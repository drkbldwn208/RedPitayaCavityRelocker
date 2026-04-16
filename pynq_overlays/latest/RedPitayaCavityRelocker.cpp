#include "relock.h"
#include <cmath>

void cavity_relocker(
    hls::stream<axis_t> &adc_in,
    hls::stream<axis_t> &dac_out,
    int threshold,
    volatile int *ps_status_flag,
    int lock_voltage_in,
    volatile int *pdh_waveform,
    int arm_enable,
    volatile int *pi_rail_interrupt,
    int sideband_offset,
    int servo_offset,
    volatile int *offset_estimate_out
) {
    #pragma HLS PIPELINE II=4
    #pragma HLS INTERFACE axis port=adc_in
    #pragma HLS INTERFACE axis port=dac_out
    #pragma HLS INTERFACE s_axilite port=threshold
    #pragma HLS INTERFACE s_axilite port=ps_status_flag
    #pragma HLS INTERFACE s_axilite port=lock_voltage_in
    #pragma HLS INTERFACE s_axilite port=arm_enable
    #pragma HLS INTERFACE s_axilite port=sideband_offset
    #pragma HLS INTERFACE s_axilite port=servo_offset
    #pragma HLS INTERFACE s_axilite port=pi_rail_interrupt
    #pragma HLS INTERFACE s_axilite port=offset_estimate_out
    #pragma HLS INTERFACE s_axilite port=return
    #pragma HLS INTERFACE m_axi port=pdh_waveform depth=32768 offset=slave bundle=gmem

    static State current_state = D;
    static unsigned long long timer = 0;
    static int rolling_timer = 0;
    static long long avg_accumulator = 0;
    static int rolling_avg = 0;
    static int held_vco_voltage = 0;
    static int held_sweep_voltage = 0;
    static int mem_index = 0;
    static int decim_counter = 0;
    static int internal_lock_voltage = 0;
    static int offload_timer = 0;    
    static bool from_interrupt = false;

    axis_t val_adc = adc_in.read();
    axis_t val_dac;    

    short val_pdh = (short)(val_adc.data & 0xFFFF); // Channel A
    short val_pi = (short)((val_adc.data >> 16) & 0xFFFF); // Right shift reads the upper 16 bits, so this is channel B

    short out_pi = 0;
    short out_vco = 0;

    static short hold_ramp_step = 0;
    static int long long hold_ramp_accum = 0;

    static short reengage_ramp_step = 0;
    static int long long reengage_ramp_accum = 0;

    static short d_state_counter = 0;
    static short rail_detect_counter = 0;

    static int adc_offset_estimate = 0;

    // Calculate rolling average at 1 Hz (roughly 2 << 27)
    // Wait two accumulator cycles after switching from state R
    if (current_state == D) { 
        if (rolling_timer < ROLLING_AVG_LIMIT) {
            avg_accumulator += val_pi;
            rolling_timer++;
        } else {
            rolling_avg = avg_accumulator >> ROLLING_AVG_SHIFT;
            avg_accumulator = 0;
            rolling_timer = 0;
            if (d_state_counter < D_STATE_DEAD_CYCLES) {
                d_state_counter++;
            } else {
                int offset_error = rolling_avg - adc_offset_estimate;
                adc_offset_estimate = adc_offset_estimate + (offset_error >> EMA_SHIFT); 
            }
            if (std::abs(rolling_avg) > 6000) {
                rail_detect_counter++;                
            } else {
                rail_detect_counter = 0;
            }
        }
    }

    int active_offset = (servo_offset == 0) ? adc_offset_estimate : servo_offset;
    *offset_estimate_out = adc_offset_estimate;    
    short zeroing_servo = ((val_pi - active_offset) >> ZERO_SERVO_SHIFT);
    
    switch (current_state) {
        case D: { // Detecting state
            out_pi = val_pdh;
            out_vco = val_pi + internal_lock_voltage;

            if (offload_timer > 31250) {
                if (internal_lock_voltage > sideband_offset) {
                    internal_lock_voltage--;
                } else if (internal_lock_voltage < sideband_offset) {
                    internal_lock_voltage++;                    
                }
                offload_timer = 0;
            } else {
                offload_timer++;
            }

            bool armed = (arm_enable == 1 || arm_enable == 2);
            bool rail_detect_active = (arm_enable == 1 || arm_enable == 3);

            if (!rail_detect_active) {
                rail_detect_counter = 0;
            }            
            
            if ((rail_detect_counter >= RAIL_DETECT_CONFIRM_CYCLES) && 
                (d_state_counter >= D_STATE_DEAD_CYCLES) &&
                rail_detect_active) {
                //Change state to interrupt, toggle interrupt flag
                current_state = I;
                timer = 0;
                *pi_rail_interrupt = 1;    
                held_vco_voltage = out_vco;
                rail_detect_counter = 0;
            } else if ((std::abs(val_pi - rolling_avg) > threshold) && 
                        armed && 
                        (d_state_counter >= D_STATE_DEAD_CYCLES)) {
                //Change state to hold, capture last voltage
                current_state = H;
                timer = 0;
                held_vco_voltage = out_vco; 
                *ps_status_flag = 0;
                hold_ramp_accum = (long long)held_vco_voltage << HOLD_TIMEOUT_CYCLES_SHIFT;
                hold_ramp_step = 8192 + held_vco_voltage;
            }
            break;
        }

        case H: // Hold
            if (timer > HOLD_TIMEOUT_CYCLES) {
                out_pi = zeroing_servo;
                out_vco = -8192;
            } else {
                out_pi = zeroing_servo;
                out_vco = (short)(hold_ramp_accum >> HOLD_TIMEOUT_CYCLES_SHIFT);
                hold_ramp_accum -= hold_ramp_step;
            }
            
            timer ++;

            // Wait a few seconds (5s = 625000000) and switch to sweep
            if (timer > HOLD_TIMEOUT_CYCLES + HOLD_TIMEOUT_PAUSE_CYCLES) {
                current_state = S;
                timer = 0;
                mem_index = 0;
                decim_counter = 0;
                if (from_interrupt) {
                    *pi_rail_interrupt = 0;
                    from_interrupt = false;                    
                }
            }
            break;

        case S: //Sweep
            out_pi = zeroing_servo;
            out_vco = -8192 + ((timer << 14) >> SWEEP_TIMEOUT_CYCLES_SHIFT);

            decim_counter++;
            if (decim_counter >= DECIMATION_FACTOR) {
                pdh_waveform[mem_index] = val_pdh;
                mem_index++;
                decim_counter = 0;
            }

            held_sweep_voltage = out_vco;

            timer++;
            if (timer >= SWEEP_TIMEOUT_CYCLES) {
                // Switch to wait and flip PS status
                current_state = W;
                *ps_status_flag = 1;
            }
            break;

        case W: // Wait for PS to flip status flag to 2
            out_pi = zeroing_servo;
            out_vco = held_sweep_voltage; 

            if (*ps_status_flag == 2) {
                current_state = R;
                timer = 0;
                reengage_ramp_accum = (long long)held_sweep_voltage << REENGAGE_TIMEOUT_CYCLES_SHIFT;
                reengage_ramp_step = held_sweep_voltage - lock_voltage_in;
            }
            break;
        
        case R: // Re-engage
            timer ++;
            // 10 ms = 1250000
            if (timer > REENGAGE_TIMEOUT_CYCLES) {
                out_vco = lock_voltage_in;
                out_pi = zeroing_servo;
            } else {
                out_pi = zeroing_servo;
                out_vco = (short)(reengage_ramp_accum >> REENGAGE_TIMEOUT_CYCLES_SHIFT);
                reengage_ramp_accum -= reengage_ramp_step;
            }
            if (timer > REENGAGE_TIMEOUT_CYCLES + REENGAGE_DWELL_CYCLES) {      
                internal_lock_voltage = lock_voltage_in;
                current_state = D;
                timer = 0;
                avg_accumulator = 0;
                rolling_timer = 0;
                *ps_status_flag = 0;
                d_state_counter = 0;
                adc_offset_estimate = 0;
            }
            break;

        case I: //PI rail interrupt block, do nothing until user manually resets all lockboxes
            out_pi = zeroing_servo;
            out_vco = held_vco_voltage;
            if (*pi_rail_interrupt == 2) {
                current_state = H;
                timer = 0;
                hold_ramp_accum = (long long)held_vco_voltage << HOLD_TIMEOUT_CYCLES_SHIFT;
                hold_ramp_step = 8192 + held_vco_voltage;
                d_state_counter = 0;
                internal_lock_voltage = 0;
                from_interrupt = true;
            }
            break;

    }

    val_dac.data = (((uint32_t)out_vco & 0xFFFF) << 16) | ((uint32_t)out_pi & 0xFFFF); //Out_vco goes to channel B, out_pi goes to channel A

    val_dac.keep = 0xF;
    val_dac.strb = 0xF;
    val_dac.last = 0;

    dac_out.write(val_dac);
}