#ifndef CAVITY_RELOCKER_H
#define CAVITY_RELOCKER_H

#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include <stdint.h>

typedef ap_axiu<32,0,0,0> axis_t;
enum State {D, H, S, W, R, I};

#ifndef ROLLING_AVG_SHIFT
    #define ROLLING_AVG_SHIFT 23
#endif

#ifndef ROLLING_AVG_LIMIT
    #define ROLLING_AVG_LIMIT (1 << ROLLING_AVG_SHIFT)
#endif

#ifndef ZERO_SERVO_SHIFT
    #define ZERO_SERVO_SHIFT 3
#endif

#ifndef HOLD_TIMEOUT_CYCLES_SHIFT
    #define HOLD_TIMEOUT_CYCLES_SHIFT 25
#endif

#ifndef HOLD_TIMEOUT_CYCLES
    #define HOLD_TIMEOUT_CYCLES (1 << HOLD_TIMEOUT_CYCLES_SHIFT)
#endif

#ifndef HOLD_TIMEOUT_PAUSE_CYCLES
    #define HOLD_TIMEOUT_PAUSE_CYCLES (1 << 24)
#endif

#ifndef SWEEP_TIMEOUT_CYCLES_SHIFT
    #define SWEEP_TIMEOUT_CYCLES_SHIFT 25
#endif

#ifndef SWEEP_TIMEOUT_CYCLES
    #define SWEEP_TIMEOUT_CYCLES (1 << SWEEP_TIMEOUT_CYCLES_SHIFT)
#endif

#ifndef REENGAGE_TIMEOUT_CYCLES_SHIFT
    #define REENGAGE_TIMEOUT_CYCLES_SHIFT 21
#endif

#ifndef REENGAGE_TIMEOUT_CYCLES
    #define REENGAGE_TIMEOUT_CYCLES (1 << REENGAGE_TIMEOUT_CYCLES_SHIFT)
#endif

#ifndef DECIMATION_FACTOR_SHIFT
    #define DECIMATION_FACTOR_SHIFT 8
#endif

#ifndef DECIMATION_FACTOR
    #define DECIMATION_FACTOR (1 << DECIMATION_FACTOR_SHIFT)
#endif

#ifndef SWEEP_TIMEOUT_DECIM_CYCLES 
    #define SWEEP_TIMEOUT_DECIM_CYCLES (1 << (SWEEP_TIMEOUT_CYCLES_SHIFT - DECIMATION_FACTOR_SHIFT))
#endif

#ifndef D_STATE_DEAD_CYCLES
    #define D_STATE_DEAD_CYCLES 5
#endif

#ifndef RAIL_DETECT_CONFIRM_CYCLES
    #define RAIL_DETECT_CONFIRM_CYCLES 3
#endif 

#ifndef REENGAGE_DWELL_CYCLES_SHIFT
    #define REENGAGE_DWELL_CYCLES_SHIFT 21
#endif

#ifndef REENGAGE_DWELL_CYCLES
    #define REENGAGE_DWELL_CYCLES (1 << REENGAGE_DWELL_CYCLES_SHIFT)
#endif 

#ifndef EMA_SHIFT 
    #define EMA_SHIFT 3
#endif

void cavity_relocker(
    hls::stream<axis_t> &adc_in,
    hls::stream<axis_t> &dac_out,
    int threshold,
    volatile int *ps_status_flag,
    int lock_voltage_in,
    volatile int *pdh_waveform,
    int arm_enable,
    int sideband_offset
);

#endif