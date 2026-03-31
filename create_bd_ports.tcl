# Auto-generated TCL script to create BD ports
create_bd_port -dir O adc_enc_p_o
create_bd_port -dir I adc_clk_n_i
create_bd_port -dir O -from 13 -to 0 dac_dat_o
create_bd_port -dir O adc_enc_n_o
create_bd_port -dir O adc_csn_o
create_bd_port -dir I -from 15 -to 0 adc_dat_a_i
create_bd_port -dir I -from 15 -to 0 adc_dat_b_i
create_bd_port -dir I adc_clk_p_i
