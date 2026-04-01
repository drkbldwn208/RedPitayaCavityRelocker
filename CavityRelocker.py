import time
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from pynq import Overlay, allocate

bitfile = "RedPitayaCavityRelocker.bit"

print("Loading overlay...")
overlay = Overlay(bitfile)
relocker = overlay.cavity_relocker_0

print("Register map check:")
print(relocker.register_map)

# HLS IP writes 31.25 M samples during the sweep, int data type, so 125 MB of DDR4
print("Allocating memory for sweep data...")
sweep_buffer = allocate(shape=(31250000,), dtype='i4')

relocker.register_map.pdh_waveform_1 = sweep_buffer.physical_address
relocker.register_map.pdh_waveform_2 = sweep_buffer.physical_address >> 32

print(f"Buffer allocated at physical address: {hex(sweep_buffer.physical_address)}")

def pdh_dispersive(x, A, x0, gamma, offset):
  return A * (x - x0) / ((x - x0)**2 + gamma**2) + offset

def find_lock_voltage(raw_data, timer_max = 31250000, vmin = -8192, vmax = 8191):
  decimation_factor = 1000
  y_data = raw_data[::decimation_factor]

  timer_indices = np.arange(0, timer_max, decimation_factor)
  x_voltage = v_min + (vmax - vmin) * timer_indices / timer_max

  v_offset = np.mean(y_data)
  v_amplitude = (np.max(y_data) - np.min(y_data))

  zero_crossings = np.where(np.diff(np.sign(y_data - v_offset)))[0]
  guess_center = x_voltage[len(zero_crossings) // 2] if len(zero_crossings) > 0 else 0
  guess_gamma = (v_max - v_min) / 10

  p0 = [v_amplitude, guess_center, guess_gamma, v_offset]

  try:
    popt, _ = curve_fit(pdh_dispersive, x_voltage, y_data, p0=p0)
    lock_voltage = popt[1]
    lock_point = np.clip(lock_voltage, vmin, vmax)
    return int(lock_point), x_voltage, y_data, popt
  
  except RuntimeError:
    print("Curve fitting failed, returning default lock voltage.")
    return 0, x_voltage, y_data, p0
  
relocker.register_map.threshold = 1500

relocker.register_map.ps_status_flag = 0

relocker.write(0x00, 1)  # Arm relocker
print("Relocker is armed")

try:
  while True:
    if relocker.register_map.ps_status_flag == 1:
      start_time = time.time()
      print("Unlock detected, sweep complete and processing data...")

      sweep_data = np.copy(sweep_buffer)

      lock_voltage, x_volts, y_data, popt = find_lock_voltage(sweep_data)
      print(f"Calculated lock voltage: {lock_voltage}")

      relocker.register_map.lock_voltage = lock_voltage

      relocker.register_map.ps_status_flag = 2

      calc_time = time.time() - start_time
      print(f"Data processing time: {calc_time:.3f} seconds")

      time.sleep(.1)

  time.sleep(.01)
except KeyboardInterrupt:
  print("Exiting...")
  relocker.write(0x00, 0x00)
  sweep_buffer.freebuffer()
  print("Resources cleaned up, goodbye!")



