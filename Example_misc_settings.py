"""
Example: Miscellaneous settings and environment logging

This example demonstrates:
  - Oversampling (OS) selection and its effect on max sampling rate
  - Safe sampling rate frequency configuration
  - Reading ambient temperature and humidity
"""

from coredaq_py_api import CoreDAQ
import time

# ----------------------------------------------------------------------
# Connect to device
# ----------------------------------------------------------------------
daq = CoreDAQ("/dev/tty.usbmodem2057396453331") # Set your CoreDAQ port here

print("Device:", daq.idn())
print("Frontend:", daq.frontend_type())

# ----------------------------------------------------------------------
# Oversampling & sampling frequency
# ----------------------------------------------------------------------
# Oversampling trades bandwidth for improved SNR.
#
# OS index → max supported sampling rate:
#   OS 0 → 100 kS/s
#   OS 1 → 100 kS/s
#   OS 2 → 50  kS/s
#   OS 3 → 25  kS/s
#   OS 4 → 12.5 kS/s
#   OS 5 → 6.25 kS/s
#   OS 6 → 3.125 kS/s
#
# Higher OS = lower noise, lower bandwidth.

daq.set_oversampling(4)     #  OS 0 gives you low noise measurements but incase higher SNR is required, increase OS index
daq.set_freq(10_000)        # 10 kS/s is valid at OS4

print(f"Oversampling index : {daq.get_oversampling()}")
print(f"Sampling frequency: {daq.get_freq_hz()} Hz")


# ----------------------------------------------------------------------
# Read environmental sensors # CoreDAQ is equipped with temp/humidity sensors
# ----------------------------------------------------------------------
T_ambient = daq.get_head_temperature_C()
RH = daq.get_head_humidity()
T_die = daq.get_die_temperature_C()

print(f"Ambient temperature : {T_ambient:.2f} °C")
print(f"Ambient humidity    : {RH:.1f} %")
print(f"Device die temp     : {T_die:.2f} °C")

