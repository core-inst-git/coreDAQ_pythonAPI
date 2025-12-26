from coredaq_py_api import CoreDAQ
import time

daq = CoreDAQ("/dev/tty.usbmodem2050396453331") # Set your CoreDAQ port here

print("Device:", daq.idn())

# Snapshot (quick voltage read) with 5 frames averaging
print("Snapshot 5 frames (mV):", daq.snapshot_mV(5)) # Output in mv


daq.close() # Close the connection cleanly


