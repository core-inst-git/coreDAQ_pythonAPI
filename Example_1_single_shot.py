from coredaq_py_api import CoreDAQ
import time
import numpy as np

daq = CoreDAQ("/dev/tty.usbmodem2054396453331") # Set your CoreDAQ port here
daq.set_freq(100000)
print("Device:", daq.idn())

# Snapshot (quick voltage read) with 5 frames averaging
print("Snapshot 5 frames :", np.array(daq.snapshot_adc(32)[0])) # Output in mv


daq.close() # Close the connection cleanly


