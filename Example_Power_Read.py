from coredaq_py_api import CoreDAQ
import time


daq = CoreDAQ("/dev/tty.usbmodem2057396453331") # Set your CoreDAQ port here

print("Device:", daq.idn()) # Identification string

print("Device Frontend Type:", daq.frontend_type()) # Identify connected frontend variant

daq.set_gain1(0)  # CH1 gain index = 0
                  # Transimpedance / power range mapping:
                  #   G0 → ~1 kΩ   (≈ 4 mW max)
                  #   G1 → ~2 kΩ   (≈ 2 mW)
                  #   G2 → ~5 kΩ   (≈ 800 µW)
                  #   G3 → ~10 kΩ  (≈ 400 µW)
                  #   G4 → ~50 kΩ  (≈ 80 µW)
                  #   G5 → ~100 kΩ (≈ 40 µW)
                  #   G6 → ~1 MΩ   (≈ 4 µW)
                  #   G7 → ~10 MΩ  (≈ 400 nW)
                  # Higher index = higher transimpedance (more sensitivity, lower max power)

# Sampling rate
daq.set_freq(50_000)

# Measure a snapshot with average of 5 frames 
print("Snapshot 5 frames (mV):", daq.snapshot_mV(5)[0]) # Output in mV 

print("Gains : ", daq.snapshot_mV(5)[1]) # Returns current gain indices

print("Snapshot 5 frames (Watts):", daq.snapshot_W(5)) # Output in Watts

daq.close() # Close the connection cleanly






