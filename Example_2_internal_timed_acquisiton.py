from coredaq_py_api import CoreDAQ
import time
from matplotlib import pyplot as plt
import numpy as np


daq = CoreDAQ("/dev/tty.usbmodem2054396453331") # Set your CoreDAQ port here
daq.set_freq(100000)
print("Device:", daq.idn())

# Streaming capture

daq.set_freq(100_000) 
# Sampling rate

#No External Trigger
daq.arm_acquisition(200_000, use_trigger=False)

daq.start_acquisition()



time.sleep(3)  # Wait for acquisition to complete

# Bulk transfer raw data → Watt arrays
ch = daq.transfer_frames_raw(200_000)

print("Ch1 first 5 samples (W):", ch[0][:5])
print("Ch2 first 5 samples (W):", ch[1][:5])
print("Ch3 first 5 samples (W):", ch[2][:5])
print("Ch4 first 5 samples (W):", ch[3][:5])


LOW  = 10
HIGH = 50

for ci, ch_data in enumerate(ch):
    arr = np.asarray(ch_data)

    bad_idx = np.where((arr < LOW) | (arr > HIGH))[0]

    if len(bad_idx) == 0:
        print(f"CH{ci+1}: OK (no out-of-range samples)")
    else:
        print(f"CH{ci+1}: {len(bad_idx)} out-of-range samples")
        print(f"  First few bad indices: {bad_idx[:10].tolist()}")
        print(f"  Values: {arr[bad_idx[:10]].tolist()}")

daq.close()  # Close the connection cleanly

