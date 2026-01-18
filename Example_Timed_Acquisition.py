from matplotlib import use
from coredaq_py_api import CoreDAQ
import time

daq = CoreDAQ("/dev/tty.usbmodem2057396453331") # Set your CoreDAQ port here

print("Device:", daq.idn()) # Print device identification

print("Device Frontend Type:", daq.frontend_type()) # Identify connected frontend variant


NUM_SAMPLES = 500_000
SAMPLING_RATE = 50_000

# Sampling rate
daq.set_freq(SAMPLING_RATE) 

daq.arm_acquisition(NUM_SAMPLES) #Arm - No External Trigger
daq.start_acquisition() #Start Acquisition

#Incase of an external trigger starting acquisiton, eg. A swept laset
#daq.arm_acquisition(NUM_SAMPLES, use_trigger=True, trigger_rising=True)

start=time.time()

time.sleep(NUM_SAMPLES/SAMPLING_RATE + 0.5 )  # wait for acquisition to complete

end = time.time()

print(f"Acquired 100_000 frames in {end-start:.2f} seconds")

ch = daq.transfer_frames_W(NUM_SAMPLES) # Dump acquired frames in Watts

#Data strucutre is a list of 4 lists, each containing the samples for that channel
print("Ch1 first 5 samples (W):", ch[0][:5])
print("Ch2 first 5 samples (W):", ch[1][:5])
print("Ch3 first 5 samples (W):", ch[2][:5])
print("Ch4 first 5 samples (W):", ch[3][:5])


daq.close()  # Close the connection cleanly

