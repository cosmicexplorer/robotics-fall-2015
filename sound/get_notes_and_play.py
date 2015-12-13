from wavebender import *

rate = 44100
num_secs = 2

channels = ((square_wave(440, amplitude=.1),),)
samples = compute_samples(channels, rate * num_secs)
write_wavefile(stdout, samples, rate * num_secs, nchannels=1)
