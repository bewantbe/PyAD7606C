import time
import numpy as np

from M3F20xm import M3F20xmADC

def log10_(x):
    # log of 0 is -inf, don't raise error or warning
    with np.errstate(divide='ignore'):
        return np.log10(x)

# set user parameter
sample_rate = 500e3
#per_period_size = 16384
#per_period_size = 0.1 * sample_rate  # this controls RMS update rate
per_period_size = 4096
t_sampling = 3600.0 * 24 * 365  # seconds
volt_range = [0, 5]
ends = 'single'

# initialize ADC
adc = M3F20xmADC(reset = True)
adc.set_input_range(volt_range, ends)
adc.set_sampling_rate(sample_rate)
n_channel = 8
sample_rate = 1.0 / adc.get_sampling_interval()
print('Actual sample rate:', sample_rate)

# set for-loop parameters
n_sample = sample_rate * t_sampling
n_round = int(n_sample // per_period_size) + 10

# sample counting
n_read = 0
ok = False

# start ADC
adc.start(int(per_period_size))
t1 = time.time()

try:
    for i in range(n_round):
        # wait for a peroid of data
        if 0:
            # frame rate by host wall time
            tt = time.time() - t1
            t_wait = (i + 2) * per_period_size / sample_rate - tt
            # can be used to test FIFO buffer size and behavior.
            n_frames_left = adc.get_fifo_frames_left()
        else:
            # frame rate by ADC FIFO
            n_frames_left = adc.get_fifo_frames_left()
            buffer_period_margin = 2 * per_period_size
            t_wait = (buffer_period_margin - n_frames_left) / sample_rate
        if t_wait > 0:
            time.sleep(t_wait)
        # read from ADC
        ivolt = adc.read()
        # Compute RMS in dB and show it as simple ASCII art
        v = np.reshape(ivolt, (-1, n_channel)) / 32768.0
        # ASCII volume bar
        res_n = 8  # character length of each dB bar
        bl = -70
        bu = -20
        print(f'i: {i:4d} |', end='')
        print(f't ={i*per_period_size/sample_rate:6.1f} |', end='')
        print(f' n_r = {v.shape[0]} | t_r ={v.shape[0]/sample_rate:8.5f} |', end='')
        print(f' n_l = {n_frames_left} | t_l ={n_frames_left/sample_rate:8.5f} |', end='')
        for c in range(n_channel):
            u = v[:, c] - np.mean(v[:, c])
            dbu = 20.0 * log10_(np.sqrt(np.mean(u**2)))
            dbu_scaled = int(np.clip((dbu - bl)/(bu-bl) * res_n, 0, res_n))
            print('#'*dbu_scaled + ' '*(res_n - dbu_scaled), end='|')
        print()
        # get out of the loop if we have enough data
        n_read += len(ivolt) // n_channel
        if n_read >= n_sample:
            ok = True
            break
except KeyboardInterrupt:
    print('Interrupted by user.')

t2 = time.time()
adc.stop()

print('Read', n_read, 'samples.', ' i =', i, ' of n_round =', n_round)
print(f'Wall   time: {t2 - t1:.6g} s')
print(f'Sample time: {n_read / sample_rate:.6g} s')
if ok:
    print('Enough data was collected.')

adc.close()