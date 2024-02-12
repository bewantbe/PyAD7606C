import time
import numpy as np

from M3F20xm import M3F20xmADC


volt_range = [-2.5, 2.5]
ends = 'single'
sample_rate = 500e3
per_period_size = 16384

adc = M3F20xmADC(reset = True)
adc.set_input_range(volt_range, ends)
adc.set_sampling_rate(sample_rate)
n_channel = 8
sample_rate = 1.0 / adc.get_sampling_interval()
print('Actual sample rate:', sample_rate)

t_sampling = 3600.0  # seconds
n_sample = sample_rate * t_sampling
n_round = int(n_sample // per_period_size) + 10

sz_arr = -np.ones(n_round, dtype=np.int32)

n_read = 0
ok = False

s_dummy = np.zeros(n_channel, dtype = np.complex128)

adc.start(per_period_size)
t1 = time.time()

for i in range(n_round):
    tt = time.time() - t1
    t_wait = (i + 2) * per_period_size / sample_rate - tt
    if t_wait > 0:
        time.sleep(t_wait)
    ivolt = adc.read()
    sz_arr[i] = len(ivolt)
    n_read += len(ivolt) // n_channel
    if n_read >= n_sample:
        ok = True
        break
    # payload
    s_dummy += np.fft.fft(np.reshape(ivolt, (-1, n_channel))/32768.0, axis=0)[2]

print('s_dummy =', s_dummy)

t2 = time.time()
adc.stop()

print('sz', sz_arr[:80])
print('')

print('Read', n_read, 'samples.', ' i =', i, ' of n_round =', n_round)
print('time:', t2 - t1, 's')
if ok:
    print('Enough data was collected.')

adc.close()