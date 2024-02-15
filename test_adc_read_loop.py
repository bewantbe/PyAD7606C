import time
import numpy as np

n_trial = 3
n_retry_max = 4
arr_npass = np.zeros(n_trial, dtype=np.int32)

for jj in range(n_trial):
    npass = 0
    for ii in range(n_retry_max):
        try:
            exec(open('test_adc_read_p.py').read())
        except Exception as e:
            print('Exception:', e)
            adc.close()
            break
        if adc._retry_count_ == 0:
            npass = ii+1
        else:
            break
    arr_npass[jj] = npass
    print('npass:', npass)

print('n pass arr: ', arr_npass)
print(f'Mean pass: {np.mean(arr_npass):.3g}')
