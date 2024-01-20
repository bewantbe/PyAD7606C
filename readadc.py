#!/usr/bin/env python3
# Note: copy USB2DaqsB.dll along with this script.
# Run: python readadc.py

import time
import numpy as np

from M3F20xm import M3F20xmADC, dbg_print

if __name__ == '__main__':

    t0 = time.time()
    adc = M3F20xmADC()    # cost about 2~3 seconds
    t1 = time.time()
    print('time cost: ', t1-t0, 's (open device)')

    if 1:
        conf = adc.get_config()
        byADCOptions = conf.byADCOptions & (0xFF ^ 0x20) | 0x20
        adc.config(byADCOptions = byADCOptions,
                   wPeriod = 1000,
                   dwCycleCnt = 0,
                   dwMaxCycles = 10)

        adc.set_register('reset')

    if 0:
        reg_list = list(adc.get_register()) + [0xff]
        print('reg:')
        for ii in range(6):
            for jj in range(8):
                print(f"0x{reg_list[ii*8+jj]:02X}", end=' ')
            print('')
        print('')

    if 1:
        print('')
        adc.show_config()     # cost about 0.0011 seconds
        #print('')
        #adc.show_reg()        # cost about 0.0025 seconds

    if 0:
        t0 = time.time()
        adc.sampling()        # cost about 1 seconds
        t1 = time.time()
        print('time cost: ', t1-t0, 's (single sampling)')

    if 1:
        n_frame_total = 0
        n_continued_empty_frames = 0
        t_wait = 4 * adc.get_sampling_interval()
        ii = 0

        t0 = time.time()
        adc.start()
        while n_frame_total < adc.get_config().dwMaxCycles:
            ii += 1
            dbg_print(4, '------- loop', ii)
            v, n_left = adc.read()   # non-block
            if len(v) == 0 and n_left == 0:
                n_continued_empty_frames += 1
                if n_continued_empty_frames > 1:
                    break
                time.sleep(max(0.01, t_wait))
                continue
            else:
                n_continued_empty_frames = 0
            print(f'data size {len(v)} = {len(v)//8}*8,  data left: {n_left}')
            if len(v) > 0:
                print('data mean', np.mean(v))
            n_frame_total += len(v) // 8
        adc.stop()

        print('total frame', n_frame_total)
        t1 = time.time()
        print('time cost: ', t1-t0, 's, ii = ', ii)

    #print('')
    #adc.show_config()     # cost about 0.0011 seconds

    if 0:
        # waiting Ctrl-C in a while loop
        while True:
            time.sleep(10)
            dbg_print(4, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
            pass

    adc.close()
