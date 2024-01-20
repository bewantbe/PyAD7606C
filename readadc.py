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

    conf = adc.get_config()
    byADCOptions = conf.byADCOptions & (0xFF ^ 0x20)
    adc.config(byADCOptions = byADCOptions,
               dwPeriod = 2**31-1,
               dwCycleCnt = 0,
               dwMaxCycles = 2000)

    adc.set_register('reset')
    reg_list = list(adc.get_register()) + [0xff]
    print('reg:')
    for ii in range(6):
        for jj in range(8):
            print(f"0x{reg_list[ii*8+jj]:02X}", end=' ')
        print('')
    print('')

    if 1:
        t0 = time.time()
        print('')
        adc.show_config()     # cost about 0.0011 seconds
        print('')
        #adc.show_reg()        # cost about 0.0025 seconds
        t1 = time.time()
        print('time cost: ', t1-t0, 's (show config, reg)')

    if 0:
        t0 = time.time()
        adc.sampling()        # cost about 1 seconds
        t1 = time.time()
        print('time cost: ', t1-t0, 's (single sampling)')

    if 1:
        n_frame_total = 0
        n_continued_empty_frames = 0

        t0 = time.time()
        adc.start()
        for i in range(100):
            dbg_print(4, '------- loop', i)
            v, n_left = adc.read()   # non-block
            if len(v) == 0 and n_left == 0:
                n_continued_empty_frames += 1
            else:
                n_continued_empty_frames = 0
            if n_continued_empty_frames > 10:
                break
            if len(v) == 0:
                time.sleep(0.01)
                continue
            print(f'data size {len(v)} = {len(v)//8}*8,  data left: {n_left}')
            print('data mean', np.mean(v))
            n_frame_total += len(v) // 8
            if n_left > 8192:
                pass
            else:
                time.sleep(0.1)
        adc.stop()

        dbg_print(4, '------- loop ends')
        v, n_left = adc.read()
        dbg_print(4, 'data size', len(v))
        if len(v):
            dbg_print(4,'data mean', np.mean(v))
        n_frame_total += len(v) // 8
        print('total frame', n_frame_total)
        t1 = time.time()
        print('time cost: ', t1-t0, 's')

    if 0:
        # waiting Ctrl-C in a while loop
        while True:
            time.sleep(10)
            dbg_print(4, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
            pass

    adc.close()
