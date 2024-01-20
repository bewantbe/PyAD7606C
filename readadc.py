#!/usr/bin/env python3
# Note: copy USB2DaqsB.dll along with this script.
# Run: python readadc.py

import time
import numpy as np

from M3F20xm import M3F20xmADC

if __name__ == '__main__':

    adc = M3F20xmADC()
    adc.sampling()

    adc.show_config()
    adc.show_reg()

    if 0:
        adc.start()
        for i in range(10):
            dbg_print(4, '------- loop', i)
            v = adc.read()
            dbg_print(4, 'data size', len(v))
            dbg_print(4, 'data mean', np.mean(v))
            time.sleep(0.1)
        adc.stop()

        dbg_print(4, '------- loop ends')
        v = adc.read()
        dbg_print(4, 'data size', len(v))
        dbg_print(4,'data mean', np.mean(v))

    if 0:
        # waiting Ctrl-C in a while loop
        while True:
            time.sleep(10)
            dbg_print(4, time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
            pass

    adc.close()
