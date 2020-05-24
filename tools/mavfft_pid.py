#!/usr/bin/env python

'''
fit estimate of PID oscillations
'''
from __future__ import print_function

import numpy
import pylab

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="select packets by condition")
parser.add_argument("--sample-length", type=int, default=0, help="number of samples to run FFT over")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil

def fft(logfile):
    '''display fft for PID data in logfile'''

    print("Processing log %s" % filename)
    mlog = mavutil.mavlink_connection(filename)

    data = {'PIDR.rate' : 400,
            'PIDP.rate' : 400,
            'PIDY.rate' : 400, }
    sample_rate = 400
    for gyr in ['PIDR','PIDP', 'PIDY']:
        for ax in ['P', 'I', 'D']:
            data[gyr+'.'+ax] = []

    # now gather all the data
    while True:
        m = mlog.recv_match(condition=args.condition)
        if m is None:
            break
        type = m.get_type()
        if type.startswith("PIDR"):
            data[type+'.P'].append(m.P)
            data[type+'.I'].append(m.I)
            data[type+'.D'].append(m.D)
        if type.startswith("PIDP"):
            data[type+'.P'].append(m.P)
            data[type+'.I'].append(m.I)
            data[type+'.D'].append(m.D)
        if type.startswith("PIDY"):
            data[type+'.P'].append(m.P)
            data[type+'.I'].append(m.I)
            data[type+'.D'].append(m.D)
        if type.startswith("PARM") and m.Name.startswith("SCHED_LOOP_RATE"):
            sample_rate = m.Value


    print("Extracted %u data points" % len(data['PIDR.P']))

    for msg in ['PIDR', 'PIDP', 'PIDY']:
        pylab.figure()

        for axis in ['P', 'I', 'D']:
            field = msg + '.' + axis
            d = data[field]
            fs = int(sample_rate)
            window = numpy.hanning(fs)
            sum_fft = numpy.zeros(fs//2+1)
            for i in range(0, len(d) // fs):
                dx = d[i*fs:(i+1)*fs]
                dx = numpy.array(dx)
                dx *= window
                if len(dx) == 0:
                    continue
                d_fft = numpy.fft.rfft(dx)
                d_fft = numpy.square(abs(d_fft))
                d_fft[0] = 0
                d_fft[-1] = 0
                sum_fft += d_fft
                freq  = numpy.fft.rfftfreq(fs, 1.0 / sample_rate)

            sum_fft /= (len(d) // fs)
            pylab.plot( freq, numpy.abs(sum_fft), label=field )
        pylab.legend(loc='upper right')

for filename in args.logs:
    fft(filename)

pylab.show()
