#!/usr/bin/env python
"""
/proc/net/dev
Inter-|   Receive                                                |  Transmit
 face |bytes    packets errs drop fifo frame compressed multicast|bytes    packets errs drop fifo colls carrier compressed
  dvb0: 2025233886  989970   38    0    0 61064460          0         0 46790380  248885    0    0    0     0 1833788          0
    lo: 304812933 2547545    0    0    0     0          0         0 304812933 2547545    0    0    0     0       0          0
  eth1: 47689869441 176087325    1    0    0     1          0     55726 28763513507 197100289    0    0    0     0       0          0
  eth0: 48080420986 177082381    0    0    0     0          0     55831 28751598346 197101556    0    0    0     0       0          0
 bond0: 95770290427 353169706    1    0    0     1          0    111557 57515111853 394201845    0    0    0     0       0          0

"""

import time, os, sys

def findIF(devName):
    with open('/proc/net/dev') as f:
        for line in f:
            l = line.split(':')
            name = l[0].strip()
            if devName == name:
                return l[1].strip()

def main(argv):
    devName = argv[0];
    seconds = int(argv[1])
    for i in range(0, seconds):
        print(','.join(findIF(argv[0]).split())) 
        time.sleep(1)

if __name__ == '__main__':
    main(sys.argv[1:])
