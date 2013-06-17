#!/usr/bin/env python
import os
import Gnuplot
import csv
import sys


def plot(fileName, maxCount):
    l = list()
    with open(fileName) as f:
        i = 0
        for row in csv.DictReader(f, ["1", "2", "3", "4", "5", "6", "7", "8", "9"]):
            l.append(int(row['9']) / 1024 / 1024)
            i += 1
            if i > maxCount:
                break

    print(l)
    g = Gnuplot.Gnuplot()
    #g('set style data linespoints')
    g('set yr [0:26]')
    #g('set terminal png')
    #g('set output "%s.png"' % fileName)
    g.plot(l)
    raw_input('...\n')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('usage: ./%s [file name]\n' % os.path.basename(sys.argv[0]))
        sys.exit(1)

    plot(sys.argv[1], 99999)
