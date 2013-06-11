#!/usr/bin/env python
import Gnuplot, os, csv

g = Gnuplot.Gnuplot()
fileName = 'dvbout'
f = open(fileName, 'r')
l = list()
#g('set style data linespoints')
i = 0
for row in csv.DictReader(f, ["1", "2", "3","4","5","6","7","8","9"]):
    l.append(int(row['9'])/1024/1024)
    i = i +1
    if (i > 120):
        break

# g('set terminal png')
g('set yr [0:26]')
g('set output "%s.png"' % fileName)
g.plot(l)
f.close()
raw_input('...\n')
