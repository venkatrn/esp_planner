#!/usr/bin/env python

import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import sys
from collections import defaultdict
from scipy import interpolate
import pprint

pp = pprint.PrettyPrinter(indent=4)

if len(sys.argv) < 2:
    print "USAGE: analyze_results.py <esp.txt> <lazy.txt>"
    exit(1)

ESP_FILENAME = sys.argv[1]
LAZY_FILENAME = sys.argv[2]

f_esp = open(ESP_FILENAME)
f_lazy = open(LAZY_FILENAME)

csv_esp = csv.reader(f_esp)
csv_lazy = csv.reader(f_lazy)

all_lines = f_esp.read().split('\n')
# Skip last empty line
num_lines = len(all_lines) - 1
print 'Num Trials: {}'.format(num_lines)

time_axis = np.linspace(0, 25.0, 5000)
MAX_BOUND = 5
MAX_COST = 1e9


avg_trace = np.zeros(time_axis.shape)
time_to_optimal_sol = np.zeros(1)
time_to_first_sol = np.zeros(1)
num_valid_trials = 0
mpl.rc('text', usetex = True)
for ii in xrange(num_lines / 2):
    # print ii
    row1 = all_lines[2 * ii]
    row2 = all_lines[(2 * ii) + 1]
    times = np.fromstring(row1, dtype=float, count=-1, sep=' ')
    times = np.cumsum(times)
    costs = np.fromstring(row2, dtype=float, count=-1, sep=' ')
    opt_cost = costs[-1]
    if opt_cost == MAX_COST:
        continue
    times = np.hstack(([0], times, [1000.0]))
    costs = np.hstack(([MAX_COST], costs, [opt_cost]))
    bounds = costs / opt_cost
    mask = (costs > MAX_COST - 1)
    mask[-1] = 0
    mask[-2] = 0
    # bounds = bounds[mask]
    # times = times[mask]
    bounds[mask] = MAX_BOUND
    # print bounds
    # Interpolate intermediate values
    f = interpolate.interp1d(times, bounds, kind='zero')
    trace = f(time_axis)
    plt.plot(time_axis, trace, 'o-', color='r', markersize=2)
    plt.yticks([1.0, 2.0, 3.0, 5.0], ['$1.0$', '$2.0$', '$3.0$', '$\infty$'])
    avg_trace += trace
    num_valid_trials = num_valid_trials + 1
    time_to_optimal_sol = np.hstack((time_to_optimal_sol, times[-2]))
    valid_times = times[~mask]
    # print costs
    print valid_times
    time_to_first_sol = np.hstack((time_to_first_sol, valid_times[0]))
avg_trace /= num_valid_trials
avg_time_to_optimal_sol = np.mean(time_to_optimal_sol)
avg_time_to_first_sol = np.mean(time_to_first_sol)
# plt.plot(time_axis, avg_trace, '-o')
plt.show()
# print avg_trace
print avg_time_to_optimal_sol
print avg_time_to_first_sol
