#!/usr/bin/env python

import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import sys
from collections import defaultdict
from scipy import interpolate
from scipy import stats
import pprint

pp = pprint.PrettyPrinter(indent=4)

if len(sys.argv) < 2:
    print "USAGE: analyze_results.py <esp.txt> <lazy.txt>"
    exit(1)

ESP_FILENAME = sys.argv[1]
LAZY_FILENAME = sys.argv[2]

f_esp = open(ESP_FILENAME)
f_lazy = open(LAZY_FILENAME)
files = {f_esp, f_lazy}

csv_esp = csv.reader(f_esp)
csv_lazy = csv.reader(f_lazy)

time_axis = np.linspace(0, 25.0, 5000)
avg_trace = np.zeros(time_axis.shape)
mpl.rc('text', usetex = True)

t_data = np.zeros((2,100))
print t_data

for idx, f in enumerate(files):
    all_lines = f.read().split('\n')
    # Skip last empty line
    num_lines = len(all_lines) - 1
    print 'Num Trials: {}'.format(num_lines / 2)
    time_to_optimal_sol = []
    time_to_first_sol = []
    num_valid_trials = 0

    for ii in xrange(num_lines / 2):
        row1 = all_lines[2 * ii]
        row2 = all_lines[(2 * ii) + 1]
        times = np.fromstring(row1, dtype=float, count=-1, sep=' ')
        times = np.cumsum(times)
        costs = np.fromstring(row2, dtype=float, count=-1, sep=' ')
        opt_cost = costs[-1]
        bounds = costs / opt_cost
        time_to_opt = times[bounds < 1+1e-10][0]
        time_to_optimal_sol.append(time_to_opt)
        time_to_first_sol.append(times[0])
    t_data[idx,:] = np.array(time_to_optimal_sol)


    avg_time_to_optimal_sol = np.mean(time_to_optimal_sol)
    avg_time_to_first_sol = np.mean(time_to_first_sol)
    print avg_time_to_optimal_sol
    print avg_time_to_first_sol
    # print t_data[idx, :]

speedups = t_data[1,:] / t_data[0,:]
print speedups
print stats.gmean(speedups)
print np.mean(speedups)
plt.plot(t_data[0,:], t_data[1,:], 'o')
print np.sum(t_data[1,:] > t_data[0,:])

# plt.plot(time_axis, avg_trace, '-o')
plt.xlim(0, 10)
plt.ylim(0, 10)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
# print time_to_optimal_sol


# bounds = costs / opt_cost
    # times = np.hstack(([0], times, [1000.0]))
    # costs = np.hstack(([MAX_COST], costs, [opt_cost]))
# f = interpolate.interp1d(times, bounds, kind='zero')
#     trace = f(time_axis)
#     plt.plot(time_axis, trace, 'o-', color='r', markersize=2)
#     plt.yticks([1.0, 2.0, 3.0, 5.0], ['$1.0$', '$2.0$', '$3.0$', '$\infty$'])
#     avg_trace += trace
#     num_valid_trials = num_valid_trials + 1
#     time_to_optimal_sol = np.hstack((time_to_optimal_sol, times[-2]))
#     # print costs
#     time_to_first_sol = np.hstack((time_to_first_sol, times[1]))
