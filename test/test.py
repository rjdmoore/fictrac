import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
import glob
import subprocess
import os

if (len(sys.argv) < 2):
    print('Needs at least test name!')
    sys.exit()

test_name = sys.argv[1]

print('Processing test {}'.format(test_name))

cfg_fn = test_name+'.cfg'

print('Using cfg: {}'.format(cfg_fn))

gt_csv = test_name+'_gt.csv'

print('Using gt: {}'.format(gt_csv))

if os.name == 'nt':
    cmd = "..\\bin\\Release\\fictrac.exe"
else:
    cmd = "..\\bin\\Release\\fictrac"

subprocess.run([cmd, cfg_fn])

dat_list = glob.glob(test_name+'-*.dat')

print('Found {} dat files matching pattern {}'.format(len(dat_list), test_name))

dat_fn = dat_list[-1]

print('Using dat: {}'.format(dat_fn))

with open(gt_csv, 'r') as f:
    tmp = list(csv.reader(f))
    gt = np.array([[float(i) for i in x] for x in tmp])
    
with open(dat_fn, 'r') as f:
    tmp = list(csv.reader(f))
    dat = np.array([[float(i) for i in x] for x in tmp])
    dat = dat[:,8:11]

err = np.linalg.norm(gt - dat, axis=1)
max_err = np.max(err)
end_err = err[-1]

print('max_err: {}  end_err: {}'.format(max_err, end_err))

plt.figure(1)
plt.plot(gt)
plt.plot(dat)
plt.grid()

plt.figure(2)
plt.plot(err)
plt.grid()

plt.show()
