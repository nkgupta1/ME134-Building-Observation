# Data given like: .csv file
# id, timestamp, x, y, z
# if id=0 goal, id=1 odom
# to plot: location in 3D and 1D in each axis over time

import csv
import matplotlib.pyplot as plt
import numpy as np 
import time
np.random.seed(444)

odom_data = './bebop.csv' # Set the file name to be read

# Make the lists which will be used for data.
odom = []
t_odom = []
x_odom = []
y_odom = []
z_odom = []

x_goal = []
y_goal = []
z_goal = []

data = []

# Read through the CSV file
with open(odom_data, 'r+') as cvsfile:
    reader = csv.reader(cvsfile)
    for line in reader:
        if len(line) != 5:
            continue
        entry = []
        for elt in line:
            try:
                entry.append(float(elt))
            except Exception as e:
                entry = []
                break
        if entry != []:
            data.append(entry)

# Assign the values in data to either goals or odom
for x in data:
    if x[0] == 0:
        if len(x) != 5:
            continue
        x_goal.append(x[2])
        y_goal.append(x[3])
        z_goal.append(x[4])
    elif x[0] == 1:
        if len(x) != 5:
            continue
        odom.append(x)
        t_odom.append(x[1])
        x_odom.append(x[2])
        y_odom.append(x[3])
        z_odom.append(x[4])
    else:
        print "Invalid ID."

# Plot the 1D odom data
fig, axes = plt.subplots(2,2)

axes[0,0].plot(t_odom, x_odom, '-')
axes[0,0].set(xlabel='t', ylabel='x')
axes[0,0].axhline(x_goal[0])

axes[0,1].plot(t_odom, y_odom, '-')
axes[0,1].set(xlabel='t', ylabel='y')
axes[0,1].axhline(y_goal[0])

axes[1,0].plot(t_odom, z_odom, '-')
axes[1,0].set(xlabel='t', ylabel='z')
axes[1,0].axhline(z_goal[0])

t = time.time()
plt.savefig('graphs/%5d'%t)
plt.show()

print '%5d.png'%t

