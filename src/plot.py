# Data given like: .csv file
# id, timestamp, x, y, z
# if id=0 goal, id=1 odom
# to plot: location in 3D and 1D in each axis over time

import csv
import matplotlib.pyplot as plt
import numpy as np 
np.random.seed(444)

odom_data = './bebop.csv' # Set the file name to be read

# Make the lists which will be used for data.
odom = []
t_odom = []
x_odom = []
y_odom = []
z_odom = []
goals = []

data = []

# Read through the CSV file
with open(odom_data, 'r+') as cvsfile:
    reader = csv.reader(cvsfile)
    for line in reader:
        entry = []
        for elt in line:
            entry.append(float(elt))
        data.append(entry)

# Assign the values in data to either goals or odom
for x in data:
    if x[0] == 0:
        pass
        # goals.append(x)
    elif x[0] == 1:
        odom.append(x)
        t_odom.append(x[1])
        x_odom.append(x[2])
        y_odom.append(x[3])
        z_odom.append(x[4])
    else:
        print "Invalid ID."

# Plot the 1D odom data
fig, axes = plt.subplots(2,2)

axes[0,0].plot(t_odom, x_odom, 'o-')
axes[0,0].set(xlabel='t', ylabel='x')

axes[0,1].plot(t_odom, y_odom, 'o-')
axes[0,1].set(xlabel='t', ylabel='x')

axes[1,0].plot(t_odom, z_odom, 'o-')
axes[1,0].set(xlabel='t', ylabel='x')

plt.show()

