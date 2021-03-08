import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

fig = plt.figure()
ax = plt.axes(projection ='3d')

with open('proba.csv') as csvfile:
    data = [[frame, time, x, y, z] for frame, time, x, y, z in csv.reader(csvfile, delimiter= ',')]
ax.plot3D([float(elem[2]) for elem in data], [float(elem[3]) for elem in data], [float(elem[4]) for elem in data], 'green')
ax.set_title('proba')
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.show()


plt.plot([float(elem[2]) for elem in data])
plt.plot([float(elem[3]) for elem in data])
plt.plot([float(elem[4]) for elem in data])
plt.show()