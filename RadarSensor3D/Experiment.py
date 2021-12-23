from DataGenerationRadar3D import RadarSensor, Target
import numpy as np
import matplotlib.pyplot as plt
from DBScan import DBSCAN
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D

'''
Example for creating a target and design its path
'''
path = [[0, 5, 0],
        [0, 5, 0.5],
        [1, 5, 1],
        [1, 5, 0.5],
        [0.5, 2, 0.1]]

vel = 3 * np.ones((1, 5))
vel[0, 2] = 1

InitialPosition = np.array([-1, 5, 0])

opt_1 = {
    'InitialPosition': InitialPosition,
    'Path': np.array(path).transpose(),
    'Velocities': vel
}

opt_2 = {
    'InitialPosition': np.array([10, 10, 0]),
    'Path': np.array(path).transpose(),
    'Velocities': vel
}

x_1 = Target(opt_1)
x_2 = Target(opt_2)

'''
You can add multiple targets to the list
'''
# targets = [x_1]
targets = [x_1, x_2]

'''
Setup the radar sensor
The radar sensor points always to the direction along the y axis
(see diagram in the note)
'''

optRadar = {
    'Position': np.array([0, 0, 0.5]),
    'OpeningAngle': np.array([120, 90]),  # [Horizontal, Vertical]
    'FalseDetection': True
}
sensor = RadarSensor(optRadar)

'''
Here we loop through all targets and get the current radar detection
- apply your DBScan here 
- apply your Kalman Filter here
'''
getNext = True
vel = list()
points = list()

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
while (getNext == True):
    for target in targets:
        target.Step(1 / sensor.opt['MeasurementRate'])
        getNext = getNext & ~target.reachedEnd

    dets = sensor.Detect(targets)
    for det in dets:
        points.append(det[:3])
        ax.scatter(det[0], det[1], det[2])

plt.show()

model = DBSCAN()
points = np.array(points)
prediction = model.fit_predict(points)
print(prediction)

v = prediction
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=prediction)

plt.show()
