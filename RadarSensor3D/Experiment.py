from DataGenerationRadar3D import RadarSensor, Target
import numpy as np
import matplotlib.pyplot as plt
from DBScan import DBSCAN
from RadarSensor1D.KalmanFilter import KalmanFilter
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
import queue

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
model = DBSCAN(minpts=3, eps=0.5)
predictions = list()
points = list()
number_points = 15
predict_queue = queue.Queue(number_points)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
while (getNext == True):
    for target in targets:
        target.Step(1 / sensor.opt['MeasurementRate'])
        getNext = getNext & ~target.reachedEnd

    dets = sensor.Detect(targets)

    for det in dets:
        predict_queue.put(det[:3])
        if predict_queue.full():
            pred = model.fit_predict(np.array(list(predict_queue.queue)))

            points.append(det[:3])
            predictions.append(pred[-1])
            predict_queue.get()

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
points_np = np.array(points)
ax.scatter(points_np[:, 0], points_np[:, 1], points_np[:, 2], c=predictions)

plt.show()
