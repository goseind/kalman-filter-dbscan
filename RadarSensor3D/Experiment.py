from numpy.linalg import det
from DataGenerationRadar3D import RadarSensor, Target
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Defining 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

'''
Example for creating a target and design its path
'''
path_1 = [[0,5,0],
        [0,5,0.5],
        [1,5,1],
        [1,5,0.5],
        [0.5, 2, 0.1]]

# Adding targets
path_2 = [[0,4,0],
        [0,3,0.5],
        [1,2,1],
        [1,1,0.5],
        [0.5, 1, 0.1]]
path_3 = [[2,7,0],
        [0,3,0.3],
        [2,6,4],
        [2,4,0.6],
        [0.6, 4, 0.2]]

vel_1 = 3 * numpy.ones((1,5))
vel_1[0,2] = 1

# Adding targets
vel_2 = 3 * numpy.ones((1,5))
vel_2[0,2] = 1
vel_3 = 3 * numpy.ones((1,5))
vel_3[0,2] = 1

InitialPosition_1 = numpy.array([-1,5,0])

# Adding targets
InitialPosition_2 = numpy.array([6,7,4])
InitialPosition_3 = numpy.array([9,8,9])

opt_t_1 = {
    'InitialPosition' : InitialPosition_1,
    'Path' : numpy.array(path_1).transpose(),
    'Velocities' : vel_1
}

# Adding targets
opt_t_2 = {
    'InitialPosition' : InitialPosition_2,
    'Path' : numpy.array(path_2).transpose(),
    'Velocities' : vel_2
}
opt_t_3 = {
    'InitialPosition' : InitialPosition_3,
    'Path' : numpy.array(path_3).transpose(),
    'Velocities' : vel_3
}

t_1 = Target(opt_t_1)

# Adding targets
t_2 = Target(opt_t_2)
t_3 = Target(opt_t_3)

'''
You can add multiple targets to the list
'''
targets = [t_1, t_2, t_3] # Adding targets

'''
Setup the radar sensor
The radar sensor points always to the direction along the y axis
(see diagram in the note)
'''

optRadar = {
    'Position' : numpy.array([0,0,0.5]),
    'OpeningAngle' : numpy.array([120,90]), # [Horizontal, Vertical]
    'FalseDetection': True
}
sensor = RadarSensor(optRadar)

'''
Here we loop through all targets and get the current radar detection
- apply your DBScan here 
- apply your Kalman Filter here
'''

getNext = True
while(getNext == True):
    for target in targets:
        target.Step(1/sensor.opt['MeasurementRate'])
        getNext = getNext & ~target.reachedEnd    

    dets = sensor.Detect(targets)
    
    # Adding detections to plot
    for det in dets:
        ax.scatter(det[0], det[1], det[2])

    '''
    Ideas: To include previous detections?
    '''

# Plot result
plt.show()