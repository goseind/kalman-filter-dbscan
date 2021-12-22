from numpy.linalg import det
from DataGenerationRadar3D import RadarSensor, Target
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Defining plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

'''
Example for creating a target and design its path
'''
path = [[0,5,0],
        [0,5,0.5],
        [1,5,1],
        [1,5,0.5],
        [0.5, 2, 0.1]]

vel = 3 * numpy.ones((1,5))
vel[0,2] = 1

InitialPosition = numpy.array([-1,5,0])

opt = {
    'InitialPosition' : InitialPosition,
    'Path' : numpy.array(path).transpose(),
    'Velocities' : vel
}

x = Target(opt)
y = Target(opt) # Adding target 
z = Target(opt) # Adding target

'''
You can add multiple targets to the list
'''
targets = [x] # Just one target for the moment

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