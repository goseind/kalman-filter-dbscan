from DataGenerationRadar3D import RadarSensor, Target
import numpy
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D

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

'''
You can add multiple targets to the list
'''
targets = [x]

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

    '''
    Ideas: To include previous detections?
    '''