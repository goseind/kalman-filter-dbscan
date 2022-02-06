import numpy as np
import matplotlib.pyplot as plt
from DataGenerationRadar3D import *
from KalmanFilter import *

def onlyKalman(Q, R, falsedets):
    transition_model = np.array([[1, 0.01, 0.01/2],
                                [0, 1,    0.01  ],
                                [0, 0,    0.01 ]])
    
    H =  np.array([[1., 0., 0.]])

    path1 = [[0,   5, 0  ],
            [0,   5, 0.5],
            [1,   4, 1  ],
            [2,   3, 2  ],
            [1,   5, 3  ],
            [1,   5, 0.5],
            [0.5, 2, 0.1]]

    vel1 = 3 * np.ones((1,len(path1)))
    vel1[0,2] = 1

    InitialPosition1 = np.array([-1,5,0])

    opt1 = {
        'InitialPosition' : InitialPosition1,
        'Path' : np.array(path1).transpose(),
        'Velocities' : vel1
    }

    x = Target(opt1)

    targets = [x]

    falsedets = False

    optRadar = {
        'Position' : np.array([0, 0, 0.5]),
        'OpeningAngle' : np.array([120,90]), # [Horizontal, Vertical]
        'FalseDetection': falsedets
    }
    sensor = RadarSensor(optRadar)
    Detections = np.array([0,0,0,0])

    # Count number of iterations
    i = 0
    pred = []
    getNext = True
    while(getNext == True):
        
        for target in targets:
            target.Step(1/sensor.opt['MeasurementRate'])
            getNext = getNext & ~target.reachedEnd  

        dets = sensor.Detect(targets)
        # Exclude radialVelocity for the moment. (todo: include it.)
        for det in dets:
            #det = det[:-1]
            Detections = np.vstack((det, Detections))
        
        #print(Detections)
        #print(Detections[0,:])
        s0 = np.vstack((Detections[0,:-1], np.zeros((2,3))))
        
        if i == 0:
            f = KalmanFilter(s0, transition_model, H, Q, R)
            pred.append(s0[0,:])
        
        s = Detections[0,:-1].reshape(1,3)
        s_hat = f.step(s)
        pred = np.vstack((s_hat[0,:], pred))
        i += 1
        
    return pred, Detections


Q_1 = np.diag([0, 0, 0])
R_1 = np.diag([rangeAccuracy**2])/3 # Soll gut sein

res = onlyKalman(Q_1, R_1, True)

# Visualize trajectory.
T1 = res[0][:-1]

# Plot Trajectory
fig = plt.figure(figsize=(8, 4))
ax1 = fig.add_subplot(121, projection='3d')   
ax2 = fig.add_subplot(122, projection='3d')

#ax.view_init(20, 35) 
ax1.plot3D(T1[:,0], T1[:,1], T1[:,2], 'bo')
ax2.plot3D(res[1][:,0], res[1][:,1], res[1][:,2], 'r*')