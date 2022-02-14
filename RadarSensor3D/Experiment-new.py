from DataGenerationRadar3D import RadarSensor, Target, rangeAccuracy
import numpy as np
from DBScan import DBSCAN
from KalmanFilter import KalmanFilter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Use DBSCAN to track the paths and Kalman to correct the measurment errors
def scan_with_filter(model, pt_history, targets, R, Q, transition_model, H):
    getNext = True
    detections = np.array([0,0,0,0])
    # Count number of iterations
    i = 0
    labeled = {}
    predictions = {}
    filters = {}

    while(getNext == True):
        i += 1
        for target in targets:
            target.Step(1/sensor.opt['MeasurementRate'])
            getNext = getNext & ~target.reachedEnd  

        dets = sensor.Detect(targets)
        for det in dets:
            detections = np.vstack((det, detections))

        if i >= pt_history:
            # First application of DBSCAN.
            clusters = model.fit_predict(detections[:pt_history])
            # Determine number of targets (objects tracked).
            num_objs = set(clusters)

            for j in num_objs:
                # find index of first occurence of target j in clusters. This line is needed to filter out false detections
                obj_idx = np.where(clusters == j)[0][0]
                try:
                    last_obj_idx = np.where(clusters == j)[0][1]
                except:
                    last_obj_idx = -1

                if j not in labeled.keys():
                    labeled[j] = detections[obj_idx]
                    s0 = np.vstack((detections[obj_idx,:-1], np.zeros((2,3))))
                    filters[j] = KalmanFilter(s0, transition_model, H, Q, R)
                    predictions[j] = [s0[0,:]]
                else:
                    # try to check if label swap occured + correct it
                    # check if last detection is in the cluster if its not an outlier and we had enough found clusters (last_obj_idx)
                    if (not detections[last_obj_idx] in labeled[j]) and (last_obj_idx != -1):
                        for l in labeled.keys():
                            if detections[last_obj_idx] in labeled[l]:
                                j = l
                                break
                            

                s = filters[j].step(detections[obj_idx,:-1])
                predictions[j] = np.vstack((s[0,:], predictions[j]))
                labeled[j] = np.vstack((detections[obj_idx], labeled[j]))
                
    return predictions


'''
Example for creating a target and design its path
'''

# Parameters first target.
path1 = [[0,5,0],
         [0,5,0.5],
         [1,4,1],
         [2,3,2],
         [1,5,3],
         [1,5,0.5],
         [0.5, 2, 0.1]]

vel1 = 3 * np.ones((1,len(path1)))
vel1[0,2] = 1

InitialPosition1 = np.array([-1,5,0])

opt1 = {
    'InitialPosition' : InitialPosition1,
    'Path' : np.array(path1).transpose(),
    'Velocities' : vel1
}


# Parameters second target.
path2 = [[1. , 4. , 1. ],
         [1. , 5. , 1.7],
         [2. , 5. , 1. ],
         [3. , 4. , 2. ],
         [3. , 4. , 1.5],
         [2. , 4. , 2. ]]

vel2 = 2 * np.ones((1,len(path2)))
vel2[0,4] = 0.5

InitialPosition2 = np.array([2,4,1])

opt2 = {
    'InitialPosition' : InitialPosition2,
    'Path' : np.array(path2).transpose(),
    'Velocities' : vel2
}


# Instantiate targets
x = Target(opt1)
y = Target(opt2)


targets = [x, y]

'''
Setup the radar sensor
The radar sensor points always to the direction along the y axis
(see diagram in the note)
'''

optRadar = {
    'Position' : np.array([0,0,0.5]),
    'OpeningAngle' : np.array([120,90]), # [Horizontal, Vertical]
    'FalseDetection': True
}
sensor = RadarSensor(optRadar)

# Measurement error.
## Variance of a uniform distribution is given by (b-a)**2/12.
R = np.diag([rangeAccuracy**2])/3
# Process error.
Q = np.diag([0.05,0.05,0.05])
# Process/transition model.
transition_model = np.array([[1, 0.01, 0.01/2],
                             [0, 1, 0.01],
                             [0, 0, 0.01]])
# Transformation matrix
## Transforms predicted quantities into outputs that can be compared to the measurements
H =  np.array([[1., 0., 0.]])

model = DBSCAN(eps=0.7, minpts=2)
# Number of previous measurements to consider for DBSCAN().
pt_history = 20


predictions = scan_with_filter(model, pt_history, targets, R, Q, transition_model, H)
            
# Visualize trajectory.
T1 = predictions[0][:-1]
T2 = predictions[1][:-1]

# Plot Trajectory
fig = plt.figure()
ax = plt.axes(projection='3d')
#ax.view_init(20, 35) 
ax.plot3D(T1[:,0], T1[:,1], T1[:,2], 'blue')   
ax.plot3D(T2[:,0], T2[:,1], T2[:,2], 'red')    

# show plot
plt.show()