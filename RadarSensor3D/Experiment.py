from DataGenerationRadar3D import RadarSensor, Target, rangeAccuracy
import numpy as np
from DBScan import DBSCAN
from KalmanFilter import KalmanFilter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

'''
Example for creating a target and design its path
'''
# Parameters first target.
path1 = [[0. ,5. ,0. ],
         [0. ,5. ,0.5],
         [1. ,4. ,1. ],
         [2. ,3. ,2. ],
         [1. ,5. ,3. ],
         [1. ,5. ,0.5],
]

#path1 = list(np.array(path1)*4)

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
The radar sensor points always to the direction along the y axis, e.g Sensor is not movable.
(see diagram in the note)
'''

optRadar = {
    'Position' : np.array([0,0,0.5]),
    'OpeningAngle' : np.array([120,90]), # [Horizontal, Vertical] # todo: Use these informations.
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
H =  np.array([[1., 0., 0.],
               [0., 1., 0.]]) 


getNext = True
Detections = np.array([0,0,0,0])
model = DBSCAN(eps=0.7, minpts=2)
# Number of previous measurements to consider for DBSCAN().
ante = 10
# Count number of iterations
i = 0

while(getNext == True):
    i += 1
    for target in targets:
        target.Step(1/sensor.opt['MeasurementRate'])
        getNext = getNext & ~target.reachedEnd  

    dets = sensor.Detect(targets)
    # Exclude radialVelocity for the moment. (todo: include it.)
    for det in dets:
        Detections = np.vstack((det, Detections))
    
    # Execute once to initialize filters etc. todo: Is there a smarter way to do all below ?
    if i == ante:
        # First application of DBSCAN.
        clusters = model.fit_predict(Detections[:ante])
        # Determine number of targets (objects tracked).
        num_objs = len(set(clusters[clusters > -1]))
        
        # "Filters" contains a kalman filter for each target.
        Filters = []
        # Sensor_Dets will contain lists of the detected positions of each object.
        Sensor_dets = []
        # "Preds" contains the predictions of the path of each target.
        Preds = []
        # Iterate over the targets.
        for j in range(num_objs):
            # Find index of first occurence of target j in clusters. This line is needed to filter out false detections
            obj_idx = np.where(clusters == j)[0][0]
            det = Detections[obj_idx,:-1]
            Sensor_dets.append(det)
            # Add placeholder values for speed and acceleration in each component to the detection.
            s0 = np.vstack((det, np.zeros((2,3)))) 
            # Add velocity to s0.
            Filters.append(KalmanFilter(s0, transition_model, H=H, Q=Q, R=R))
            Preds.append(s0[0,:])

    # Cluster and predict position via Kalman filter.
    elif i > ante:
        clusters = model.fit_predict(Detections[:ante])
        for j in range(num_objs):
            # try/ except prevents non-detection of existing object from breaking the program.
            try:
                obj_idx = np.where(clusters == j)[0][0]
                det = Detections[obj_idx,:-1]
                Sensor_dets[j] = np.vstack((Sensor_dets[j], det))
                vel = (Sensor_dets[j][-1] - Sensor_dets[j][-2]) 
                # Check whether estimated speed can be reconciled with radial velocity.
                bVector = -det
                # Estimate radial velocity
                est_rad_vel = np.dot(vel, bVector)/np.dot(bVector, bVector)
                if det[-1] - est_rad_vel < det[-1]*0.1:
                   s = np.vstack((det, vel))
                else:
                   s = det.reshape(1,3)
                s_hat = Filters[j].step(s)
                Preds[j] = np.vstack((Preds[j], s_hat[0,:] ))
            except IndexError:
                print(f"Object {j} not found!")
                continue
         
            

#%%
if __name__ == "__main__":    
#%% Compare sensor measurments to true trajectory.
    true_path_x = np.array(x.Trajectory)
    true_path_y = np.array(y.Trajectory)
    
    diff_sensor_x = true_path_x[-len(Sensor_dets[0]):] - Sensor_dets[0]
    diff_sensor_y = true_path_y[-len(Sensor_dets[1]):] - Sensor_dets[1]
    
    mse_sens_x = np.sum(diff_sensor_x**2)/ np.prod(diff_sensor_x.shape)
    mse_sens_y = np.sum(diff_sensor_y**2)/ np.prod(diff_sensor_y.shape)
    
    mse_sens = (mse_sens_x + mse_sens_y)/ 2
    print(f"Sensor Mean squared Error: {mse_sens}\n")
    
#%% Compare kalman filter predictions to true trajectory.
    # Reuse true_path_x/y from cell above.
    diff_filter_x = true_path_x[-len(Preds[0]):] - Preds[0]
    diff_filter_y = true_path_y[-len(Preds[1]):] - Preds[1]
    
    mse_filter_x = np.sum(diff_filter_x**2)/ np.prod(diff_filter_x.shape)
    mse_filter_y = np.sum(diff_filter_y**2)/ np.prod(diff_filter_y.shape)
    
    mse_filter = (mse_filter_x + mse_filter_y)/ 2
    print(f"Filter Mean squared Error: {mse_filter}")
    
#%% Plot Predictions.
    T1 = Preds[0]
    T2 = Preds[1]
    fig = plt.figure()
    ax = plt.axes(projection='3d')   
    #ax.view_init(20, 35) 
    ax.plot3D(T1[:,0], T1[:,1], T1[:,2], 'blue')   
    ax.plot3D(T2[:,0], T2[:,1], T2[:,2], 'red')    
    
#%% Other previous visualization experiments.
    # model = DBSCAN(eps=0.2, minpts=7)        
    # T = np.vstack((T1,T2))
    # clusters = model.fit_predict(T)    
    # fig = plt.figure()
    # ax = plt.axes(projection ="3d")  
    # ax.scatter(T[:,0], T[:,1], T[:,2], c = clusters)
    
    
    # fig2 = plt.figure()
    # ax2 = plt.axes(projection='3d')   
    # #ax.view_init(20, 35) 
    # ax2.plot3D(Detections[:,0], Detections[:,1], Detections[:,2], 'blue')       
    
    # ax2.set_xlim3d(0, 5)
    # ax2.set_ylim3d(0, 5)
    # ax2.set_zlim3d(0, 5)
    
    # model = DBSCAN(eps=0.2, minpts=3)  
    # ante = -10   
    # clusters = model.fit_predict(Detections[:ante])    
    # fig = plt.figure()
    # ax3 = plt.axes(projection ="3d") 
    
    # ax3.set_xlim3d(0, 5)
    # ax3.set_ylim3d(0, 5)
    # ax3.set_zlim3d(0, 5)
    
    # ax3.scatter(Detections[:ante,0], Detections[:ante,1], Detections[:ante,2], c = clusters)
    
#%matplotlib auto
#%matplotlib inline    
    