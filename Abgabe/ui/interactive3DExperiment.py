# Modul Import

## Externe Module
from ipywidgets import *
import numpy as np
import matplotlib.pyplot as plt

## Eigen Module
import DataGenerationRadar1D as gen
from KalmanFilter import KalmanFilter
from DataGenerationRadar3D import *
from DBScan import *

eps_ia = widgets.FloatSlider(
    value=.2,
    min=.1,
    max=1.,
    step=.1,
    description='$\epsilon$',
    disabled=False,
    continuous_update=False,
    orientation='horizontal',
    readout=True,
    readout_format='.1f',
)

minpts_ia = widgets.IntSlider(
    value=2,
    min=1,
    max=10,
    step=1,
    description='Min. Points',
    disabled=False,
    continuous_update=False,
    orientation='horizontal',
    readout=True,
    readout_format='d'
)

targets_ia = widgets.SelectMultiple(
    options=['Target 1', 'Target 2', 'Target 3', 'Target 4'],
    value=['Target 1'],
    #rows=10,
    description='Targets',
    disabled=False
)

plt_fdets_ia = widgets.Checkbox(
    value=False,
    description='Plot false detections',
    disabled=False,
    indent=False
)

def plot_3DExperiment():
    def update(eps, minpts, targ_select, plt_fdets):
        
        # initialize DBScan
        model = DBSCAN(eps, minpts)
        
        # clear graph on update
        ax.clear()
        
        # Parameters first target
        path1 = [[0,5,0],
                [0,5,0.5],
                [1,5,1],
                [1,5,0.5],
                [0.5, 2, 0.1]]
        
        vel1 = 3 * np.ones((1,5))
        vel1[0,2] = 1

        InitialPosition1 = np.array([-1,5,0])

        opt1 = {
            'InitialPosition' : InitialPosition1,
            'Path' : np.array(path1).transpose(),
            'Velocities' : vel1
        }
        
        # Parameters second target
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

        # Parameters third target
        path3 = [[1. , 4. , 1. ],
                [1. , 5. , 1.],
                [2. , 5. , 1. ],
                [3.5 , 4. , 2. ],
                [3. , 4.1 , 1.],
                [2.2 , 4. , 2. ]]

        vel3 = 2 * np.ones((1,len(path3)))
        vel3[0,4] = 0.3

        InitialPosition3 = np.array([3,4,5])

        opt3 = {
            'InitialPosition' : InitialPosition3,
            'Path' : np.array(path3).transpose(),
            'Velocities' : vel3
        }
        
        # Parameters fourth target
        path4 = [[1.5 , 4.2 , 1.5 ],
                [1.5 , 5.2 , 1.2],
                [2.5 , 5.2 , 1.2 ],
                [3.5 , 4.2 , 2.2 ],
                [3.5 , 4.1 , 1.4],
                [2.2 , 4.2 , 2.3 ]]

        vel4 = 2 * np.ones((1,len(path4)))
        vel4[0,4] = 0.7

        InitialPosition4 = np.array([4,4,4])

        opt4 = {
            'InitialPosition' : InitialPosition4,
            'Path' : np.array(path4).transpose(),
            'Velocities' : vel4
        }
        
        # Instantiate targets
        t_1 = Target(opt1)
        t_2 = Target(opt2)
        t_3 = Target(opt3)
        t_4 = Target(opt4)
        
        # Target and colors dictionary for interact
        target_dict = {'Target 1': t_1, 'Target 2': t_2, 'Target 3': t_3, 'Target 4': t_4}
        target_colors = {'Target 1': 'limegreen', 'Target 2': 'darkcyan', 'Target 3': 'darkviolet', 'Target 4': 'deeppink'}
        colors_list = []
        targets = list()
        
        # target selection for interact
        for target in targ_select:
            if target in target_dict:
                targets.append(target_dict[target])
                colors_list.append(target_colors[target])

        '''
        Setup the radar sensor
        The radar sensor points always to the direction along the y axis
        (see diagram in the note)
        '''
        
        sens_position = np.array([0,0,0.5])
        ax.plot3D(sens_position[0], sens_position[1], sens_position[2], 'ro')
        
        optRadar = {
            'Position' : sens_position,
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


        getNext = True
        Detections = np.array([0,0,0])
        
        # Number of previous measurements to consider for DBSCAN().
        ante = 20
        # Count number of iterations
        i = 0
        
        while(getNext == True):
            i += 1
            for target in targets:
                target.Step(1/sensor.opt['MeasurementRate'])
                getNext = getNext & ~target.reachedEnd  

            dets = sensor.Detect(targets)

            # Allow display of false detections only with one target
            if len(targets) > 1:
                plt_fdets_ia.disabled=True
                plt_fdets_ia.value=False
            else:
                plt_fdets_ia.disabled=False

            # plot flase dets only if checkbox is activated and only one target selected
            if plt_fdets == True and len(targets) == 1:
                try:
                    p = dets[1]
                    ax.scatter(p[0], p[1], p[2], s=50, c='tomato')
                except IndexError:
                    pass

            for det in dets:
                det = det[:-1]
                Detections = np.vstack((det, Detections))

            # Execute once to initialize filters etc. todo: Is there a smarter way to do all below ?
            if i == ante:
                # First application of DBSCAN.
                clusters = model.fit_predict(Detections[:ante])
                # Determine number of targets (objects tracked).
                num_objs = len(set(clusters[clusters > -1]))

                # "Filters" contains a kalman filter for each target.
                Filters = []
                # "Preds" contains the predictions of the path of each target.
                Preds = []
                # Iterate over the targets.
                for j in range(num_objs):
                    # Find index of first occurence of target j in clusters. This line is needed to filter out false detections
                    obj_idx = np.where(clusters == j)[0][0]
                    # Add placeholder values for speed and acceleration in each component to the detection.
                    s0 = np.vstack((Detections[obj_idx], np.zeros((2,3))))
                    Filters.append(KalmanFilter(s0, transition_model, H, Q, R))
                    # For the moment only the predicted position is relevant. todo: incorporate velocity.
                    Preds.append(s0[0,:])

            # Cluster and predict position via Kalman filter.
            elif i > ante:
                clusters = model.fit_predict(Detections[:ante])
                for j in range(num_objs):
                    # try/ except prevents non-detection of existing object from breaking the program.
                    try:
                        obj_idx = np.where(clusters == j)[0][0]
                        # Reshape is needed to make matrix multiplication inside the kalman filter work.
                        s = Detections[obj_idx].reshape(1,3)
                        s_hat = Filters[j].step(s)
                        Preds[j] = np.vstack((s_hat[0,:], Preds[j]))
                    except IndexError:
                        print(f"Object {j} not found!")
                        continue

        # Visualize trajectory.
        for pred, color in zip(Preds, colors_list):
            T = pred[:-1]
            ax.plot3D(T[:,0], T[:,1], T[:,2], color) 
        
        ax.set_xlim3d(0, 5)
        ax.set_ylim3d(0, 5)
        ax.set_zlim3d(0, 5)
        
        fig.canvas.draw_idle()

    fig = plt.figure() #figsize=(9, 8), dpi=100
    ax = plt.axes(projection='3d')

    ui = widgets.VBox([eps_ia, 
                    minpts_ia, 
                    targets_ia,
                    plt_fdets_ia])
    out = interactive_output(update, {'eps': eps_ia, 'minpts': minpts_ia, 'targ_select': targets_ia, 'plt_fdets': plt_fdets_ia})
    display(ui, out)