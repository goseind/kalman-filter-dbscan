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
    value=1.5,
    min=.1,
    max=3,
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
    # rows=10,
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
    # Use DBSCAN to track the paths and Kalman to correct the measurment errors
    def scan_with_filter(model, pt_history, sensor, targets, R, Q, transition_model, H):
        getNext = True
        detections = np.array([0, 0, 0, 0])
        # Count number of iterations
        i = 0
        labeled = {}
        predictions = {}
        filters = {}

        while (getNext == True):
            i += 1
            for target in targets:
                target.Step(1 / sensor.opt['MeasurementRate'])
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
                        s0 = np.vstack((detections[obj_idx, :-1], np.zeros((2, 3))))
                        filters[j] = KalmanFilter(s0, transition_model, H, Q, R)
                        predictions[j] = [s0[0, :]]
                    else:
                        # try to check if label swap occured + correct it
                        # check if last detection is in the cluster if its not an outlier and we had enough found clusters (last_obj_idx)
                        if (not detections[:pt_history][last_obj_idx] in labeled[j]) and (last_obj_idx != -1):
                            for l in labeled.keys():
                                if detections[last_obj_idx] in labeled[l]:
                                    j = l
                                    break

                    s = filters[j].step(detections[obj_idx, :-1])
                    predictions[j] = np.vstack((s[0, :], predictions[j]))
                    labeled[j] = np.vstack((detections[obj_idx], labeled[j]))

        return predictions, labeled

    def update(eps, minpts, targ_select, plt_fdets):
        if len(targ_select) > 1:
            plt_fdets = False
            plt_fdets_ia.disabled = True
            plt_fdets_ia.value = False
        else:
            plt_fdets_ia.disabled = False
         

        # initialize DBScan
        model = DBSCAN(eps, minpts)

        # clear graph on update
        ax.clear()

        # Parameters first target
        path1 = [[0, 5, 0],
                 [0, 5, 0.5],
                 [1, 5, 1],
                 [1, 5, 0.5],
                 [0.5, 2, 0.1]]

        vel1 = 3 * np.ones((1, 5))
        vel1[0, 2] = 1

        InitialPosition1 = np.array([-1, 5, 0])

        opt1 = {
            'InitialPosition': InitialPosition1,
            'Path': np.array(path1).transpose(),
            'Velocities': vel1
        }

        # Parameters second target
        path2 = [[1., 4., 1.],
                 [1., 5., 1.7],
                 [2., 5., 1.],
                 [3., 4., 2.],
                 [3., 4., 1.5],
                 [2., 4., 2.]]

        vel2 = 2 * np.ones((1, len(path2)))
        vel2[0, 4] = 0.5

        InitialPosition2 = np.array([2, 4, 1])

        opt2 = {
            'InitialPosition': InitialPosition2,
            'Path': np.array(path2).transpose(),
            'Velocities': vel2
        }

        # Parameters third target
        path3 = [[1., 4., 2.],
                 [1., 3., 1.2],
                 [2., 3., 1.],
                 [2.5, 3., 2.],
                 [3., 3.1, 1.5],
                 [2.5, 3., 2.]]

        vel3 = 2 * np.ones((1, len(path3)))
        vel3[0, 4] = 0.3

        InitialPosition3 = np.array([3, 4, 5])

        opt3 = {
            'InitialPosition': InitialPosition3,
            'Path': np.array(path3).transpose(),
            'Velocities': vel3
        }

        # Parameters fourth target
        path4 = [[1.5, 4.2, 1.5],
                 [1.5, 5.2, 1.2],
                 [2.5, 5.2, 1.2],
                 [3.5, 4.2, 2.2],
                 [3.5, 4.1, 1.4],
                 [2.2, 4.2, 2.3]]

        vel4 = 2 * np.ones((1, len(path4)))
        vel4[0, 4] = 0.7

        InitialPosition4 = np.array([4, 4, 4])

        opt4 = {
            'InitialPosition': InitialPosition4,
            'Path': np.array(path4).transpose(),
            'Velocities': vel4
        }

        # Instantiate targets
        t_1 = Target(opt1)
        t_2 = Target(opt2)
        t_3 = Target(opt3)
        t_4 = Target(opt4)

        # Target and colors dictionary for interact
        target_dict = {'Target 1': t_1, 'Target 2': t_2, 'Target 3': t_3, 'Target 4': t_4}
        target_colors = {'Target 1': 'limegreen', 'Target 2': 'darkcyan', 'Target 3': 'darkviolet',
                         'Target 4': 'deeppink'}
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

        sens_position = np.array([0, 0, 0.5])
        ax.plot3D(sens_position[0], sens_position[1], sens_position[2], 'ms')

        optRadar = {
            'Position': sens_position,
            'OpeningAngle': np.array([120, 90]),  # [Horizontal, Vertical]
            'FalseDetection': True
        }
        sensor = RadarSensor(optRadar)

        # Measurement error.
        ## Variance of a uniform distribution is given by (b-a)**2/12.
        R = np.diag([rangeAccuracy ** 2]) / 3
        # Process error.
        Q = np.diag([0.05, 0.05, 0.05])
        # Process/transition model.
        transition_model = np.array([[1, 0.01, 0.01 / 2],
                                     [0, 1, 0.01],
                                     [0, 0, 0.01]])
        # Transformation matrix
        ## Transforms predicted quantities into outputs that can be compared to the measurements
        H = np.array([[1., 0., 0.]])

        pt_history = 20

        predictions, labeled = scan_with_filter(model, pt_history, sensor, targets, R, Q, transition_model, H)

        # Determine mse.

        if plt_fdets:
            T = predictions[-1]
            ax.plot3D(T[:, 0], T[:, 1], T[:, 2], 'ro')

        del predictions[-1]
        
        # Visualize trajectory.
        for pred, color in zip(predictions, colors_list):
            T = predictions[pred]
            if len(targets) == 1:
                sensor_dets = labeled[pred][:,:-1]
                true_path = np.array(targets[pred].Trajectory) - sensor.opt["Position"]
                mse_KF = mse([T],[true_path])
                mse_Sensor = mse([sensor_dets],[true_path])
                print(f"Mean squared Error of Filter: {mse_KF}")
                print(f"Mean squared Error of Sensor: {mse_Sensor}")
            ax.plot3D(T[:, 0], T[:, 1], T[:, 2], color)
            print("")
        


        ax.set_xlim3d(0, 5)
        ax.set_ylim3d(0, 5)
        ax.set_zlim3d(0, 5)

        fig.canvas.draw_idle()

    fig = plt.figure(facecolor='w')  # figsize=(9, 8), dpi=100
    ax = plt.axes(projection='3d')

    ui = widgets.VBox([eps_ia,
                       minpts_ia,
                       targets_ia,
                       plt_fdets_ia])
    out = interactive_output(update,
                             {'eps': eps_ia, 'minpts': minpts_ia, 'targ_select': targets_ia, 'plt_fdets': plt_fdets_ia})
    display(ui, out)


def mse(paths: list, true_paths: list):
    """
    Predict the mean square error between a path prediction and the true path of the objects

    """
    
    Diffs = []
    total_elements = 0
    # For-loop: Simple difference between a predicted path and a true measurement.
    for i, true_path in enumerate(true_paths):
        diff = true_path[-len(paths[i]):] - paths[i]
        Diffs.append(diff)
        total_elements += len(diff)
        
         
    # For-loop: Square and average the differences.
    mse = 0
    for i, diff in enumerate(Diffs):
        # List diff is mapped to an integer (mse).
        mse += np.sum(diff**2)/ np.prod(diff.shape) * (len(diff)/total_elements)

    return mse