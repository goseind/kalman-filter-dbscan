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

eps_ia_dbscan = widgets.FloatSlider(
    value=.7,
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

minpts_ia_dbscan = widgets.IntSlider(
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

pt_history_ia_dbscan = widgets.IntSlider(
    value=20,
    min=10,
    max=50,
    step=1,
    description='PT History',
    continuous_update=False,
    orientation='horizontal',
    readout=True,
    readout_format='d'
)

pttargets_ia_dbscan = widgets.SelectMultiple(
    options=['Target 1', 'Target 2'],
    value=['Target 1', 'Target 2'],
    #rows=10,
    description='Targets',
    disabled=False
)

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

def scan(model, pt_history, targets):
    getNext = True
    detections = np.array([0,0,0,0])
    # Count number of iterations
    i = 0
    labeled = {}

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
                # Find index of first occurence of target j in clusters. This line is needed to filter out false detections
                obj_idx = np.where(clusters == j)[0][0]

                if j not in labeled.keys():
                    labeled[j] = detections[obj_idx,:-1]

                s = detections[obj_idx,:-1].reshape(1,3)
                labeled[j] = np.vstack((s, labeled[j]))
                
    return labeled

colors = { -1: 'red', 0: 'green', 1: 'yellow', 2: 'blue', 3: 'purple', 4: 'orange', 5: 'pink', 6: 'black', 7: 'brown' }

def plot_interactive_dbscan():
    def update_dbscan(eps, minpts, pt_history, target_select):
        ax.clear()
        x = Target(opt1)
        y = Target(opt2)
        target_dict = {'Target 1': x, 'Target 2': y}
        
        targets = list()
        for target in target_select:
            if target in target_dict:
                targets.append(target_dict[target])
                
        
        labeled = scan(DBSCAN(minpts=minpts, eps=eps), pt_history, targets)
        print(f'Found {len(labeled.keys())-1} clusters and {len(labeled[-1])} outlier!')

        # Plot Trajectories   
        for label in labeled.keys():
            T = labeled[label];
            ax.scatter(T[:,0], T[:,1], T[:,2], c=f'{colors[label]}')   

        # show plot
        fig.canvas.draw_idle()


    fig= plt.figure(figsize=(8,6), dpi= 100, facecolor='w')
    ax = plt.axes(projection='3d')

    #print(type(plt_fdets_ia.value))

    ui = widgets.VBox([eps_ia_dbscan, 
                    minpts_ia_dbscan,
                    pt_history_ia_dbscan,
                    pttargets_ia_dbscan,
                    ])

    out = interactive_output(update_dbscan, {'eps': eps_ia_dbscan, 'minpts': minpts_ia_dbscan, 'target_select': pttargets_ia_dbscan, 'pt_history': pt_history_ia_dbscan})

    display(ui, out)