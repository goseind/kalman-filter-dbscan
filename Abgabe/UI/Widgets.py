
# Modul Import

## Externe Module
from ipywidgets import *
import numpy as np
import matplotlib.pyplot as plt

# Widgets
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

from ipywidgets import *
import numpy as np
import matplotlib.pyplot as plt
import DataGenerationRadar1D as gen
from KalmanFilter import KalmanFilter

style = {'description_width': 'initial'}

type_widget = widgets.Dropdown(
    options=['Static', 'ConstantVelocity', 'ConstantAcceleration', 'Sinus', 'Triangle', ],
    value='Static',
    description='Type:',
    disabled=False,
    style=style,
)

initialDistance_widget = widgets.IntSlider(
    value=8,
    min=0,
    max=10,
    description='InitialDistance:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    continuous_update=False,
    style=style,
)

stopTime_widget = widgets.FloatSlider(
    value=1,
    min=0.1,
    max=10.0,
    step=0.1,
    description='StopTime:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.1f',
    continuous_update=False,
    style=style,
)

movementRange_widget = widgets.FloatSlider(
    value=1,
    min=0,
    max=10.0,
    step=0.1,
    description='MovementRange:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.1f',
    continuous_update=False,
    style=style,
)

frequency_widget = widgets.FloatSlider(
    value=2,
    min=1,
    max=10.0,
    step=0.1,
    description='Frequency:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.1f',
    continuous_update=False,
    style=style,
)

sporadicError_widget = widgets.IntSlider(
    value=5,
    min=0,
    max=10,
    description='SporadicError:',
    disabled=False,
    orientation='horizontal',
    continuous_update=False,
    style=style,
)

velocity_widget = widgets.FloatSlider(
    value=3,
    min=0,
    max=10.0,
    step=0.1,
    description='Velocity:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.1f',
    continuous_update=False,
    style=style,
)

initialVelocity_widget = widgets.FloatSlider(
    value=3,
    min=0,
    max=100.0,
    step=0.5,
    description='InitialVelocity:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.1f',
    continuous_update=False,
    style=style,
)

acceleration_widget = widgets.FloatSlider(
    value=3,
    min=0,
    max=10.0,
    step=0.1,
    description='Acceleration:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.1f',
    continuous_update=False,
    style=style,
)
option_widgets = [HTML(value="<h4>Data settings</h4>"), type_widget, initialDistance_widget, stopTime_widget,
                  movementRange_widget, frequency_widget,
                  sporadicError_widget, velocity_widget, initialVelocity_widget, acceleration_widget]

option_ui = widgets.VBox(option_widgets)

minRange_widget = widgets.FloatSlider(
    value=0.3,
    min=0,
    max=10.0,
    step=0.1,
    description='MinRange:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.1f',
    continuous_update=False,
    style=style,
)

maxRange_widget = widgets.FloatSlider(
    value=25.0,
    min=0.5,
    max=50,
    step=0.5,
    description='MaxRange:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.1f',
    continuous_update=False,
    style=style,
)

maxVelocity_widget = widgets.IntSlider(
    value=25,
    min=0,
    max=100,
    description='MaxVelocity:',
    disabled=False,
    orientation='horizontal',
    continuous_update=False,
    style=style,
)

rangeAccuracy_widget = widgets.FloatSlider(
    value=0.02,
    min=0.01,
    max=1.0,
    step=0.01,
    description='RangeAccuracy:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.02f',
    continuous_update=False,
    style=style,
)

velocityAccuracy_widget = widgets.FloatSlider(
    value=0.005,
    min=0,
    max=0.5,
    step=0.001,
    description='VelocityAccuracy:',
    disabled=False,
    orientation='horizontal',
    readout=True,
    readout_format='.03f',
    continuous_update=False,
    style=style,
)

measurementRate_widget = widgets.IntSlider(
    value=100,
    min=10,
    max=500,
    step=10,
    description='MeasurementRate:',
    disabled=False,
    orientation='horizontal',
    continuous_update=False,
    style=style,
)
sensor_widgets = [HTML(value="<h4>Sensor settings</h4>"), minRange_widget, maxRange_widget, maxVelocity_widget,
                  rangeAccuracy_widget, velocityAccuracy_widget,
                  measurementRate_widget]
sensor_ui = widgets.VBox(sensor_widgets)

# kalman widget
item_layout = Layout(height='100%', width='100px')
q0_widget = widgets.BoundedFloatText(
    value=0,
    min=0,
    max=10.0,
    description='Q:',
    disabled=False,
    style={'description_width': '10px'},
    layout=Layout(height='100%', width='110px')
)

q1_widget = widgets.BoundedFloatText(
    value=0,
    min=0,
    max=10.0,
    description='',
    disabled=False,
    layout=item_layout
)

q2_widget = widgets.BoundedFloatText(
    value=0,
    min=0,
    max=10.0,
    disabled=False,
    layout=item_layout
)

r0_widget = widgets.BoundedFloatText(
    value=gen.rangeAccuracy ** 2 / 3,
    min=0,
    max=10.0,
    description='R:',
    disabled=False,
    style={'description_width': '10px'},
    layout=Layout(height='100%', width='110px')
)

r1_widget = widgets.BoundedFloatText(
    value=gen.velocityAccuracy ** 2 / 3,
    min=0,
    max=10.0,
    description='',
    disabled=False,
    layout=item_layout
)