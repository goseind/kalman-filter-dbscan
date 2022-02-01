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

q = widgets.HBox([q0_widget, q1_widget, q2_widget])
r = widgets.HBox([r0_widget, r1_widget])

kalman_widgets = [HTML(value="<h4>Kalman settings</h4>"), q, r]
kalman_ui = widgets.VBox(kalman_widgets)

update_dict = {'type_value': type_widget,
               'initialDistance': initialDistance_widget,
               'stopTime': stopTime_widget,
               'movementRange': movementRange_widget,
               'frequency': frequency_widget,
               'SporadicError': sporadicError_widget,
               'velocity': velocity_widget,
               'initialVelocity': initialVelocity_widget,
               'acceleration': acceleration_widget,
               'minRange_value': minRange_widget,
               'maxRange_value': maxRange_widget,
               'maxVelocity_value': maxVelocity_widget,
               'rangeAccuracy_value': rangeAccuracy_widget,
               'velocityAccuracy_value': velocityAccuracy_widget,
               'measurementRate_value': measurementRate_widget,
               'q0': q0_widget,
               'q1': q1_widget,
               'q2': q2_widget,
               'r0': r0_widget,
               'r1': r1_widget, }


def setup_ui(type_value):
    for widget in option_widgets:
        if widget.description == 'SporadicError:' or widget.description == 'Type:':
            widget.disabled = False
        else:
            widget.disabled = True

    if type_value is 'Static':
        initialDistance_widget.disabled = False
        stopTime_widget.disabled = False
    if type_value is 'ConstantVelocity':
        initialDistance_widget.disabled = False
        stopTime_widget.disabled = False
        velocity_widget.disabled = False
    if type_value is 'ConstantAcceleration':
        initialDistance_widget.disabled = False
        stopTime_widget.disabled = False
        initialVelocity_widget.disabled = False
        acceleration_widget.disabled = False
    if type_value is 'Sinus':
        movementRange_widget.disabled = False
        stopTime_widget.disabled = False
        frequency_widget.disabled = False
    if type_value is 'Triangle':
        initialDistance_widget.disabled = False
        stopTime_widget.disabled = False
        frequency_widget.disabled = False
        movementRange_widget.disabled = False


def update_sensor_settings(minRange_value, maxRange_value, maxVelocity_value, rangeAccuracy_value,
                           velocityAccuracy_value, measurementRate_value):
    gen.minRange = minRange_value
    gen.maxRange = maxRange_value
    gen.maxVelocity = maxVelocity_value
    gen.rangeAccuracy = rangeAccuracy_value
    gen.velocityAccuracy = velocityAccuracy_value
    gen.measurementRate = measurementRate_value


def foo():
    data_dist.set_xdata(timeAxis)
    true_dist.set_xdata(timeAxis)
    kalman_1D_dist.set_xdata(timeAxis)
    data_vel.set_xdata(timeAxis)
    true_vel.set_xdata(timeAxis)
    kalman_1D_vel.set_xdata(timeAxis)


def update_x_axis(values, graphs):
    for graph in graphs:
        graph.set_xdata(values)


def update_y_axis(values, graphs):
    for value, graph in zip(values, graphs):
        graph.set_ydata(value)


def setup_opt(initialDistance, initialVelocity, stopTime, movementRange, frequency, SporadicError, velocity,
              acceleration):
    return {
        "initialDistance": initialDistance,
        "initialVelocity": initialVelocity,
        "stopTime": stopTime,
        "movementRange": movementRange,
        "frequency": frequency,
        "SporadicError": SporadicError,
        "velocity": velocity,
        "acceleration": acceleration

    }


def update_predictions(kalman_filter, time_axis, s0, dist_values, vel_values):
    Predictions = [s0]
    for i in range(1, np.size(time_axis)):
        s = np.array([dist_values[i], vel_values[i]])
        pred = kalman_filter.step(s)
        Predictions.append(pred)

    Predictions = np.reshape(Predictions, (3, np.size(time_axis)))
    return Predictions
