# Modul Import

## Externe Module
from ipywidgets import *
import numpy as np
import matplotlib.pyplot as plt

## Eigen Module
import DataGenerationRadar1D as gen
from KalmanFilter import KalmanFilter
from Widgets import *

# Helper Functions for Juypter Notebook
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