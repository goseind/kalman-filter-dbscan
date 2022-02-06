from ipywidgets import *
import numpy as np
import matplotlib.pyplot as plt
import DataGenerationRadar1D as gen
from KalmanFilter import KalmanFilter, compute_mse

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

truth_values_checkbox = widgets.Checkbox(value=True,
                                         description='Truth Values',
                                         disabled=False,
                                         )

values_checkbox = widgets.Checkbox(value=True,
                                   description='Values',
                                   disabled=False)

pred_values_checkbox = widgets.Checkbox(value=True,
                                        description='Prediction Values',
                                        disabled=False)

legend_widgets = [HTML(value="<h4>Legende</h4>"), values_checkbox, truth_values_checkbox, pred_values_checkbox]
legend_ui = widgets.VBox(legend_widgets)

kalman_legend_ui = widgets.VBox([legend_ui, kalman_ui])

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

    return Predictions


def plot_interactive_kalaman_filter():
    fig = plt.figure()
    ax_dist = fig.add_subplot(211)
    ax_vel = fig.add_subplot(212)

    opt = {
        "initialDistance": 8,
        "stopTime": 1,
        "movementRange": 1,
        "frequency": 2,
        "SporadicError": 5,
        "velocity": 3
    }

    timeAxis, distValues, velValues, truthDistValues, truthVelValues = gen.GenerateData(type="Static", options=opt)

    R = np.diag([gen.rangeAccuracy ** 2, gen.velocityAccuracy ** 2]) / 3
    Q = np.diag([0, 0, 0])
    s0 = np.array([distValues[0], velValues[0], 0])
    transition_model = np.array([[1, 0.01, 0.01 / 2],
                                 [0, 1, 0.01],
                                 [0, 0, 0.01]])
    H = np.array([[1., 0., 0.],
                  [0., 1., 0.]])
    kalmanFilter1D = KalmanFilter(s0, transition_model, H, Q, R)
    Predictions = [s0]
    for i in range(1, np.size(timeAxis)):
        s = np.array([distValues[i], velValues[i]])
        pred = kalmanFilter1D.step(s)
        Predictions.append(pred)
    Predictions = np.array(Predictions)
    kalman_1D_dist, = ax_dist.plot(timeAxis, Predictions[:, 0])
    kalman_1D_vel, = ax_vel.plot(timeAxis, Predictions[:, 1])
    data_dist, = ax_dist.plot(timeAxis, distValues)
    data_vel, = ax_vel.plot(timeAxis, velValues)
    true_dist, = ax_dist.plot(timeAxis, truthDistValues)
    true_vel, = ax_vel.plot(timeAxis, truthVelValues)

    def update_visibility(values, true_values, pred_values):
        data_dist.set_visible(values)
        data_vel.set_visible(values)
        kalman_1D_dist.set_visible(pred_values)
        kalman_1D_vel.set_visible(pred_values)
        true_vel.set_visible(true_values)
        true_dist.set_visible(true_values)

    def update(initialDistance, initialVelocity, stopTime, movementRange, frequency, SporadicError, velocity,
               acceleration, minRange_value, maxRange_value, maxVelocity_value, rangeAccuracy_value,
               velocityAccuracy_value, measurementRate_value, q0, q1, q2, r0, r1,
               type_value='Static', ):
        opt = setup_opt(initialDistance, initialVelocity, stopTime, movementRange, frequency, SporadicError, velocity,
                        acceleration)
        setup_ui(type_value)
        update_sensor_settings(minRange_value, maxRange_value, maxVelocity_value, rangeAccuracy_value,
                               velocityAccuracy_value, measurementRate_value)

        timeAxis, distValues, velValues, truthDistValues, truthVelValues = gen.GenerateData(type=type_value,
                                                                                            options=opt)

        s0 = np.array([distValues[0], velValues[0], 0])
        R = np.diag([r0, r1])
        Q = np.diag([q0, q1, q2])
        kalmanFilter1D = KalmanFilter(s0, transition_model, H, Q, R)
        Predictions = update_predictions(kalmanFilter1D, timeAxis, s0, distValues, velValues)
        Predictions = np.array(Predictions)

        update_x_axis(timeAxis, [data_dist, true_dist, kalman_1D_dist, data_vel, true_vel, kalman_1D_vel])
        update_y_axis([truthDistValues, truthVelValues, Predictions[:, 0], Predictions[:, 1], distValues, velValues],
                      [true_dist, true_vel, kalman_1D_dist, kalman_1D_vel, data_dist, data_vel])

        print("Position pred,data: ", compute_mse(Predictions[:, 0], truthDistValues, distValues))
        print("Velocity pred,data: ", compute_mse(Predictions[:, 1], truthVelValues, velValues))

        fig.canvas.draw_idle()

    out = widgets.interactive_output(update, update_dict)
    out_vis = widgets.interactive_output(update_visibility,
                                         {'values': values_checkbox, 'true_values': truth_values_checkbox,
                                          'pred_values': pred_values_checkbox})
    ui = widgets.HBox([option_ui, sensor_ui, kalman_legend_ui])
    display(out, out_vis, ui)
