from __future__ import print_function
from ipywidgets import interact, interactive, interactive_output, fixed, interact_manual
import ipywidgets as widgets

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