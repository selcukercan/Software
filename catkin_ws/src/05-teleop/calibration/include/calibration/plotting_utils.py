from itertools import izip
from os.path import join

import plotly.graph_objs as go
import plotly.offline as opy
import plotly.plotly as py
import plotly
import rospy
from numpy import arange, array, cos, sin, pi
from utils import get_param_from_config_file, x_in_np, get_workspace_param, deg


save_results = get_param_from_config_file('save_experiment_results')
upload_results = get_param_from_config_file('upload_plots')
plotly.tools.set_credentials_file(username='selcukercan', api_key='uiHkN3x2e7AfTnF8YwHk')

if not upload_results:
    opy.init_notebook_mode(connected=True)
else:
    pass

SLEEP_DURATION = 1

# High Level
def multiplot(states_list=None, time_list=None, input_list=None, experiment_name_list=None, plot_title='', save=False,
              save_dir=None, upload_this=False):
    plot_datas = []
    # generate arange time data if time is not provided to facilitate debugging
    if time_list is None:
        time_list = []
        for i in range(len(states_list)):
            time_list.append(arange(states_list[i][1, :].shape[0]))

    if input_list is not None:
        for states, time, input, experiment_name in izip(states_list, time_list, input_list, experiment_name_list):
            tek_plot_data = single_plot_data(states=states, time=time, input=input, experiment_name=experiment_name)
            plot_datas.extend(tek_plot_data)
    else:
        for states, time, experiment_name in izip(states_list, time_list, experiment_name_list):
            tek_plot_data = single_plot_data(states=states, time=time, experiment_name=experiment_name)
            plot_datas.extend(tek_plot_data)
    if len(plot_datas) is not 0:
        layout = dict(title=plot_title)
        fig = dict(data=plot_datas, layout=layout)
        if save:
            if save_dir == None:  # default save location
                save_dir = get_workspace_param("results_dir")
            opy.plot(fig, auto_open=False, filename=join(save_dir, plot_title + ".html"))
        else:
            opy.plot(fig)
        if upload_this:
            py.plot(fig)
    else:
        rospy.loginfo('[plotting_utils] unable to plot as data no data provided')
    shall_i_sleep()

"""
def path_plot(single_experiment, plot_name=''):
    data = x_in_np(single_experiment)
    plot_data = single_path_data_polar(data)
    layout = go.Layout(
        showlegend=True,
        polar=dict(
            sector=[min(data[1, :]) - 10, max(data[1, :]) + 10],
            radialaxis=dict(
                range=[min(data[0, :]) - 0.05, max(data[0, :]) + 0.05]
            )
        ),
        title=plot_name
    )
    fig = go.Figure(data=plot_data, layout=layout)
    if not upload_results:
        opy.plot(fig)
    else:
        py.plot(fig)
    shall_i_sleep()
"""

def single_plot_data(states=None, time=None, input=None, experiment_name=""):
    return single_plot_data_polar(states=states, time=time, input=input, experiment_name=experiment_name)


# Polar
def single_plot_data_polar(states=None, time=None, input=None, experiment_name=""):
    data = []

    if time is None:
        rospy.logwarn('[plotting_utils] time vector is left out, using array indexes (0, 1, ..., n) for plotting')
        t = arange(states[1, :].shape[0])
    else:
        t = time

    if states is not None:
        rho = states[0, :]
        yaw = states[1, :]

        p3 = go.Scatter(
            x=t,
            y=rho,
            mode='markers',
            name='rho ' + experiment_name.split('_')[-1]
        )

        p4 = go.Scatter(
            x=t,
            y=yaw,
            mode='markers',
            name='yaw pos ' + experiment_name.split('_')[-1]
        )

        data.extend([p3, p4])

    if input is not None:
        r = input[0, :]
        l = input[1, :]

        # Create a trace
        p1 = go.Scatter(
            x=t,
            y=r,
            name='right wheel commands'
        )

        p2 = go.Scatter(
            x=t,
            y=l,
            name='left wheel commands'
        )

        data.extend([p1, p2])

    return data


def single_path_data_polar(states=None, color="#182844", experiment_name=""):
    x_states = x_in_np(states)
    data = [
        go.Scatterpolar(
            r=x_states[0, :],
            theta=deg(x_states[1, :]),
            mode='markers',
            marker=dict(
                color=color
            ),
            name=experiment_name
        )
    ]
    return data


def multi_path_plot(experiment_list, experiment_name_list=[], plot_title="", save=False, upload_this=False):
    plot_data = []
    upper_rho = -10000  # unreasonably large number
    lower_rho = 10000  # unreasonably large number
    upper_theta = -10000  # unreasonably large number
    lower_theta = 10000  # unreasonably large number

    colorway = ['#8B0000', '#000080', '#006400']

    for i, exp in enumerate(experiment_list):
        plot_data_exp = x_in_np(exp)
        plot_data_single = single_path_data_polar(plot_data_exp, color=colorway[i],
                                                  experiment_name=experiment_name_list[i])
        plot_data.extend(plot_data_single)

        if max(plot_data_exp[1, :]) > upper_theta:
            upper_theta = max(plot_data_exp[1, :])
        if min(plot_data_exp[1, :]) < lower_theta:
            lower_theta = min(plot_data_exp[1, :])
        if max(plot_data_exp[0, :]) > upper_rho:
            upper_rho = max(plot_data_exp[0, :])
        if min(plot_data_exp[0, :]) < lower_rho:
            lower_rho = min(plot_data_exp[0, :])

    layout = go.Layout(
        showlegend=True,
        polar=dict(
            sector=[deg(lower_theta) - 10, deg(upper_theta) + 10],
            radialaxis=dict(
                range=[lower_rho - 0.05, upper_rho + 0.05]
            )
        ),
        title=plot_title
    )

    fig = go.Figure(data=plot_data, layout=layout)

    if save:
        save_dir = get_workspace_param("results_dir")
        opy.plot(fig, auto_open=False, filename=join(save_dir, plot_title + ".html"))
    else:
        opy.plot(fig)
    if upload_this:
        py.plot(fig)

    shall_i_sleep()


# General
def single_plot_generic(x=None, y=None, plot_title=None):
    data = []

    if x is None:
        x = arange(states[1, :].shape[0])
    else:
        t = x

    data_obj = go.Scatter(
        x=t,
        y=y,
        mode='markers',
        name=plot_title
    )
    data.append(data_obj)
    return data


def simple_plot(x_val, y_val, plot_name="", save_dir="", x_axis_name="", y_axis_name=""):
    data = []

    if x_val is None:
        x_val = arange(len(y_val))

    p1 = go.Scatter(
        x=x_val,
        y=y_val,
        mode='markers'
    )
    data.extend([p1])
    layout = dict(
    title=go.layout.Title(
        text=plot_name,
        font=dict(
            family='Courier New, monospace',
            size=20,
            color='#7f7f7f'
        )
    ),
    xaxis=go.layout.XAxis(
        title=go.layout.xaxis.Title(
            text=x_axis_name,
            font=dict(
                family='Courier New, monospace',
                size=18,
                color='#7f7f7f'
            )
        )
    ),
    yaxis=go.layout.YAxis(
        title=go.layout.yaxis.Title(
            text=y_axis_name,
            font=dict(
                family='Courier New, monospace',
                size=18,
                color='#7f7f7f'
            )
        )
    )
    )
    fig = dict(data=data, layout=layout)
    save_plot = get_param_from_config_file("save_experiment_results")
    if save_plot:
        opy.plot(fig, auto_open=False, filename=join(save_dir, plot_name + ".html"))
    else:
        opy.plot(fig)
        shall_i_sleep()


def param_convergence_plot(param_hist, plot_name="", save_dir=""):
    for param in param_hist.keys():
        iter = range(len(param_hist[param]))
        simple_plot(iter, param_hist[param], 'Convergence Plot For Parameter {}'.format(param), save_dir=save_dir)
    shall_i_sleep()


def param_space_cost_plot(cost, params_space_list):
    a = array(params_space_list)
    trace1 = go.Scatter3d(
        x=a[:, 0],
        y=a[:, 1],
        z=a[:, 2],
        mode='markers',
        marker=dict(
            size=12,
            color=array(cost) / a.size,  # set color to an array/list of desired values
            colorscale='Viridis',  # choose a colorscale
            opacity=0.8,
            showscale=True,
            cmax=0.03,
            cmin=0,
            colorbar=dict(title='Total nMSE Error'))
    )

    data = [trace1]
    layout = go.Layout(
        title='Parameter Space and Cost Function Value [Horizon Length:{}]'.format(a.shape),
        scene=dict(xaxis=dict(title='dr'),
                   yaxis=dict(title='dl'),
                   zaxis=dict(title='L')
                   ),
        margin=dict(
            l=0,
            r=0,
            b=0,
            t=0
        )
    )
    fig = go.Figure(data=data, layout=layout)
    opy.plot(fig)
    shall_i_sleep()


def path_plot_plotly(experiment, plot_name=''):
    import plotly.figure_factory as ff
    path_data = x_in_np(experiment)  # make sure that we use numpy representation
    # Create quiver figure
    fig = ff.create_quiver(path_data[0, :], path_data[1, :], cos(path_data[2, :] * pi / 180),
                           sin(path_data[2, :] * pi / 180),
                           scale=.008,
                           arrow_scale=.05,
                           name='quiver',
                           line=dict(width=1)
                           )
    layout = go.Layout(
        title=plot_name,
        xaxis=dict(
            title='x position [m]',
            ticklen=1,
            zeroline=False,
            gridwidth=2,
        ),
        yaxis=dict(
            title='y position [m]',
            ticklen=1,
            gridwidth=2,
        )
    )

    fig['layout'] = layout
    # fig = dict(data=data, layout=layout)
    opy.plot(fig)
    shall_i_sleep()

def shall_i_sleep():
    if not save_results:
        rospy.sleep(SLEEP_DURATION)
    else:
        pass
