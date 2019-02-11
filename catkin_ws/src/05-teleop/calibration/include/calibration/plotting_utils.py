import plotly.graph_objs as go
import plotly.offline as opy
import plotly.figure_factory as ff
import rospy
from os.path import join
from numpy import arange, array, cos, sin, pi
from itertools import izip
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize
from matplotlib.pyplot import figure
from data_adapter_utils import x_polar_to_cart
from utils import get_param_from_config_file, x_in_np

opy.init_notebook_mode(connected=True)

measurement_coordinate_frame = get_param_from_config_file("express_measurements_in")
SLEEP_DURATION = 1

# High Level

def path_plot(experiment, plot_name=''):
    if measurement_coordinate_frame == "polar":
        path_plot_polar(experiment, plot_name=plot_name)
    elif measurement_coordinate_frame == "cartesian":
        path_plot_cartesian(experiment, plot_name=plot_name)
    else:
        rospy.loginfo('invalid plot name defined at path_plot function')

def multiplot(states_list=None, time_list=None, input_list=None, experiment_name_list=None, plot_title='', save=False, save_dir=""):
    plot_datas = []
    # generate arange time data if time is not provided to faciliate debugging
    if time_list is None:
        time_list = []
        for i in range(len(states_list)):
            time_list.append(arange(states_list[i][1, :].shape[0]))

    if input_list is not None:
        for states, time, input, experiment_name in izip(states_list,time_list,input_list, experiment_name_list):
            tek_plot_data = single_plot_data(states=states, time=time, input=input, experiment_name=experiment_name)
            plot_datas.extend(tek_plot_data)
    else:
        for states, time, experiment_name in izip(states_list,time_list, experiment_name_list):
            tek_plot_data = single_plot_data(states=states, time=time, experiment_name=experiment_name)
            plot_datas.extend(tek_plot_data)
    if len(plot_datas) is not 0:
        layout = dict(title=plot_title)
        fig = dict(data=plot_datas, layout=layout)
        if save:
            opy.plot(fig, auto_open=False, filename=join(save_dir, plot_title + ".html"))
        else:
            opy.plot(fig)
    else:
            rospy.loginfo('[plotting_utils] unable to plot as data no data provided')

def single_plot_data(states=None, time=None, input=None, experiment_name=""):
    if measurement_coordinate_frame == "cartesian":
        return single_plot_data_cartesian(states= states, time= time, input = input , experiment_name=experiment_name)
    elif measurement_coordinate_frame == "polar":
        return single_plot_data_polar(states= states, time= time, input = input , experiment_name=experiment_name)


# Cartesian
def single_plot_data_cartesian(states= None, time= None, input = None, experiment_name=""):
    data = []

    if time is None:
        rospy.logwarn('[plotting_utils] time vector is left out, using array indexes (0, 1, ..., n) for plotting')
        t = arange(states[1,:].shape[0])
    else:
        t = time

    if states is not None:
        px = states[0,:]
        py = states[1,:]
        rz = states[2,:]

        p3 = go.Scatter(
            x=t,
            y=px,
            name='x pos ' + experiment_name.split('_')[-1]
        )

        p4 = go.Scatter(
            x=t,
            y=py,
            name='y pos ' + experiment_name.split('_')[-1]
        )

        p5 = go.Scatter(
            x=t,
            y=rz,
            name='yaw ang ' + experiment_name.split('_')[-1]
        )

        data.extend([p3, p4, p5])
    if input is not None:
        r = input[0,:]
        l = input[1,:]

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

def path_plot_cartesian(experiment, plot_name=''):
    path_data = x_in_np(experiment)

    fig, ax = plt.subplots()
    #q = ax.quiver(path_data[0,:], path_data[1,:], cos(path_data[2,:] * pi / 180), sin(path_data[2,:] * pi / 180))
    q = ax.quiver([path_data[0,:], path_data[0,:] * 1.1], [path_data[1,:], path_data[1,:] * 1.1],
                  [cos(path_data[2,:] * pi / 180), cos(path_data[2,:] * pi / 180)], [sin(path_data[2,:] * pi / 180), sin(path_data[2,:] * pi / 180)])

    ax.quiverkey(q, X=0.3, Y=1.1, U=10,
                 label='Quiver key, length = 10', labelpos='E')
    plt.savefig("path_plot.png")
    plt.show()
    rospy.sleep(SLEEP_DURATION)

def path_plot_cartesian_plotly(data_sets, data_set_names):
    fig, ax = plt.subplots()
    colormaps = [cm.Reds, cm.Blues, cm.Greens]
    for i, data in enumerate(data_sets):
        path_data = x_in_np(data)

        if measurement_coordinate_frame == "polar":
            path_data = x_polar_to_cart(path_data)

        x_datas = path_data[0, :]
        y_datas = path_data[1, :]
        u_datas = cos(path_data[2, :] * pi / 180)
        v_datas = sin(path_data[2, :] * pi / 180)

        colors = arange(len(x_datas))
        norm = Normalize()
        norm.autoscale(colors)

        colormap = colormaps[i]

        q = ax.quiver(x_datas, y_datas, u_datas, v_datas, color=colormap(norm(colors)))
        #ax.quiverkey(q, X=0.3, Y=1.1, U=10, label='Quiver key', length='10', labelpos='E')

    ax.legend(data_set_names)
    figure(num=1, figsize=(20, 12), dpi=80, facecolor='w', edgecolor='k')
    fig.suptitle('Measurements and Model Predictions', fontsize=20)
    plt.xlabel('X [m]', fontsize=18)
    plt.ylabel('Y [m]', fontsize=18)
    plt.savefig("new.png")
    rospy.sleep(SLEEP_DURATION)


# Polar
def single_plot_data_polar(states= None, time= None, input = None, experiment_name=""):
    data = []

    if time is None:
        rospy.logwarn('[plotting_utils] time vector is left out, using array indexes (0, 1, ..., n) for plotting')
        t = arange(states[1,:].shape[0])
    else:
        t = time

    if states is not None:
        rho = states[0,:]
        yaw = states[1,:]

        p3 = go.Scatter(
            x=t,
            y=rho,
            name='rho ' + experiment_name.split('_')[-1]
        )

        p4 = go.Scatter(
            x=t,
            y=yaw,
            name='yaw pos ' + experiment_name.split('_')[-1]
        )

        data.extend([p3, p4])

    if input is not None:
        r = input[0,:]
        l = input[1,:]

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

def single_path_data_polar(states=None):
    x_states = x_in_np(states)
    data = [
        go.Scatterpolar(
            r=x_states[0,:],
            theta=x_states[1,:],
            mode='markers',
            marker=dict(
                color='peru'
            )
        )
    ]
    return data

def path_plot_polar(single_experiment, plot_name=''):
    data = x_in_np(single_experiment)
    plot_data = single_path_data_polar(data)
    layout = go.Layout(
        showlegend=False,
        polar=dict(
            sector=[min(data[1,:]) - 10, max(data[1,:]) + 10],
            radialaxis=dict(
                range=[min(data[0,:]) - 0.05, max(data[0,:]) + 0.05]
            )
        ),
        title=plot_name
    )
    fig = go.Figure(data=plot_data, layout=layout)
    opy.plot(fig)
    rospy.sleep(SLEEP_DURATION)


# General
def simple_plot(x_val, y_val, plot_name=""):
    data = []

    if x_val is None:
        x_val= arange(len(y_val))

    p1 = go.Scatter(
        x=x_val,
        y=y_val,
        mode='markers'
    )
    data.extend([p1])
    layout = dict(title=plot_name)
    fig = dict(data=data, layout=layout)
    opy.plot(fig)
    rospy.sleep(SLEEP_DURATION)

def param_convergence_plot(param_hist):
    for param in param_hist.keys():
        iter = range(len(param_hist[param]))
        simple_plot(iter, param_hist[param], 'Parameter {}'.format(param))
    rospy.sleep(SLEEP_DURATION)

def param_space_cost_plot(cost, params_space_list):
    a = array(params_space_list)
    trace1 = go.Scatter3d(
        x=a[:,0],
        y=a[:,1],
        z=a[:,2],
        mode='markers',
        marker=dict(
            size=12,
            color=array(cost)/a.size,  # set color to an array/list of desired values
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
    rospy.sleep(SLEEP_DURATION)

def plot_system(states=None, time=None, input=None, experiment_name=""):
    data = single_plot_data_cartesian(states=states, time=time, input=input, experiment_name=experiment_name)
    if len(data) is not 0:
        layout = dict(title=experiment_name)
        fig = dict(data=data, layout=layout)
        opy.plot(fig)
    else:
        rospy.loginfo('[plotting_utils] unable to plot as data no data provided')

# Void?

def path_plot_plotly(experiment, plot_name=''):

    path_data = x_in_np(experiment) #make sure that we use numpy representation
    # Create quiver figure
    fig = ff.create_quiver(path_data[0,:], path_data[1,:], cos(path_data[2,:] * pi / 180), sin(path_data[2,:] * pi / 180),
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
    #fig = dict(data=data, layout=layout)
    opy.plot(fig)
    rospy.sleep(SLEEP_DURATION)
