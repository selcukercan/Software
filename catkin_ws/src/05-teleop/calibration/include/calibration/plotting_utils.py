import plotly.graph_objs as go
import plotly.offline as opy
import rospy
from numpy import arange
from itertools import izip

opy.init_notebook_mode(connected=True)

def single_plot_data(states= None, time= None, input = None, experiment_name=""):
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

def plot_system(states= None, time= None, input = None, experiment_name=""):
        data = single_plot_data(states=states, time=time, input=input,experiment_name=experiment_name)
        if len(data) is not 0:
            layout = dict(title=experiment_name)
            fig = dict(data=data, layout=layout)
            opy.plot(fig)
        else:
            rospy.loginfo('[plotting_utils] unable to plot as data no data provided')

def multiplot(states_list=None, time_list=None, input_list=None, experiment_name_list=None, mode='single_view', plot_title=''):
    if mode == 'single_view':
        multiplot_single_view(states_list=states_list, time_list=time_list, input_list=input_list, experiment_name_list=experiment_name_list, plot_title=plot_title)
    elif mode == 'separate_views':
        multiplot_seperate_views(states_list=states_list, time_list=time_list, input_list=input_list,experiment_name_list=experiment_name_list)
    else:
        rospy.loginfo('[plotting_utils] invalid mode passed to multiplot')

def multiplot_seperate_views(states_list=None, time_list=None, input_list=None, experiment_name_list=None):
    if input_list is not None:
        for states, time, input, experiment_name in izip(states_list,time_list,input_list, experiment_name_list):
            tek_plot_data = single_plot_data(states=states, time=time, input=input, experiment_name=experiment_name)
    else:
        for states, time, experiment_name in izip(states_list,time_list, experiment_name_list):
            plot_system(states=states, time=time, experiment_name=experiment_name)

def multiplot_single_view(states_list=None, time_list=None, input_list=None, experiment_name_list=None, plot_title=''):
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
        opy.plot(fig)
    else:
        rospy.loginfo('[plotting_utils] unable to plot as data no data provided')

def simple_plot(x_val, y_val, plot_name=""):
    data = []
    p1 = go.Scatter(
        x=x_val,
        y=y_val,
    )
    data.extend([p1])
    layout = dict(title=plot_name)
    fig = dict(data=data, layout=layout)
    opy.plot(fig)
