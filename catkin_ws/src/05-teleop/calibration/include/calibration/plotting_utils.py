import plotly.graph_objs as go
import plotly.offline as opy
import rospy
from itertools import izip

opy.init_notebook_mode(connected=True)

def single_plot_data(states= None, time= None, input = None, experiment_name=""):

    if time is None:
        rospy.logwarn('[plotting_utils] time vector cannot be left-out, it evaluates None')
    else:
        data = []
        t = time

    if states is not None:
        px = states[0,:]
        py = states[1,:]
        rz = states[2,:]

        p3 = go.Scatter(
            x=t,
            y=px,
            name='x position'
        )

        p4 = go.Scatter(
            x=t,
            y=py,
            name='y position'
        )

        p5 = go.Scatter(
            x=t,
            y=rz,
            name='yaw angle'
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

def multiplot(states_list=None, time_list=None, input_list=None, experiment_name_list=None, mode='single_view'):
    if mode == 'single_view':
        multiplot_single_view(states_list=states_list, time_list=time_list, input_list=input_list, experiment_name_list=experiment_name_list)
    elif mode == 'separate_views':
        multiplot_seperate_views(states_list=states_list, time_list=time_list, input_list=input_list,experiment_name_list=experiment_name_list)
    else:
        rospy.loginfo('[plotting_utils] invalid mode passed to multiplot')

def multiplot_seperate_views(states_list=None, time_list=None, input_list=None, experiment_name_list=None):
    if input_list is not None:
        for states, time, input, experiment_name in states_list,time_list,input_list, experiment_name_list:
            tek_plot_data = single_plot_data(states=states, time=time, input=input, experiment_name=experiment_name)
    else:
        for states, time, experiment_name in izip(states_list,time_list, experiment_name_list):
            plot_system(states=states, time=time, experiment_name=experiment_name)

def multiplot_single_view(states_list=None, time_list=None, input_list=None, experiment_name_list=None):
    plot_datas = []

    if input_list is not None:
        for states, time, input, experiment_name in states_list,time_list,input_list, experiment_name_list:
            tek_plot_data = single_plot_data(states=states, time=time, input=input, experiment_name=experiment_name)
    else:
        for states, time, experiment_name in izip(states_list,time_list, experiment_name_list):
            tek_plot_data = single_plot_data(states=states, time=time, experiment_name=experiment_name)
            plot_datas.extend(tek_plot_data)

    if len(plot_datas) is not 0:
        layout = dict(title='ALL PLOTS')
        fig = dict(data=plot_datas, layout=layout)
        opy.plot(fig)
    else:
        rospy.loginfo('[plotting_utils] unable to plot as data no data provided')

