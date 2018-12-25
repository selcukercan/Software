import plotly.graph_objs as go
import plotly.offline as opy
import rospy

opy.init_notebook_mode(connected=True)

def plot_system(states= None, time= None, input = None):

    if time == None:
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

    if len(data) is not 0:
        opy.plot(data)
    else:
        rospy.loginfo('[plotting_utils] unable to plot as data no data provided')

