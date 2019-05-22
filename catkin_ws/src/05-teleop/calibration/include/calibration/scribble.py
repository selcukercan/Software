import plotly.graph_objs as go
import plotly.plotly as py
import plotly.offline as opy
import time
#import plotly
#plotly.tools.set_credentials_file(username='selcukercan', api_key='uiHkN3x2e7AfTnF8YwHk')
import numpy as np
from scipy.interpolate import interp1d
from scipy.optimize import curve_fit

def exponential_decay(x, a, c, d):
    return a*np.exp(-c*x)+d

plot_zero = True
plot_first = False
plot_second = False
plot_third = False
plot_fourth = False
plot_fifth  = False # manually play with exponential parameters
plot_sixth = False
data = []
# autobot 2
#dr = [0.4092447641, 0.39738684, 0.32507522, 0.32147515, 0.30004273, 0.30507412, 0.30174047, 0.28353484, 0.29834763, 0.31358348]
#dl = [0.3943144616, 0.37012756, 0.32790826, 0.31194245, 0.31402889, 0.30280042, 0.29636639, 0.29794934, 0.30412467, 0.31554642]

# mete
dr = np.array([0.75204397, 0.54466192, 0.44755904, 0.43031595, 0.41497138, 0.3809711, 0.36190473, 0.3744712, 0.34637772, 0.34837032])
dl = np.array([0.72504231, 0.51723504, 0.4111301, 0.40478754, 0.38660895, 0.36979044, 0.34373081, 0.35664163, 0.33244699, 0.33843113])
#dr = np.array([0.75204397, 0.54466192, 0.44755904, 0.43031595, 0.41497138])
#dl = np.array([0.72504231, 0.51723504, 0.4111301, 0.40478754, 0.38660895])

L = 0.0522

d = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
#d = np.array([0.1, 0.2, 0.3, 0.4, 0.5])

vx = dr * d + dl * d
omega = (dr * d - dl * d) / L

x = np.arange(0,2,0.01)
current = 1 - np.exp(-3 * x)

power = 2
vel = power / current

if plot_zero:
    data  = []
    # Drive Constant Variation for Different Duty Cycles
    p1 = go.Scatter(
            x=x,
            y=vel,
            mode='markers')

    data.extend([p1])
    fig = dict(data=data)
    opy.plot(fig)
    time.sleep(1)


if plot_first:
    # Drive Constant Variation for Different Duty Cycles
    p1 = go.Scatter(
            x=d,
            y=dl,
            mode='markers')
    p2 = go.Scatter(
            x=d,
            y=dr,
            mode='markers')
    data.extend([p1, p2])

    layout = dict(
        title=go.layout.Title(
            text="Drive Constant Variation for Different Duty Cycles",
            font=dict(
                family='Courier New, monospace',
                size=20,
                color='#7f7f7f'
            )
        ),
        xaxis=go.layout.XAxis(
            title=go.layout.xaxis.Title(
                text="Duty Cycle",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        ),
        yaxis=go.layout.YAxis(
            title=go.layout.yaxis.Title(
                text="Drive Constant [m/s]",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        )
    )
    fig = dict(data=data, layout=layout)
    opy.plot(fig)
    time.sleep(1)


if plot_second:
    # Drive constant vs Velocity
    data2 = []

    p3 = go.Scatter(
            x=d,
            y=vx,
            mode='markers')

    data2.extend([p3])

    layout2 = dict(
        title=go.layout.Title(
            text="Drive constant vs Longitudinal Velocity",
            font=dict(
                family='Courier New, monospace',
                size=20,
                color='#7f7f7f'
            )
        ),
        xaxis=go.layout.XAxis(
            title=go.layout.xaxis.Title(
                text="Duty Cycle",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        ),
        yaxis=go.layout.YAxis(
            title=go.layout.yaxis.Title(
                text="Velocity [m/s]",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        )
    )
    fig2 = dict(data=data2, layout=layout2)
    py.plot(fig2)

if plot_third:
    # Drive constant vs Velocity
    data3 = []

    p4 = go.Scatter(
            x=d,
            y=omega,
            mode='markers')

    data3.extend([p4])

    layout3 = dict(
        title=go.layout.Title(
            text="Drive constant vs Rotational Velocity",
            font=dict(
                family='Courier New, monospace',
                size=20,
                color='#7f7f7f'
            )
        ),
        xaxis=go.layout.XAxis(
            title=go.layout.xaxis.Title(
                text="Duty Cycle",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        ),
        yaxis=go.layout.YAxis(
            title=go.layout.yaxis.Title(
                text="Omega [rad/s]",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        )
    )
    fig3 = dict(data=data3, layout=layout3)
    py.plot(fig3)


x_test = x = xnew = np.linspace(0.1, 1, num=100, endpoint=True)
#f_exp = exponential_decay(x_test, 1 ,7, 0.35)
popt_right, pcov = curve_fit(exponential_decay, d, dr, p0=(1 ,7, 0.35))
popt_left, pcov = curve_fit(exponential_decay, d, dl, p0=(1 ,7, 0.35))
#f_r = interp1d(d, dr, kind='cubic')

if plot_fourth:
    # Drive constant vs Velocity
    data3 = []

    p4 = go.Scatter(
            x=x_test,
            y=f_r(x_test),
            mode='markers')

    data3.extend([p4])

    layout3 = dict(
        title=go.layout.Title(
            text="Fitting to Drive Constant Variation",
            font=dict(
                family='Courier New, monospace',
                size=20,
                color='#7f7f7f'
            )
        ),
        xaxis=go.layout.XAxis(
            title=go.layout.xaxis.Title(
                text="Duty Cycle",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        ),
        yaxis=go.layout.YAxis(
            title=go.layout.yaxis.Title(
                text="Omega [rad/s]",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        )
    )
    fig3 = dict(data=data3, layout=layout3)
    py.plot(fig3)

if plot_fifth:
    # Drive constant vs Velocity
    data3 = []

    p4 = go.Scatter(
            x=x_test,
            y=f_exp,
            mode='markers')

    data3.extend([p4])

    layout3 = dict(
        title=go.layout.Title(
            text="Fitting to Drive Constant Variation",
            font=dict(
                family='Courier New, monospace',
                size=20,
                color='#7f7f7f'
            )
        ),
        xaxis=go.layout.XAxis(
            title=go.layout.xaxis.Title(
                text="Duty Cycle",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        ),
        yaxis=go.layout.YAxis(
            title=go.layout.yaxis.Title(
                text="Omega [rad/s]",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        )
    )
    fig3 = dict(data=data3, layout=layout3)
    opy.plot(fig3)

if plot_sixth:
    # Drive constant vs Velocity
    data = []

    # Drive Constant Variation for Different Duty Cycles
    p1 = go.Scatter(
            x=d,
            y=dl,
            mode='markers',
            name="left drive constants")
    p2 = go.Scatter(
            x=d,
            y=dr,
            mode='markers',
            name="right drive constants")

    p3 = go.Scatter(
            x=x_test,
            y=exponential_decay(x_test, popt_left[0], popt_left[1], popt_left[2]),
            mode='markers',
            name="exponential fit left")

    p4 = go.Scatter(
            x=x_test,
            y=exponential_decay(x_test, popt_right[0], popt_right[1], popt_right[2]),
            mode='markers',
            name="exponential fit right")

    data.extend([p1,p2,p3,p4])

    layout = dict(
        title=go.layout.Title(
            text="Fitting to Drive Constant Variation",
            font=dict(
                family='Courier New, monospace',
                size=20,
                color='#7f7f7f'
            )
        ),
        xaxis=go.layout.XAxis(
            title=go.layout.xaxis.Title(
                text="Duty Cycle",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        ),
        yaxis=go.layout.YAxis(
            title=go.layout.yaxis.Title(
                text="Omega [rad/s]",
                font=dict(
                    family='Courier New, monospace',
                    size=18,
                    color='#7f7f7f'
                )
            )
        )
    )
    fig = dict(data=data, layout=layout)
    opy.plot(fig)
