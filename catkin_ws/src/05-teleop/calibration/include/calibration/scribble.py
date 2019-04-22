import plotly.graph_objs as go
import plotly.plotly as py
import plotly
import numpy as np
from scipy.interpolate import interp1d
plotly.tools.set_credentials_file(username='selcukercan', api_key='uiHkN3x2e7AfTnF8YwHk')


plot_first = False
plot_second = False
plot_third = False
plot_fourth = True

data = []
# autobot 2
#dr = [0.4092447641, 0.39738684, 0.32507522, 0.32147515, 0.30004273, 0.30507412, 0.30174047, 0.28353484, 0.29834763, 0.31358348]
#dl = [0.3943144616, 0.37012756, 0.32790826, 0.31194245, 0.31402889, 0.30280042, 0.29636639, 0.29794934, 0.30412467, 0.31554642]

# mete
dr = np.array([0.75204397, 0.54466192, 0.44755904, 0.43031595, 0.41497138, 0.3809711, 0.36190473, 0.3744712, 0.34637772, 0.34837032])
dl = np.array([0.72504231, 0.51723504, 0.4111301, 0.40478754, 0.38660895, 0.36979044, 0.34373081, 0.35664163, 0.33244699, 0.33843113])
L = 0.0522

d = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
vx = dr * d + dl * d
omega = (dr * d - dl * d) / L

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
    py.plot(fig)

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

x_test = x = xnew = np.linspace(0.1, 1, num=41, endpoint=True)
f_r = interp1d(d, dr, kind='cubic')

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

