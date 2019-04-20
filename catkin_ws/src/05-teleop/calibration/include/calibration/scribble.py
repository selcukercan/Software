import plotly.graph_objs as go
import plotly.plotly as py
import plotly
plotly.tools.set_credentials_file(username='selcukercan', api_key='uiHkN3x2e7AfTnF8YwHk')

data = []

dl = [0.3943144616, 0.37012756, 0.32790826, 0.31194245, 0.31402889, 0.30280042, 0.29636639, 0.29794934, 0.30412467, 0.31554642]
dr = [0.4092447641, 0.39738684, 0.32507522, 0.32147515, 0.30004273, 0.30507412, 0.30174047, 0.28353484, 0.29834763, 0.31358348]
d = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

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
