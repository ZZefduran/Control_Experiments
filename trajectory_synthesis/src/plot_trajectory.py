import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
# Read the CSV data into a DataFrame
def plot_traj(df,freq):
    # Create a subplot grid with 3 rows and 1 column
    fig = make_subplots(
        rows=3, cols=1,
        subplot_titles=('Position', 'Velocity', 'Torque')
    )
    time = np.arange(len(next(iter(df.values())))) / freq
    idx = 0
    # Add Position plot
    if df.get('motor_pos') is not None:
        fig.add_trace(
            go.Scatter(x=time, y=df['motor_pos'], mode='lines', name='Position'),
            row=1+idx, col=1
        )
        idx+=1

    if df.get('motor_vel') is not None:
        fig.add_trace(
            go.Scatter(x=time, y=df['motor_vel'], mode='lines', name='Velocity'),
            row=idx+1, col=1
        )
        idx+=1

    if df.get('motor_torque') is not None:
        # Add Torque plot
        fig.add_trace(
            go.Scatter(x=time, y=df['motor_torque'], mode='lines', name='Torque'),
            row=idx+1, col=1
        )
        idx+=1

    # Update layout
    fig.update_layout(
        title_text='Time Series Data',
        height=800, # Adjust height as needed
        xaxis_title='Time',
        yaxis_title='Value'
    )

    # Show the figure
    fig.show()
