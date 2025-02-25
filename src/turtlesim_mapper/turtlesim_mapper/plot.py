import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import time
from pathlib import Path

# Set up the figure and subplots with specified layout
fig = plt.figure(figsize=(15, 10))
gs = fig.add_gridspec(2, 3)

# Create subplots with specific positions
ax_xy = fig.add_subplot(gs[0, 0])  # X-Y Trajectory
ax_angle = fig.add_subplot(gs[0, 1])  # Angle vs Time
ax_x = fig.add_subplot(gs[1, 0])  # X Position vs Time
ax_y = fig.add_subplot(gs[1, 1])  # Y Position vs Time
ax_angular_vel = fig.add_subplot(gs[1, 2])  # Angular Velocity vs Time

fig.suptitle('Real-time Pose Data Visualization')

def read_data():
    try:
        data = pd.read_csv('pose_data.txt', header=None, 
                          names=['time', 'x', 'y', 'angle', 'angular_velocity'])
        return data
    except:
        return None

def animate(i):
    data = read_data()
    if data is None:
        return
    
    # Calculate the time window (last 20 seconds)
    current_time = data['time'].iloc[-1]
    time_cutoff = current_time - 20
    
    # Filter data for last 20 seconds
    mask = data['time'] >= time_cutoff
    recent_data = data[mask]
    
    # Convert timestamps to relative time (seconds ago)
    relative_times = recent_data['time'] - current_time
    
    # Clear all axes
    ax_xy.clear()
    ax_x.clear()
    ax_y.clear()
    ax_angle.clear()
    ax_angular_vel.clear()
    
    # Plot X-Y trajectory with fixed limits
    ax_xy.plot(recent_data['x'], recent_data['y'], 'b-')
    ax_xy.set_title('X-Y Trajectory')
    ax_xy.set_xlabel('X Position')
    ax_xy.set_ylabel('Y Position')
    ax_xy.grid(True)
    ax_xy.set_xlim(-0.5, 11.5)
    ax_xy.set_ylim(-0.5, 11.5)
    
    # Plot X position vs time
    ax_x.plot(relative_times, recent_data['x'], 'r-')
    ax_x.set_title('X Position vs Time')
    ax_x.set_xlabel('Time (s)')
    ax_x.set_ylabel('X Position')
    ax_x.grid(True)
    ax_x.set_ylim(-0.5, 11.5)
    
    # Plot Y position vs time
    ax_y.plot(relative_times, recent_data['y'], 'g-')
    ax_y.set_title('Y Position vs Time')
    ax_y.set_xlabel('Time (s)')
    ax_y.set_ylabel('Y Position')
    ax_y.grid(True)
    ax_y.set_ylim(-0.5, 11.5)
    
    # Plot angle vs time with fixed y limits
    ax_angle.plot(relative_times, recent_data['angle'], 'purple')
    ax_angle.set_title('Angle vs Time')
    ax_angle.set_xlabel('Time (s)')
    ax_angle.set_ylabel('Angle (rad)')
    ax_angle.grid(True)
    ax_angle.set_ylim(-3.5, 3.5)
    
    # Plot angular velocity vs time with fixed y limits
    ax_angular_vel.plot(relative_times, recent_data['angular_velocity'], 'orange')
    ax_angular_vel.set_title('Angular Velocity vs Time')
    ax_angular_vel.set_xlabel('Time (s)')
    ax_angular_vel.set_ylabel('Angular Velocity (rad/s)')
    ax_angular_vel.grid(True)
    ax_angular_vel.set_ylim(-2.5, 2.5)
    
    # Adjust layout
    plt.tight_layout()

# Create animation
ani = animation.FuncAnimation(fig, animate, interval=1000)  # Update every 1000ms (1 second)

# Show the plot
plt.show()