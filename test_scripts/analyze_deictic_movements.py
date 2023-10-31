import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import pandas as pd
import numpy as np
# data = pd.read_csv("C:\dev\SpotDockerImage\data\deictic_movements\deictic_data_2023-10-30T163636.csv")
# data = pd.read_csv("C:\dev\SpotDockerImage\data\deictic_movements\deictic_data_2023-10-30T164145.csv")
# data = pd.read_csv("C:\dev\SpotDockerImage\data\deictic_movements\deictic_data_2023-10-31T101517.csv")
# data = pd.read_csv("C:\dev\SpotDockerImage\data\deictic_movements\deictic_data_2023-10-31T104708.csv")
data = pd.read_csv("C:\dev\SpotDockerImage\data\deictic_movements\deictic_data_2023-10-31T110440.csv")
print(data)

fig, ax = plt.subplots(figsize=(12, 12))

# Plotting the person's positions
ax.scatter(data['x_person'], data['y_person'], color='blue', s=150, marker='o', label='Person')
for i, txt in enumerate(data.index):
    ax.annotate(f'P{i}', (data['x_person'].iloc[i], data['y_person'].iloc[i]), fontsize=9, ha='left')

# Plotting the goal (object's) positions
ax.scatter(data['x_object'], data['y_object'], color='green', s=150, marker='s', label='Goal (Object)')
for i, txt in enumerate(data.index):
    ax.annotate(f'G{i}', (data['x_object'].iloc[i], data['y_object'].iloc[i]), fontsize=9, ha='left')

# Plotting the robot's initial positions
ax.scatter(data['x_robot'], data['y_robot'], color='red', s=150, marker='^', label='Robot Initial')
for i, txt in enumerate(data.index):
    ax.annotate(f'Ri{i}', (data['x_robot'].iloc[i] + 0.1, data['y_robot'].iloc[i]), fontsize=9, ha='left')

# Plotting the robot's final positions
ax.scatter(data['x_robot_new'], data['y_robot_new'], color='orange', s=150, marker='v', label='Robot Final')
for i, txt in enumerate(data.index):
    ax.annotate(f'Rf{i}', (data['x_robot_new'].iloc[i], data['y_robot_new'].iloc[i]), fontsize=9, ha='right')

# Plotting the desired trajectory based on the object (goal) locations
ax.plot(data['x_object'], data['y_object'], color='green', linestyle='--', label='Desired Trajectory (Goal)')

# Plotting the robot's trajectory based on its new locations
ax.plot(data['x_robot_new'], data['y_robot_new'], color='orange', linestyle='--', label='Robot Trajectory')

# Plotting the operator/person's trajectory
ax.plot(data['x_person'], data['y_person'], color='blue', linestyle='--', label='Operator/Person Trajectory')


try:
    # Plotting arrows for orientations
    arrow_length = 0.15
    for i in range(len(data)):
        ax.arrow(data['x_object'].iloc[i], data['y_object'].iloc[i], 
                arrow_length * np.cos(data['goal_rotation'].iloc[i]), 
                arrow_length * np.sin(data['goal_rotation'].iloc[i]), 
                head_width=0.1, head_length=0.15, fc='green', ec='green')
        
        ax.arrow(data['x_robot_new'].iloc[i], data['y_robot_new'].iloc[i], 
                arrow_length * np.cos(data['robot_rotation_new'].iloc[i]), 
                arrow_length * np.sin(data['robot_rotation_new'].iloc[i]), 
                head_width=0.1, head_length=0.15, fc='orange', ec='orange')
except:
    pass

# Setting axis limits and other plot details
# ax.set_xlim(-3, 5)
# ax.set_ylim(-3, 3)
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.set_title('Visualization of All Data Points with Trajectories')
ax.legend()
ax.grid(True)

plt.tight_layout()
plt.show()