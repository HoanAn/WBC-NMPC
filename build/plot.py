import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
import os

# --- Configuration ---
LOG_FILE_PATH = "log_state.csv"
NQ_JOINTS = 3       # Number of elements in a group (i.e., number of subplots)
NUM_DATA_GROUPS = 11 # Total number of data groups 
HISTORY_POINTS = 200 # How many past points to show on the persistent plots
TIME_SHIFT_STEP = 10.0 # The amount of time to shift each consecutive predicted group by (e.g., 10)
# ---

# --- Plotting Configuration ---
# Format strings for line labels, using placeholder names for clarity
LABEL_FMT_PERSISTENT = 'Group {k_index} E{j_index} (Persistent)'
LABEL_FMT_SHIFTED = 'Group {k_index} E{j_index} (Shifted/Instantaneous)'

# Initialize the figure and axes (3 subplots)
fig, axs = plt.subplots(NQ_JOINTS, 1, sharex=True, figsize=(10, 8))

# Define colors for the lines. 
COLOR_GROUP_1 = 'tab:blue' # Current state
COLOR_GROUP_2 = 'tab:green' # Desired state / Reference
COLOR_PREDICTION_GROUPS = 'tab:red' # Future predicted states

# Initialize a 2D list for lines: lines[j] is the list of lines for subplot j+1.
lines = []
for j in range(NQ_JOINTS):
    lines.append([])
    for k in range(NUM_DATA_GROUPS):
        # Line properties
        color = COLOR_GROUP_1 if k == 0 else (COLOR_GROUP_2 if k == 1 else COLOR_PREDICTION_GROUPS)
        
        # KEY FIX: Use 'None' for linestyle and a marker for prediction groups
        linestyle = '-' if k < 2 else 'None' 
        alpha = 1.0 if k < 2 else 0.8
        
        marker = 'None' if k < 2 else 'o'
        markersize = 0 if k < 2 else 4

        # Calculate indices for formatting
        j_index = j + 1
        k_index = k + 1
        
        # Construct label using str.format()
        if k < 2:
            label = LABEL_FMT_PERSISTENT.format(j_index=j_index, k_index=k_index)
        else:
            label = LABEL_FMT_SHIFTED.format(j_index=j_index, k_index=k_index)

        line, = axs[j].plot(
            [], [], 
            label=label, 
            color=color, 
            linestyle=linestyle, 
            alpha=alpha,
            marker=marker,
            markersize=markersize
        )
        lines[j].append(line)


# Initialize data buffers
times = []
# 3D list for data: data_history[j][k] is the list of values for element j of group k.
data_history = [[[] for _ in range(NUM_DATA_GROUPS)] for _ in range(NQ_JOINTS)]


def parse_log_line(line, NQ_JOINTS):
    """
    Parses a single line from log_state.csv.
    """
    if not line or line.startswith('#'):
        return None

    groups = line.strip().split(',')
    
    if not groups:
        return None

    try:
        iteration_str = groups[0].strip()
        if not iteration_str:
            return None
            
        time_value = float(iteration_str)
        data_groups = groups[1:]
        
        parsed_data = []
        for group in data_groups:
            elements = [float(x) for x in group.strip().split() if x]
            
            if not elements and group.strip() == '':
                continue
                
            if len(elements) != NQ_JOINTS:
                return None 

            parsed_data.append(np.array(elements))

        if len(parsed_data) != NUM_DATA_GROUPS:
             return None

        return time_value, parsed_data

    except ValueError:
        return None
    except IndexError:
        return None


def read_last_valid_data(file_path):
    """Reads the last valid data entry from the log file."""
    try:
        with open(file_path, 'r') as f:
            all_lines = f.readlines()
            
            if not all_lines:
                return None

            for line in reversed(all_lines):
                data = parse_log_line(line, NQ_JOINTS)
                if data:
                    return data
            
            return None
            
    except Exception as e:
        return None


def update_plot(frame):
    """Callback function to update the plot data."""
    new_data = read_last_valid_data(LOG_FILE_PATH)
    
    if new_data:
        new_time, new_all_groups_data = new_data
        
        if times and new_time <= times[-1]:
             return [line_obj for sublist in lines for line_obj in sublist] 

        # --- Update Persistent Groups (k=0, 1) ---
        times.append(new_time)
        
        for j in range(NQ_JOINTS):
            for k in range(min(NUM_DATA_GROUPS, 2)): # Only for k=0 and k=1
                data_history[j][k].append(new_all_groups_data[k][j])
            
        # Trim old data for persistent groups
        # if len(times) > HISTORY_POINTS:
        #     times.pop(0)
        #     for j in range(NQ_JOINTS):
        #         for k in range(min(NUM_DATA_GROUPS, 2)):
        #             data_history[j][k].pop(0)


        # --- Update Plot Lines and Axes ---
        all_artists = []
        max_time_shifted = new_time
        
        for j in range(NQ_JOINTS): # j is the subplot index (0, 1, 2)
            for k in range(NUM_DATA_GROUPS): # k is the group index (0 to M-1)
                
                if k < 2:
                    # Persistent Groups (k=0 and k=1)
                    lines[j][k].set_data(times, data_history[j][k])
                    
                else:
                    # Instantaneous/Shifted Groups (k=2 onwards)
                    # Index k=2 (Group 3): Offset should be 1 * 10 = 10
                    # Index k=3 (Group 4): Offset should be 2 * 10 = 20
                    time_offset = (k - 1) * TIME_SHIFT_STEP
                    shifted_time = new_time + time_offset
                    
                    # Update max_time for X-axis scaling
                    max_time_shifted = max(max_time_shifted, shifted_time)
                    
                    # Set line data to a single point at the shifted time.
                    lines[j][k].set_data([shifted_time], [new_all_groups_data[k][j]])
                
                all_artists.append(lines[j][k])
            
            # --- Axis Scaling ---
            if times:
                min_x = times[0]
            else:
                min_x = new_time
                
            # Set X limits based on all visible data points
            axs[j].set_xlim(min_x, max_time_shifted + TIME_SHIFT_STEP * 2)
            
            # Y-axis: Autoscale based on current data
            axs[j].relim()
            axs[j].autoscale_view(True, True, True)

            # Set labels and title
            axs[j].set_ylabel(f'Element {j+1} Value')
            
            # --- Legend ---
            if j == 0:
                handles = [lines[j][0]]
                labels = [f'Group 1 (Current State)']
                
                if NUM_DATA_GROUPS > 1:
                    handles.append(lines[j][1])
                    labels.append(f'Group 2 (Predicted 1)')

                if NUM_DATA_GROUPS > 2:
                    # Create a proxy line/point for the shifted groups for the legend
                    # Use a dummy artist to represent the style of the prediction points
                    proxy_artist = plt.Line2D(
                        [], [], 
                        color=COLOR_PREDICTION_GROUPS, 
                        marker='o', 
                        linestyle='None', 
                        markersize=4
                    )
                    handles.append(proxy_artist)
                    labels.append(f'Groups 3-{NUM_DATA_GROUPS} (Predicted 2 to 10)')
                
                axs[j].legend(handles=handles, labels=labels, loc='upper right', fontsize='small')

            axs[j].grid(True, linestyle=':', alpha=0.6)
            
        # Only set the xlabel on the bottom subplot
        axs[-1].set_xlabel('Iteration/Time Step (Shifted for Prediction)')
        
        fig.suptitle('Real-Time Log Data Plot (Persistent vs. Shifted Prediction)')
        fig.tight_layout(rect=[0, 0.03, 1, 0.95])
        
        return all_artists

# Create the animation that calls update_plot every 50 milliseconds
ani = animation.FuncAnimation(
    fig, 
    update_plot, 
    interval=10, 
    blit=True 
)

# Run the plot
plt.show()