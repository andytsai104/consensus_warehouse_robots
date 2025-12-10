import numpy as np
import matplotlib.pyplot as plt
import math

# --- 1. SETUP PARAMETERS ---
num_robots = 5
num_tasks = 4
iterations = 10
epsilon = 0.1

# --- Robot's adjacency matrix ---
adjacency_matrix = np.eye(num_robots)
adjacency_matrix[0,1] = 1; adjacency_matrix[1,0] = 1
adjacency_matrix[1,2] = 1; adjacency_matrix[2,1] = 1
adjacency_matrix[2,3] = 1; adjacency_matrix[3,2] = 1 
adjacency_matrix[3,4] = 1; adjacency_matrix[4,3] = 1 

# --- Robot/Task allocation ---
robot_positions = np.array([
    [0.0, 2.5],   # R0
    [-2.0, 7.5],  # R1
    [-7.5, -7.5], # R2
    [2.5, -5.0],  # R3
    [8.0, -3.0]   # R4
])

task_positions = np.array([
    [1.0,   1.0], # T0
    [-1.0, -1.0], # T1
    [7.5,   3.0], # T2
    [-4.0, -6.0], # T3
])

robot_availability = np.ones(num_robots)
history = [] 

# --- ITERATIVE CONSENSUS LOOP ---
print("Starting Iterative Consensus...")

for k in range(num_tasks):
    # Initialize Bids
    current_bids = np.zeros(num_robots)
    for i in range(num_robots):
        if robot_availability[i] == 1:
            dist = np.linalg.norm(robot_positions[i] - task_positions[k])
            current_bids[i] = 1.0 / (dist + epsilon)
        else:
            current_bids[i] = -1.0 # Busy robots bid -1

    consensus_state = current_bids.copy()
    task_history = np.zeros((iterations + 1, num_robots))
    task_history[0, :] = consensus_state

    # Run Max-Consensus
    for t in range(iterations):
        new_state = consensus_state.copy()
        for i in range(num_robots):
            neighbors = np.where(adjacency_matrix[i] == 1)[0]
            local_max = np.max(consensus_state[neighbors])
            new_state[i] = local_max
        consensus_state = new_state
        task_history[t+1, :] = consensus_state

    # Determine Winner
    final_value = consensus_state[0]
    winner_idx = -1
    for i in range(num_robots):
        if robot_availability[i] == 1:
            dist = np.linalg.norm(robot_positions[i] - task_positions[k])
            bid = 1.0 / (dist + epsilon)
            if np.abs(bid - final_value) < 1e-6:
                winner_idx = i
                break
    
    print(f"Task {k} Assigned to: Robot {winner_idx} (Bid: {final_value:.4f})")
    
    if winner_idx != -1:
        robot_availability[winner_idx] = 0
    
    # CHANGE 1: Save both the history AND the winner_idx as a tuple
    history.append((task_history, winner_idx))

# --- PLOT RESULTS ---
plots_per_image = 2
num_images = math.ceil(num_tasks / plots_per_image)

print(f"Generating {num_images} image(s)...")

for img_idx in range(num_images):
    fig, axes = plt.subplots(2, 1, figsize=(15, 8))
    axes = axes.flatten()
    
    start_task = img_idx * plots_per_image
    end_task = min((img_idx + 1) * plots_per_image, num_tasks)
    
    for i, task_idx in enumerate(range(start_task, end_task)):
        ax = axes[i]
        
        # CHANGE 2: Unpack the data and the winner
        data, winner = history[task_idx]
        
        for r in range(num_robots):
            # Style 1: Busy Robot (Dotted, faint)
            if data[0, r] < 0:
                ax.plot(range(iterations + 1), data[:, r], 
                        linestyle=':', alpha=0.3, label=f'R{r} (Busy)')
            
            # Style 2: The WINNER (Thick line, Solid, Explicit Label)
            elif r == winner:
                ax.plot(range(iterations + 1), data[:, r], 
                        marker='o', markersize=4, linewidth=3, color='red', 
                        label=f'R{r} (WINNER)')
            
            # Style 3: Loser (Thin line, regular)
            else:
                ax.plot(range(iterations + 1), data[:, r], 
                        marker='o', markersize=3, linewidth=1, alpha=0.6, 
                        label=f'R{r}')
        
        # Add winner info to title
        ax.set_title(f'Task {task_idx + 1} (Winner: R{winner})')
        ax.set_xlabel('Iteration')
        ax.set_ylabel('Bid Value')
        ax.grid(True)
        
        # Show legend on all plots so we can see who the winner is for each
        ax.legend(fontsize='small', loc='lower right')

    for j in range(end_task - start_task, plots_per_image):
        fig.delaxes(axes[j])

    plt.tight_layout()
    # plt.savefig('consensus_result.png')
    print("Plot generated.")
    plt.show()