#!/usr/bin/env python3

"""
Visualization script to show the dual TB3 setup and road network.
This helps understand the robot positions and network topology.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

def visualize_dual_tb3_setup():
    """Create a visualization of the dual TB3 road network setup."""
    
    # Road network nodes (from tb3_road_network_mover.cpp)
    nodes = {
        0: (-1.5,  1.0, 180),
        1: ( 0.0,  1.0, 180),
        2: ( 2.1,  1.0,  90),
        3: (-1.5, -1.0, 270),
        4: ( 0.0, -1.0,  90),
        5: ( 0.3,  0.0,   0),
        6: ( 1.7,  0.0,   0),
        7: ( 2.1, -1.0, 270),
    }
    
    # Transition matrix connections
    transitions = [
        (0, 4),  # Node 0 → Node 4
        (1, 0),  # Node 1 → Node 0
        (1, 3),  # Node 1 → Node 3
        (2, 1),  # Node 2 → Node 1
        (3, 0),  # Node 3 → Node 0
        (4, 5),  # Node 4 → Node 5
        (5, 6),  # Node 5 → Node 6
        (6, 2),  # Node 6 → Node 2
        (7, 4),  # Node 7 → Node 4
    ]
    
    # Starting positions
    tb3_1_start = (-1.5, 1.0)  # Node 0
    tb3_2_start = (0.3, 0.0)   # Node 5
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Draw transitions (arrows)
    for start_node, end_node in transitions:
        x_start, y_start, _ = nodes[start_node]
        x_end, y_end, _ = nodes[end_node]
        
        ax.annotate('',
                   xy=(x_end, y_end),
                   xytext=(x_start, y_start),
                   arrowprops=dict(arrowstyle='->', lw=1.5, color='gray', alpha=0.6))
    
    # Draw road network nodes
    for node_id, (x, y, yaw) in nodes.items():
        circle = plt.Circle((x, y), 0.1, color='blue', alpha=0.5, zorder=3)
        ax.add_patch(circle)
        ax.text(x + 0.2, y + 0.2, f'N{node_id}', fontsize=10, fontweight='bold')
        
        # Draw orientation arrow
        arrow_len = 0.15
        dx = arrow_len * np.cos(np.radians(yaw))
        dy = arrow_len * np.sin(np.radians(yaw))
        ax.arrow(x, y, dx, dy, head_width=0.08, head_length=0.08, 
                fc='blue', ec='blue', alpha=0.7, zorder=4)
    
    # Draw TB3 Robot 1
    tb3_1 = plt.Circle(tb3_1_start, 0.15, color='red', alpha=0.7, zorder=5)
    ax.add_patch(tb3_1)
    ax.text(tb3_1_start[0] - 0.3, tb3_1_start[1] + 0.3, 
            'TB3_1\n(Start)', fontsize=11, fontweight='bold', color='red',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Draw TB3 Robot 2
    tb3_2 = plt.Circle(tb3_2_start, 0.15, color='green', alpha=0.7, zorder=5)
    ax.add_patch(tb3_2)
    ax.text(tb3_2_start[0] - 0.3, tb3_2_start[1] - 0.4, 
            'TB3_2\n(Start)', fontsize=11, fontweight='bold', color='green',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Add title and labels
    ax.set_title('Dual TurtleBot3 Road Network Setup\n' + 
                'Blue nodes: Road network | Arrows: Possible transitions | ' +
                'Red/Green: TB3 robots',
                fontsize=14, fontweight='bold', pad=20)
    ax.set_xlabel('X Position (meters)', fontsize=12)
    ax.set_ylabel('Y Position (meters)', fontsize=12)
    
    # Add legend
    legend_elements = [
        patches.Circle((0, 0), 0.1, color='blue', alpha=0.5, label='Road Network Node'),
        patches.Circle((0, 0), 0.15, color='red', alpha=0.7, label='TB3 Robot 1 (/tb3_1)'),
        patches.Circle((0, 0), 0.15, color='green', alpha=0.7, label='TB3 Robot 2 (/tb3_2)'),
        patches.FancyArrow(0, 0, 0.3, 0, width=0.05, color='gray', alpha=0.6, label='Transition'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=10)
    
    # Grid and limits
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-2.5, 3.0)
    ax.set_ylim(-1.8, 1.8)
    ax.set_aspect('equal')
    
    # Add information box
    info_text = (
        "Namespace Architecture:\n"
        "━━━━━━━━━━━━━━━━━━━━━━━━\n"
        "TB3_1 Topics:\n"
        "  • /tb3_1/cmd_vel\n"
        "  • /tb3_1/odom\n"
        "  • /tb3_1/current_goal\n"
        "\n"
        "TB3_2 Topics:\n"
        "  • /tb3_2/cmd_vel\n"
        "  • /tb3_2/odom\n"
        "  • /tb3_2/current_goal\n"
    )
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
            fontsize=9, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    
    # Save figure
    output_path = '/home/basestation/ros2_ws/src/drive_drone/dual_tb3_visualization.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Visualization saved to: {output_path}")
    
    plt.show()

if __name__ == '__main__':
    print("Generating dual TB3 road network visualization...")
    visualize_dual_tb3_setup()
