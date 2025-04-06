#!/usr/bin/env python3
import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
import tempfile
import subprocess
import os
import threading
import time
import copy
import argparse
import signal

class InteractiveCBS:
    def __init__(self, map_file):
        # Load map data
        with open(map_file, 'r') as f:
            self.map_data = yaml.safe_load(f)
        
        # Original map data (keep track for reset)
        self.original_map_data = copy.deepcopy(self.map_data)
        
        # Set up workspace paths
        self.map_file = map_file
        self.temp_map_file = None
        self.temp_output_file = None
        self.cbs_executable = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build", "run_cbs")
        
        # Animation variables
        self.current_schedule = None
        self.animation_running = False
        self.animation_thread = None
        self.solution_found = False
        
        # Planning variables
        self.search_process = None
        self.search_running = False
        self.map_modified = False
        self.current_agent_positions = None
        self.initial_positions_visible = True
        
        # Set up the interactive figure with a single pane
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title("Interactive CBS Path Planning")
        
        # Configure axes
        self.ax.set_title("Click to add/remove obstacles, then click 'Run Planning'")
        self.ax.set_aspect('equal')
        
        # Initialize the grid representation
        self.init_grid()
        
        # Add UI elements
        self.init_ui()
        
        # Connect mouse click event to the axis
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # Create an initial temporary map file
        self.save_temp_map()
        
        # Run initial planning
        self.run_cbs()

    def init_grid(self):
        # Get dimensions
        width, height = self.map_data["map"]["dimensions"]
        
        # Clear previous elements
        self.ax.clear()
        
        # Set up grid
        self.ax.set_xlim(-0.5, width - 0.5)
        self.ax.set_ylim(-0.5, height - 0.5)
        self.ax.set_xticks(range(width))
        self.ax.set_yticks(range(height))
        self.ax.grid(True, linestyle='-', linewidth=1)
        
        # Reset patches
        self.map_obstacles = []
        self.map_agents = []
        self.map_goals = []
        self.agent_circles = []
        
        # Draw obstacles
        for x, y in self.map_data["map"]["obstacles"]:
            rect = Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='black')
            self.ax.add_patch(rect)
            self.map_obstacles.append((x, y, rect))
        
        # Draw agents and goals
        for i, agent in enumerate(self.map_data["agents"]):
            # Agent start
            start_x, start_y = agent["start"]
            color = f"C{i}"
            circle = Circle((start_x, start_y), 0.3, facecolor=color, edgecolor='black')
            self.ax.add_patch(circle)
            text = self.ax.text(start_x, start_y, f"{i}", ha='center', va='center')
            self.map_agents.append((start_x, start_y, circle, text))
            
            # Agent goal
            goal_x, goal_y = agent["goal"]
            goal_rect = Rectangle((goal_x - 0.25, goal_y - 0.25), 0.5, 0.5, 
                                  facecolor='none', edgecolor=color, linewidth=2, linestyle='--')
            self.ax.add_patch(goal_rect)
            self.ax.text(goal_x, goal_y, f"G{i}", ha='center', va='center', fontsize=8)
            self.map_goals.append((goal_x, goal_y, goal_rect))
        
        # Reset initial positions flag
        self.initial_positions_visible = True
        
        # Set up grid appearance
        self.ax.set_title("Click to add/remove obstacles, then click 'Run Planning'")
        self.ax.set_aspect('equal')

    def init_ui(self):
        # Position buttons at the bottom
        self.run_button_ax = plt.axes([0.35, 0.01, 0.2, 0.05])
        self.run_button = plt.Button(self.run_button_ax, 'Run Planning')
        self.run_button.on_clicked(self.on_run_button)
        
        # Add save button
        self.save_button_ax = plt.axes([0.6, 0.01, 0.15, 0.05]) 
        self.save_button = plt.Button(self.save_button_ax, 'Save Map')
        self.save_button.on_clicked(self.on_save_button)
        
        # Add clear button
        self.clear_button_ax = plt.axes([0.15, 0.01, 0.15, 0.05])
        self.clear_button = plt.Button(self.clear_button_ax, 'Clear Animation')
        self.clear_button.on_clicked(self.on_clear_button)

    def hide_initial_positions(self):
        """Hide the initial static agent positions"""
        if self.initial_positions_visible:
            for _, _, circle, text in self.map_agents:
                circle.set_visible(False)
                text.set_visible(False)
            self.initial_positions_visible = False
            self.fig.canvas.draw_idle()

    def show_initial_positions(self):
        """Show the initial static agent positions"""
        if not self.initial_positions_visible:
            for _, _, circle, text in self.map_agents:
                circle.set_visible(True)
                text.set_visible(True)
            self.initial_positions_visible = True
            self.fig.canvas.draw_idle()

    def on_click(self, event):
        # Check if click was in the main axis
        if event.inaxes == self.ax:
            # Get grid coordinates from click (ensure proper rounding)
            x, y = int(round(event.xdata)), int(round(event.ydata))
            
            # Check if this position is valid (not already an agent or goal)
            for agent_x, agent_y, _, _ in self.map_agents:
                if x == agent_x and y == agent_y:
                    print(f"Cannot place obstacle at agent position ({x}, {y})")
                    return
                    
            for goal_x, goal_y, _ in self.map_goals:
                if x == goal_x and y == goal_y:
                    print(f"Cannot place obstacle at goal position ({x}, {y})")
                    return
                    
            # Check if clicked on an existing obstacle
            for i, (obs_x, obs_y, rect) in enumerate(self.map_obstacles):
                if x == obs_x and y == obs_y:
                    # Remove obstacle
                    rect.remove()
                    self.map_obstacles.pop(i)
                    self.map_data["map"]["obstacles"].remove([obs_x, obs_y])
                    print(f"Removed obstacle at ({x}, {y})")
                    self.fig.canvas.draw_idle()
                    
                    # Set flag to indicate map was modified
                    self.map_modified = True
                    
                    # If animation is running, update planning
                    if self.animation_running and self.current_agent_positions:
                        self.trigger_replanning()
                    return
            
            # Add new obstacle
            rect = Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='black')
            self.ax.add_patch(rect)
            self.map_obstacles.append((x, y, rect))
            self.map_data["map"]["obstacles"].append([x, y])
            print(f"Added obstacle at ({x}, {y})")
            self.fig.canvas.draw_idle()
            
            # Set flag to indicate map was modified
            self.map_modified = True
            
            # If animation is running, update planning
            if self.animation_running and self.current_agent_positions:
                self.trigger_replanning()

    def trigger_replanning(self):
        """Trigger replanning based on current agent positions and updated map"""
        print("Map modified - triggering replanning")
        
        # Stop current animation
        self.animation_running = False
        if self.animation_thread and self.animation_thread.is_alive():
            self.animation_thread.join()
        
        # Update map with current agent positions
        if self.update_map_with_current_positions():
            # Save updated map
            self.save_temp_map()
            # Run CBS planning again
            self.run_cbs()
        else:
            print("Could not update agent positions for replanning")
            
    def update_map_with_current_positions(self):
        """Update map with current agent positions for replanning"""
        if not self.current_agent_positions:
            return False
            
        # Create a copy of the original map data
        updated_map = copy.deepcopy(self.map_data)
        
        # Update start positions to current positions
        for i, agent in enumerate(updated_map["agents"]):
            if i < len(self.current_agent_positions):
                x, y = self.current_agent_positions[i]
                agent["start"] = [int(round(x)), int(round(y))]
        
        # Save as the new map data
        self.map_data = updated_map
        return True

    def reset_agent_positions(self):
        """Reset agent start positions to their original values"""
        # Make a fresh copy preserving obstacles but resetting agents
        new_map = copy.deepcopy(self.map_data)
        
        # Reset agent start positions to original values
        for i, agent in enumerate(new_map["agents"]):
            if i < len(self.original_map_data["agents"]):
                agent["start"] = copy.deepcopy(self.original_map_data["agents"][i]["start"])
        
        # Update map data
        self.map_data = new_map
        print("Reset agent positions to original start locations")
        return True

    def on_run_button(self, event):
        print("Running planning with current obstacles...")
        # Save temp map file
        self.save_temp_map()
        # Run CBS planning
        self.run_cbs()
    
    def on_clear_button(self, event):
        # Stop any running animation
        self.animation_running = False
        if self.animation_thread and self.animation_thread.is_alive():
            self.animation_thread.join()
        
        # Remove agent circles
        for agent_circle, label in self.agent_circles:
            agent_circle.remove()
            label.remove()
        
        self.agent_circles = []
        self.current_agent_positions = None
        
        # Reset agent positions to original start locations
        self.reset_agent_positions()
        
        # Show initial positions again
        self.show_initial_positions()
        
        self.fig.canvas.draw_idle()
        
    def on_save_button(self, event):
        """Save the current map to a file"""
        from tkinter import Tk, filedialog
        
        # Create Tkinter root window and hide it
        root = Tk()
        root.withdraw()
        
        # Ask user for file location
        save_path = filedialog.asksaveasfilename(
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")],
            title="Save Map As"
        )
        
        # If user didn't cancel
        if save_path:
            with open(save_path, 'w') as f:
                yaml.dump(self.map_data, f)
            print(f"Map saved to {save_path}")
            
        root.destroy()

    def save_temp_map(self):
        # Create a temporary map file
        if self.temp_map_file:
            try:
                os.remove(self.temp_map_file)
            except:
                pass
        
        self.temp_map_file = os.path.join(tempfile.gettempdir(), "temp_cbs_map.yaml")
        print(f"Saving temporary map to {self.temp_map_file}")
        
        with open(self.temp_map_file, 'w') as f:
            yaml.dump(self.map_data, f)
            
        # Create a temporary output file
        if self.temp_output_file:
            try:
                os.remove(self.temp_output_file)
            except:
                pass
                
        self.temp_output_file = os.path.join(tempfile.gettempdir(), "temp_cbs_output.yaml")

    def run_cbs(self):
        # Run CBS planner
        print("Running CBS planner...")
        self.run_button.label.set_text("Planning...")
        self.fig.canvas.draw_idle()
        
        # Reset flag
        self.map_modified = False
        
        # Build command
        cmd = [
            self.cbs_executable,
            "-i", self.temp_map_file,
            "-o", self.temp_output_file
        ]
        
        # Run command
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate()
            
            if process.returncode != 0:
                print(f"CBS planning failed: {stderr.decode()}")
                self.solution_found = False
                self.run_button.label.set_text("Planning Failed")
                self.fig.canvas.draw_idle()
                return
            
            print(f"CBS planning finished: {stdout.decode()}")
            
            # Load the solution
            try:
                with open(self.temp_output_file, 'r') as f:
                    self.current_schedule = yaml.safe_load(f)
                self.solution_found = True
                
                # Visualize solution
                self.visualize_solution()
                
                self.run_button.label.set_text("Run Planning")
                self.fig.canvas.draw_idle()
            except Exception as e:
                print(f"Error loading solution: {e}")
                self.solution_found = False
                self.run_button.label.set_text("Solution Error")
                self.fig.canvas.draw_idle()
        except Exception as e:
            print(f"Error running CBS planner: {e}")
            self.solution_found = False
            self.run_button.label.set_text("Execution Error")
            self.fig.canvas.draw_idle()

    def visualize_solution(self):
        # Clear previous animation
        self.animation_running = False
        if self.animation_thread and self.animation_thread.is_alive():
            self.animation_thread.join()
        
        # Remove existing agent circles
        for agent_circle, label in self.agent_circles:
            agent_circle.remove()
            label.remove()
        
        self.agent_circles = []
        
        if not self.solution_found or not self.current_schedule:
            self.ax.set_title("No valid solution found")
            self.fig.canvas.draw_idle()
            return
        
        # Draw agent paths
        for i, agent in enumerate(self.map_data["agents"]):
            name = agent["name"]
            color = f"C{i}"
            
            # Draw path
            path = self.current_schedule["schedule"][name]
            x_points = [step["x"] for step in path]
            y_points = [step["y"] for step in path]
            
            # Draw path line
            self.ax.plot(x_points, y_points, 'o-', color=color, alpha=0.5, markersize=3)
            
        # Start a new animation thread
        self.animation_running = True
        self.animation_thread = threading.Thread(target=self.animate_solution)
        self.animation_thread.daemon = True
        self.animation_thread.start()
        
        # Show makespan and cost
        makespan = self.current_schedule["statistics"]["makespan"]
        cost = self.current_schedule["statistics"]["cost"]
        self.ax.set_title(f"Solution: Makespan={makespan}, Cost={cost}")
        self.fig.canvas.draw_idle()

    def animate_solution(self):
        if not self.solution_found or not self.current_schedule:
            return
            
        # Find maximum timestep (makespan)
        max_t = self.current_schedule["statistics"]["makespan"]
        
        # Hide initial agent positions when animation starts
        self.hide_initial_positions()
        
        # Create agent circles
        self.agent_circles = []
        self.current_agent_positions = []
        
        for i, agent in enumerate(self.map_data["agents"]):
            name = agent["name"]
            color = f"C{i}"
            
            # Initial position
            start_pos = self.current_schedule["schedule"][name][0]
            circle = Circle((start_pos["x"], start_pos["y"]), 0.3, facecolor=color, edgecolor='black')
            self.ax.add_patch(circle)
            label = self.ax.text(start_pos["x"], start_pos["y"], str(i), ha='center', va='center')
            
            self.agent_circles.append((circle, label))
            self.current_agent_positions.append((start_pos["x"], start_pos["y"]))
            
        self.fig.canvas.draw_idle()
        
        # Get frame interval
        frame_interval = 0.1  # seconds
        
        # For each timestep
        for t in range(int(max_t * 10)):
            if not self.animation_running:
                return
                
            current_t = t / 10.0
            
            # Update agent positions
            for i, ((circle, label), agent) in enumerate(zip(self.agent_circles, self.map_data["agents"])):
                name = agent["name"]
                path = self.current_schedule["schedule"][name]
                
                # Find correct position at time t through interpolation
                pos = self.interpolate_position(current_t, path)
                circle.center = pos
                label.set_position(pos)
                
                # Update current position for this agent
                if i < len(self.current_agent_positions):
                    self.current_agent_positions[i] = (pos[0], pos[1])
            
            # Check if map was modified and we need to replan
            if self.map_modified:
                print(f"Map modified during animation at time {current_t}")
                self.map_modified = False
                self.trigger_replanning()
                return
            
            self.fig.canvas.draw_idle()
            time.sleep(frame_interval)
        
        self.animation_running = False

    def interpolate_position(self, t, path):
        # Find two path points between which time t falls, then interpolate
        idx = 0
        while idx < len(path) and path[idx]["t"] < t:
            idx += 1

        if idx == 0:
            return np.array([path[0]["x"], path[0]["y"]])
        elif idx < len(path):
            p1 = np.array([path[idx - 1]["x"], path[idx - 1]["y"]])
            p2 = np.array([path[idx]["x"], path[idx]["y"]])
            dt = path[idx]["t"] - path[idx - 1]["t"]
            alpha = (t - path[idx - 1]["t"]) / dt
            return (1 - alpha) * p1 + alpha * p2
        else:
            return np.array([path[-1]["x"], path[-1]["y"]])

    def show(self):
        # Avoid using tight_layout which caused warnings
        plt.subplots_adjust(bottom=0.1)
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interactive CBS Path Planning")
    parser.add_argument("map", help="YAML file with map and agents")
    args = parser.parse_args()
    
    app = InteractiveCBS(args.map)
    app.show() 