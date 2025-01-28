import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib import animation
import json
import random

class Animation:
    def __init__(self, map_json, solution_json):
        self.map = map_json
        self.solution = solution_json

        # Prepare figure
        aspect = self.map["dimX"] / self.map["dimY"]
        self.fig = plt.figure(frameon=False, figsize=(4*aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

        self.patches = []
        self.agents = {}
        self.agent_colors = {}

        # Set up the grid boundary
        xmin, ymin = -0.5, -0.5
        xmax, ymax = self.map["dimX"] - 0.5, self.map["dimY"] - 0.5
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)

        # Border rectangle for the entire grid
        self.patches.append(
            Rectangle((xmin, ymin),
                      xmax - xmin,
                      ymax - ymin,
                      facecolor="none",
                      edgecolor='black')
        )

        # Draw obstacles
        for (ox, oy) in self.map["obstacles"]:
            rect = Rectangle((ox - 0.5, oy - 0.5),
                             1, 1,
                             facecolor='red',
                             edgecolor='black')
            self.patches.append(rect)

        # Random color per agent
        num_agents = len(self.map["starts"])
        for agent_id in range(num_agents):
            color = (random.random(), random.random(), random.random())
            self.agent_colors[agent_id] = color

        # Draw goals (match each goal color to its agent)
        for i, g in enumerate(self.map["goals"]):
            gx, gy = g
            goal_rect = Rectangle((gx - 0.4, gy - 0.4),
                                  0.8, 0.8,
                                  facecolor=self.agent_colors[i],
                                  edgecolor='black')
            self.patches.append(goal_rect)

        # Place agents at their start positions
        self.max_path_length = 0
        for i, start_pos in enumerate(self.map["starts"]):
            sx, sy = start_pos
            circle = Circle((sx, sy),
                            0.3,
                            facecolor=self.agent_colors[i],
                            edgecolor='black')
            self.agents[i] = circle
            self.patches.append(circle)

            path_len = len(self.solution['solutions'][i]['path'])
            self.max_path_length = max(self.max_path_length, path_len)

        # We want sub-frames for smooth dragging:
        self.frames_per_step = 20  # how many frames to "move" one cell
        # If max_path_length = L, we have (L-1) moves. Each move gets frames_per_step.
        # Add +1 to handle the final position.
        self.total_frames = (self.max_path_length - 1) * self.frames_per_step + self.frames_per_step

        # Create animation
        self.animation = animation.FuncAnimation(
            self.fig,
            self.animate_func,
            init_func=self.init_func,
            frames=self.total_frames,
            interval=100,   # ms between frames; increase if you want even slower motion
            blit=True
        )

    def init_func(self):
        # Add all patches to the axes
        for patch in self.patches:
            self.ax.add_patch(patch)
        return self.patches

    def animate_func(self, frame):
        """
        frame goes from 0 to total_frames-1.
        We figure out which segment of the path we're in (t),
        how far along that segment we are (substep), and interpolate.
        """
        t = frame // self.frames_per_step       # which segment (integer)
        substep = frame % self.frames_per_step  # sub‐frame within that segment
        alpha = substep / float(self.frames_per_step)  # fraction [0..1]

        for agent_id, circle in self.agents.items():
            path = self.solution['solutions'][agent_id]['path']
            # If there's only 1 coordinate, do nothing
            if len(path) < 2:
                # Stays at the start forever
                circle.center = path[0]
                continue

            # If we've exceeded the path length, just stay at the last position
            if t >= len(path) - 1:
                circle.center = path[-1]
            else:
                # Interpolate from path[t] to path[t+1]
                (x1, y1) = path[t]
                (x2, y2) = path[t+1]
                x_pos = x1 + alpha * (x2 - x1)
                y_pos = y1 + alpha * (y2 - y1)
                circle.center = (x_pos, y_pos)

        return self.patches

    def show(self):
        plt.show()


if __name__ == "__main__":
    # EXAMPLE: Even if a path has only 2 steps, it will be dragged across sub-frames.
    # Use your actual JSON files if you have them.
    #
    # Here is a minimal example of a 'small' path for each of two agents:

    with open("test/sampleSchedule.json") as solution_json_file:
        solution_json = json.load(solution_json_file)
    with open("test/sampleMap.json") as map_json_file:
        map_json = json.load(map_json_file)

    anim = Animation(map_json, solution_json)
    anim.show()