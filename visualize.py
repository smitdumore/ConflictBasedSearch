import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib import animation
import numpy as np
import argparse


class Animation:
    def __init__(self, map_data, schedule):
        self.map = map_data
        self.schedule = schedule

        self.colors = ['orange']  # List of agent colors (expand as needed)
        self.agents = {}          # Stores Circle patches for agents
        self.agent_labels = {}    # Stores text labels for agents
        self.patches = []         # Stores all shape patches to be drawn
        self.artists = []         # Stores additional drawable objects like text

        # Get aspect ratio for correct figure sizing
        aspect = map_data["map"]["dimensions"][0] / map_data["map"]["dimensions"][1]
        self.fig, self.ax = plt.subplots(figsize=(4 * aspect, 4))
        self.ax.set_aspect('equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

        self.setup_map()              # Set up background and obstacles
        self.T = self.setup_agents()  # Draw agents and get animation duration

        # Create animation function
        self.anim = animation.FuncAnimation(
            self.fig,
            self.animate_func,
            init_func=self.init_func,
            frames=int(self.T + 1) * 10,
            interval=100,
            blit=True
        )

    def setup_map(self):
        # Set plot limits based on map size
        width, height = self.map["map"]["dimensions"]
        self.ax.set_xlim(-0.5, width - 0.5)
        self.ax.set_ylim(-0.5, height - 0.5)

        # Draw red border around the map
        self.patches.append(Rectangle((-0.5, -0.5), width, height, facecolor='none', edgecolor='red'))

        # Draw each obstacle as a red square
        for x, y in self.map["map"]["obstacles"]:
            self.patches.append(Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red'))

    def setup_agents(self):
        max_t = 0
        for i, agent in enumerate(self.map["agents"]):
            color = self.colors[i % len(self.colors)]

            # Draw agent goals or potential goals
            goals = agent.get("potentialGoals", [agent["goal"]])
            for goal in goals:
                self.patches.append(Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5,
                                              facecolor=color, edgecolor='black', alpha=0.5))

        for i, agent in enumerate(self.map["agents"]):
            name = agent["name"]
            color = self.colors[i % len(self.colors)]

            # Draw agent's start position
            circle = Circle(agent["start"], 0.3, facecolor=color, edgecolor='black')
            circle.original_face_color = color
            self.agents[name] = circle
            self.patches.append(circle)

            # Add text label over the agent
            label = self.ax.text(*agent["start"], name.replace("agent", ""), ha='center', va='center')
            self.agent_labels[name] = label
            self.artists.append(label)

            # Track latest timestamp to set animation duration
            max_t = max(max_t, self.schedule["schedule"][name][-1]["t"])

        return max_t

    def init_func(self):
        # Add static elements to the plot
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, frame_idx):
        # Animate each frame at time t
        t = frame_idx / 10.0

        for name, path in self.schedule["schedule"].items():
            pos = self.interpolate_position(t, path)
            self.agents[name].center = pos
            self.agent_labels[name].set_position(pos)
            self.agents[name].set_facecolor(self.agents[name].original_face_color)

        # Check for collisions between agents
        agents_list = list(self.agents.values())
        for i in range(len(agents_list)):
            for j in range(i + 1, len(agents_list)):
                if np.linalg.norm(np.array(agents_list[i].center) - np.array(agents_list[j].center)) < 0.7:
                    agents_list[i].set_facecolor('red')
                    agents_list[j].set_facecolor('red')
                    print(f"COLLISION! (agent-agent) ({i}, {j})")

        return self.patches + self.artists

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

    def save(self, filename, speed):
        # Save animation as video file
        self.anim.save(filename, writer="ffmpeg", fps=10 * speed, dpi=200)

    def show(self):
        # Show animation window
        plt.show()


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="YAML file with map and agents")
    parser.add_argument("schedule", help="YAML file with agents' schedule")
    parser.add_argument("--video", help="Output video file (leave empty to show instead)")
    parser.add_argument("--speed", type=int, default=1, help="Speed-up factor")
    args = parser.parse_args()

    # Load map and schedule data
    with open(args.map) as f:
        map_data = yaml.safe_load(f)
    with open(args.schedule) as f:
        schedule = yaml.safe_load(f)

    # Run animation
    anim = Animation(map_data, schedule)
    if args.video:
        anim.save(args.video, args.speed)
    else:
        anim.show()
