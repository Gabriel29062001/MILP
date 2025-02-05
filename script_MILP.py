from MILP_function import run_simulation
from coord_transform_functions import mapping
import gurobipy as gp
from gurobipy import GRB
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import scipy.io

# Parameters
robots = {
    "robot1": {"start": [0, 2], "end": [12, 2]},
    "robot2": {"start": [0, 0], "end": [12, 0]},
    #"robot3": {"start": [0, 1], "end": [16, 3]},
   
}

obstacles = {
    "obstacle1": {"A": np.array([[1, 0], [0, -1], [0, 1], [-1, 0]]), "b": [2, -3, -1.5, -4], "polytope" :[[2, -1], [2, 3], [4, 3], [4, -1]]},
    #"obstacle2": {"A": np.array([[1, 0], [0, -1], [0, 1], [-1, 0]]), "b": [8, -5, 1.5, -10], "polytope" :[[8, 1.5], [8, 5], [10, 5], [10, 1.5]]},
    #"obstacle3": {"A": np.array([[1, 0], [0, -1], [0, 1], [-1, 0]]), "b": [8, -0.5, -3, -10], "polytope" :[[8, -3], [8, 0.5], [10, 0.5], [10, -3]]},
    #"obstacle4": {"A": np.array([[1, 0], [0, -1], [0, 1], [-1, 0]]), "b": [8, -0.5, -3, -10], "polytope" :[[8, -3], [8, 0.5], [10, 0.5], [10, -3]]},
   
}

output_directory_simulation_data = "/home/gabriel/Documents/epfl/MA3/Semester_project/MILP/simulations/data/"
output_directory_simulation_map = "/home/gabriel/Documents/epfl/MA3/Semester_project/MILP/simulations/map/"


total_time_simulation = []

for i in range(1, len(robots) + 1):
    robots_config = {key: robots[key] for key in list(robots.keys())[:i]}  # Subset of robots
    for j in range(1, len(obstacles) + 1):
        obstacles_config = {key: obstacles[key] for key in list(obstacles.keys())[:j]}  # Subset of obstacles
        
        # Generate file names
        data_file = output_directory_simulation_data + f"{i}_robots_{j}_obstacles.mat"
        map_file = output_directory_simulation_map + f"{i}_robots_{j}_obstacles.png"
        positions_file = output_directory_simulation_map + f"{i}_robots_{j}_obstacles.mat"
        
        # Run simulation and mapping
        time_simulation = run_simulation(robots_config, obstacles_config, data_file)
        mapping(data_file, map_file, positions_file)
        total_time_simulation.append(time_simulation)

#

time_file = output_directory_simulation_data + "time.mat"

scipy.io.savemat(time_file, {"total_time_simulation": total_time_simulation})