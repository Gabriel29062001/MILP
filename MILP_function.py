import gurobipy as gp
from gurobipy import GRB
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import scipy.io
import time

# Parameters
robots = {
    "robot1": {"start": [0, 2], "end": [12, 2]},
    "robot2": {"start": [0, 0], "end": [12, 0]},
    "robot3": {"start": [0, 1], "end": [16, 3]},
   
}

obstacles = {
    "obstacle1": {"A": np.array([[1, 0], [0, -1], [0, 1], [-1, 0]]), "b": [2, -3, -1.5, -4], "polytope" :[[2, -1], [2, 3], [4, 3], [4, -1]]},
    "obstacle2": {"A": np.array([[1, 0], [0, -1], [0, 1], [-1, 0]]), "b": [8, -5, 1.5, -10], "polytope" :[[8, 1.5], [8, 5], [10, 5], [10, 1.5]]},
    "obstacle3": {"A": np.array([[1, 0], [0, -1], [0, 1], [-1, 0]]), "b": [8, -0.5, -3, -10], "polytope" :[[8, -3], [8, 0.5], [10, 0.5], [10, -3]]},

   
}


def run_simulation(robots, obstacles, output_file):
    start_time = time.time()
    safe_d = 0.5
    M = 5e2  # Big-M constant
    w_ref = 0.05  # Weight for reference trajectory
    w_smooth = 0.3  # Weight for smoothness
    w_goal = 0.4  # Weight for goal reaching


    n = 35  # Number of timesteps
    dt = 0.1

    number_obstacles = len(obstacles)
    k = 0
    for obstacle in obstacles :
        k += len(obstacles[obstacle]["polytope"])

    # Gurobi model
    model = gp.Model("Collision Avoidance")

    # Variables for each robot
    robot_vars = {}
    for robot_name, data in robots.items():
        robot_vars[robot_name] = {
            "x": model.addVars(n, lb=-GRB.INFINITY, name=f"{robot_name}_x"),
            "y": model.addVars(n, lb=-GRB.INFINITY, name=f"{robot_name}_y"),
            "v": model.addVars(n, lb=-GRB.INFINITY, name=f"{robot_name}_v"),
            "omega": model.addVars(n, lb=-GRB.INFINITY, name=f"{robot_name}_omega"),
            "phi": model.addVars(n, lb=-GRB.INFINITY, name=f"{robot_name}_phi"),
            "z": model.addVars(k, n, vtype=GRB.BINARY, name=f"{robot_name}_z"),
        }


    # Add initial conditions and dynamics constraints for each robot
    for robot_name, data in robots.items():
        start = data["start"]
        end = data["end"]

        reference_trajectories = np.linspace(data["start"], data["end"], n)
        x_ref = reference_trajectories[:, 0]
        y_ref = reference_trajectories[:, 1]
        phi_ref = np.arctan2(np.diff(y_ref, append=y_ref[-1]), np.diff(x_ref, append=x_ref[-1]))

        v_ref = np.sqrt(np.diff(x_ref, append=x_ref[-1])**2 + np.diff(y_ref, append=y_ref[-1])**2) / (1 / n)
        omega_ref = np.diff(phi_ref, append=phi_ref[-1]) / (1 / n)

        x = robot_vars[robot_name]["x"]
        y = robot_vars[robot_name]["y"]
        z = robot_vars[robot_name]["z"]

        v = robot_vars[robot_name]["v"]
        omega = robot_vars[robot_name]["omega"]
        phi = robot_vars[robot_name]["phi"]

        # Initial conditions
        model.addConstr(x[0] == start[0])
        model.addConstr(y[0] == start[1])
        model.addConstr(phi[0] == phi_ref[0])
        model.addConstr(v[0] == v_ref[0])
        model.addConstr(omega[0] == omega_ref[0])


        # Dynamics constraints
        for t in range(n - 1):
            
            phi_k = phi_ref[t]
            v_k = v_ref[t]
            A_k = np.array([
                [1, 0, -dt * v_k * np.sin(phi_k)],
                [0, 1, dt * v_k * np.cos(phi_k)],
                [0, 0, 1]
            ])

            B_k = np.array([
                [dt * np.cos(phi_k), 0],
                [dt * np.sin(phi_k), 0],
                [0, dt]
            ])

            model.addConstr(x[t+1] == x_ref[t+1] + A_k[0, 0] * (x[t] - x_ref[t]) +
                        A_k[0, 1] * (y[t] - y_ref[t]) +
                        A_k[0, 2] * (phi[t]- phi_ref[t]) +
                        B_k[0, 0] * (v[t] - v_ref[t]) +
                        B_k[0, 1] * (omega[t] - omega_ref[t]))
            model.addConstr(y[t+1] == y_ref[t+1] + A_k[1, 0] * (x[t] - x_ref[t]) +
                        A_k[1, 1] * (y[t] - y_ref[t]) +
                        A_k[1, 2] * (phi[t]- phi_ref[t]) +
                        B_k[1, 0] * (v[t] - v_ref[t]) +
                        B_k[1, 1] * (omega[t] - omega_ref[t]))
            model.addConstr(phi[t+1] == phi_ref[t+1] + A_k[2, 0] * (x[t] - x_ref[t]) +
                        A_k[2, 1] * (y[t] - y_ref[t]) +
                        A_k[2, 2] * (phi[t]- phi_ref[t]) +
                        B_k[2, 0] * (v[t] - v_ref[t]) +
                        B_k[2, 1] * (omega[t] - omega_ref[t]))



        for t in range(n):
            total_faces = 0
            for obstacle in obstacles:
                print(len(obstacles))
                print(obstacle)
                faces = len(obstacles[obstacle]["polytope"])
                A = obstacles[obstacle]["A"]
                b = obstacles[obstacle]["b"]
                for i in range(faces):
                    print("i : ", i)
                    
                    model.addConstr(
                        A[i, 0] * x[t] + A[i, 1] * y[t] - b[i] <= -safe_d + M * z[total_faces+i, t]  
                    )

                model.addConstr(gp.quicksum(z[total_faces+i, t] for i in range(faces)) == faces - 1)
                total_faces += faces
                
    # Collision avoidance between robots
    robot_names = list(robot_vars.keys())
    for i in range(len(robot_names)):
        for j in range(i + 1, len(robot_names)):  # Avoid duplicate pairs
            robot1 = robot_vars[robot_names[i]]
            robot2 = robot_vars[robot_names[j]]
            for t in range(n):
                model.addQConstr(
                    (robot1["x"][t] - robot2["x"][t]) * (robot1["x"][t] - robot2["x"][t])
                    + (robot1["y"][t] - robot2["y"][t]) * (robot1["y"][t] - robot2["y"][t])
                    >= safe_d ** 2
                )

    # Objective: Minimize the sum of smoothness, reference tracking, and goal reaching costs
    objective = gp.quicksum(
        w_ref * ((robot_vars[robot_name]["x"][t] - np.linspace(data["start"][0], data["end"][0], n)[t]) ** 2 +
                (robot_vars[robot_name]["y"][t] - np.linspace(data["start"][1], data["end"][1], n)[t]) ** 2) +
        w_smooth * ((robot_vars[robot_name]["x"][t + 1] - robot_vars[robot_name]["x"][t]) ** 2 +
                    (robot_vars[robot_name]["y"][t + 1] - robot_vars[robot_name]["y"][t]) ** 2) +
        w_goal * ((robot_vars[robot_name]["x"][n - 1] - data["end"][0]) ** 2 +
                (robot_vars[robot_name]["y"][n - 1] - data["end"][1]) ** 2)
        for robot_name, data in robots.items() for t in range(n - 1)
    )

    model.setObjective(objective, GRB.MINIMIZE)



    model.Params.MIPGap = 0.25  # Allows a 20% relative gap

    # Optimize the model
    model.optimize()
    end_time = time.time()
    execution_time = end_time - start_time

    # Number of frames (time steps)
    n_frames = n


    # Set up the figure and axis
    fig, ax = plt.subplots()
    ax.set_xlim(0, 12)
    ax.set_ylim(-5, 6)


    # Plot obstacles (static)
    for obstacle in obstacles:
        polytopes_obstacles = obstacles[obstacle]["polytope"]
        number_polytopes = len(polytopes_obstacles)
        for i in range(number_polytopes):
            plt.scatter(polytopes_obstacles[i][0], polytopes_obstacles[i][1], color="blue")
            next_i = (i + 1) % number_polytopes
            plt.plot(
                [polytopes_obstacles[i][0], polytopes_obstacles[next_i][0]],
                [polytopes_obstacles[i][1], polytopes_obstacles[next_i][1]],
                color="blue",
            )

    # Prepare robot data
    lines = {}
    points = {}
    for robot_name, data in robots.items():
        x_values = [robot_vars[robot_name]["x"][t].X for t in range(n_frames)]
        y_values = [robot_vars[robot_name]["y"][t].X for t in range(n_frames)]
        traj = np.linspace(data["start"], data["end"], n_frames)
        
        # Plot reference trajectory
        plt.plot(traj[:, 0], traj[:, 1], "--", label=f"{robot_name} Reference Trajectory")
        
        # Initialize path and current position markers for animation
        line, = ax.plot([], [], "o-", label=f"{robot_name} Optimal Path")
        point, = ax.plot([], [], "o", label=f"{robot_name} Current Position")
        lines[robot_name] = (line, point)

    # Configure plot
    #plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Optimal Paths with Collision Avoidance")

    # Animation update function
    def update(frame):
        for robot_name, data in robots.items():
            x_values = [robot_vars[robot_name]["x"][t].X for t in range(n_frames)]
            y_values = [robot_vars[robot_name]["y"][t].X for t in range(n_frames)]

            
            # Update lines and points for each robot
            line, point = lines[robot_name]
            line.set_data(x_values[:frame], y_values[:frame])  # Path so far
            point.set_data(x_values[frame], y_values[frame])  # Current position
        
        return [line for line, point in lines.values()] + [point for line, point in lines.values()]

    # Create the animation

    ani = FuncAnimation(fig, update, frames=n_frames, blit=True, interval=200, repeat = False)


    # Display the animation
    plt.show()


    for robot_name, data in robots.items():
        print(x_values)
        print(y_values)

    scipy.io.savemat(output_file, {
        "robot_positions": {
            robot_name: [
                [robot_vars[robot_name]["x"][t].X, robot_vars[robot_name]["y"][t].X] for t in range(n)
            ]
            for robot_name in robots
        },
        "polytopes": {
            obstacle_name: obstacles[obstacle_name]["polytope"]
            for obstacle_name in obstacles
        },
        "references": {
            robot_name: [
                [x, y]
                for x, y in zip(
                    np.linspace(data["start"][0], data["end"][0], n),
                    np.linspace(data["start"][1], data["end"][1], n)
                )
            ]
            for robot_name, data in robots.items()
        },

        "time execution": execution_time
    })

    print(f"Temps d'exécution : {execution_time:.2f} secondes")
    return execution_time


#run_simulation(robots, obstacles, "datatest.mat")