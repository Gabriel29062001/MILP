import numpy as np
import cv2
import scipy.io
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt



def mapping(initial_map_file, output_map_file, output_positions_file):

    # Transformation setup
    nuscenes_points = np.array([[0, -5], [0, 6], [12, -5], [12, 6]], dtype=np.float32)
    mcs_points = np.array([[-3, -1.5], [-3, 1.5], [3, -1.5], [3, 1.5]], dtype=np.float32)

    # Compute the transformation matrix
    transformation_matrix, status = cv2.findHomography(nuscenes_points, mcs_points)

    # Save transformation matrix
    np.save('transformation_matrix_s556.npy', transformation_matrix)

    # Load transformation matrix
    loaded_transformation_matrix = np.load('transformation_matrix_s556.npy')

    # Load .mat file
    data = scipy.io.loadmat(initial_map_file)

    # Extract the structured array for robot positions
    structured_src_points = data["robot_positions"]
    obstacles_src_points = data["polytopes"]
    references_src_points = data["references"]

    # Extract the actual points (assuming they are in a tuple inside the array)
    src_points = structured_src_points[0][0]  # Unpack the array
    print(src_points)

    obstacles_points = obstacles_src_points[0][0]  # Unpack the array
    print(obstacles_points)

    references_src_points = data["references"][0][0]
    print("references",references_src_points)



    obstacles_transform = []

    for i in range(len(obstacles_points)):
        dst_obstacles = np.zeros((len(obstacles_points[i]), 2))
        print(obstacles_points[i])
        for j in range(len(obstacles_points[i])):
            x = obstacles_points[i][j][0]
            y = obstacles_points[i][j][1]

            src_obstacle_homogenous = np.array([x,y,1])
            dst_obstacle_homogenous =  np.dot(loaded_transformation_matrix, src_obstacle_homogenous)
            dst_obstacle= dst_obstacle_homogenous[:2] / dst_obstacle_homogenous[2]

            dst_obstacles[j, :] = dst_obstacle
            print(dst_obstacles)
        obstacles_transform.append(dst_obstacles)


    # Initialize an empty array for the destination points

    robots_points = []
    references_points = []
    # Loop through and apply the transformation
    for i in range(len(src_points)):
        dst_points = np.zeros((len(src_points[i]), 2))
        dst_ref_points = np.zeros((len(references_src_points[i]), 2))
        for j in range(len(src_points[i])):
    
        # Extract [x, y] coordinates
            x = src_points[i][j][0]
            y = src_points[i][j][1]
            #print(x)
            #print(y)
            # Add the third coordinate for homogeneous coordinates
            src_point_homogenous = np.array([x, y, 1])

            # Apply the transformation
            dst_point_homogenous = np.dot(loaded_transformation_matrix, src_point_homogenous)

            # Convert back from homogeneous coordinates to 2D
            dst_point = dst_point_homogenous[:2] / dst_point_homogenous[2]

            # Store the destination point
            dst_points[j, :] = dst_point


                    # Extract [x, y] coordinates
            x_ref =references_src_points[i][j][0]
            y_ref = references_src_points[i][j][1]
            #print(x)
            #print(y)
            # Add the third coordinate for homogeneous coordinates
            src_ref_point_homogenous = np.array([x_ref, y_ref, 1])

            # Apply the transformation
            dst_ref_point_homogenous = np.dot(loaded_transformation_matrix,src_ref_point_homogenous)

            # Convert back from homogeneous coordinates to 2D
            dst_ref_point = dst_ref_point_homogenous[:2] / dst_ref_point_homogenous[2]

            # Store the destination point
            dst_ref_points[j, :] = dst_ref_point


        print("Robot", i)
        print(dst_points)
        robots_points.append(dst_points)
        references_points.append(dst_ref_points)


    print(robots_points)
    print(references_points)




    #print("Obstacles Points:\n", robots_points)

    #print("Transformed Points:\n", obstacles_transform)

    plt.figure(figsize=(1920 / 100, 1080 / 100))  # Taille en pouces (dpi de 100 par défaut)

    colors = ["green","blue","cyan","magenta","yellow"]
    color_variable = 0

    for robot in robots_points:
        robot = np.array(robot)  # Assurez-vous que chaque robot est un tableau NumPy
        plt.scatter(robot[:, 0], robot[:, 1], color=colors[color_variable], marker = "o", linewidths=5, label='Robot Positions')
        color_variable = color_variable+1
    color_variable = 0

    for ref in references_points:
        ref = np.array(ref)  # Assurez-vous que chaque robot est un tableau NumPy
        plt.scatter(ref[:, 0], ref[:, 1], color=colors[color_variable], marker = "_", linewidths=5, label='Reference Trajectories')
        color_variable = color_variable+1

    # Plot transformed obstacles
    for transformed_polytope in obstacles_transform:
        transformed_polytope = np.array(transformed_polytope)
        plt.plot(np.append(transformed_polytope[:, 0], transformed_polytope[0, 0]),  # Close the polygon
                np.append(transformed_polytope[:, 1], transformed_polytope[0, 1]), 'r-', label='Obstacles')

                        
    plt.xlim(-3, 3)  # Limit x-axis to [-3, 3]
    plt.ylim(-1.5, 1.5)  # Limit y-axis to [-1.5, 1.5]
    # Add labels and legend
    plt.xlabel('Y Coordinate')
    plt.ylabel('X Coordinate')
    plt.title('Original and Transformed Points')

    plt.grid(True)
    plt.legend()

    # Save the plot
    plt.savefig(output_map_file, dpi = 100)

    print("Plot saved as ", output_map_file)

    scipy.io.savemat(output_positions_file, {
        "robot_positions": robots_points
    })

    