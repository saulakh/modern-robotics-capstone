import modern_robotics as mr
import numpy as np
import csv

def trajectory(X_start, X_end, grip, Tf, writer):
    """
    Inputs:
    - X_start: Starting end-effector configuration
    - X_end: Goal end-effector configuration
    - grip: Gripper state for this section of trajectory
    - Tf: total time of motion for this section (in seconds)
    - writer: add each step of the trajectory to this csv writer
    Output:
    - Returns trajectory and appends values to csv file
    """
    dt = 0.01
    N = Tf/dt # number of iterations between start and end
    traj = mr.ScrewTrajectory(X_start, X_end, Tf, N, 3)
    
    # Extract rotation and position values from traj
    for i in range(len(traj)):
        rot = traj[i][:-1,0:-1].flatten()
        pos = traj[i][:-1,-1].flatten()
        state = np.concatenate((rot,pos))
        # Write to csv file
        writer.writerow(np.append(state,grip))
    pass

def grasp_position(cube_config, theta):
    """
    Returns end-effector grasp configuration, rotated about y-axis from cube position
    """
    grasp = np.identity(4)
    # Rotate theta (in radians) about y-axis in space frame
    rot_y = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    # Change end-effector orientation
    grasp[:3,:3] = rot_y
    # Lower center of gripper to center height of cube
    grasp[2,3] -= 0.0215

    return cube_config @ grasp

def standoff_position(cube_config, theta, dist):
    """
    Returns the end-effector standoff configuration, relative to cube position
    """
    standoff = np.identity(4)
    rot_y = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    # Change end-effector orientation
    standoff[:3,:3] = rot_y
    # Standoff at desired distance above cube (in meters)
    standoff[2,3] += dist

    return cube_config @ standoff

def chassis_to_endeffector(T_sb):
    """
    Input:
    - Transformation matrix from space frame to chassis frame
    Output:
    - End effector configuration in the space frame, when all joint angles are zero
    """
    # Chassis frame to base frame of arm 0
    T_b0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
    # End effector frame relative to base frame
    M_0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
    # Initial configuration of end-effector
    Tse_ini = T_sb @ T_b0 @ M_0e

    return Tse_ini

def space_to_chassis(phi,x,y):
    """
    Transformation matrix from space frame to chassis frame
    """
    T_sb = np.array([[np.cos(phi),-np.sin(phi),0,x],[np.sin(phi),np.cos(phi),0,y],[0,0,1,0.0963],[0,0,0,1]])
    
    return T_sb

def motion_planning(robot_initial, cube_initial, cube_final):
    """
    Inputs:
    - robot_initial: End-effector configuration at robot's initial position
    - cube_initial: Cube configuration at its initial position
    - cube_final: Cube configuration at its final position
    Ouput:
    - Appends full trajectory to csv file, moving the cube from inital to goal position
    """
    # Open csv file
    f = open('./results/trajectory.csv', 'w', newline='')
    writer = csv.writer(f)

    # 1) Move gripper to standoff configuration over initial cube location
    standoff1 = standoff_position(cube_initial, 3*np.pi/4, 0.075)
    grip, Tf = 0, 5
    trajectory(robot_initial, standoff1, grip, Tf, writer)

    # 2) Move gripper down to grasp position
    grasp1 = grasp_position(cube_initial, 3*np.pi/4)
    trajectory(standoff1, grasp1, grip, 1, writer)

    # 3) Close gripper
    grip = 1
    trajectory(grasp1, grasp1, grip, 1, writer)

    # 4) Move gripper back up to initial standoff configuration
    trajectory(grasp1, standoff1, grip, 1, writer)

    # 5) Move gripper to standoff configuration over goal cube location
    standoff2 = standoff_position(cube_final, 3*np.pi/4, 0.075)
    trajectory(standoff1, standoff2, grip, Tf, writer)

    # 6) Move gripper to final configuration of the cube
    grasp2 = grasp_position(cube_final, 3*np.pi/4)
    trajectory(standoff2, grasp2, grip, 1, writer)

    # 7) Open gripper
    grip = 0
    trajectory(grasp2, grasp2, grip, 1, writer)

    # 8) Move gripper back to final standoff configuration
    trajectory(grasp2, standoff2, grip, 1, writer)

    f.close()
    pass

def main():
    # Initial chassis position
    T_sb = space_to_chassis(0,0,0)
    # Initial end effector position
    Tse_ini = chassis_to_endeffector(T_sb)

    # Initial and goal configurations of the cube
    Tsc_ini = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
    Tsc_goal = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])

    # Generate full trajectory
    motion_planning(Tse_ini, Tsc_ini, Tsc_goal)
    
    pass

if __name__ == "__main__":
    main()