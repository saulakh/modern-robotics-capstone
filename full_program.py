import modern_robotics as mr
import numpy as np
import csv
from youBot_init import youBot
from next_state import nextState
import trajectory_generation
from feedback_control import feedbackControl

def endEffectorConfig(config, M0e, Tb0, Blist):
    """
    Returns SE(3) transformation matrix from 13-vector of current configuration
    """
    phi,x,y = config[0:3]
    arm_config = config[3:8]
    Tsb = trajectory_generation.space_to_chassis(phi,x,y)
    T0e = mr.FKinBody(M0e, Blist, arm_config)
    Tse = Tsb @ Tb0 @ T0e
    return Tse

def trajToSE3(traj):
    """
    Returns SE(3) transformation matrix from flattened trajectory
    """
    rot = traj[0:9].reshape(3,3)
    pos = traj[9:12]

    SE3 = np.zeros((4,4))
    SE3[:3,:3] = rot
    SE3[:3,3] = pos
    SE3[3,:] = [0,0,0,1]

    return SE3

def main():
    """
    This function returns the full path of the youBot, with these helper functions:
    - nextState: takes current configuration and controls as input, returns next configuration
    - trajectory: takes initial and goal SE(3) transformation matrices, and returns full trajectory
    - feedbackControl: returns commanded end-effector twist and controls
    """
    robot = youBot()

    # Get initial end-effector SE(3) configuration from chassis phi,x,y and joint angles
    Tsb = trajectory_generation.space_to_chassis(0,0,0)
    T0e = mr.FKinBody(robot.M0e, robot.Blist, robot.thetalist)
    X_initial = Tsb @ robot.Tb0 @ T0e

    # Trajectory with SE(3) configurations
    trajectory_generation.motion_planning(X_initial, robot.cube_initial, robot.cube_final)

    # Write to csv file (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4,gripper)
    f = open('./results/youBot.csv', 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(robot.initial_config)

    # Start second writer for error twists
    f2 = open('./results/error.csv', 'w', newline='')
    writer2 = csv.writer(f2)
    writer2.writerow(robot.errorIntegral)

    # Load configurations from trajectory csv
    traj = np.loadtxt('./results/trajectory.csv',delimiter=',')
    # Loop through trajectory for X, Xd, and Xd_next
    for i in range(len(traj)-1):
        X = robot.X
        Xd = trajToSE3(traj[i])
        Xd_next = trajToSE3(traj[i+1])
    
        # Get controls needed for nextState
        controls, errorIntegral = feedbackControl(X, Xd, Xd_next, robot.Kp, robot.Ki, robot.dt, robot.current_config, robot.errorIntegral)
        robot.controls = controls
        robot.errorIntegral = errorIntegral

        # Get next configuration and add to csv file
        robot.current_config = np.append((nextState(robot.current_config, robot.controls, robot.dt, robot.max_speed)), 0)
        writer.writerow(np.around(robot.current_config, 8))
        writer2.writerow(np.around(robot.errorIntegral, 8))

        # Set Xd as X
        robot.X = trajToSE3(traj[i])

    f.close()
    f2.close()
    pass

if __name__ == "__main__":
    main()