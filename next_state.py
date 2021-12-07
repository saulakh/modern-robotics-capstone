import numpy as np
import csv

def eulerStep(angles,speeds,dt):
    """
    Inputs:
    - angles: includes 5 old joint arm angles, and 4 old wheel angles
    - speeds: includes 5 joint speeds and 4 wheel speeds
    - dt: timestep Δt
    Output:
    - angles: 5 new arm joint angles and 4 new wheel angles
    """
    new_angles = []
    for i in range(len(angles)):
        new_angles.append(angles[i] + speeds[i] * dt)

    return new_angles

def odometery(q_k, d_theta):
    """
    Inputs:
    - q_k: current chassis state (phi,x,y)
    - d_theta: change in wheel angles
    Output:
    - q_k1: new chassis state (phi,x,y)
    """
    
    # Chassis dimensions (meters)
    r = 0.0475 # wheel radius
    l = 0.235 # forward-backward distance between wheels
    w = 0.15 # side-to-side distance between wheels

    # H(0) matrix
    H = (1/r) * np.array([[-l-w,1,-1],[l+w,1,1],[l+w,1,-1],[-l-w,1,1]])
    # F is pseudo inverse of H(0)
    F = np.linalg.pinv(H)

    # Vb = H_inv * u
    Vb = F @ d_theta
    omega_bz = Vb[0]
    v_bx = Vb[1]
    v_by = Vb[2]

    # Set up delta_q matrix
    if omega_bz < 0.00001:
        delta_qb = Vb
    else:
        delta_xb = (v_bx*np.sin(omega_bz) + v_by*(np.cos(omega_bz) - 1)) / omega_bz
        delta_yb = (v_by*np.sin(omega_bz) + v_bx*(1 - np.cos(omega_bz))) / omega_bz
        delta_qb = np.array([omega_bz, delta_xb, delta_yb]).T

    phi = q_k[0]
    delta_q = np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]]) @ delta_qb

    # New chassis state
    q_k1 = q_k + delta_q

    return q_k1

def nextState(current, controls, dt, max_speed):
    """
    Inputs:
    - current: 12-vector for current robot configuration (phi,x,y for chassis, 5 joint angles, and 4 wheel angles)
    - controls: 9-vector of controls indicating the 5 joint speeds theta_dot and the 4 wheel speeds u
    - dt: timestep Δt
    - max_speed: limits the maximum angular speed of arm joints and wheels to [-max_speed rad/s, max_speed rad/s]
    Output:
    - config: 12-vector representing the configuration of the robot time Δt later
    """
    new_state = np.ones(12)

    # Extract values from current configuration
    phi,x,y = current[0:3]
    angles = current[3:12]

    # Limit joint and wheel speeds
    for i in range(len(controls)):
        if controls[i] < -max_speed:
            controls[i] = -max_speed
        elif controls[i] > max_speed:
            controls[i] = max_speed

    # Use odometry for new chassis configuration
    new_state[0:3] = odometery(current[0:3], dt*controls[5:9])
    # Use euler step for new joint and wheel angles
    new_state[3:12] = eulerStep(angles,controls,dt)
    
    return new_state

def main():
    # Test nextState function
    config = np.zeros(12)
    controls = np.array([0.5,0.5,0.5,0.5,0.5,10,10,10,10])
    dt = 0.01 # seconds
    max_speed = 10 # rad/s

    # Open csv file
    f = open('./results/path.csv', 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(config)

    # Run for 100 iterations
    for i in range(101):
        config = nextState(config,controls,dt,max_speed)
        writer.writerow(config)

    f.close()
    
    pass

if __name__ == "__main__":
    main()