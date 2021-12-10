import modern_robotics as mr
import numpy as np

def Jacobian(T0e, F, Blist, thetalist):
    """
    Inputs:
    - Tbe: transformation matrix from base to end-effector
    - F: psuedoinverse of chassis kinematic model H(0)
    - Blist: screw axes when arm is at home configuration
    - thetalist: list of 5 joint angles for arm configuration
    Output:
    - J_e: Full Jacobian matrix
    """
    Te0 = np.linalg.inv(T0e)
    Adj_Teb = mr.Adjoint(Te0)
    # Build F6 matrix
    m = F.shape[1]
    F6 = np.zeros((6,m))
    F6[2:5,:] = F
    # Get full Jacobian matrix
    J_base = Adj_Teb @ F6
    J_arm = mr.JacobianBody(Blist,thetalist)
    J_e = np.concatenate((J_base, J_arm), axis=1)
    #print("\nFull jacobian:\n", np.around(J_e,3))
    # Switch to using J_arm then J_base, to match order of controls and config
    J = np.concatenate((J_arm, J_base), axis=1)
    return J

def testJointLimit():
    pass

def feedbackControl(X, Xd, Xd_next, Kp, Ki, dt, currentConfig, errorIntegral):
    """
    Inputs:
    - X: current actual end-effector configuration (Tse)
    - Xd: current end-effector reference configuration Xd (Tse_d)
    - Xd_next: reference configuration at next timestep in ref trajectory
    - Kp, Ki: PI gain matrices
    - dt: timestep between ref trajectory configurations
    - currentConfig: (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4)
    - errorIntegral: sum of all error twists over time
    Outputs:
    - V: commanded end-effector twist, expressed in end-effector frame
    """
    # Chassis dimensions (meters)
    r = 0.0475 # wheel radius
    l = 0.235 # forward-backward distance between wheels
    w = 0.15 # side-to-side distance between wheels
    # Get F matrix
    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],[1,1,1,1],[-1,1,-1,1]])

    M0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
    Blist = np.array([[0,0,1,0,0.033,0],[0,-1,0,-0.5076,0,0],[0,-1,0,-0.3526,0,0],[0,-1,0,-0.2176,0,0],[0,0,1,0,0,0]]).T
    thetalist = currentConfig[3:8].T
    # Get end-effector configuration relative to base frame (for test input angles)
    T0e = mr.FKinBody(M0e, Blist, thetalist)
    J_e = Jacobian(T0e, F, Blist, thetalist)

    X_inv = np.linalg.inv(X)
    Xd_inv = np.linalg.inv(Xd)
    # Error twist between current and desired state
    X_err = mr.se3ToVec(mr.MatrixLog6(X_inv @ Xd))
    print("\nError twist:", np.around(X_err,3))
    # Error integral is sum of all X_err*dt over time
    errorIntegral = sum(errorIntegral, X_err * dt)
    # Feedforward reference twist
    Vd = mr.se3ToVec((1/dt) * mr.MatrixLog6(Xd_inv @ Xd_next))
    print("\nVd:", Vd)
    Adj_Vd = mr.Adjoint(X_inv @ Xd) @ Vd
    print("\nAdj_Vd:", Adj_Vd)

    # Get commanded end-effector twist V
    V = Adj_Vd + Kp @ X_err + Ki @ errorIntegral
    print("\nV:", V)
    # Note: these controls are (u, thetadot), but nextState uses (thetadot, u)
    #controls = np.linalg.pinv(J_e) @ V
    #print("\nControls:", np.around(controls,3))

    # After flipping the Jacobian to [J_arm J_base]:
    controls = np.linalg.pinv(J_e) @ V
    print("\nControls:", np.around(controls,3))
    return controls, errorIntegral

def main():
    # Test input (phi,x,y,J1,J2,J3,J4,J5)
    robotConfig = np.array([0,0,0,0,0,0.2,-1.6,0])
    errorIntegral = np.zeros((1,6))
    dt = 0.01

    # Given Xd, Xd_next, X, Kp, Ki:
    Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
    Xd_next = np.array([[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]])
    X = np.array([[0.17,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.17,0.57],[0,0,0,1]])
    Kp = np.zeros(6)
    Ki = np.zeros(6)

    # Test feedback control
    output = feedbackControl(X, Xd, Xd_next, Kp, Ki, dt, robotConfig, errorIntegral)
    pass

if __name__ == "__main__":
    main()