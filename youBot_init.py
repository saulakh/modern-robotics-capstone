import numpy as np

class youBot():
    def __init__(self):
        self.dt = 0.01 # seconds
        self.max_speed = 15 # rad/s
        self.errorIntegral = np.zeros(6)

        # Chassis dimensions (meters)
        self.r = 0.0475 # wheel radius
        self.l = 0.235 # forward-backward distance between wheels
        self.w = 0.15 # side-to-side distance between wheels

        # Initial youBot configuration
        self.initial_config = np.array([np.pi/6,-0.1,0.1,0,-0.2,0.2,-1.6,0,0,0,0,0,0]) # (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4,gripper)
        self.Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
        self.M0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])
        self.Blist = np.array([[0,0,1,0,0.033,0],[0,-1,0,-0.5076,0,0],[0,-1,0,-0.3526,0,0],[0,-1,0,-0.2176,0,0],[0,0,1,0,0,0]]).T
        self.thetalist = self.initial_config[3:8].T

        # Starting configuration and controls
        self.current_config = self.initial_config
        self.controls = np.zeros(13)

        # Initial and goal configurations of the cube
        self.cube_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
        self.cube_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])
        # Cube positions for new task
        self.cube_initial2 = np.array([[1,0,0,1],[0,1,0,1],[0,0,1,0.025],[0,0,0,1]])
        self.cube_final2 = np.array([[0,1,0,1],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])

        # Kp and Ki gains
        self.Kp_gain = 5
        self.Ki_gain = 0
        self.Kp = self.Kp_gain * np.identity(6)
        self.Ki = self.Ki_gain * np.identity(6)

        # Initial X, Xd, Xd_next:
        self.X = np.array([[0.17,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.17,0.57],[0,0,0,1]])
        self.Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
        self.Xd_next = np.array([[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]])
        
        pass