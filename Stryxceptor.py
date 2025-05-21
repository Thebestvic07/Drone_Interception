'''

Class to simulate the Interceptor's physical behavior and host the control logic.

'''

############       Libs       ############
import numpy as np
from numpy.linalg import norm


##############       Interceptor Class       ############
class Stryxceptor:
    def __init__(self, init_pose=np.zeros(3), init_velocity=np.zeros(3), init_acceleration=np.zeros(3), mass=1.0, dt=0.1):
        self.pose = np.array(init_pose, dtype=float)
        self.velocity = np.array(init_velocity, dtype=float)
        self.acceleration = np.array(init_acceleration, dtype=float)
        self.mass = mass
        self.target = None

        # Trajectories
        self.past_states = []
        self.future_states = []
        self.future_times = []
        self.time = 0.0
        self.intercepted = False

        # Constants
        self.max_speed = 83.0  # m/s  --> 300 km/h 
        self.max_accel = 13.0  # m/s² (derived offline using drag balance at max speed) 
        self.max_angle_delta = 3.5*dt   # 3.5 rad/s = 200°/s --> Good angular speed for a quadrotor 
        self.drag_coef = 0.002  # F_drag = -k * v * |v| // approx coeff for q Cx of 0.1 and apparent area of 0.03 m²

        # Commands
        self.command = [0,  # throttle in N/kg = m/s²
                        0,  # delta roll in rad
                        0]  # delta pitch in rad

    def update_target(self, target):
        self.target = target

    def get_drag(self):
        speed = norm(self.velocity)
        if speed > 0:
            drag_force = -self.drag_coef * speed * self.velocity
            return drag_force / self.mass
        return np.zeros(3)
       
    def get_weight(self):
        g = 9.81  # m/s²
        weight = np.array([0, 0, -self.mass * g])
        return weight / self.mass
    
    def get_rotation(self):
        roll = self.command[1] 
        pitch = self.command[2]

        # Rotation matrices
        R_roll = np.array([[1, 0, 0],
                           [0, np.cos(roll), -np.sin(roll)],
                           [0, np.sin(roll), np.cos(roll)]])
        R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])
        
        # Combined rotation matrix
        return np.dot(R_pitch, R_roll)
    
    def get_thrust(self):
        accel_normalized = self.acceleration / norm(self.acceleration)  if norm(self.acceleration) > 0 else np.zeros(3)
        thrust_vector = self.command[0] * self.get_rotation() @ accel_normalized
        return thrust_vector
    
    def update(self, dt):
        # Update past states and time 
        self.past_states.append(self.pose.copy())
        self.time += dt


        # ---- Controlled Dynamics ----
        # # Get command
        # self.command = Controller(self, dt)

        # # Update dynamics
        # self.acceleration = self.get_weight() + self.get_drag() + self.get_thrust()
        # self.velocity += self.acceleration * dt
        # self.pose += self.velocity * dt

        # ---- Trajectory Follower ----
        if self.intercepted == False:
            if len(self.future_states) == 0:
                self.pose = self.pose 
            else:

                if self.time >= self.future_times[0]:
                    self.pose = self.future_states.pop(0)
                    self.future_times.pop(0)
                if len(self.future_states) == 0:   
                    self.intercepted = True
                    print("Intercepted!")
        else:
            self.pose += self.velocity * dt 



