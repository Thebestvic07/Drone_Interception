'''
Class to simulate a UFO to be intercepted.

'''

############       Libs       ############
import numpy as np
from numpy.linalg import norm

##############       UFO Class       ############
class UFO:
    def __init__(self, init_pose, init_velocity=np.zeros(3), init_acceleration=np.zeros(3)):
        self.pose = np.array(init_pose, dtype=float)
        self.velocity = np.array(init_velocity, dtype=float)
        self.acceleration = np.array(init_acceleration, dtype=float)

        self.past_states = []
        self.future_states = []

    def update(self, dt):
        # Store past state
        self.past_states.append(self.pose.copy())
        
        # Update Dynamics
        self.velocity += self.acceleration * dt
        self.pose += self.velocity * dt

    def predict_pose(self, t):
        # Predict future pose at time t (assuming constant accel)
        return (self.pose +
                self.velocity * t +
                0.5 * self.acceleration * t**2)
    
    def predict_speed(self, t):
        # Predict future pose at time t (assuming constant accel)
        return (self.velocity +
                self.acceleration * t)