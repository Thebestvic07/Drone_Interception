'''
Main Simulation File

This file runs the simulation loop and handles the time-stepping for the UFO and interceptor.

'''

############       Arguments       ############
# Initial UFO values
ufo_position = [-1000, -500, 100]
ufo_velocity = [30.0, -10.0, 0.0]
ufo_acceleration = [0.0, 0.0, 0.0]

base_position = [0, 0, 0]  # Interceptor base position

# Rendering options
live_rendering = True
animation_speed = 3          # Adjust this to speed up or slow down the animation
map_limits = [(-1000, 1000), (-1000, 1000), (0, 200)]  # X, Y, Z limits for the plot

# Simulation parameters
dt = 0.1  # seconds
sim_time = 40.0  # total simulation time in seconds

############       Libs       ############
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import argparse
import time

##############      Classes       ############
from UFO import UFO
from Stryxceptor import Stryxceptor
from Motion_Planner import Motion_Planner

##############     Simulation & Rendering      ############

def update(dt, ufo, planner, interceptor):
    ufo.update(dt)
    planner.update(dt)
    interceptor.update(dt)

def render(subplot, object, style='r'):
    # Plot current position
    subplot.scatter(object.pose[0], object.pose[1], object.pose[2], c=style, label=object.__class__.__name__)

    # Past states as a line
    if hasattr(object, 'past_states'):
        past = list(zip(*object.past_states))
        if past:
            subplot.plot(past[0], past[1], past[2], c=style, alpha=0.5)

    # Target as cross
    if hasattr(object, 'target') and object.target is not None:
        subplot.scatter(object.target[0], object.target[1], object.target[2], c=style, marker='x', label='Target')

    # Future predictions
    if hasattr(object, 'future_states'):
        for idx in range(len(object.future_states)):
            state = object.future_states[idx]
            subplot.scatter(state[0], state[1], state[2], c=style, s=0.1, alpha=0.5)

def render_interception(dt, sim_time, ufo, planner, interceptor):
    steps = int(sim_time / dt) 

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    def render_init():
        ax.set_xlim(map_limits[0])
        ax.set_ylim(map_limits[1])
        ax.set_zlim(map_limits[2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('UFO Interception Simulation')
        ax.legend()
        return []

    def render_update(frame):
        ax.cla()  # Clear previous frame
        render_init()

        # Simulation step
        update(dt, ufo, planner, interceptor)

        # Render current state
        render(ax, ufo, 'r')
        render(ax, interceptor, 'b')
        return []

    ani = FuncAnimation(fig, render_update, frames=steps,
                        init_func=render_init,
                        interval=dt*1000/animation_speed,
                        blit=False)        
    plt.show()


def render_interception_static(ufo, interceptor):
        # Create figure and axis
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(map_limits[0])
        ax.set_ylim(map_limits[1])
        ax.set_zlim(map_limits[2])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('UFO Interception Simulation')
        # Render UFO
        render(ax, ufo, 'r')
        # Render Interceptor
        render(ax, interceptor, 'b') 
        plt.legend()
        plt.tight_layout()
        plt.show()


# Helpers to parse input or use default
def get_input(prompt, default):
    user_input = input(f"{prompt} (default: {' '.join(map(str, default))}): ").strip()
    if not user_input:
        return default
    return list(map(float, user_input.split()))

def get_bool_input(prompt, default):
    user_input = input(f"{prompt} (y/n, default: {'y' if default else 'n'}): ").strip().lower()
    if not user_input:
        return default
    return user_input == 'y'

##############      Simulation Execution       ############
if __name__ == "__main__":

    # --- Initialize Simulation ---
    steps = int(sim_time / dt)

    # Init Objects 
    ufo = UFO(init_pose =ufo_position, 
              init_velocity=ufo_velocity,
              init_acceleration=ufo_acceleration)
    
    interceptor = Stryxceptor(init_pose=base_position,
                              dt=dt)
    
    planner = Motion_Planner(ufo, interceptor, dt)

    print("--- Simulation Initialized ---")
    print("UFO Position: ", ufo.pose, "Velocity: ", ufo.velocity, "Acceleration: ", ufo.acceleration)

    # --- Simulation Loop ---
    # Static simulation
    if not live_rendering:
        # for _ in range(steps):
        update(dt, ufo, planner, interceptor)

        # Plot Simulation Output
        render_interception_static(ufo, interceptor)
        # Return the simulation data 
        # print("UFO :", ufo.past_states)
        # print("Interceptor :", interceptor.past_states)    

    # Live simulation
    else:
        render_interception(dt, sim_time, ufo, planner, interceptor)

