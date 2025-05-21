'''
Class to compute a smooth optimal interception trajectory.

'''
############       Libs       ############
import numpy as np
from numpy.linalg import norm

##############      Classes       ############
from UFO import UFO
from Stryxceptor import Stryxceptor

##############       Motion Planner Class       ############
class Motion_Planner:
    def __init__(self, ufo: UFO, interceptor: Stryxceptor, dt, horizon=30.0, vel_final= np.inf, acc_final=np.inf):
        self.ufo = ufo
        self.interceptor = interceptor
        self.horizon = int(horizon / dt)    # Number of seconds to predict forward | default: 30s 
        self.dt = dt                        # Time step for prediction 

        self.ufo_prediction = []
        self.interception_traj = []
        self.times = []

        self.disc_steps = 50 #Integer number steps to divide every path segment into to provide the reference positions for control 
        self.t_start = 0 # Start time of the trajectory 
        self.times = [] # Time vector for the path segments

        self.vel_final = vel_final if vel_final != np.inf else self.interceptor.max_speed  # m/s
        self.acc_final = acc_final if acc_final != np.inf else 0.0  # m/s²


    def update(self, dt):       
        # if np.round(self.t_start, 2) - int(self.t_start) == 0:  # Recompute planning every second
        if self.t_start == 0:                                     # Compute planning only once at the start
            # Recompute the UFO's future position
            self.predict_ufo(dt, self.horizon)
            for idx in range(len(self.ufo_prediction)): # append some of predicted positions to the UFO's future states
                if idx % 30 == 0:
                    self.ufo.future_states.append(self.ufo_prediction[idx])

            # Recompute the interception trajectory
            self.compute_interception_trajectory()

        # Update time 
        self.t_start += dt


    def predict_ufo(self, dt, horizon):
        # Predict UFO's future position assuming constant acceleration
        self.ufo_prediction = []
        for i in range(horizon):
            t = i * dt
            predicted_pos = self.ufo.predict_pose(t)
            self.ufo_prediction.append(predicted_pos)

    def compute_interception_trajectory(self):
        # Compute the interception trajectory based on the UFO's predicted position
        self.interception_traj = []

        # Compute the interception waypoints
        waypoints = self.find_interception_waypoint()
        print("Waypoints: ", waypoints)
        print("Times: ", self.times)

        # Compute the velocity and acceleration at the interception point
        vel_intercept = self.ufo.predict_speed(self.times[-1] - self.times[0])  # UFO's predicted velocity at the interception point
        vel_intercept = vel_intercept / norm(vel_intercept) * self.vel_final if norm(vel_intercept) > 0 else np.array([self.vel_final, 0, 0])  
        acc_intercept = self.ufo.acceleration / norm(self.ufo.acceleration) * self.acc_final if norm(self.ufo.acceleration) > 0 else np.zeros(3)  # constant acceleration
        print("vel_intercept: ", vel_intercept)
        print("acc_intercept: ", acc_intercept)

        # Compute a smooth trajectory to the waypoints
        self.interception_traj, _ = self.fit_trajectory(waypoints, vel_intercept, acc_intercept)

    def find_interception_waypoint(self):
        # Find the interception waypoint by solving the optimization problem
        waypoints = []

        # First point is the interceptor's current position
        waypoints.append([self.interceptor.pose[0], self.interceptor.pose[1], self.interceptor.pose[2]])
        self.times = [0]

        # Second point is the UFO's current height
        height_point = [self.interceptor.pose[0], self.interceptor.pose[1], self.ufo.pose[2]]
        waypoints.append(height_point)
        up_time = np.sqrt(2/3 * self.ufo.pose[2])   # Time estimation to reach the UFO's height with max thrust and cst drag (10m/s)
        
        # Check feasibility of the interception
        print("Up time: ", up_time)
        self.times.append(up_time)

        # UFO's predicted pose in the x-y plane
        intercept_point = np.zeros(3)
        intercept_distance = 10.0  # Distance to intercept point
        ufo_pos = self.ufo.predict_pose(up_time)  # UFO's predicted position when the interceptor reaches its height

        delta = norm(np.array(ufo_pos) - np.array(intercept_point))  # Distance between predicted pose of UFO and current intercept point
        iter = 0
        while delta > intercept_distance:
            # Update the intercept point
            intercept_point = ufo_pos
                    
            # Compute approx time to get to the ney interception point 
            distance = norm(np.array(intercept_point) - np.array(height_point))
            horizontal_time = distance / (self.interceptor.max_speed *0.50)   # Add a safety factor since trajectory is not straight

            print("Horizontal time: ", horizontal_time)

            # Compute actual position of the UFO at this time
            ufo_pos = self.ufo.predict_pose(horizontal_time + up_time)

            # Update delta
            delta = norm(np.array(ufo_pos) - np.array(intercept_point))
            print("Delta: ", delta, " Intercept Point: ", intercept_point, " UFO Position: ", ufo_pos)
            iter += 1
            if iter > 1000:
                raise ValueError("UFO is too fast to intercept.")

        waypoints.append(intercept_point)
        self.times.append(horizontal_time + up_time)

        # Update interceptor target
        self.interceptor.update_target(waypoints[-1])

        return waypoints
    
    def fit_trajectory(self, waypoints, vel_intercept, acc_intercept):
        # Fit a smooth trajectory to the waypoints
        coeffs = self.compute_poly_coefficients(waypoints, vel_intercept, acc_intercept)
        trajectory, times = self.poly_setpoint_extraction(coeffs)

        # Update the interceptor's future states
        self.interceptor.future_states = trajectory.tolist()
        self.interceptor.future_times = times.tolist()

        return trajectory, times

    ##############       Polynomial Fitting       ############
    
    def compute_poly_matrix(self, t):
        '''
        Computes the polynomial constraints matrix for the trajectory fitting.
        Inputs:
        - t: The time of evaluation of the A matrix (t=0 at the start of a path segment, else t >= 0)
        Outputs:
        - The constraint matrix "A(t)" [5 x 6] representing the equations for position, velocity, acceleration, jerk and snap.
        '''
        A_m = np.array([
            [1, t, t**2, t**3, t**4, t**5],
            [0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4],
            [0, 0, 2, 6*t, 12*t**2, 20*t**3],
            [0, 0, 0, 6, 24*t, 60*t**2],
            [0, 0, 0, 0, 24, 120*t]
            
        ])

        return A_m
    
    def compute_poly_coefficients(self, path_waypoints, vel_intercept, acc_intercept):
        '''
        Computes the polynomial coefficients for the trajectory given the path waypoints and the final velocity and acceleration at the interception point.
        Inputs:
        - path_waypoints: The sequence of input path waypoints provided by the path-planner
        - vel_intercept: The final velocity at the interception point [3 x 1]
        - acc_intercept: The final acceleration at the interception point [3 x 1]
        Outputs:
        - poly_coeffs: The polynomial coefficients for each segment of the path [6(m-1) x 3]
        '''

        seg_times = np.diff(self.times) #The time taken to complete each path segment
        m = len(path_waypoints) #Number of path waypoints (including start and end)
        poly_coeffs = np.zeros((6*(m-1),3)) 

        for dim in range(3):  # Compute for x, y, and z separately
            A = np.zeros((6*(m-1), 6*(m-1)))
            b = np.zeros(6*(m-1))
            pos = np.array([p[dim] for p in path_waypoints])

            A_i = self.compute_poly_matrix(0)  # Constraints for beginning of the segment
            for i in range(m-1):
                A_f = self.compute_poly_matrix(seg_times[i])    # Constraints for end of the segment
                if i == 0:
                    # Initial Position constraints
                    A[i*6, i*6:(i+1)*6] = A_i[0]
                    b[i*6] = pos[i]
                    # Final position constraint
                    A[i*6+1, i*6:(i+1)*6] = A_f[0]
                    b[i*6+1] = pos[i+1]                       
                    # Continuity constraints
                    A[i*6+2:i*6+6, i*6:(i+1)*6] = A_f[1:5]
                    A[i*6+2:i*6+6, (i+1)*6:(i+2)*6] = -A_i[1:5]

                    # Initial velocity + acceleration constraints added at the end 
                    A[6*(m-1)-2, 0:6] = A_i[1] #velocity
                    b[6*(m-1)-2] = self.interceptor.velocity[dim]
                    A[6*(m-1)-1, 0:6] = A_i[2] #acceleration
                    b[6*(m-1)-1] = self.interceptor.acceleration[dim]
                            
                elif i == (m-2):
                    # Initial Position constraints
                    A[i*6, i*6:(i+1)*6] = A_i[0]
                    b[i*6] = pos[i]
                    # Final position constraint
                    A[i*6+1, i*6:(i+1)*6] = A_f[0]
                    b[i*6+1] = pos[i+1]   
                    # Final velocity + acceleration constraints 
                    A[i*6+2, i*6:(i+1)*6] = A_f[1] #velocity
                    b[i*6+2] = vel_intercept[dim]
                    A[i*6+3, i*6:(i+1)*6] = A_f[2] #acceleration
                    b[i*6+3] = acc_intercept[dim]

                else: 
                    # Initial position constraint
                    A[i*6, i*6:(i+1)*6] = A_i[0]
                    b[i*6] = pos[i]
                    # Final position constraint
                    A[i*6+1, i*6:(i+1)*6] = A_f[0]
                    b[i*6+1] = pos[i+1]
                    # Continuity constraints
                    A[i*6+2:i*6+6, i*6:(i+1)*6] = A_f[1:5]
                    A[i*6+2:i*6+6, (i+1)*6:(i+2)*6] = -A_i[1:5]

            poly_coeffs[:,dim] = np.linalg.solve(A, b) # Solve the equation A * poly_coeffs = b

        return poly_coeffs

    def poly_setpoint_extraction(self, poly_coeffs):
        '''
        Extract the trajectory setpoints from the polynomial coefficients
        Inputs:
        - poly_coeffs: The polynomial coefficients for each segment of the path [6(m-1) x 3]
        Outputs:
        - trajectory_setpoints: The trajectory setpoints for the interceptor [N x 3]
        - time_setpoints: The time setpoints for the trajectory [N x 1]
        '''

        # Uses the class features: self.disc_steps, self.times, self.poly_coeffs, self.vel_lim, self.acc_lim
        x_vals, y_vals, z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))
        v_x_vals, v_y_vals, v_z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))
        a_x_vals, a_y_vals, a_z_vals = np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1)), np.zeros((self.disc_steps*len(self.times),1))

        # Define the time reference in self.disc_steps number of segements
        time_setpoints = np.linspace(self.times[0], self.times[-1], self.disc_steps*len(self.times))  # Fine time intervals

        # Extract the x,y and z direction polynomial coefficient vectors
        coeff_x = poly_coeffs[:,0]
        coeff_y = poly_coeffs[:,1]
        coeff_z = poly_coeffs[:,2]

        for i,t in enumerate(time_setpoints):
            seg_idx = min(max(np.searchsorted(self.times, t)-1,0), len(coeff_x) - 1)
            # Determine the x,y and z position reference points at every refernce time
            x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_x[seg_idx*6:(seg_idx+1)*6])
            y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_y[seg_idx*6:(seg_idx+1)*6])
            z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[0],coeff_z[seg_idx*6:(seg_idx+1)*6])
            # Determine the x,y and z velocities at every reference time
            v_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_x[seg_idx*6:(seg_idx+1)*6])
            v_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_y[seg_idx*6:(seg_idx+1)*6])
            v_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[1],coeff_z[seg_idx*6:(seg_idx+1)*6])
            # Determine the x,y and z accelerations at every reference time
            a_x_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_x[seg_idx*6:(seg_idx+1)*6])
            a_y_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_y[seg_idx*6:(seg_idx+1)*6])
            a_z_vals[i,:] = np.dot(self.compute_poly_matrix(t-self.times[seg_idx])[2],coeff_z[seg_idx*6:(seg_idx+1)*6])

        # Find the maximum absolute velocity during the segment
        vel_max = np.max(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
        vel_mean = np.mean(np.sqrt(v_x_vals**2 + v_y_vals**2 + v_z_vals**2))
        acc_max = np.max(np.sqrt(a_x_vals**2 + a_y_vals**2 + a_z_vals**2))
        print("Maximum flight speed: " + str(vel_max))
        print("Average flight speed: " + str(vel_mean))
        print("Maximum flight acceleration: " + str(acc_max))
        print("Maximum Height: " + str(np.max(z_vals)))
        print("Expected time to intercept: " + str(time_setpoints[-1]))
        
        trajectory_setpoints = np.hstack((x_vals, y_vals, z_vals))

        return trajectory_setpoints, time_setpoints