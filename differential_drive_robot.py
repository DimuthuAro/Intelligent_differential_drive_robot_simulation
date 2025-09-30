"""
Differential Drive Robot Simulation with P, PD, and PID Controllers

This simulation implements a differential drive robot that navigates to target positions
using three different control strategies: 

1) P (Proportional) - This controller adjusts the robot's velocity based on the current error.
    [** The Current Error is the distance to target.]
    
2) PD (Proportional-Derivative) - This controller adds a term that accounts for the 
    rate of change of the error, providing damping. -- Used Derivation of Error.
    
3) PID (Proportional-Integral-Derivative) - This controller adds an integral term that 
    accumulates past errors,

    ** Integral is got by adding all past errors and multiplying by time step.

Mathematical Model:
The differential drive robot has two wheels with velocities v_left and v_right.
    v_left --> velocity of the left vheel
    v_right --> velocity of the right wheel
    v is LINEAR VELOCITY of our simulating robot vehicle
    v = (v_left + v_right) / 2
    
I used to get the average velocity by taking the average of two simulated wheel velocities.
Then I got the difference between the two wheels and its have to be diveded by the distance 
between the center of the two wheels to get the angular velocity.
    ω is ANGULAR VELOCITY of our simulating robot vehicle
    L = distance between the two wheels
    ω = (v_right - v_left) / L

The robot's linear velocity v and angular velocity ω are:
    v = (v_left + v_right) / 2
    ω = (v_right - v_left) / L

The robot's kinematics in global coordinates:
When we dervie the equation for x we got horizontal part of the velocity of the simulating robot
also same to the y and get vertical part of the velocity of the simulating robot
but in mechanical physics we use sin for vertical and cos for horizontal.

USING DERIVATION:
Vertical_velocity = v * sin(θ) also = dx/dt where x is horizontal position.
Horizontal_velocity = v * cos(θ) same as above = dy/dt where y is vertical position.
And for angle same as above two by derivation we get angular velocity ω(Omega) = dθ/dt,
Where θ is the robot's orientation angle.

    dx/dt = v * cos(θ)
    dy/dt = v * sin(θ)
    dθ/dt = ω

Control Laws:

** u is the signal we use to Correct the position of our robot vehicle to reach the target position.
Its a Velocity in terms.

trial and error method:
* Start with a small value for Kp (e.g., 0.1).
* Gradually increase Kp until the system responds quickly to changes in error.
** Means We Test the System until We are At The value That correct the error

** Kp is the Ratio of the constant 
that determines how much the control signal responds to the current error.
its get by trial and error method.

** Kd is the Derivative constant
that determines how much the control signal responds to the rate of change of the error.
We got that by also trial and error method.

Difference in Kp and Kd is:

- Kp reacts to the current error, providing immediate correction.

- Kd reacts to the rate of change of the error, providing damping and reducing overshoot. 
(Using Derivation of Error)

- Ki is the Integral constant
that determines how much the control signal responds to the accumulation of past errors.
(Using Integration of Error)

** Ki is the Integral constant
that determines how much the control signal responds to the accumulation of past errors.
We got that by also trial and error method.

1. P Controller: u = Kp * error
2. PD Controller: u = Kp * error + Kd * d(error)/dt
3. PID Controller: u = Kp * error + Ki * ∫error dt + Kd * d(error)/dt
"""

import pygame
import numpy as np
import math
import time
import matplotlib.pyplot as plt

from controllers import PController, PDController, PIDController, ControllerType, RobotController

class DifferentialDriveRobot:
    """
    Differential Drive Robot Model
    
    This class implements the kinematics and dynamics of a differential drive robot.
    The robot has two wheels separated by distance L, and can move forward/backward
    and turn by controlling individual wheel velocities.
    """

    def __init__(self, x : float=0, y : float=0, theta : float=0.0, wheel_base : float=0.3, max_velocity : float=50.0):
        """
        Initialize the differential drive robot
        
        Args:
            x (float): Initial x position (m)
            y (float): Initial y position (m)
            theta (float): Initial orientation (radians)
            wheel_base (float): Distance between wheels (m)
            max_velocity (float): Maximum wheel velocity (m/s)
        """
        # Robot state
        self.x : float = x
        self.y : float = y
        self.theta : float = theta

        # Robot parameters
        self.wheel_base : float = wheel_base  # The Distance Between the two centers of the wheels Notated as L in First Comment
        self.max_velocity : float = max_velocity # Maximum wheel velocity (m/s)
        self.radius : float = 0.2  # Robot radius for visualization Only For Visualization (Not Used in Calculations)

        # Velocity constraints
        self.v_left : float = 0.0 # The Velocity of Left Wheel
        self.v_right : float = 0.0 # The Velocity of Right Wheel

        # History for trajectory plotting : Stores the path taken by the robot For Visualization Only
        self.trajectory_x : list[float] = [x] # List to store x positions
        self.trajectory_y : list[float] = [y] # List to store y positions

    def set_wheel_velocities(self, v_left : float, v_right : float) -> None:
        """
        Set the velocities of left and right wheels with smoothing
        
        Args:
            v_left (float): Left wheel velocity (m/s)
            v_right (float): Right wheel velocity (m/s)
        """
        # Apply velocity constraints
        target_v_left = np.clip(v_left, -self.max_velocity, self.max_velocity)
        target_v_right = np.clip(v_right, -self.max_velocity, self.max_velocity)
        
        # Add velocity smoothing to prevent jerky movements
        max_acceleration = 5.0  # m/s^2
        dt = 1.0 / 60.0  # Assume 60 FPS for smoothing
        max_velocity_change = max_acceleration * dt
        
        # Smooth left wheel velocity
        vel_diff_left = target_v_left - self.v_left
        if abs(vel_diff_left) > max_velocity_change:
            vel_diff_left = math.copysign(max_velocity_change, vel_diff_left)
        self.v_left += vel_diff_left
        
        # Smooth right wheel velocity
        vel_diff_right = target_v_right - self.v_right
        if abs(vel_diff_right) > max_velocity_change:
            vel_diff_right = math.copysign(max_velocity_change, vel_diff_right)
        self.v_right += vel_diff_right
    
    def get_velocities(self) -> tuple[float, float]:
        """
        Get linear and angular velocities from wheel velocities
        
        Returns:
            tuple: (linear_velocity, angular_velocity)
        """
        # Linear velocity: average of wheel velocities
        # As in the first comment v = (v_left + v_right) / 2 Implementation
        v = (self.v_left + self.v_right) / 2.0
        
        # Angular velocity: difference in wheel velocities divided by wheel base
        # As in the first comment ω = (v_right - v_left) / L Implementation
        omega = (self.v_right - self.v_left) / self.wheel_base # ω --> omega is angular velocity
        
        return v, omega # Return linear and angular velocities as tuple of two floats

    def update(self, dt: float) -> None:
        """
        Update robot position and orientation using kinematic model
        
        dt ---> Delta Time --> Time step in seconds (How much time has passed since last update)
        dt is used to scale the velocity to get the change in position and orientation.
        
        Args:
            dt (float): Time step (seconds)
        """
        
        # Get current velocities
        # Get from the get_velocities method as a tuple of two floats
        v, omega = self.get_velocities()
        
        # Cache trigonometric calculations for performance
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        
        # Update position and orientation using kinematic equations
        # Using the equations from the first comment
        # dx/dt = v * cos(θ)
        # dy/dt = v * sin(θ)  
        # dθ/dt = ω
        
        self.x += v * cos_theta * dt # Update x position
        self.y += v * sin_theta * dt # Update y position
        self.theta += omega * dt # Update orientation
        
        # Normalize angle to [-π, π] for consistency
        # If theta exceeds this range, it wraps around to stay within it.
        """ Example: If theta = 4π, then it will be normalized to 0 (since 4π is equivalent to 0 in a circular system)."""
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Store trajectory Only For Visualization (limit size for memory efficiency)
        # Append current position to trajectory lists
        self.trajectory_x.append(self.x)
        self.trajectory_y.append(self.y)
        
        # Limit trajectory history to prevent memory issues
        max_trajectory_points = 1000
        if len(self.trajectory_x) > max_trajectory_points:
            self.trajectory_x.pop(0)
            self.trajectory_y.pop(0)


    def get_position(self) -> tuple[float, float, float]:
        """Get current position and orientation"""
        # Return current position (x, y) and orientation (theta) as a tuple of three floats
        return self.x, self.y, self.theta
    
    def pythagorean_theorem(self, a: float, b: float) -> float:
        """Calculate hypotenuse using Pythagorean theorem"""
        # Used to calculate Euclidean distance using Pythagorean theorem
        return math.sqrt(a**2 + b**2)
    
    def distance_to_target(self, target_x, target_y) -> float:
        """Calculate Euclidean distance to target"""
        # Calculate Euclidean distance to target position (target_x, target_y)
        dx = target_x - self.x
        dy = target_y - self.y
        
        # Use optimized distance calculation
        return math.sqrt(dx * dx + dy * dy)
    
    def distance_squared_to_target(self, target_x, target_y) -> float:
        """Calculate squared distance to target (faster for comparisons)"""
        dx = target_x - self.x
        dy = target_y - self.y
        return dx * dx + dy * dy

    def angle_normalization(self, angle: float) -> float:
        """Normalize angle to [-π, π]"""
        # Normalize angle to be within [-π, π]
        # math.atan2(math.sin(angle), math.cos(angle)) is a compact way to do this
        """ 
        if angle > π:
        while angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -π:
            while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        """
        angle_sine = math.sin(angle)
        angle_cosine = math.cos(angle)
        normalized_angle = math.atan2(angle_sine, angle_cosine)
        return normalized_angle

    def angle_to_target(self, target_x, target_y) -> float:
        """Calculate angle to target relative to current orientation"""
        target_angle = math.atan2(target_y - self.y, target_x - self.x)
        angle_error = target_angle - self.theta
        
        # Normalize angle error to [-π, π] using optimized method
        return math.atan2(math.sin(angle_error), math.cos(angle_error))

    # Test the robot direction - add this to your main_state.py temporarily
    def test_robot_direction(self):
        """Test function to verify robot direction"""
        print(f"Robot position: {self.get_position()}")
        print(f"Robot angle: {self.theta} radians = {math.degrees(self.theta)} degrees")
        
        # Test: move robot forward for 1 second
        self.set_wheel_velocities(1.0, 1.0)  # Both wheels same speed = straight line
        self.update(1.0)  # Update for 1 second
        
        new_pos = self.get_position()
        print(f"After moving forward: {new_pos}")
        print(f"Expected: robot should move in direction of angle {math.degrees(self.theta)} degrees")

