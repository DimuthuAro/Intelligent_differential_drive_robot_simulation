import numpy as np
from enum import Enum

class ControllerType(Enum):
    """Enumeration for different controller types"""
    P = "P Controller"
    PD = "PD Controller"
    PID = "PID Controller"

class PController:
    """
    Proportional Controller
    
    Control law: u = Kp * error
    
    This is the simplest controller that produces a control signal proportional
    to the current error. It provides fast response but may have steady-state error
    and can be oscillatory.
    """
    
    def __init__(self, kp=1.0):
        """
        Initialize P controller
        
        Args:
            kp (float): Proportional gain
        """
        self.kp = kp
        self.name = "P Controller"
    
    def compute_control(self, error, dt):
        """
        Compute control signal using proportional control
        
        Args:
            error (float): Current error
            dt (float): Time step (not used in P controller)
            
        Returns:
            float: Control signal
        """
        return self.kp * error
    
    def reset(self):
        """Reset controller state (no state to reset for P controller)"""
        pass


class PDController:
    """
    Proportional-Derivative Controller
    
    Control law: u = Kp * error + Kd * d(error)/dt
    
    Adds derivative term to P controller for improved stability and reduced overshoot.
    The derivative term provides damping by opposing rapid changes in error.
    """
    
    def __init__(self, kp=1.0, kd=0.1):
        """
        Initialize PD controller
        
        Args:
            kp (float): Proportional gain
            kd (float): Derivative gain
        """
        self.kp = kp
        self.kd = kd
        self.previous_error = 0.0
        self.name = "PD Controller"
    
    def compute_control(self, error, dt):
        """
        Compute control signal using PD control
        
        Args:
            error (float): Current error
            dt (float): Time step for derivative calculation
            
        Returns:
            float: Control signal
        """
        # Calculate derivative of error
        if dt > 0:
            error_derivative = (error - self.previous_error) / dt
        else:
            error_derivative = 0.0
        
        # Update previous error for next iteration
        self.previous_error = error
        
        # PD control law
        return self.kp * error + self.kd * error_derivative
    
    def reset(self):
        """Reset derivative term"""
        self.previous_error = 0.0


class PIDController:
    """
    Proportional-Integral-Derivative Controller
    
    Control law: u = Kp * error + Ki * ∫error dt + Kd * d(error)/dt
    
    Full PID controller with integral term to eliminate steady-state error.
    The integral term accumulates past errors to provide zero steady-state error.
    """
    
    def __init__(self, kp=1.0, ki=0.01, kd=0.1, integral_limit=10.0):
        """
        Initialize PID controller
        
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain
            kd (float): Derivative gain
            integral_limit (float): Maximum integral term to prevent windup
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        
        self.previous_error = 0.0
        self.integral = 0.0
        self.name = "PID Controller"
    
    def compute_control(self, error, dt):
        """
        Compute control signal using PID control
        
        Args:
            error (float): Current error
            dt (float): Time step for integration and derivation
            
        Returns:
            float: Control signal
        """
        # Update integral term
        self.integral += error * dt
        
        # Apply integral windup protection
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        
        # Calculate derivative of error
        if dt > 0:
            error_derivative = (error - self.previous_error) / dt
        else:
            error_derivative = 0.0
        
        # Update previous error for next iteration
        self.previous_error = error
        
        # PID control law
        return self.kp * error + self.ki * self.integral + self.kd * error_derivative
    
    def reset(self):
        """Reset integral and derivative terms"""
        self.integral = 0.0
        self.previous_error = 0.0


class RobotController:
    """
    High-level robot controller that uses P/PD/PID controllers for navigation
    
    This controller manages both linear and angular motion to guide the robot
    to target positions using the selected control algorithm.
    """
    
    def __init__(self, controller_type=ControllerType.PID):
        """
        Initialize robot controller with optimized parameters
        
        Args:
            controller_type (ControllerType): Type of controller to use
        """
        self.controller_type = controller_type
        
        # Optimized controller parameters based on system characteristics
        if controller_type == ControllerType.P:
            self.linear_controller = PController(kp=1.5)  # Reduced for stability
            self.angular_controller = PController(kp=4.0)  # Increased for responsiveness
        elif controller_type == ControllerType.PD:
            self.linear_controller = PDController(kp=2.5, kd=0.8)  # Better damping
            self.angular_controller = PDController(kp=4.5, kd=1.2)  # Improved stability
        else:  # PID - Optimized parameters
            self.linear_controller = PIDController(kp=3.0, ki=0.05, kd=0.7, integral_limit=5.0)
            self.angular_controller = PIDController(kp=5.0, ki=0.02, kd=1.0, integral_limit=3.0)
        
        self.tolerance = 0.03  # Tighter tolerance for better precision
        self.angle_tolerance = 0.1  # radians (~5.7 degrees)
        self.previous_distance = float('inf')  # For adaptive control
        
    def compute_control(self, robot, target_x, target_y, dt):
        """
        Compute wheel velocities to reach target position with adaptive behavior
        
        Args:
            robot (DifferentialDriveRobot): Robot instance
            target_x (float): Target x position
            target_y (float): Target y position
            dt (float): Time step
            
        Returns:
            tuple: (v_left, v_right) wheel velocities
        """
        # Calculate errors
        distance_error = robot.distance_to_target(target_x, target_y)
        angle_error = robot.angle_to_target(target_x, target_y)
        
        # Check if target is reached
        if distance_error < self.tolerance and abs(angle_error) < self.angle_tolerance:
            self.reset()  # Reset controllers when target reached
            return 0.0, 0.0
        
        # Adaptive control strategy based on distance
        if distance_error > 1.0:  # Far from target - prioritize reaching
            # Use full linear control, moderate angular
            linear_scale = 1.0
            angular_scale = 0.7
        elif distance_error > 0.2:  # Medium distance - balanced approach
            linear_scale = 0.8
            angular_scale = 1.0
        else:  # Close to target - prioritize orientation
            linear_scale = 0.4
            angular_scale = 1.2
        
        # Compute control signals with adaptive scaling
        linear_control = self.linear_controller.compute_control(distance_error, dt) * linear_scale
        angular_control = self.angular_controller.compute_control(angle_error, dt) * angular_scale
        
        # Apply velocity limits based on distance (smoother approach)
        max_linear_vel = min(3.0, distance_error * 2.0)  # Scale velocity with distance
        linear_control = np.clip(linear_control, -max_linear_vel, max_linear_vel)
        angular_control = np.clip(angular_control, -5.0, 5.0)
        
        # Convert to differential drive wheel velocities
        v = linear_control
        omega = angular_control
        
        v_left = v - omega * robot.wheel_base / 2
        v_right = v + omega * robot.wheel_base / 2
        
        self.previous_distance = distance_error
        return v_left, v_right
    
    def reset(self):
        """Reset controller states"""
        if hasattr(self.linear_controller, 'reset'):
            self.linear_controller.reset()
        if hasattr(self.angular_controller, 'reset'):
            self.angular_controller.reset()

