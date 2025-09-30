"""
Advanced robot behaviors and path planning algorithms
"""

import math
import numpy as np
from typing import List, Tuple, Optional
from enum import Enum

class PathPlanningMode(Enum):
    DIRECT = "Direct Path"
    SMOOTH_CURVE = "Smooth Curve"
    OBSTACLE_AVOIDANCE = "Obstacle Avoidance"

class RobotBehavior:
    """Enhanced robot behaviors for more intelligent navigation"""
    
    def __init__(self):
        self.path_planning_mode = PathPlanningMode.DIRECT
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 0.1  # meters
        
    def set_path_planning_mode(self, mode: PathPlanningMode):
        """Set the path planning mode"""
        self.path_planning_mode = mode
        
    def generate_smooth_path(self, start: Tuple[float, float], end: Tuple[float, float], 
                           num_waypoints: int = 10) -> List[Tuple[float, float]]:
        """
        Generate a smooth curved path between start and end points
        
        Args:
            start: Starting position (x, y)
            end: Ending position (x, y)
            num_waypoints: Number of intermediate waypoints
            
        Returns:
            List of waypoint coordinates
        """
        waypoints = []
        
        # Calculate control points for Bezier curve
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        
        # Create control points that make a smooth S-curve
        control1 = (start[0] + dx * 0.3, start[1] + dy * 0.1)
        control2 = (start[0] + dx * 0.7, start[1] + dy * 0.9)
        
        # Generate waypoints along Bezier curve
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            
            # Cubic Bezier curve calculation
            x = (1-t)**3 * start[0] + 3*(1-t)**2*t * control1[0] + 3*(1-t)*t**2 * control2[0] + t**3 * end[0]
            y = (1-t)**3 * start[1] + 3*(1-t)**2*t * control1[1] + 3*(1-t)*t**2 * control2[1] + t**3 * end[1]
            
            waypoints.append((x, y))
            
        return waypoints
    
    def generate_spiral_approach(self, start: Tuple[float, float], end: Tuple[float, float], 
                               spiral_radius: float = 0.5, spiral_turns: float = 1.5) -> List[Tuple[float, float]]:
        """
        Generate a spiral approach to the target (useful for precise positioning)
        
        Args:
            start: Starting position (x, y)
            end: Ending position (x, y)
            spiral_radius: Radius of spiral approach
            spiral_turns: Number of spiral turns
            
        Returns:
            List of waypoint coordinates
        """
        waypoints = []
        
        # Calculate approach angle
        approach_angle = math.atan2(end[1] - start[1], end[0] - start[0])
        
        # Generate spiral points
        num_points = int(spiral_turns * 20)  # 20 points per turn
        
        for i in range(num_points):
            t = i / num_points
            
            # Spiral parameters
            angle = approach_angle + spiral_turns * 2 * math.pi * t
            radius = spiral_radius * (1 - t)  # Shrinking radius
            
            # Calculate spiral point
            spiral_x = end[0] + radius * math.cos(angle)
            spiral_y = end[1] + radius * math.sin(angle)
            
            waypoints.append((spiral_x, spiral_y))
            
        # Add final target
        waypoints.append(end)
        
        return waypoints
    
    def plan_path(self, robot_pos: Tuple[float, float], target_pos: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Plan a path from robot position to target based on selected mode
        
        Args:
            robot_pos: Current robot position (x, y)
            target_pos: Target position (x, y)
            
        Returns:
            List of waypoints to follow
        """
        if self.path_planning_mode == PathPlanningMode.DIRECT:
            return [target_pos]
        elif self.path_planning_mode == PathPlanningMode.SMOOTH_CURVE:
            return self.generate_smooth_path(robot_pos, target_pos)
        elif self.path_planning_mode == PathPlanningMode.OBSTACLE_AVOIDANCE:
            # For now, use smooth curve as placeholder
            return self.generate_smooth_path(robot_pos, target_pos)
        else:
            return [target_pos]
    
    def get_current_target(self, robot_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        Get the current waypoint target for the robot
        
        Args:
            robot_pos: Current robot position (x, y)
            
        Returns:
            Current target waypoint or None if no path is set
        """
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return None
            
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        # Check if current waypoint is reached
        distance = math.sqrt((robot_pos[0] - current_waypoint[0])**2 + 
                           (robot_pos[1] - current_waypoint[1])**2)
        
        if distance < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                return self.waypoints[self.current_waypoint_index]
            else:
                return None  # Path completed
                
        return current_waypoint
    
    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """Set new waypoints and reset navigation"""
        self.waypoints = waypoints
        self.current_waypoint_index = 0
    
    def is_path_complete(self) -> bool:
        """Check if the current path is complete"""
        return self.current_waypoint_index >= len(self.waypoints)
    
    def reset_path(self):
        """Reset the current path"""
        self.waypoints.clear()
        self.current_waypoint_index = 0

class AdaptiveController:
    """
    Adaptive controller that adjusts gains based on performance
    """
    
    def __init__(self, base_kp: float = 2.0, base_ki: float = 0.1, base_kd: float = 0.5):
        self.base_kp = base_kp
        self.base_ki = base_ki  
        self.base_kd = base_kd
        
        # Adaptation parameters
        self.performance_history = []
        self.adaptation_rate = 0.1
        self.min_gain_factor = 0.5
        self.max_gain_factor = 2.0
        
        # Current adaptive gains
        self.adaptive_kp_factor = 1.0
        self.adaptive_ki_factor = 1.0
        self.adaptive_kd_factor = 1.0
        
    def update_performance(self, error: float, settling_time: float, overshoot: float):
        """
        Update controller gains based on performance metrics
        
        Args:
            error: Current steady-state error
            settling_time: Time to settle within tolerance
            overshoot: Maximum overshoot observed
        """
        # Calculate performance score (lower is better)
        performance_score = abs(error) + settling_time * 0.1 + overshoot * 2.0
        
        self.performance_history.append(performance_score)
        
        # Keep only recent history
        if len(self.performance_history) > 10:
            self.performance_history.pop(0)
            
        # Adapt gains if we have enough history
        if len(self.performance_history) >= 3:
            recent_trend = self.performance_history[-1] - self.performance_history[-3]
            
            # If performance is getting worse, adjust gains
            if recent_trend > 0:
                # Increase derivative gain to reduce overshoot
                if overshoot > 0.1:
                    self.adaptive_kd_factor = min(self.max_gain_factor, 
                                                self.adaptive_kd_factor + self.adaptation_rate)
                
                # Adjust proportional gain based on settling time
                if settling_time > 3.0:
                    self.adaptive_kp_factor = min(self.max_gain_factor,
                                                self.adaptive_kp_factor + self.adaptation_rate)
                elif settling_time < 1.0:
                    self.adaptive_kp_factor = max(self.min_gain_factor,
                                                self.adaptive_kp_factor - self.adaptation_rate)
                
                # Adjust integral gain based on steady-state error
                if abs(error) > 0.05:
                    self.adaptive_ki_factor = min(self.max_gain_factor,
                                                self.adaptive_ki_factor + self.adaptation_rate)
    
    def get_adaptive_gains(self) -> Tuple[float, float, float]:
        """
        Get current adaptive controller gains
        
        Returns:
            Tuple of (kp, ki, kd) with adaptive factors applied
        """
        kp = self.base_kp * self.adaptive_kp_factor
        ki = self.base_ki * self.adaptive_ki_factor
        kd = self.base_kd * self.adaptive_kd_factor
        
        return kp, ki, kd
    
    def reset_adaptation(self):
        """Reset adaptive factors to baseline"""
        self.adaptive_kp_factor = 1.0
        self.adaptive_ki_factor = 1.0
        self.adaptive_kd_factor = 1.0
        self.performance_history.clear()

class RobotFormation:
    """
    Multi-robot formation control (for future extensions)
    """
    
    def __init__(self, formation_type: str = "line"):
        self.formation_type = formation_type
        self.robots = []
        self.leader_index = 0
        self.formation_spacing = 1.0  # meters
        
    def add_robot(self, robot_id: str):
        """Add a robot to the formation"""
        self.robots.append(robot_id)
        
    def get_formation_target(self, robot_id: str, leader_pos: Tuple[float, float], 
                           leader_orientation: float) -> Tuple[float, float]:
        """
        Calculate target position for a robot in formation
        
        Args:
            robot_id: ID of the robot
            leader_pos: Position of formation leader
            leader_orientation: Orientation of formation leader
            
        Returns:
            Target position for the robot
        """
        try:
            robot_index = self.robots.index(robot_id)
        except ValueError:
            return leader_pos
            
        if robot_index == self.leader_index:
            return leader_pos
            
        # Calculate formation offset based on type
        if self.formation_type == "line":
            offset_distance = (robot_index - self.leader_index) * self.formation_spacing
            offset_x = -offset_distance * math.cos(leader_orientation)
            offset_y = -offset_distance * math.sin(leader_orientation)
        elif self.formation_type == "wedge":
            if robot_index < self.leader_index:
                side = -1
                offset_rank = self.leader_index - robot_index
            else:
                side = 1
                offset_rank = robot_index - self.leader_index
                
            offset_distance = offset_rank * self.formation_spacing
            offset_angle = leader_orientation + side * math.pi / 6  # 30 degrees
            offset_x = -offset_distance * math.cos(offset_angle)
            offset_y = -offset_distance * math.sin(offset_angle)
        else:
            offset_x = 0
            offset_y = 0
            
        target_x = leader_pos[0] + offset_x
        target_y = leader_pos[1] + offset_y
        
        return (target_x, target_y)