"""
Performance monitoring and analysis tools for robot simulation optimization
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from dataclasses import dataclass
from typing import List, Tuple, Optional

@dataclass
class PerformanceMetrics:
    """Container for robot performance metrics"""
    arrival_time: float
    path_length: float
    energy_consumption: float
    overshoot_distance: float
    settling_time: float
    steady_state_error: float
    control_effort: float

class PerformanceMonitor:
    """
    Monitor and analyze robot performance metrics for optimization
    """
    
    def __init__(self, max_history_size: int = 1000):
        """
        Initialize performance monitor
        
        Args:
            max_history_size: Maximum number of data points to keep in memory
        """
        self.max_history_size = max_history_size
        
        # Performance tracking
        self.start_time: Optional[float] = None
        self.target_reached_time: Optional[float] = None
        self.mission_start_pos: Optional[Tuple[float, float]] = None
        self.target_pos: Optional[Tuple[float, float]] = None
        
        # Real-time metrics
        self.position_history = deque(maxlen=max_history_size)
        self.velocity_history = deque(maxlen=max_history_size)
        self.control_history = deque(maxlen=max_history_size)
        self.error_history = deque(maxlen=max_history_size)
        self.time_history = deque(maxlen=max_history_size)
        
        # Cumulative metrics
        self.total_path_length = 0.0
        self.total_energy = 0.0
        self.max_overshoot = 0.0
        self.previous_position: Optional[Tuple[float, float]] = None
        
    def start_mission(self, robot_pos: Tuple[float, float], target_pos: Tuple[float, float]):
        """Start a new mission and reset metrics"""
        self.start_time = time.time()
        self.target_reached_time = None
        self.mission_start_pos = robot_pos
        self.target_pos = target_pos
        self.previous_position = robot_pos
        
        # Reset cumulative metrics
        self.total_path_length = 0.0
        self.total_energy = 0.0
        self.max_overshoot = 0.0
        
        # Clear history
        self.position_history.clear()
        self.velocity_history.clear()
        self.control_history.clear()
        self.error_history.clear()
        self.time_history.clear()
        
    def update(self, robot, v_left: float, v_right: float, distance_error: float, dt: float):
        """
        Update performance metrics with current robot state
        
        Args:
            robot: DifferentialDriveRobot instance
            v_left: Left wheel velocity command
            v_right: Right wheel velocity command  
            distance_error: Current distance error
            dt: Time step
        """
        if self.start_time is None:
            return
            
        current_time = time.time() - self.start_time
        current_pos = (robot.x, robot.y)
        
        # Calculate path length increment
        if self.previous_position is not None:
            dx = current_pos[0] - self.previous_position[0]
            dy = current_pos[1] - self.previous_position[1]
            path_increment = np.sqrt(dx**2 + dy**2)
            self.total_path_length += path_increment
            
        # Calculate energy consumption (simplified model)
        v, omega = robot.get_velocities()
        energy_increment = (abs(v_left) + abs(v_right)) * dt
        self.total_energy += energy_increment
        
        # Track overshoot
        if self.target_pos is not None:
            target_distance = np.sqrt((current_pos[0] - self.target_pos[0])**2 + 
                                    (current_pos[1] - self.target_pos[1])**2)
            if hasattr(self, 'min_distance_to_target'):
                if target_distance < self.min_distance_to_target:
                    self.min_distance_to_target = target_distance
                else:
                    overshoot = target_distance - self.min_distance_to_target
                    self.max_overshoot = max(self.max_overshoot, overshoot)
            else:
                self.min_distance_to_target = target_distance
        
        # Store history
        self.position_history.append(current_pos)
        self.velocity_history.append((v, omega))
        self.control_history.append((v_left, v_right))
        self.error_history.append(distance_error)
        self.time_history.append(current_time)
        
        self.previous_position = current_pos
        
    def target_reached(self):
        """Mark that target has been reached"""
        if self.target_reached_time is None and self.start_time is not None:
            self.target_reached_time = time.time() - self.start_time
            
    def get_metrics(self) -> PerformanceMetrics:
        """
        Calculate and return comprehensive performance metrics
        
        Returns:
            PerformanceMetrics object with all calculated metrics
        """
        if self.start_time is None:
            return PerformanceMetrics(0, 0, 0, 0, 0, 0, 0)
            
        # Arrival time
        arrival_time = self.target_reached_time or (time.time() - self.start_time)
        
        # Calculate straight-line distance for efficiency comparison
        if self.mission_start_pos and self.target_pos:
            straight_line_distance = np.sqrt(
                (self.target_pos[0] - self.mission_start_pos[0])**2 + 
                (self.target_pos[1] - self.mission_start_pos[1])**2
            )
            path_efficiency = straight_line_distance / max(self.total_path_length, 1e-6)
        else:
            path_efficiency = 1.0
            
        # Calculate settling time (time to stay within tolerance)
        settling_time = self._calculate_settling_time()
        
        # Calculate steady-state error
        steady_state_error = float(np.mean(list(self.error_history)[-10:])) if len(self.error_history) >= 10 else 0.0
        
        # Calculate total control effort
        control_effort = sum(abs(left) + abs(right) for left, right in self.control_history)
        
        return PerformanceMetrics(
            arrival_time=arrival_time,
            path_length=self.total_path_length,
            energy_consumption=self.total_energy,
            overshoot_distance=self.max_overshoot,
            settling_time=settling_time,
            steady_state_error=steady_state_error,
            control_effort=control_effort
        )
        
    def _calculate_settling_time(self, tolerance: float = 0.05) -> float:
        """Calculate time to settle within tolerance"""
        if len(self.error_history) < 10:
            return 0.0
            
        # Find the last time error exceeded tolerance
        last_violation_idx = -1
        for i, error in enumerate(reversed(self.error_history)):
            if abs(error) > tolerance:
                last_violation_idx = len(self.error_history) - 1 - i
                break
                
        if last_violation_idx == -1:
            return 0.0  # Always within tolerance
            
        return self.time_history[-1] - self.time_history[last_violation_idx]
        
    def plot_performance(self, save_path: Optional[str] = None):
        """
        Generate comprehensive performance plots
        
        Args:
            save_path: Optional path to save the plot
        """
        if len(self.position_history) < 2:
            return
            
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle('Robot Performance Analysis', fontsize=16)
        
        # Convert deque to lists for plotting
        positions = list(self.position_history)
        velocities = list(self.velocity_history)
        controls = list(self.control_history)
        errors = list(self.error_history)
        times = list(self.time_history)
        
        # 1. Trajectory plot
        x_pos = [pos[0] for pos in positions]
        y_pos = [pos[1] for pos in positions]
        axes[0, 0].plot(x_pos, y_pos, 'b-', linewidth=2, label='Actual Path')
        if self.mission_start_pos:
            axes[0, 0].plot(self.mission_start_pos[0], self.mission_start_pos[1], 'go', markersize=8, label='Start')
        if self.target_pos:
            axes[0, 0].plot(self.target_pos[0], self.target_pos[1], 'ro', markersize=8, label='Target')
        axes[0, 0].set_xlabel('X Position (m)')
        axes[0, 0].set_ylabel('Y Position (m)')
        axes[0, 0].set_title('Robot Trajectory')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        axes[0, 0].axis('equal')
        
        # 2. Error vs Time
        axes[0, 1].plot(times, errors, 'r-', linewidth=2)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Distance Error (m)')
        axes[0, 1].set_title('Distance Error Over Time')
        axes[0, 1].grid(True)
        
        # 3. Velocities vs Time
        linear_vels = [vel[0] for vel in velocities]
        angular_vels = [vel[1] for vel in velocities]
        axes[0, 2].plot(times, linear_vels, 'b-', label='Linear Velocity', linewidth=2)
        axes[0, 2].plot(times, angular_vels, 'r-', label='Angular Velocity', linewidth=2)
        axes[0, 2].set_xlabel('Time (s)')
        axes[0, 2].set_ylabel('Velocity')
        axes[0, 2].set_title('Robot Velocities')
        axes[0, 2].legend()
        axes[0, 2].grid(True)
        
        # 4. Control signals
        left_controls = [ctrl[0] for ctrl in controls]
        right_controls = [ctrl[1] for ctrl in controls]
        axes[1, 0].plot(times, left_controls, 'b-', label='Left Wheel', linewidth=2)
        axes[1, 0].plot(times, right_controls, 'r-', label='Right Wheel', linewidth=2)
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Wheel Velocity (m/s)')
        axes[1, 0].set_title('Control Signals')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # 5. Performance metrics summary
        metrics = self.get_metrics()
        metrics_text = f"""
        Arrival Time: {metrics.arrival_time:.2f} s
        Path Length: {metrics.path_length:.2f} m
        Energy: {metrics.energy_consumption:.2f}
        Max Overshoot: {metrics.overshoot_distance:.3f} m
        Settling Time: {metrics.settling_time:.2f} s
        Steady State Error: {metrics.steady_state_error:.3f} m
        Control Effort: {metrics.control_effort:.2f}
        """
        axes[1, 1].text(0.1, 0.5, metrics_text, fontsize=10, verticalalignment='center',
                        transform=axes[1, 1].transAxes)
        axes[1, 1].set_title('Performance Metrics')
        axes[1, 1].axis('off')
        
        # 6. Energy consumption over time
        if len(controls) > 0:
            energy_over_time = np.cumsum([abs(ctrl[0]) + abs(ctrl[1]) for ctrl in controls]) * (times[1] - times[0] if len(times) > 1 else 0.016)
            axes[1, 2].plot(times, energy_over_time, 'g-', linewidth=2)
            axes[1, 2].set_xlabel('Time (s)')
            axes[1, 2].set_ylabel('Cumulative Energy')
            axes[1, 2].set_title('Energy Consumption')
            axes[1, 2].grid(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.show()

class OptimizationSuggestions:
    """
    Analyze performance metrics and suggest optimizations
    """
    
    @staticmethod
    def analyze_and_suggest(metrics: PerformanceMetrics, controller_type: str) -> List[str]:
        """
        Analyze performance metrics and return optimization suggestions
        
        Args:
            metrics: PerformanceMetrics object
            controller_type: Type of controller being used
            
        Returns:
            List of optimization suggestions
        """
        suggestions = []
        
        # Analyze arrival time
        if metrics.arrival_time > 10.0:
            suggestions.append("🐌 Slow arrival time detected. Consider increasing proportional gain (Kp) for faster response.")
        
        # Analyze path efficiency
        if metrics.path_length > 0:
            # Assume we know the straight-line distance somehow, or this would be passed in
            suggestions.append("📏 Consider implementing path planning for more direct routes.")
        
        # Analyze energy consumption
        if metrics.energy_consumption > 50.0:
            suggestions.append("⚡ High energy consumption. Consider reducing maximum velocities or using smoother control.")
        
        # Analyze overshoot
        if metrics.overshoot_distance > 0.1:
            suggestions.append("🎯 Significant overshoot detected. Consider increasing derivative gain (Kd) for better damping.")
        
        # Analyze settling time
        if metrics.settling_time > 5.0:
            suggestions.append("⏰ Long settling time. Consider tuning integral gain (Ki) or tightening tolerances.")
        
        # Analyze steady-state error
        if abs(metrics.steady_state_error) > 0.05:
            suggestions.append("🎚️ Steady-state error present. Consider increasing integral gain (Ki).")
        
        # Controller-specific suggestions
        if controller_type == "P Controller":
            suggestions.append("🔧 P Controller active: Consider upgrading to PD for better stability or PID for zero steady-state error.")
        elif controller_type == "PD Controller":
            suggestions.append("🔧 PD Controller active: Consider adding integral term (PID) to eliminate steady-state error.")
        
        if not suggestions:
            suggestions.append("✅ Performance looks good! Consider fine-tuning parameters for specific scenarios.")
            
        return suggestions