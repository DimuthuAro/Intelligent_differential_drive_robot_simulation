from simulation_state import SimulationState
from differential_drive_robot import DifferentialDriveRobot
from controllers import RobotController, ControllerType
from performance_monitor import PerformanceMonitor, OptimizationSuggestions
import time , math , pygame as pg
from config import *

class MainState(SimulationState):
    def __init__(self):
        super().__init__()
        
        # Coordinate conversion parameters
        self.meters_to_pixels = 100  # 100 pixels per meter
        self.screen_center_x = SCREEN_WIDTH // 2
        self.screen_center_y = SCREEN_HEIGHT // 2
        
        # Initialize robot at screen center in world coordinates
        robot_world_x = 0  # meters
        robot_world_y = 0  # meters
        self.robot = DifferentialDriveRobot(robot_world_x, robot_world_y, ROBOT_STARTING_ANGLE)
        
        self.controller_type = ControllerType.PID
        self.controller = RobotController(self.controller_type)
        
        # Performance monitoring
        self.performance_monitor = PerformanceMonitor()
        self.show_performance = False
        self.last_target_pos = None
        
        # Path visualization
        self.show_path = True
        self.path_points = []  # Store screen coordinates of robot path
        self.max_path_points = 500  # Limit path length for performance
        
        # Visualization Parameters
        self.robot_position = self.robot.get_position()
        self.destination_position = (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
        self.robot_radius = 20 # pixels
        self.destination_radius = 10 # pixels
        self.font = pg.font.SysFont("Arial", 16)
        self.start_button: pg.Rect = pg.Rect(10, SCREEN_HEIGHT - 30, 60, 20)
        self.stop_button: pg.Rect = pg.Rect(80, SCREEN_HEIGHT - 30, 60, 20)
        self.p_button: pg.Rect = pg.Rect(150, SCREEN_HEIGHT - 30, 40, 20)
        self.pd_button: pg.Rect = pg.Rect(200, SCREEN_HEIGHT - 30, 50, 20)
        self.pid_button: pg.Rect = pg.Rect(260, SCREEN_HEIGHT - 30, 50, 20)
        self.perf_button: pg.Rect = pg.Rect(320, SCREEN_HEIGHT - 30, 80, 20)
        self.analyze_button: pg.Rect = pg.Rect(410, SCREEN_HEIGHT - 30, 70, 20)
        self.path_button: pg.Rect = pg.Rect(490, SCREEN_HEIGHT - 30, 60, 20)
        self.clear_button: pg.Rect = pg.Rect(560, SCREEN_HEIGHT - 30, 50, 20)    
    
    def handle_event(self, event):
        super().handle_event(event)
        mouse_position = pg.mouse.get_pos()
        if event.type == pg.KEYDOWN:
            if event.key == pg.K_t:  # Press 'T' key to test movement
                self._test_robot_movement()
        elif event.type == pg.MOUSEBUTTONDOWN:
            if mouse_position[0] >= 0 and mouse_position[0] <= SCREEN_WIDTH and \
                mouse_position[1] >= 0 and mouse_position[1] <= SCREEN_HEIGHT - 40:
                    mouse_x, mouse_y = event.pos
                    self.destination_position = (mouse_x, mouse_y)
                    world_x, world_y = self._screen_to_world(mouse_x, mouse_y)
                    # Clear path when new destination is set
                    self.path_points.clear()
                    print(f"New destination set to: {self.destination_position} (screen) / ({world_x:.2f}, {world_y:.2f}) (world)")
                    print("Path cleared for new destination")
            elif self.stop_button.collidepoint(mouse_position):
                    print("Stop button pressed")
                    self.robot.set_wheel_velocities(0.0, 0.0)
                    self.robot.update(0)  # Update robot state immediately to stop
                    self.controller.linear_controller.reset()
                    self.controller.angular_controller.reset()
                    self.robot_position = self.robot.get_position()
            elif self.start_button.collidepoint(mouse_position):
                    print("Start button pressed")
                    # Reset robot position and controller states
                    self.robot = DifferentialDriveRobot(0, 0, ROBOT_STARTING_ANGLE)  # Reset to world center
                    self.controller.linear_controller.reset()
                    self.controller.angular_controller.reset()
                    self.robot_position = self.robot.get_position()
                    # Clear path on reset
                    self.path_points.clear()
            elif self.p_button.collidepoint(mouse_position):
                    print("P Controller selected")
                    self._switch_controller(ControllerType.P)
            elif self.pd_button.collidepoint(mouse_position):
                    print("PD Controller selected")
                    self._switch_controller(ControllerType.PD)
            elif self.pid_button.collidepoint(mouse_position):
                    print("PID Controller selected")
                    self._switch_controller(ControllerType.PID)
            elif self.perf_button.collidepoint(mouse_position):
                    print("Performance monitoring toggled")
                    self.show_performance = not self.show_performance
            elif self.analyze_button.collidepoint(mouse_position):
                    print("Analyzing performance...")
                    self._analyze_performance()
            elif self.path_button.collidepoint(mouse_position):
                    print("Path visualization toggled")
                    self.show_path = not self.show_path
            elif self.clear_button.collidepoint(mouse_position):
                    print("Path cleared")
                    self.path_points.clear()
                
    def _world_to_screen(self, world_x, world_y):
        """Convert world coordinates (meters) to screen coordinates (pixels)"""
        screen_x = self.screen_center_x + world_x * self.meters_to_pixels
        screen_y = self.screen_center_y - world_y * self.meters_to_pixels  # Flip Y axis
        return int(screen_x), int(screen_y)
    
    def _screen_to_world(self, screen_x, screen_y):
        """Convert screen coordinates (pixels) to world coordinates (meters)"""
        world_x = (screen_x - self.screen_center_x) / self.meters_to_pixels
        world_y = -(screen_y - self.screen_center_y) / self.meters_to_pixels  # Flip Y axis
        return world_x, world_y
    
    def _switch_controller(self, controller_type):
        """Switch to a different controller type"""
        if self.controller_type != controller_type:
            self.controller_type = controller_type
            self.controller = RobotController(controller_type)
            # Reset performance monitoring when switching controllers
            self.performance_monitor = PerformanceMonitor()
            print(f"Switched to {controller_type.value}")
    
    def _test_robot_movement(self):
        """Test basic robot movement - call this to debug"""
        print("Testing robot movement...")
        print(f"Initial position: {self.robot.get_position()}")
        
        # Test 1: Move straight forward
        print("Test 1: Moving straight (both wheels same speed)")
        self.robot.set_wheel_velocities(0.5, 0.5)
        for i in range(10):
            self.robot.update(0.1)  # 0.1 second steps
            pos = self.robot.get_position()
            print(f"  Step {i+1}: ({pos[0]:.3f}, {pos[1]:.3f}, {math.degrees(pos[2]):.1f}°)")
        
        # Reset
        self.robot = DifferentialDriveRobot(0, 0, 0)
        
        # Test 2: Turn in place
        print("Test 2: Turning in place (left wheel backward, right forward)")
        self.robot.set_wheel_velocities(-0.5, 0.5)
        for i in range(10):
            self.robot.update(0.1)
            pos = self.robot.get_position()
            print(f"  Step {i+1}: ({pos[0]:.3f}, {pos[1]:.3f}, {math.degrees(pos[2]):.1f}°)")
        
        self.robot.set_wheel_velocities(0, 0)  # Stop
        print("Movement test completed.")
    
    def _analyze_performance(self):
        """Analyze current performance and show suggestions"""
        metrics = self.performance_monitor.get_metrics()
        suggestions = OptimizationSuggestions.analyze_and_suggest(metrics, self.controller_type.value)
        
        print("\n" + "="*50)
        print("PERFORMANCE ANALYSIS")
        print("="*50)
        print(f"Arrival Time: {metrics.arrival_time:.2f} s")
        print(f"Path Length: {metrics.path_length:.2f} m")
        print(f"Energy Consumption: {metrics.energy_consumption:.2f}")
        print(f"Max Overshoot: {metrics.overshoot_distance:.3f} m")
        print(f"Settling Time: {metrics.settling_time:.2f} s")
        print(f"Steady State Error: {metrics.steady_state_error:.3f} m")
        print(f"Control Effort: {metrics.control_effort:.2f}")
        print("\nOPTIMIZATION SUGGESTIONS:")
        for i, suggestion in enumerate(suggestions, 1):
            print(f"{i}. {suggestion}")
        print("="*50 + "\n")
        
        # Generate performance plots
        try:
            self.performance_monitor.plot_performance()
        except Exception as e:
            print(f"Could not generate performance plots: {e}")
    
    def _render_path(self, screen):
        """Render the robot's path trajectory"""
        if self.show_path and len(self.path_points) > 1:
            # Draw path as connected lines with gradient effect
            for i in range(len(self.path_points) - 1):
                start_point = self.path_points[i]
                end_point = self.path_points[i + 1]
                
                # Create gradient effect - older points are more transparent
                alpha = int(255 * (i + 1) / len(self.path_points))
                # Use green color for the path with varying alpha
                path_color = (*GREEN[:3], alpha) if alpha > 50 else (0, 150, 0)
                
                # Draw line segment (pygame doesn't support alpha for lines directly, so use solid color with intensity)
                intensity = max(50, alpha)
                color = (0, min(255, intensity), 0)
                pg.draw.line(screen, color, start_point, end_point, 2)
            
            # Draw small circles at path points to show discrete positions
            for i, point in enumerate(self.path_points[::5]):  # Show every 5th point to avoid clutter
                alpha = int(100 * (i * 5 + 1) / len(self.path_points))
                if alpha > 30:
                    color = (0, min(255, alpha + 50), 0)
                    pg.draw.circle(screen, color, point, 2)
    
    def _render_destination(self, screen):
        pg.draw.circle(screen, (255, 0, 0), (int(self.destination_position[0]), int(self.destination_position[1])), self.destination_radius)
        
    def _render_robot(self, screen):
        world_x, world_y, theta = self.robot_position
        screen_x, screen_y = self._world_to_screen(world_x, world_y)
        
        # Draw robot body
        pg.draw.circle(screen, (0, 0, 255), (screen_x, screen_y), self.robot_radius)
        
        # Draw robot direction indicator
        end_x = screen_x + int(self.robot_radius * 0.8 * math.cos(theta))
        end_y = screen_y - int(self.robot_radius * 0.8 * math.sin(theta))  # Flip Y for screen coordinates
        pg.draw.line(screen, (255, 255, 255), (screen_x, screen_y), (end_x, end_y), 3)
        
    def _render_statusbar(self, screen):
        controller_text = self.font.render(f"Controller: {self.controller_type.name}", True, BLACK)
        screen.blit(controller_text, (10, 10))
        
        # Show performance info if monitoring is enabled
        if self.show_performance:
            metrics = self.performance_monitor.get_metrics()
            perf_text = self.font.render(f"Time: {metrics.arrival_time:.1f}s | Path: {metrics.path_length:.1f}m | Energy: {metrics.energy_consumption:.1f}", True, BLACK)
            screen.blit(perf_text, (10, 30))
        
        # Show path info
        if self.show_path:
            path_info = self.font.render(f"Path Points: {len(self.path_points)} | Show Path: ON", True, DARK_GRAY)
            screen.blit(path_info, (10, 50))

    def _render_commandbar(self, screen):
        #Render AT Bottom CommandBar Background
        pg.draw.rect(screen, GRAY, (0, SCREEN_HEIGHT - 40, SCREEN_WIDTH, 40))
        
        # Render Start Button
        pg.draw.rect(screen, GREEN, self.start_button)
        start_text = self.font.render("Start", True, BLACK)
        screen.blit(start_text, (22, SCREEN_HEIGHT - 28))

        # Render Stop Button
        pg.draw.rect(screen, RED, self.stop_button)
        stop_text = self.font.render("Stop", True, BLACK)
        screen.blit(stop_text, (98, SCREEN_HEIGHT - 28))
        
        # Render Controller Toggle Buttons
        # P Controller Button
        p_color = YELLOW if self.controller_type == ControllerType.P else WHITE
        pg.draw.rect(screen, p_color, self.p_button)
        pg.draw.rect(screen, BLACK, self.p_button, 2)  # Border
        p_text = self.font.render("P", True, BLACK)
        screen.blit(p_text, (165, SCREEN_HEIGHT - 28))
        
        # PD Controller Button
        pd_color = YELLOW if self.controller_type == ControllerType.PD else WHITE
        pg.draw.rect(screen, pd_color, self.pd_button)
        pg.draw.rect(screen, BLACK, self.pd_button, 2)  # Border
        pd_text = self.font.render("PD", True, BLACK)
        screen.blit(pd_text, (215, SCREEN_HEIGHT - 28))
        
        # PID Controller Button
        pid_color = YELLOW if self.controller_type == ControllerType.PID else WHITE
        pg.draw.rect(screen, pid_color, self.pid_button)
        pg.draw.rect(screen, BLACK, self.pid_button, 2)  # Border
        pid_text = self.font.render("PID", True, BLACK)
        screen.blit(pid_text, (275, SCREEN_HEIGHT - 28))
        
        # Performance Monitor Button
        perf_color = YELLOW if self.show_performance else WHITE
        pg.draw.rect(screen, perf_color, self.perf_button)
        pg.draw.rect(screen, BLACK, self.perf_button, 2)  # Border
        perf_text = self.font.render("Monitor", True, BLACK)
        screen.blit(perf_text, (335, SCREEN_HEIGHT - 28))
        
        # Analyze Button
        pg.draw.rect(screen, WHITE, self.analyze_button)
        pg.draw.rect(screen, BLACK, self.analyze_button, 2)  # Border
        analyze_text = self.font.render("Analyze", True, BLACK)
        screen.blit(analyze_text, (420, SCREEN_HEIGHT - 28))
        
        # Path Toggle Button
        path_color = LIGHT_GREEN if self.show_path else WHITE
        pg.draw.rect(screen, path_color, self.path_button)
        pg.draw.rect(screen, BLACK, self.path_button, 2)  # Border
        path_text = self.font.render("Path", True, BLACK)
        screen.blit(path_text, (510, SCREEN_HEIGHT - 28))
        
        # Clear Path Button
        pg.draw.rect(screen, ORANGE, self.clear_button)
        pg.draw.rect(screen, BLACK, self.clear_button, 2)  # Border
        clear_text = self.font.render("Clear", True, BLACK)
        screen.blit(clear_text, (570, SCREEN_HEIGHT - 28))

    def render(self, screen):
        super().render(screen)
        self._render_path(screen)  # Draw path first so robot appears on top
        self._render_robot(screen)
        self._render_destination(screen)
        self._render_statusbar(screen)
        self._render_commandbar(screen)
        
    def update(self, dt):
        super().update(dt)
        # Convert destination from screen to world coordinates for control
        dest_world_x, dest_world_y = self._screen_to_world(self.destination_position[0], self.destination_position[1])
        
        # Get current robot state for debugging
        current_x, current_y, current_theta = self.robot.get_position()
        
        # Calculate errors for debugging
        distance_error = self.robot.distance_to_target(dest_world_x, dest_world_y)
        angle_error = self.robot.angle_to_target(dest_world_x, dest_world_y)
        
        v_left, v_right = self.controller.compute_control(self.robot, dest_world_x, dest_world_y, dt)
        
        # Performance monitoring
        target_changed = self.last_target_pos != (dest_world_x, dest_world_y)
        if target_changed:
            self.performance_monitor.start_mission((current_x, current_y), (dest_world_x, dest_world_y))
            self.last_target_pos = (dest_world_x, dest_world_y)
        
        self.performance_monitor.update(self.robot, v_left, v_right, distance_error, dt)
        
        # Check if target reached for performance monitoring
        if distance_error < self.controller.tolerance:
            self.performance_monitor.target_reached()
        
        # Debug prints - show only when robot is turning or near target
        if self.show_performance and (distance_error < 0.5 or abs(angle_error) > math.pi/6):
            print(f"Robot: ({current_x:.2f}, {current_y:.2f}, {math.degrees(current_theta):.1f}°) → Target: ({dest_world_x:.2f}, {dest_world_y:.2f}) | Dist: {distance_error:.2f}m, Angle: {math.degrees(angle_error):.1f}°")
        
        self.robot.set_wheel_velocities(v_left, v_right)
        self.robot.update(dt)
        self.robot_position = self.robot.get_position()
        
        # Update path tracking
        current_screen_pos = self._world_to_screen(current_x, current_y)
        
        # Add point to path if robot has moved significantly (avoid cluttering)
        if not self.path_points or \
           (abs(current_screen_pos[0] - self.path_points[-1][0]) > 3 or 
            abs(current_screen_pos[1] - self.path_points[-1][1]) > 3):
            self.path_points.append(current_screen_pos)
            
            # Limit path length for performance
            if len(self.path_points) > self.max_path_points:
                self.path_points.pop(0)

