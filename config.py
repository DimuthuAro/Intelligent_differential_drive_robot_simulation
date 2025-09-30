# Configuration settings for the robot simulation

# TIME DEFINITIONS
SIMULATED_SECOND = 1000 # milliseconds in a simulated second
FPS = 60  # Frames per second

# COLOR DEFINITIONS
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (200, 200, 200)
YELLOW = (255, 255, 0)

# Extended colors for advanced features
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
LIGHT_BLUE = (173, 216, 230)
LIGHT_GREEN = (144, 238, 144)
DARK_GRAY = (169, 169, 169)
LIGHT_GRAY = (211, 211, 211)

# Path visualization colors
PATH_COLOR = (0, 200, 0)  # Bright green for path
PATH_FADE_COLOR = (0, 100, 0)  # Darker green for fading effect
WAYPOINT_COLOR = (255, 100, 100)  # Light red for waypoints

# Version info
MAJOR_VERSION = 1
MINOR_VERSION = 0
PATCH_VERSION = 0
VERSION = f"{MAJOR_VERSION}.{MINOR_VERSION}.{PATCH_VERSION}"

# Window title
TITLE = f"RIS Assignment -- Differential Drive Robot Simulation {VERSION}"

# Background color
BACKGROUND_COLOR = WHITE

# Screen dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Visualization parameters
ROBOT_STARTING_POS = (0, 0)  # Meters
ROBOT_STARTING_ANGLE = 0  # Radians

# Performance optimization settings
OPTIMIZED_RENDERING = True  # Enable rendering optimizations
TRAJECTORY_MAX_POINTS = 1000  # Limit trajectory points for memory
PERFORMANCE_MONITORING = True  # Enable performance monitoring

# Controller optimization parameters
OPTIMIZED_PID_GAINS = {
    'P': {'linear_kp': 1.5, 'angular_kp': 4.0},
    'PD': {'linear_kp': 2.5, 'linear_kd': 0.8, 'angular_kp': 4.5, 'angular_kd': 1.2},
    'PID': {'linear_kp': 3.0, 'linear_ki': 0.05, 'linear_kd': 0.7,
            'angular_kp': 5.0, 'angular_ki': 0.02, 'angular_kd': 1.0}
}

# Adaptive control settings
ADAPTIVE_CONTROL_ENABLED = True
ADAPTATION_RATE = 0.1
MIN_GAIN_FACTOR = 0.5
MAX_GAIN_FACTOR = 2.0

# Robot physical constraints (optimized)
MAX_LINEAR_VELOCITY = 3.0  # m/s
MAX_ANGULAR_VELOCITY = 5.0  # rad/s
MAX_ACCELERATION = 5.0  # m/s^2
WHEEL_BASE = 0.3  # meters

# Precision settings
POSITION_TOLERANCE = 0.03  # meters (tighter tolerance)
ANGLE_TOLERANCE = 0.1  # radians (~5.7 degrees)
WAYPOINT_TOLERANCE = 0.1  # meters