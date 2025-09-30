import pygame as pg
import numpy as np
import matplotlib.pyplot as plt
import math
import time

from controllers import ControllerType, RobotController
from differential_drive_robot import DifferentialDriveRobot

from config import BLACK, WHITE, RED, GREEN, BLUE, GRAY, YELLOW
from config import SCREEN_WIDTH, SCREEN_HEIGHT, TITLE
from config import ROBOT_STARTING_POS, ROBOT_STARTING_ANGLE

from simulation_state import SimulationState
    
# Simulation State Manager - A Finite State Machine (FSM)
class SimulationStateManager:
    def __init__(self, initial_state: SimulationState) -> None:
        self.current_state = initial_state
        self.current_state.__init__()
        
    def set_current_state(self, new_state: SimulationState) -> None:
        if self.current_state is not None:
            self.current_state.exit_state()
            
        self.current_state = new_state
        self.current_state.__init__()

    def handle_event(self, event) -> None:
        if self.current_state is not None:
            self.current_state.handle_event(event)
            
    def update(self, dt) -> None:
        if self.current_state is not None:
            self.current_state.update(dt)

    def render(self, screen) -> None:
        if self.current_state is not None:
            self.current_state.render(screen)

from main_state import MainState
from config import SIMULATED_SECOND, FPS, BACKGROUND_COLOR

# Interractive System
pg.init()
screen : pg.Surface = pg.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pg.display.set_caption(TITLE)
clock = pg.time.Clock()
main_state : SimulationState = MainState()
states_manager = SimulationStateManager(main_state)

def handle_events():
    for event in pg.event.get():
        if event.type == pg.QUIT:
            global running
            running = False
        else:
            states_manager.handle_event(event)
            
def update():
    dt: float = clock.tick(FPS) / SIMULATED_SECOND  # Convert to seconds
    states_manager.update(dt)

def render():
    screen.fill(BACKGROUND_COLOR)
    states_manager.render(screen)
    pg.display.flip()
    
# Main simulation loop
running = True
while running:
    handle_events()
    update()
    render()

pg.quit()