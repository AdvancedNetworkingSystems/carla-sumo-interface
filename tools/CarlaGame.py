#!/usr/bin/env python

from __future__ import print_function

import argparse
import logging
import random
import time

from carla import image_converter
from carla import sensor
from carla.client import make_carla_client, VehicleControl
from carla.planner.map import CarlaMap
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line

try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError(
        'cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')


WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600

def make_carla_settings(args, vehicle_tot):
    # Create a CarlaSettings object. This object is a wrapper around
    # the CarlaSettings.ini file. Here we set the configuration we
    # want for the new episode.
    settings = CarlaSettings()
    settings.set(
        SynchronousMode=False,
        SendNonPlayerAgentsInfo=True,
        NumberOfVehicles=vehicle_tot,
        NumberOfPedestrians=0,
        WeatherId=1,
        QualityLevel=args.quality_level)
    settings.randomize_seeds()

    camera0 = sensor.Camera('CameraRGB')
    camera0.set_image_size(WINDOW_WIDTH, WINDOW_HEIGHT)

    px, py, pz = 2.0, 0.0, 1.4      # Default (Front)
    rx, ry, rz = 0.0, 0.0, 0.0

    if args.camera == "Driver":
        px = -0.3
        py = -0.5
        pz = 1.1
    elif args.camera == "Hood":
        px = 1.0
        py = 0.0
        pz = 1.4
    elif args.camera == "Top":
        px = -5.5
        py = 0.0
        pz = 3.0
        rx = -10.0
    
    camera0.set_position(px, py, pz)
    camera0.set_rotation(rx, ry, rz)
    settings.add_sensor(camera0)

    return settings


class CarlaGame(object):
    def __init__(self, carla_client, args, vehicle_tot):
        self.client = carla_client
        self._carla_settings = make_carla_settings(args, vehicle_tot)
        self._display = None
        self._main_image = None
        self._is_on_reverse = False
        self._enable_autopilot = False
        self._position = None

    # INIT
    def initialize(self):
        """Launch the PyGame."""
        pygame.init()
        self._initialize_game()

    def _initialize_game(self):
        self._display = pygame.display.set_mode(
            (WINDOW_WIDTH, WINDOW_HEIGHT),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        logging.debug('pygame started')
        self._on_new_episode()

    def _on_new_episode(self):
        self._carla_settings.randomize_seeds()
        #self._carla_settings.randomize_weather()
        scene = self.client.load_settings(self._carla_settings)
        number_of_player_starts = len(scene.player_start_spots)
        player_start = np.random.randint(number_of_player_starts)
        print('Starting new episode...')
        self.client.start_episode(player_start)
        self._is_on_reverse = False

    # EXECUTING
    def execution_step(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
        measurements = self._on_loop()
        self._on_render()

        return measurements

    def _on_loop(self):

        measurements, sensor_data = self.client.read_data()

        self._main_image = sensor_data.get('CameraRGB', None)

        control = self._get_keyboard_control(pygame.key.get_pressed())

        #if control is None:
        #    self._on_new_episode()
        if self._enable_autopilot:
            self.client.send_control(
                measurements.player_measurements.autopilot_control)
        else:
            self.client.send_control(control)

        return measurements

    def _get_keyboard_control(self, keys):
        """
        Return a VehicleControl message based on the pressed keys. Return None
        if a new episode was requested.
        """
        #if keys[K_r]:
        #    return None
        control = VehicleControl()
        if keys[K_LEFT] or keys[K_a]:
            control.steer = -1.0
        if keys[K_RIGHT] or keys[K_d]:
            control.steer = 1.0
        if keys[K_UP] or keys[K_w]:
            control.throttle = 1.0
        if keys[K_DOWN] or keys[K_s]:
            control.brake = 1.0
        if keys[K_SPACE]:
            control.hand_brake = True
        if keys[K_q]:
            self._is_on_reverse = not self._is_on_reverse
        if keys[K_p]:
            self._enable_autopilot = not self._enable_autopilot
        control.reverse = self._is_on_reverse
        return control

    def _on_render(self):
        if self._main_image is not None:
            array = image_converter.to_rgb_array(self._main_image)
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self._display.blit(surface, (0, 0))

        pygame.display.flip()

    # FINISH
    def finish(self):
        pygame.quit
