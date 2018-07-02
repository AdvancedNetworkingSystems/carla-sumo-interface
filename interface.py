#!/usr/bin/env python

"""A Carla - SUMO interface"""

# Carla dependecies
from __future__ import print_function

import argparse
import logging
import random
import time

from carla.client import make_carla_client
from carla.sensor import Camera
from carla.settings import CarlaSettings
from carla.tcp import TCPConnectionError
from carla.util import print_over_same_line

# Plexe dependencies
import os
import random
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
from tools.utils import start_sumo, running

# Interface dependencies
import json
import math
from collections import namedtuple
import tools.CarlaGame as CG
import subprocess
import signal

# Global variables
Parameters = namedtuple(
    'Parameters', 'x_multiplier x_correction y_multiplier y_correction angle_correction')
Position = namedtuple(
    'Position', 'location_x location_y rotation_yaw')

vehicle_tot = 5
spawned_tot = 0
id_list = []


def read_parameters(map_name):
    x_multiplier = 1
    x_correction = 0
    y_multiplier = 1
    y_correction = 0
    angle_correction = 0

    if map_name is not None:
        data = json.load(open('./maps/' + map_name +
                              '/' + map_name + '.conf.json'))

        if "x_multiplier" in data:
            x_multiplier = data["x_multiplier"]

        if "x_correction" in data:
            x_correction = data["x_correction"]

        if "y_multiplier" in data:
            y_multiplier = data["y_multiplier"]

        if "y_correction" in data:
            y_correction = data["y_correction"]

        if "angle_correction" in data:
            angle_correction = data["angle_correction"]

    print('Read ', x_multiplier, x_correction,
          y_multiplier, y_correction, angle_correction)
    return Parameters(x_multiplier, x_correction, y_multiplier, y_correction, angle_correction)


def start_simulation(map_name, connect_to_instance, sumo_port):
    if connect_to_instance:
        traci.init(port=sumo_port)
    else:
        start_sumo('./maps/' + map_name + '/' + map_name + '.sumo.cfg', False)
    print('SUMO started')

    user_vehicle_length = 4.67999982834
    user_vehicle_width = 1.879999995
    add_vehicle("p0", user_vehicle_length, user_vehicle_width, 0, 0, 25)
    traci.gui.trackVehicle("View #0", "p0")
    traci.gui.setZoom("View #0", 1000)


def run(game, params):
    step = 0
    demo_mode = True
    while running(demo_mode, step, 100):
        step += 1

        # Read the data produced by the server this frame.
        measurements = game.execution_step()

        # Print some of the measurements.
        print_measurements(measurements)

        # Get the vehicles' position
        my_m = measurements.player_measurements.transform
        my_x = measurements.player_measurements.bounding_box.extent.x
        pos = Position(
            my_m.location.x,
            my_m.location.y,
            my_m.rotation.yaw)
        pos = getAdjustedPosition(pos, params, my_x)    # Correct the position
        moveVehicle("p0", pos)

        for agent in measurements.non_player_agents:    # Do the same for the other agents
            a_id = str(agent.id)  # unique id of the agent

            if agent.HasField('vehicle'):
                non_p_m = agent.vehicle.transform
                non_p_x = agent.vehicle.bounding_box.extent.x
                pos = Position(
                    non_p_m.location.x,
                    non_p_m.location.y,
                    non_p_m.rotation.yaw)
                pos = getAdjustedPosition(pos, params, non_p_x)

                # Check if the vehicle is spawned in SUMO
                if (not checkVehicleSpawned(a_id)):
                    global spawned_tot
                    global id_list
                    id_list.append(a_id)
                    spawned_tot += 1

                    non_p_y = agent.vehicle.bounding_box.extent.y
                    add_vehicle(a_id, non_p_x*2, non_p_y*2, 0, 0, 25)    # Spawn the vehicle in SUMO

                moveVehicle(a_id, pos)

        traci.simulationStep()


def checkVehicleSpawned(v_id):
    global vehicle_tot
    global spawned_tot
    global id_list

    if spawned_tot == vehicle_tot:
        return True

    for a_id in id_list:
        if v_id == a_id:
            return True

    return False


def add_vehicle(vid, length, width, position, lane, speed, vtype="vtypeauto"):
    traci.vehicle.add(vid, "platoon_route", pos=position, speed=speed, lane=lane, typeID=vtype)
    traci.vehicle.setColor(vid, (random.uniform(0, 255),
                                 random.uniform(0, 255),
                                 random.uniform(0, 255), 255))
    traci.vehicle.setLength(vid, length)
    traci.vehicle.setWidth(vid, width)


def getAdjustedPosition(position, params, car_dimen):
    yaw = position.rotation_yaw + params.angle_correction
    radians = yaw / 180 * math.pi
    pos_x = params.x_multiplier * position.location_x + params.x_correction + \
        math.sin(
            radians)*car_dimen  # in Carla the agent's position is the center of the car
    pos_y = params.y_multiplier * position.location_y + params.y_correction + \
        math.cos(radians)*car_dimen  # in Sumo it's the front bumper
    return Position(pos_x, pos_y, yaw)


def moveVehicle(vID, position):
    traci.vehicle.moveToXY(
        vehID=vID,
        edgeID="",
        lane=0,
        x=position.location_x,
        y=position.location_y,
        angle=position.rotation_yaw,
        keepRoute=2)


def print_measurements(measurements):
    non_player_measurements = measurements.non_player_agents
    number_of_agents = len(non_player_measurements)

    player_measurements = measurements.player_measurements
    message = 'Vehicle at ({pos_x:.1f}, {pos_y:.1f}), '
    message += 'roll {roll:.1f}, pitch {pitch:.1f}, yaw {yaw:.1f}, '
    message += '{speed:.2f} m/s, '
    message += 'Collision: {{vehicles={col_cars:.0f}, pedestrians={col_ped:.0f}, other={col_other:.0f}}}, '
    message += '{other_lane:.0f}% other lane, {offroad:.0f}% off-road, '
    message += '({agents_num:d} agents)'
    message = message.format(
        pos_x=player_measurements.transform.location.x,
        pos_y=player_measurements.transform.location.y,
        roll=player_measurements.transform.rotation.roll,
        pitch=player_measurements.transform.rotation.pitch,
        yaw=player_measurements.transform.rotation.yaw,
        speed=player_measurements.forward_speed,
        col_cars=player_measurements.collision_vehicles,
        col_ped=player_measurements.collision_pedestrians,
        col_other=player_measurements.collision_other,
        other_lane=100 * player_measurements.intersection_otherlane,
        offroad=100 * player_measurements.intersection_offroad,
        agents_num=number_of_agents)
    print_over_same_line(message)


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host server (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Epic',
        help='Graphics quality level, a lower level makes the simulation run considerably faster')
    argparser.add_argument(
        '-c', '--camera',
        choices=['Default', 'Driver', 'Hood', 'Top'],
        default='Default',
        help='Position of the camera relative to the car')
    argparser.add_argument(
        '-m', '--map-name',
        metavar='M',
        default='town03',
        help='The name of the folder in maps/ and of the files inside of it that the script should use. '
        + 'Inside the folder there should be a .conf.json file with the adjustements for the data from Carla to SUMO, a .sumo.cfg, a .rou.xml and a .net.xml file')
    argparser.add_argument(
        '--carla-shell',
        choices=['Default', 'Yes', 'Nocarla'],
        default='Default',
        help='Tell the script how to execute CARLA. If you choose the external shell, you will have to close it manually')
    argparser.add_argument(
        '--sumo-running',
        action='store_true',
        help='If set, the script will try to connect to an already opened SUMO instance')
    argparser.add_argument(
        '--sumo-port',
        default="8813",
        help='If --sumo-running is set, this will be the port on which the script will try to connect. Default is 8813')
    args = argparser.parse_args()

    # Open CARLA
    CARLA_PATH = os.environ['CARLA_PATH']
    print("CARLA_PATH is", CARLA_PATH)

    if args.carla_shell != "Nocarla":
        if args.carla_shell == "Yes":
            cmd_line = ['gnome-terminal', '-x', CARLA_PATH + '/CarlaUE4.sh', '/Game/Maps/' + args.map_name,
                        '-windowed', '-ResX=800' '-ResY=600', '-carla-settings=tools/CarlaSettings.ini']
        else:
            cmd_line = [CARLA_PATH + '/CarlaUE4.sh', '/Game/Maps/' + args.map_name,
                        '-windowed', '-ResX=800' '-ResY=600', '-carla-settings=tools/CarlaSettings.ini']

        carla_process = subprocess.Popen(
            cmd_line, stdout=subprocess.PIPE, preexec_fn=os.setsid)

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    logging.info('listening to server %s:%s', args.host, args.port)

    carla_connected = False
    traci_connected = False
    try:
        while not carla_connected:
            try:
                # We assume the CARLA server is already waiting for a client to connect at
                # host:port. To create a connection we can use the `make_carla_client`
                # context manager, it creates a CARLA client object and starts the
                # connection. It will throw an exception if something goes wrong. The
                # context manager makes sure the connection is always cleaned up on exit.
                with make_carla_client(args.host, args.port) as client:
                    print('CarlaClient connected')

                    game = CG.CarlaGame(client, args, vehicle_tot)

                    params = read_parameters(args.map_name)

                    game.initialize()

                    carla_connected = True

                    start_simulation(
                        args.map_name, args.sumo_running, int(args.sumo_port))

                    traci_connected = True

                    run(game, params)

                    print('Run ended')

            except TCPConnectionError as error:
                logging.error(error)
                time.sleep(1)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    finally:
        if carla_connected:
            print('Closing connections')
            try:
                game.finish()
            except Exception as error:
                logging.error(error)

            if traci_connected:
                try:
                    traci.close()
                except Exception as error:
                    logging.error(error)

            # Chiudi CARLA
            if args.carla_shell == "Default":
                os.killpg(os.getpgid(carla_process.pid), signal.SIGINT)

    print('Done.')


if __name__ == '__main__':
    main()
