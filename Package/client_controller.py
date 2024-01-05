from __future__ import print_function

import airsim
import os
import argparse
import logging
import random
import time
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
from model_predictive_control import MPCController
from PIL import Image
import io
import cv2

client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()
print('AirSimClient connected')


car_controls.steering = 0
car_controls.throttle = 0
car_controls.brake = 0

# I need to prepend `sys.path` with '..' to get to the carla module there.
# I'm pre-pending `sys.path` because there are other carla modules specified
# in PYTHONPATH already
import sys
sys.path = ['..'] + sys.path

# from carla.client import make_carla_client
# from carla.sensor import Camera, Lidar
# from carla.settings import CarlaSettings
# from carla.tcp import TCPConnectionError
vehicle_name = "PhysXCar"
STEER_BOUND = 1.0
STEER_BOUNDS = (-STEER_BOUND, STEER_BOUND)
IMAGE_SIZE = (256, 144)
IMAGE_DECIMATION = 4
MIN_SPEED = 5
DTYPE = 'float32'
STEER_NOISE = lambda: random.uniform(-0.1, 0.1)
THROTTLE_NOISE = lambda: random.uniform(-0.05, 0.05)
STEER_NOISE_NN = lambda: 0 #random.uniform(-0.05, 0.05)
THROTTLE_NOISE_NN = lambda: 0 #random.uniform(-0.05, 0.05)
IMAGE_CLIP_LOWER = IMAGE_SIZE[0]
IMAGE_CLIP_UPPER = 0

def clip_throttle(throttle, curr_speed, target_speed):
    return np.clip(
        throttle - 0.01 * (curr_speed-target_speed),
        0.4,
        0.9
    )

def print_measurements(measurements):
    number_of_agents = len(measurements.non_player_agents)
    player_measurements = measurements.player_measurements
    message = 'Vehicle at ({pos_x:.1f}, {pos_y:.1f}), '
    message += '{speed:.0f}, '
    message = message.format(
        pos_x=player_measurements.transform.location.x,
        pos_y=player_measurements.transform.location.y,
        speed=player_measurements.forward_speed, # m/s -> km/h
    )
    print_over_same_line(message)

def run_airsim_client(args):
    frames_per_episode = 10000
    spline_points = 10000

    report = {
        'num_episodes': args.num_episodes,
        'controller_name': args.controller_name,
        'distances': [],
        'target_speed': args.target_speed,
    }


    track_DF = pd.read_csv('path_data.txt', header=None)
    # The track data are rescaled by 100x with relation to Carla measurements
    pts_2D = track_DF.iloc[:, [0, 1]].values

    # # Generating parameter values for interpolation
    # spline_points = 100  # You can adjust this value based on your needs
    u = np.arange(pts_2D.shape[0])

    # Interpolating x and y coordinates separately
    f_x = interp1d(u, pts_2D[:, 0], kind='linear', fill_value='extrapolate')
    f_y = interp1d(u, pts_2D[:, 1], kind='linear', fill_value='extrapolate')

    # Generating new parameter values for interpolation
    u_new = np.linspace(u.min(), u.max(), spline_points)

    # Evaluating the interpolating functions at new parameter values
    x_new = f_x(u_new)
    y_new = f_y(u_new)

    # Updating the 2D points with interpolated values
    pts_2D = np.column_stack((x_new, y_new))

    car_controls.steering = 0.0
    car_controls.throttle = 0.0

    # depth_array = None

    if args.controller_name == 'mpc':
        weather_id = 2
        controller = MPCController(args.target_speed)
    episode = 0
    num_fails = 0

    while episode < args.num_episodes:
        # Start a new episode

        if args.store_data:
            depth_storage = np.zeros((
                (IMAGE_CLIP_LOWER-IMAGE_CLIP_UPPER) // IMAGE_DECIMATION,
                IMAGE_SIZE[1] // IMAGE_DECIMATION,
                frames_per_episode
            )).astype(DTYPE)
            log_dicts = frames_per_episode * [None]
        else:
            depth_storage = None
            log_dicts = None

        status, depth_storage, one_log_dict, log_dicts, distance_travelled = run_episode(
            client,
            controller,
            pts_2D,
            depth_storage,
            log_dicts,
            frames_per_episode,
            args.controller_name,
            args.store_data
        )

        status = 'TRUE'

        if 'FAIL' in status:
            num_fails += 1
            print(status)
            continue
        else:
            print('SUCCESS: ' + str(episode))
            episode += 1

    report['num_fails'] = num_fails

    # report_output = os.path.join('reports', args.report_filename)
    # pd.to_pickle(report, report_output)


def run_episode(client, controller, pts_2D, depth_storage, log_dicts, frames_per_episode, controller_name, store_data):
    num_laps = 0
    curr_closest_waypoint = None
    prev_closest_waypoint = None
    num_waypoints = pts_2D.shape[0]
    # num_steps_below_min_speed = 0

    # MIN_DEPTH_METERS = 0
    # MAX_DEPTH_METERS = 50

    # Get vehicle pose
    pose = client.simGetVehiclePose()

    # Get car state
    car_state = client.getCarState()
    one_log_dict = controller.control(pts_2D, car_state, pose)

    prev_closest_waypoint = curr_closest_waypoint
    curr_closest_waypoint = one_log_dict['which_closest']

    # Check if we made a whole lap
    if prev_closest_waypoint is not None:

        if 0.9 * prev_closest_waypoint > curr_closest_waypoint:
            num_laps += 1

    steer, throttle = one_log_dict['steer'], one_log_dict['throttle']
    car_controls.steering = steer
    car_controls.throttle = throttle

    client.setCarControls(car_controls)

    distance_travelled = num_laps + curr_closest_waypoint / float(num_waypoints)
    return 'SUCCESS', depth_storage, one_log_dict, log_dicts, distance_travelled


def main():
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '-q', '--quality-level',
        choices=['Low', 'Epic'],
        type=lambda s: s.title(),
        default='Epic',
        help='graphics quality level, a lower level makes the simulation run considerably faster.')
    argparser.add_argument(
        '-e', '--num_episodes',
        default=999,
        type=int,
        dest='num_episodes',
        help='Number of episodes')
    argparser.add_argument(
        '-s', '--speed',
        default=10.0,
        type=float,
        dest='target_speed',
        help='Target speed')
    argparser.add_argument(
        '-cont', '--controller_name',
        default='mpc',
        dest='controller_name',
        help='Controller name')
    argparser.add_argument(
        '-sd', '--store_data',
        action='store_false',
        dest='store_data',
        help='Should the data be stored?')
    # argparser.add_argument(
    #     '-rep', '--report_filename',
    #     default=None,
    #     dest='report_filename',
    #     help='Report filename')

    # For the NN controller
    argparser.add_argument(
        '-mf', '--model_dir_name',
        default=None,
        dest='model_dir_name',
        help='NN model directory name')
    argparser.add_argument(
        '-w', '--which_model',
        default='best',
        dest='which_model',
        help='Which model to load (5, 10, 15, ..., or: "best")')
    argparser.add_argument(
        '-tca', '--throttle_coeff_A',
        default=1.0,
        type=float,
        dest='throttle_coeff_A',
        help='Coefficient by which NN throttle predictions will be multiplied by')
    argparser.add_argument(
        '-tcb', '--throttle_coeff_B',
        default=0.0,
        type=float,
        dest='throttle_coeff_B',
        help='Coefficient by which NN throttle predictions will be shifted by')
    argparser.add_argument(
        '-ens', '--ensemble-prediction',
        action='store_true',
        dest='ensemble_prediction',
        help='Whether predictions for steering should be aggregated')

    args = argparser.parse_args()

    # assert args.report_filename is not None, (
    #     'You need to provide a report filename (argument -rep or'
    #     ' --report_filename)'
    # )

    # log_level = logging.DEBUG if args.debug else logging.INFO
    # logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)
    #
    # logging.info('listening to server %s:%s', args.host, args.port)

    args.out_filename_format = '_out/episode_{:0>4d}/{:s}/{:0>6d}'

    while True:
        # try:
        run_airsim_client(args)

        print('Done.')
        return

        # except TCPConnectionError as error:
        #     logging.error(error)
        #     time.sleep(1)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')