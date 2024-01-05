# Python client example to get Lidar data from a car
#

import setup_path 
import airsim

import sys
import math
import time
import argparse
import pprint
import numpy as np
from simple_pid import PID

# Makes the drone fly and get Lidar dcata
class LidarTest:

    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
        self.pid = PID(0.001, 0.02, 0.2, setpoint=6.0)
        self.pid.output_limits = (-0.1, 0.1)  # Steering limits
        self.speed_pid = PID(0.1, 0.01, 0.05, setpoint=5)
        self.speed_pid.output_limits = (0, 1)
    def vehicle_pose(self):
        client1 = airsim.VehicleClient()
        client1.confirmConnection()
        pose = client1.simGetVehiclePose()
        return pose,client1.simGetVehiclePose().orientation
    def find_nearest_obstacle_and_direction(self, world_points, car_position):
        closest_point = None
        nearest_distance = float('inf')
        direction = 0  # 0 for straight, -1 for left, 1 for right

        for point in world_points:
            # Calculate the Euclidean distance from the car to the point
            distance = np.linalg.norm(point - np.array([car_position.position.x_val, car_position.position.y_val, car_position.position.z_val]))
            if distance < nearest_distance:
                nearest_distance = distance
                closest_point = point
                # print(point)
                direction = -1 if point[0] < 0 else 1  
        return nearest_distance, direction
    def execute(self):
        for i in range(3):
            state = self.client.getCarState()
            s = pprint.pformat(state)
            #print("state: %s" % s)
            self.car_controls.throttle = 0.5
            self.car_controls.steering = 0
            self.client.setCarControls(self.car_controls)
            print("Go Forward")
            time.sleep(3)   # let car drive a bit
            while True:
                # Get initial LIDAR data
                lidarData = self.client.getLidarData()
                if (len(lidarData.point_cloud) < 3):
                    print("\tNo points received from Lidar data")
                    continue  # If no data received, skip to the next iteration
                self.client.setCarControls(self.car_controls)
                # Parse LIDAR data
                points = self.parse_lidarData(lidarData)
                pose,quaternion= self.vehicle_pose()
                # R=self.to_rotation_matrix(quaternion)
                points=self.get_world_frame_points(points,pose,quaternion)
                # If an obstacle is detected within the avoidance threshold
                distance_to_obstacle, obstacle_direction = self.find_nearest_obstacle_and_direction(points, pose)
                error = self.pid.setpoint - distance_to_obstacle
                correction = self.pid(distance_to_obstacle)
                if obstacle_direction == -1:
                #     # Obstacle is to the left, steer right
                    self.car_controls.steering = correction  # Ensure we steer right
                elif obstacle_direction == 1:
                #     # Obstacle is to the right, steer left
                    self.car_controls.steering = -correction # Ensure we steer left
                # else:
                #     # Obstacle is straight ahead, apply correction directly
                #     self.car_controls.steering = correction

                       
                print(obstacle_direction)
                # Print debugging information
                print(f"Distance to obstacle: {distance_to_obstacle}, Error: {error}")
                print(f"PID Correction: {correction}")
                car_speed = self.client.getCarState().speed
                # Calculate the throttle value using the PID controller
                # error = self.speed_pid.setpoint - car_speed
                throttle = self.speed_pid(car_speed)
                self.car_controls.throttle = throttle
                self.client.setCarControls(self.car_controls)
                print(self.car_controls)
                time.sleep(2)
    def to_rotation_matrix(self,quaternion):
        # Ensure the quaternion is normalized
        # quaternion = quaternion / np.linalg.norm(quaternion)
        w, x, y, z = quaternion

        # Compute the rotation matrix
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])
        return R
    def get_world_frame_points(self,lidar_points, car_pose,quaternion):
    # Convert the car's quaternion to a rotation matrix
        # quaternion = np.array([
        #     car_pose.orientation.w_val,
        #     car_pose.orientation.x_val,
        #     car_pose.orientation.y_val,
        #     car_pose.orientation.z_val
        # ])
        rotation_matrix = self.to_rotation_matrix(quaternion)

        # Extract the car's position as a translation vector
        translation_vector = np.array([
            car_pose.position.x_val,
            car_pose.position.y_val,
            car_pose.position.z_val
        ])

        # Transform the LIDAR points to the world frame
        world_frame_points = []
        for point in lidar_points:
            # Convert the point to a np array
            local_point = np.array([point[0], point[1], point[2]])

            # Apply the rotation and translation to transform the point
            world_point = np.dot(rotation_matrix, local_point) + translation_vector

            # Append the transformed point to the list
            world_frame_points.append(world_point)

        return np.array(world_frame_points)
    def find_nearest_obstacle(self, world_points, car_position):
        distances = []
        for point in world_points:
            # Calculate the Euclidean distance from the car to the point
            distance = np.linalg.norm(point - np.array([car_position.position.x_val, car_position.position.y_val, car_position.position.z_val]))
            distances.append(distance)
            nearest_distance = min(distances)
        for dis in distances:
            dis 
            direction = -1 if point[1] > 0 else 1  # Assuming point[1] is the lateral position (y-axis)

        return nearest_distance,direction
    
    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

    def write_lidarData_to_disk(self, points,filename="lidar_data.txt"):
        with open(filename, 'w') as f:
            for point in points:
                f.write(f"{point[0]},{point[1]},{point[2]}\n")
            print(f"Lidar data saved to {filename}")
            print("not yet implemented")

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes car move and gets Lidar data")

    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)
  
    args = arg_parser.parse_args(args)    
    lidarTest = LidarTest()
    # try:
    lidarTest.execute()
    # finally:
        # lidarTest.stop()