import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import tensorflow as tf
from keras.models import load_model
import sys
import time
import numpy as np

import airsim

import keras.backend as K
from keras.preprocessing import image
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt

# Trained model path
MODEL_PATH = 'models/fresh_models/model_model.80-0.0088993.h5'

model = load_model(MODEL_PATH)

# Connect to AirSim 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

# Start driving
car_controls.steering = 0
car_controls.throttle = 0
car_controls.brake = 0
client.setCarControls(car_controls)

# Initialize image buffer
image_buf = np.zeros((1, 90, 255, 3))
state_buf = np.zeros((1,4))

def get_image():
    """
    Get image from AirSim client
    """
    image_response = client.simGetImages([airsim.ImageRequest(0, airsim.ImageType.Scene, False, False)])[0]
    # print(image_response.height, image_response.width)
    image1d = np.frombuffer(image_response.image_data_uint8, dtype=np.uint8)
    # print(image1d.shape)
    image_rgba = image1d.reshape(image_response.height, image_response.width, 3)
    return image_rgba[50:140,0:255,0:3].astype(float)


while (True):
    car_state = client.getCarState()
    
    if (car_state.speed < 6):
        car_controls.throttle = 1.0
    else:
        car_controls.throttle = 0.0
    
    image_buf[0] = get_image()
    image_buf[0] /= 255
    state_buf[0] = np.array([car_controls.steering, car_controls.throttle, car_controls.brake, car_state.speed])
    model_output = model.predict([image_buf, state_buf])
    # print(model_output)
    if (model_output[0][0] > 0):
        
        car_controls.steering = round(float(model_output[0][0]), 2)
    else:
        car_controls.steering = round(0.8*float(model_output[0][0]), 2)
    
    print('Sending steering = {0}, throttle = {1}'.format(car_controls.steering, car_controls.throttle))
    
    client.setCarControls(car_controls)