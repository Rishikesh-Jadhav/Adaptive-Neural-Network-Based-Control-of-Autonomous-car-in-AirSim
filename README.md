# Adaptive-Neural-Network-Based-Control-in-AirSim-Simulator

## Overview

This project focuses on enhancing autonomous vehicle control using Neural Network-Based Control in the AirSim simulator. The primary goal is to improve steering inputs for the vehicle by augmenting conventional PID and Model Predictive Controller (MPC) approaches with adaptive multi-layered neural networks. The project showcases a method that imitates the behavior of PID or MPC controllers rather than human inputs, resulting in a more robust and bias-free model.

## Project Members
- Nishant Pandey (UID: 119247556, Email: npandey2@umd.edu)
- Rishikesh Jadhav (UID: 119256534, Email: rjadhav1@umd.edu)
- Aaqib Barodawala (UID:  119348710, Email: abarodaw@umd.edu)
- Hrugved Pawar (UID: 120118074, Email: hpawar@umd.edu)

## Abstract

The project employs a systematic approach to integrate imitation learning enhanced by a Model Predictive Controller for autonomous driving in a simulated environment. The hybrid neural network architecture combines convolutional and dense layers, demonstrating efficacy in synthesizing visual and contextual information. Techniques such as dropout, data augmentation, and specific Region of Interest (ROI) selection address challenges inherent in real-world applications and diverse dataset characteristics.

## Key Features

- **Simulation Environment:** Utilized the AirSim simulator for autonomous vehicle control simulation.
  
- **Data Collection:** Gathered sensor data from the simulator, including front-facing camera images and state variables like steering, throttle, speed, and brake.

- **Neural Network Training:** Implemented a multi-layered neural network architecture suitable for control tasks, providing a jittery-free output.

- **Evaluation Metrics:** Assessed model performance using Mean Squared Error (MSE) and qualitative indicators for real-world scenarios.

## Methodology

The project employs a systematic approach to augment conventional PID and MPC controllers with adaptive multi-layered neural networks. The neural network architecture combines convolutional layers for image analysis with additional numerical data, resulting in improved steering inputs. The method imitates the behavior of controllers rather than human inputs, ensuring unbiased data distribution and jitter-free outputs.

## Experiments

- **Data Collection Strategies:** Explored data collection using LIDAR-based obstacle detection, and waypoint-driven scenarios with PID and MPC controllers.

   Data Collected: https://drive.google.com/drive/folders/1_nsHW8zgRbLLXc5W6bpPYwH0OgZFlNOx

- **Fine-Tuning and Optimization:** Experimentation with dropout values, ROI selection, and optimization using the Nadam optimizer to enhance model performance.

## Limitations

- Simulation-to-reality gap and environmental variability.
- Real-time adaptability constraints.
- Dependency on vision-based perception.

## Conclusion

This project successfully integrates imitation learning with Model Predictive Control, demonstrating a comprehensive approach to neural network architecture, training, and deployment. Challenges identified provide valuable insights for future exploration. The robustness and adaptability of the model, as well as its real-world limitations, contribute to ongoing research in deep learning for autonomous vehicles.

## Testing Results

### Click to watch the videos

### Gradual Turns

[![Click to watch the video](https://img.youtube.com/vi/fY8KmGNrRWA/0.jpg)](https://youtu.be/fY8KmGNrRWA)

### Sharp Turns

[![Click to watch the video](https://img.youtube.com/vi/I7ccYnLa4MY/0.jpg)](https://youtu.be/I7ccYnLa4MY)

### MPC Output - Scene 1

[![Click to watch the video](https://img.youtube.com/vi/vMtDc_j6Wno/0.jpg)](https://youtu.be/vMtDc_j6Wno)

### MPC Output - Scene 2

[![Click to watch the video](https://img.youtube.com/vi/bGjK_GMijDw/0.jpg)](https://youtu.be/bGjK_GMijDw)


## How to Use

1. Clone the repository.
2. Install dependencies using `requirements.txt`.

## Environment Setup

1. [Install Anaconda](https://conda.io/docs/user-guide/install/index.html) with Python 3.5 or higher.
2. [Install CNTK](https://docs.microsoft.com/en-us/cognitive-toolkit/Setup-CNTK-on-your-machine) or [install Tensorflow](https://www.tensorflow.org/install/install_windows).
3. [Install h5py](http://docs.h5py.org/en/latest/build.html).
4. [Install Keras](https://keras.io/#installation) and [configure the Keras backend](https://keras.io/backend/) to work with TensorFlow (default) or CNTK.
5. [Install AzCopy](https://docs.microsoft.com/en-us/azure/storage/common/storage-use-azcopy). Be sure to add the location for the AzCopy executable to your system path.
6. Install the other dependencies. From your Anaconda environment, run "InstallPackages.py" as root or administrator. This installs the following packages into your environment:
    * jupyter
    * matplotlib v. 2.1.2
    * image
    * keras_tqdm
    * opencv
    * msgpack-rpc-python
    * pandas
    * numpy
    * scipy

## Running the package 

1. To run the package and create waypoints for the Model predictive controller, run `python waypoints.py` and control the car manually to make it follow complex and straight paths.
2. Run `python client_controller.py` to run the Model predictive controller. Start recording while this command runs to collect the dataset.
3. Open `DataExplorationAndPreparation.ipynb` and run all the cells.
4. Open `TrainModel.ipynb` and run all the cells to train the model. Modify the Region of Interest and various hyperparameters according to the dataset size and needs.
5. Run `drive_model.py` to test the model.

## References

1. Airsim: https://github.com/microsoft/AutonomousDrivingCookbook/tree/master/AirSimE2EDeepLearning
2. Model predictive controller: https://github.com/asap-report/carla/tree/racetrack/PythonClient/racetrack 
