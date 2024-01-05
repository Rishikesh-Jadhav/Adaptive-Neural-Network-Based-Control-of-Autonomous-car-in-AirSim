### Author
Nishant AwdeshKumar Pandey

### Research Paper
The home directory has the research paper written ```Paper.pdf```

### Environment Setup

1. [Install Anaconda](https://conda.io/docs/user-guide/install/index.html) with Python 3.5 or higher.
2. [Install CNTK](https://docs.microsoft.com/en-us/cognitive-toolkit/Setup-CNTK-on-your-machine) or [install Tensorflow](https://www.tensorflow.org/install/install_windows)
3. [Install h5py](http://docs.h5py.org/en/latest/build.html)
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

### Running the package 

1. To run the package and create waypoints for the Model predictive controller. Run ```python waypoints.py``` and control the car manually and make it follow the complex and straight paths. 
2. Run ```python client_controller.py``` to run Model predictive controller. 
Start the recording while this command runs to collect the dataset.
3. Open ``` DataExplorationAndPreparation.ipynb``` and run all the cells.
4. Open ``` TrainModel.ipynb``` and run all the cells to train the model. Modify the Region of interest, and various hyperparameters according to the dataset size and needs.
5. Run ```drive_model.py``` to test the model. 

### Outputs 

![Untitled video - Made with Clipchamp](https://github.com/nishantpandey4/Enhancing-Autonomous-Vehicle-Control-Adaptive-Neural-Network-Based-Control-in-AirSim-Simulator/assets/127569735/3cba913b-22d1-473d-9950-5d50b3449010)

1. Video output: https://drive.google.com/drive/folders/1iXjCQh5sCZEpg1p6fzWvC7YgAyeC9uQF
2. Data Collected: https://drive.google.com/drive/folders/1_nsHW8zgRbLLXc5W6bpPYwH0OgZFlNOx

### Paper

[Research Paper](https://github.com/nishantpandey4/Enhancing-Autonomous-Vehicle-Control-Adaptive-Neural-Network-Based-Control-in-AirSim-Simulator/blob/main/Paper.pdf)
   
### References

1. Airsim: https://github.com/microsoft/AutonomousDrivingCookbook/tree/master/AirSimE2EDeepLearning
2. Model predictive controller: https://github.com/asap-report/carla/tree/racetrack/PythonClient/racetrack 



