# Car Control

# About
This folder contains all the code necessary to collect training data and train a neural net for the car to drive itself around our homemade track.

We used a [fisheye](https://www.amazon.com/gp/product/B01E8OWZM4/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1) camera with a 180-degree field of view.

Some of the code is run on a Raspberry Pi 3. Some of the code is run on a Jetson TX2. Some of the code is run on a workstation with a heavy-duty Nvidia GPU. We used a Titan XP. But you do not need to use such a high-end GPU for training.

To make things easier, we kept this folder on a cloud that was synced with the Raspberry Pi, Jetson TX2, and our workstation. This way we did not have to rely on a file transfer protocol like SCP to transfer our training data from the Raspberry Pi to our workstation or to transfer our models from our workstation to the Jetson TX2. It just automatically syncs. However, you need to have internet for it to sync. And you may have to wait for a few hours for synching to finish if you have a lot of images waiting to sync.

Both the Raspberry Pi 3 and Jetson TX2 are mounted on the car and communicate with each other through UDP. The Jetson TX2 takes in the frames, passes the frames through the neural net, calculates the steering angles, and then passes the steering angles to the Pi which then executes the action to change the steering angle. The speed of the car is kept constant by the Pi.

Training data is collected by driving the car around with a PS4 controller. The data is saved as a tab-delimited text file. First column is the steering angle. Second column is the speed. Third column is the path where the corresponding frame is saved.

We have not saved any of our training data on our github because it would take up too much space. This is why our `output_logs` folder is practically empty. We also included only one model (the working model) for the same reason. Our neural net models are saved as .h5 files, and including all of our trained models would have taken GB's of space. Our working model is located in this folder `models/modified_lenet/2/`.

We tried using nvidia's model for predicting steering angles, which worked on a real car. You can read about it here: https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf However, it did not perform very well on our rc car.

The neural network architecture that did work was a modified version of lenet. You can read about the original lenet architecture here: http://deeplearning.net/tutorial/lenet.html To see how we changed it, just read through `models.py`. It contains the architecture for both nvidia's model and our modified lenet model.

# How To Use

## Installing Dependencies

Make sure you ran `SelfdrivingWinterQuarter/raspberry_pi_installs/install_everything_for_raspbian_stretch.sh` on your Raspberry Pi 3. This will install all dependencies for raspberry pi.

Make sure you also ran `SelfdrivingWinterQuarter/jetson_installs/install_opencv.sh` on your Jetson TX2 to install OpenCV for the Jetson. You will also need to build Tensorflow and Keras for gpu and their dependencies from source. Unfortunately, we do not have a bash shell script to automate that process. There are plenty of tutorials online.

## Collecting Data

### Creating the output logs

Go into the `output_logs` folder and run `create_output_logs.sh` with two arguments. The first argument is the output log number for the first output log you want to create. The second argument is the output log number for the last output log you want to create. The arguments must be integers. It will create folders called `output_logX`, where X is some integer. Each `output_logX` folder will contain a subfolder called `frames` which will contain the corresponding frames for that output_log. The tab-delimited text file containing all the training data will be saved in `output_logX`.

For instance, if you ran the command below
```
./create_output_logs.sh 4 6
```
It would create three folders called `output_log4`, `output_log5`,and `output_log6`. Each of these folders would contain a sub-folder called `frames`, containing the corresponding frames for that output_log.  

Create as many output log folders as you need. You will need a new output log for each time you want to collect new data.

### Collecting the Training Data

Connect the camera to the Raspberry Pi. Make sure you have a PS4 controller. Open a terminal on the raspberry pi and run the command below to start bluetooth pairing with the PS4 controller

```
 sudo ds4drv
```

Now hold both the select and PS button on the PS4 controller until it starts flashing. The PS4 controller should eventually turn solid blue, indicating it is connected to the raspberry pi.

The program to collect data is in C++ and uses OpenCV for C++. So make sure you built OpenCV from source on the Raspberry Pi.

Here's an example command

```
./main.sh 1
```
 This will save your training data to `output_log1`, assuming you created the folder. If you wish to save to a different output_log, then change '1' to whatever number you want.


 If you are saving to an output_log that already has data, it will ask you if you are sure to overwrite. Hit 'y' to continue and execute the main program or 'n' to stop. If the output_log is empty, then the main program will execute.

Once the main program has begun, you can drive the car around using the left joy stick for going forward and reverse and the right joy stick to steer. All your steering angles, speeds, and corresponding frames will be saved to the output_log you indicated. When you are done driving, hit the triangle button on the PS4 controller to kill the program.

## Training the Model

In order to save time for the training, you are going to need a computer with a Nvidia GPU with at least 1,500 cuda cores. Do not try to train the neural nets with the Jetson TX2. The Jetson TX2 only has 256 cuda cores. It will not be very fast. And certainly do not use the Raspberry Pi for training. It does not even have a GPU.

### Converting data to pickle file

Open `save_data_to_pickle_file.py`. This python script will take all the data from all the output_logs you indicated and save them to a pickle file named `data.p`. To indicate which output_logs you want to use, change the `output_logs` variable at line 22 to include all the output logs you want to save to the pickle file.

When ready, run this command to execute the python script
```
python3 save_data_to_pickle_file.py
```

### Creating blank neural net models
To keep the code short in `train_model.py`, we do not build the neural nets in that python script. Instead, we build the models in `models.py` and leave them blank and save them as .h5 files for `train_model.py` to load and begin training.

To create the blank models for `train_model.py` run this command
```
python3 models.py
```
You only need to create the models once.

### Training the neural net
`train_model.py` contains the code for actually training the models. You can play around with the parameters, which are all capitalized and commented.
```
python3 train_model.py
```

## Executing the trained models
You are going to need to ssh into both the Raspbery Pi and Jetson TX2 to execute the models. You are also going to need to make sure the camera is attached to the Jetson TX2, not the Raspberry Pi.

Take `models` from your computer and save them to the Jetson TX2 into the same folder that `drive_jetson.py` is

### Testing your model

On the Jetson TX2, Modify the `drive_jetson.py` to select which .h5 file you want to load as your model.

One the Raspberry Pi, modify `drive_pi.py` to use the speed you want aka the `PWM` variable. We always kept the speed constant.

Once you're done modifying the code, go to the Jetson TX2 and run
```
python3 drive_jetson.py
```
Once the Jetson TX2 starts printing out the frame shape, go to the Raspberry Pi and run
```
python3 drive_pi.py
```

The car will now begin driving itself.

### Stopping the car
To stop the car, you can just kill the program running on the Raspberry Pi. If you want to switch to a different model, you have to kill the program running on the Jetson TX2, which will then kill the program running on the Raspberry Pi as well as stopping the car.
