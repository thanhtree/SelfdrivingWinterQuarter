import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import pickle
from keras.models import load_model
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Activation, Dropout, Cropping2D
from keras.layers.convolutional import Convolution2D, Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.regularizers import l2
from keras.callbacks import ReduceLROnPlateau, ModelCheckpoint
import os
import time

#Library to use UDP
import socket

#UDP_IP uses the private IP address of the Raspberry Pi
#The private IP address changes everytime so update.
#This script is the client.

UDP_IP = "192.168.2.6" # Change this to ip address of raspberry pi
UDP_PORT = 5004

#import RPi.GPIO as GPIO
#import pigpio

# os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
# comment line above and uncomment line below for linux
# export TF_CPP_MIN_LOG_LEVEL = 2

def preprocess(image):
    result = image[80:, :, :]
    result = cv2.resize(result, (200, 66), interpolation=cv2.INTER_AREA)
    result = cv2.GaussianBlur(result, (3, 3), 0)
    result = cv2.cvtColor(result, cv2.COLOR_BGR2YUV)
    return result

if __name__ == '__main__':

    model_name = "modified_lenet" # can be nvidia or modified_lenet

    # Define the input shape
    input_shape = (66, 200, 3)

    if (model_name == "modified_lenet"):
        # Using modified_lenet models from training 10 iterations
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=01_valLoss=0.017420.h5"
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=02_valLoss=0.014468.h5"
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=03_valLoss=0.012342.h5"
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=04_valLoss=0.009669.h5"
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=05_valLoss=0.008395.h5"
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=06_valLoss=0.007865.h5"
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=07_valLoss=0.007962.h5"
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=08_valLoss=0.007365.h5"
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=09_valLoss=0.006645.h5"
        #model_path = "models/modified_lenet/1/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=10_valLoss=0.007456.h5"

        # Using modified_lenet_models from training 100 iterations
        #model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=05_valLoss=0.009664.h5"
        #model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=15_valLoss=0.006044.h5"
        model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=25_valLoss=0.005425.h5" # this one works
        #model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=35_valLoss=0.004850.h5"
        #model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=45_valLoss=0.003679.h5"
        #model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=55_valLoss=0.003451.h5"
        #model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=65_valLoss=0.003595.h5"
        #model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=75_valLoss=0.003503.h5"
        #model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=85_valLoss=0.003384.h5"
        #model_path = "models/modified_lenet/2/weights_only_model_modified_lenet_lrFactor=0.100_batchSize=32_epoch=95_valLoss=0.003380.h5"

        # Define modified lenet model
        modified_lenet_model = Sequential()
        print()
        print('Modified Lenet Model:\n')
        # pixel_normalized_and_mean_centered = pixel / 255 - 0.5
        modified_lenet_model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=input_shape))
        print(modified_lenet_model.output_shape)
        # modified_lenet_model.add(Convolution2D(nb_filter=6, nb_row=5, nb_col=5,
        #                        activation='relu', border_mode='valid',
        #                        subsample=(1, 1), dim_ordering='tf'))
        modified_lenet_model.add(Conv2D(filters=6, kernel_size=(5, 5), strides=(1, 1), padding='valid', activation='relu', data_format='channels_last'))
        print(modified_lenet_model.output_shape)
        modified_lenet_model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

        print(modified_lenet_model.output_shape)
        # modified_lenet_model.add(Convolution2D(nb_filter=16, nb_row=5, nb_col=5,
        #                        activation='relu', border_mode='valid',
        #                        subsample=(1, 1)))
        modified_lenet_model.add(Conv2D(filters=16, kernel_size=(5, 5), strides=(1, 1), padding='valid', activation='relu', data_format='channels_last'))

        print(modified_lenet_model.output_shape)
        # modified_lenet_model.add(Convolution2D(nb_filter=36, nb_row=5, nb_col=5,
        #                        activation='relu', border_mode='valid',
        #                        subsample=(1, 1)))
        modified_lenet_model.add(Conv2D(filters=36, kernel_size=(5, 5), strides=(1, 1), padding='valid', activation='relu', data_format='channels_last'))
        print(modified_lenet_model.output_shape)
        modified_lenet_model.add(Flatten())
        print(modified_lenet_model.output_shape)
        modified_lenet_model.add(Dense(120, activation='relu'))
        modified_lenet_model.add(Dropout(0.25))
        print(modified_lenet_model.output_shape)
        modified_lenet_model.add(Dense(84, activation='relu'))
        modified_lenet_model.add(Dropout(0.25))
        print(modified_lenet_model.output_shape)
        modified_lenet_model.add(Dense(1))

        modified_lenet_model.compile(loss='mse', optimizer='adam', metrics=['accuracy'])
        modified_lenet_model.summary()

        modified_lenet_model.load_weights(model_path)
        model = modified_lenet_model

    if (model_name == "nvidia"):

        # Using nvidia models from training 10 iterations
        #model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=01_valLoss=0.031086.h5"
        #model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=02_valLoss=0.023957.h5"
        #model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=03_valLoss=0.025302.h5"
        #model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=04_valLoss=0.020549.h5"
        #model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=05_valLoss=0.017960.h5"
        #model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=06_valLoss=0.015531.h5"
        #model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=07_valLoss=0.018099.h5"
        #model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=08_valLoss=0.016283.h5"
        #model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=09_valLoss=0.017435.h5"
        model_path = "models/nvidia/1/weights_only_model_nvidia_lrFactor=0.100_batchSize=32_epoch=10_valLoss=0.012025.h5"




        # Define nvidia model
        nvidia_model = Sequential()
        print('Nvidia Model:\n')
        # pixel_normalized_and_mean_centered = pixel / 255 - 0.5
        nvidia_model.add(Lambda(lambda x: x / 255.0 - 0.5, input_shape=input_shape))
        print(nvidia_model.output_shape)

        nvidia_model.add(Conv2D(filters=24, kernel_size=(5, 5), strides=(2, 2), padding='valid', activation='relu', data_format='channels_last'))
        print(nvidia_model.output_shape)

        nvidia_model.add(Conv2D(filters=36, kernel_size=(5, 5), strides=(2, 2), padding='valid', activation='relu', data_format='channels_last'))
        print(nvidia_model.output_shape)

        nvidia_model.add(Conv2D(filters=48, kernel_size=(5, 5), strides=(2, 2), padding='valid', activation='relu', data_format='channels_last'))
        print(nvidia_model.output_shape)

        nvidia_model.add(Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), padding='valid', activation='relu', data_format='channels_last'))
        print(nvidia_model.output_shape)

        nvidia_model.add(Conv2D(filters=64, kernel_size=(3, 3), strides=(1, 1), padding='valid', activation='relu', data_format='channels_last'))
        print(nvidia_model.output_shape)

        nvidia_model.add(Flatten())
        print(nvidia_model.output_shape)
        nvidia_model.add(Dropout(0.25))

        nvidia_model.add(Dense(100, activation='relu'))
        print(nvidia_model.output_shape)
        nvidia_model.add(Dropout(0.25))

        nvidia_model.add(Dense(50, activation='relu'))
        print(nvidia_model.output_shape)
        nvidia_model.add(Dropout(0.25))

        nvidia_model.add(Dense(10, activation='relu'))
        print(nvidia_model.output_shape)
        nvidia_model.add(Dropout(0.25))

        nvidia_model.add(Dense(1))

        nvidia_model.compile(loss='mse', optimizer='adam', metrics=['accuracy'])
        nvidia_model.summary()

        nvidia_model.load_weights(model_path)
        model = nvidia_model

    frame_width = 320
    frame_height = 180

    cap = cv2.VideoCapture(0)

    cap.set(3, frame_width)
    cap.set(4, frame_height)

    font = cv2.FONT_HERSHEY_SIMPLEX


	#This while loop send the angle over UDP
    while(True):
        # Capture frame-by-frame
        t1 = time.time()
        ret, frame = cap.read()

        if (ret==True):
            print("frame.shape: ", frame.shape)

            preprocessed_frame = preprocess(frame)
            predicted_angle = float(model.predict(preprocessed_frame[None, :, :, :], batch_size=1))*60.0

            #converts to a string and then to a byte
            string_message = str(predicted_angle)
            bytes_message = str.encode(string_message)

            #send to server code
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(bytes_message, (UDP_IP, UDP_PORT))
