# Lane Line Detection

# About
This folder contains various python scripts and notebooks to perform lane line detection on an artificial track inside a garage. The artificial track was made by putting tape on black foam mats.

We tried different cameras and resolutions. For now we are using a [fisheye](https://www.amazon.com/gp/product/B01E8OWZM4/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1) at 640x360 resolution and 30 fps.

The lane line detection was done using traditional computer vision and was based off another person's github which is linked in the references section at the bottom.

The following steps were taken to perform the lane line detection:
- Compute the camera calibration matrix and distortion coefficients of the camera using a set of chessboard images taken using the same camera. The python notebook to perform the camera calibration can be found at `RC_Car/Camera_Calibration/Calibrate_Camera.ipynb`
- Undistort the image using the camera calibration matrix and distortion coefficients
- Convert image to birdseye view (as though looking straight down onto the track)
- Apply color and gradient thresholds to isolate the lane line pixels, creating a binary image
- Perform two simultaneous histogram-based window searches to classify pixels as either belonging to the left or right lane line
- Determine the best-fit line (a quadratic) of each lane line
- Color in the area between the best-fit lines and project the lane back onto the original undistorted image

# How To Use
## Install Dependencies
```
$ pip install numpy
$ pip install matplotlib
$ pip install opencv-python
$ pip install scikit-image
$ pip install IPython
$ pip install moviepy
```
You will also need to install jupyter notebook if you wish to run the python notebooks: http://jupyter.readthedocs.io/en/latest/install.html

## Downloading test images and videos
Go to this google drive link: https://drive.google.com/drive/folders/0AGXSliFEBeMNUk9PVA

Download the `RC_Car_Images_And_Videos` folder from the google drive link and place the folder inside the `RC_Car` folder.

This will allow you to run the lane line detection on images and videos taken from different cameras in different resolutions. For now the lane line detection is built to run only for the fisheye 640x360 test images and videos. If you want the lane line detection to work for different cameras and resolutions, you will have to re-calibrate the camera and modify the code.

## Performing Lane Line Detection on Images
Simply run the script `RC_Car/Lane_Line_Detection_For_Images.py` This will perform the lane line detection on one image. If you wish to try a different image, change the image_path variable at line 833.

Or if you want, you may run the python notebook version at `RC_Car/Lane_Line_Detection_For_Images.ipynb` This will perform lane line detection on all test images for one of the six fisheye videos.

If you run the python notebook version, run all the code blocks in order except for the second code block. The second code block is just used to extract frames from video. If you wish to try test images from another fisheye video, go to code block 4 and change the video_number variable at line 1 to any number 1-6.

## Performing Lane Line Detection on MP4s
Simply run the script  `RC_Car/Lane_Line_Detection_For_MP4s.py` This will perform lane line detection on one of the six fisheye videos. To change to a different fisheye video, go to line 929 and change the video_number variable to any number 1-6.

Or if you want, you may run the python notebook version at `RC_Car/Lane_Line_Detection_For_MP4s.ipynb` Run all the code blocks in order. This will perform lane line detection on one of the six fisheye videos. To change to a different fisheye video, go to line 1 of the third code block and change the video_number variable to any number 1-6.

Both the python script and notebook will save the video output in one of the respective video folders `RC_Car/RC_Car_Images_And_Videos/Hershels_Garage/Fisheye/videoX` where X is the video number 1-6.

# References
This is the Github we are basing the lane line detection:
https://github.com/mithi/advanced-lane-detection
