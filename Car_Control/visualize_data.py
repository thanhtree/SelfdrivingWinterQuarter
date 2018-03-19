"""
This program helps you visualize the data by displaying ten random images from the dataset and writing the corresponding speed and angle onto each image
"""

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pickle
import random
from PIL import Image

file_name = 'data.p'

with open(file_name, 'rb') as f:
    samples = pickle.load(f)

print('len(samples):', len(samples))

font = cv2.FONT_HERSHEY_SIMPLEX
fig, axs = plt.subplots(2, 5, figsize=(12, 5))
axs = axs.ravel()
count = 0
for i in range(10):
    index = random.randint(0,len(samples))
    sample = samples[index]
    angle = int(sample[0])
    speed = int(sample[1])
    image_path = sample[2]
    image = Image.open(image_path)
    image = np.asarray(image)
    print("image.shape: ", image.shape)
    image_copy = np.copy(image)
    image_copy = image_copy[80:, :, :]
    image_copy = image_copy[0:80, :, :]
    cv2.putText(image_copy, "Angle: %s Speed: %s" % (str(angle), str(speed)), (10, 20), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    axs[count].imshow(image_copy)
    count += 1
plt.show()
print('done')
