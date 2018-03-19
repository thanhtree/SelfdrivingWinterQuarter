import csv
import cv2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import pickle
def fliph_image(img):
    """
    Returns a horizontally flipped image
    """
    return cv2.flip(img, 1)


def change_image_brightness_rgb(img, s_low=0.3, s_high=0.75):
    """
    Changes the image brightness by multiplying all RGB values by the same scalacar in [s_low, s_high).
    Returns the brightness adjusted image in RGB format.
    """
    img = img.astype(np.float32)
    s = np.random.uniform(s_low, s_high)
    img *= s
    np.clip(img, 0, 255)
    return img.astype(np.uint8)


def add_random_shadow(img, w_low=0.3, w_high=0.85):
    """
    Overlays supplied image with a random shadow polygon
    The weight range (i.e. darkness) of the shadow can be configured via the interval [w_low, w_high)
    """
    cols, rows = (img.shape[0], img.shape[1])

    top_y = np.random.random_sample() * rows
    bottom_y = np.random.random_sample() * rows
    bottom_y_right = bottom_y + np.random.random_sample() * (rows - bottom_y)
    top_y_right = top_y + np.random.random_sample() * (rows - top_y)
    if np.random.random_sample() <= 0.5:
        bottom_y_right = bottom_y - np.random.random_sample() * (bottom_y)
        top_y_right = top_y - np.random.random_sample() * (top_y)

    poly = np.asarray([[[top_y, 0], [bottom_y, cols], [bottom_y_right, cols], [top_y_right, 0]]], dtype=np.int32)

    mask_weight = np.random.uniform(w_low, w_high)
    origin_weight = 1 - mask_weight

    mask = np.copy(img).astype(np.int32)
    cv2.fillPoly(mask, poly, (0, 0, 0))
    # masked_image = cv2.bitwise_and(img, mask)

    return cv2.addWeighted(img.astype(np.int32), origin_weight, mask, mask_weight, 0).astype(np.uint8)


def translate_image(img, st_angle, low_x_range, high_x_range, low_y_range, high_y_range, delta_st_angle_per_px):
    """
    Shifts the image right, left, up or down.
    When performing a lateral shift, a delta proportional to the pixel shifts is added to the current steering angle
    """
    rows, cols = (img.shape[0], img.shape[1])
    translation_x = np.random.randint(low_x_range, high_x_range)
    translation_y = np.random.randint(low_y_range, high_y_range)

    st_angle += translation_x * delta_st_angle_per_px
    translation_matrix = np.float32([[1, 0, translation_x],[0, 1, translation_y]])
    img = cv2.warpAffine(img, translation_matrix, (cols, rows))

    return img, st_angle

def augment_image(image, angle, p=0.8):
    """
    Takes in image and angle and applies augmentation functions one-by-one with a probability of p
    """
    aug_image = image
    aug_angle = angle

    if np.random.random_sample() <= p:
        aug_image = fliph_image(aug_image)
        aug_angle = -1*angle

    if np.random.random_sample() <= p:
        aug_image = change_image_brightness_rgb(aug_image)

    if np.random.random_sample() <= p:
        aug_image = add_random_shadow(aug_image, w_low=0.45)

    # if np.random.random_sample() <= p:
    #     aug_image, aug_angle = translate_image(aug_image, aug_angle, -60, 61, -20, 21, (60*0.35)/100.0)

    return aug_image, aug_angle


"""
file_name = 'data.p'
with open(file_name, 'rb') as f:
    samples = pickle.load(f)
count = 0
fig, axs = plt.subplots(5, 6, figsize=(12, 14))
axs = axs.ravel()
for i in range(0, 10000, 2000):
    sample = samples[i]
    angle = int(sample[0])
    speed = int(sample[1])
    image_path = sample[2]
    image = Image.open(image_path)
    image = np.asarray(image)

    flipped_image = fliph_image(image)
    darkened_image = change_image_brightness_rgb(image)
    random_shadow_image = add_random_shadow(image)
    shifted_image, st_angle = translate_image(image, angle, -60, 61, -20, 21, 0.35/100.0)
    aug_image, aug_angle = augment_image(image, angle)
    axs[count*6].imshow(image)
    axs[count*6+1].imshow(flipped_image)
    axs[count*6+2].imshow(darkened_image)
    axs[count*6+3].imshow(random_shadow_image)
    axs[count*6+4].imshow(shifted_image)
    axs[count*6+5].imshow(aug_image)
    count += 1
plt.show()
"""
