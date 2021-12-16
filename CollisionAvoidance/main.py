import sys
import math
import numpy as np
import pyzed.sl as sl
import cv2

# https://github.com/huy23tran11/Collision_Avoidance/blob/flight_test_2/src/main.cpp


# Find best space that closet to the center then return true, return false if there is no space at all
# img_binary = after converting depth_image_ocv to a binary image
# templ_rect = cv::Rect
# center_rect = cv::Rect
def finding_best_space(img_binary):

    img_binary_h, img_binary_w, img_binary_channels = img_binary.shape
    templ_w = 200 # dimension of the drone at 5m with 50% bigger 133/100 * 150, recommend 300 for safer avoiding
    templ_h = 83 # dimention of the drone at 5m with 50% bigger 55/100 * 150
    
    # Create a template that is all black
    temp1 = np.array((templ_h, templ_w, cv2.CV_8U), cv2.Scalar(0,0,0))

    # Top left point of the frame in the center
    center_frame_tf = ((img_binary_w / 2) - (templ_w / 2), (img_binary_h / 2) - (templ_h / 2))
    center_rect = (center_frame_tf[0], center_frame_tf[1], templ_w, templ_h)

    # binary img need to be formatted to be used for template matching algothsm
    # convert to a gray
    img_binary_formatted = cv2.cvtColor(img_binary, cv2.COLOR_BGR2GRAY)

    # Crop the center of the image with a size of (templ_h x original image width)
    img_binary_cropped = img_binary_formatted[center_frame_tf[1]:(center_frame_tf[1]+templ_h+1) , :]

    # Top left point of the cropped frame
    cropped_frame_tf = (0, center_frame_tf[1])

    # Refer to here to learn about cv::matchTemplate()
    # https://docs.opencv.org/3.4/de/da9/tutorial_template_matching.html
    img_result_templ_matching = cv2.matchTemplate(img_binary_cropped, temp1, cv2.TM_SQDIFF)

    space_loc_tf = finding_all_available_spaces(img_result_templ_matching)

    if len(space_loc_tf) <= 0:
        return False
    else:
        min_dis_index = finding_closest_space_to_center(space_loc_tf, center_frame_tf)
        top_left = (cropped_frame_tf[0] + space_loc_tf[min_dis_index][0], cropped_frame_tf[1] + space_loc_tf[min_dis_index][1])
        templ_rect = (top_left[0], top_left[1], templ_w, templ_h)
        return (templ_rect, center_rect)


# Find all available spaces and returns them as a list
def finding_all_available_spaces(img_result_templ_matching):
    space_loc_tf = []
    rows, cols, channels = img_result_templ_matching.shape
    for i in range(rows):
        for j in range(cols):
            if img_result_templ_matching[i, j] == 0:
                space_loc_tf.append((j, i))
    return space_loc_tf

def euclidean_distance(point1, point2):
    return math.sqrt(math.pow(point1[0]-point2[0]) + math.pow(point1[1]-point2[1]))

# From all available spaces, find the closet space to the center then return the index of that space rectangle in the vector
def finding_closest_space_to_center(space_loc_tf, center_frame_tf):
    distances = []

    # Go thru each space location and calculate the euclidean distance between the center frame
    for i in range(len(space_loc_tf)):
        distances.append(euclidean_distance(space_loc_tf[i], center_frame_tf))

    # Find the minimum distance and its index
    min_dist = distances[0]
    min_dist_index = 0
    for i in range(len(distances)):
        if min_dist < distances[i]:
            min_dist = distances[i]
            min_dist = i
    
    return min_dist_index


    # Need a sample image to test the above methods
    