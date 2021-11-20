import sys
import numpy as np
import pyzed.sl as sl
import cv2

# https://github.com/huy23tran11/Collision_Avoidance/blob/flight_test_2/src/main.cpp


# Find best space that closet to the center then return true, return false if there is no space at all
def finding_best_space(img_binary, temp1_rect, center_rect):

    frame_h, frame_w,_ = img_binary.shape
    templ_w = 200 # dimension of the drone at 5m with 50% bigger 133/100 * 150, recommend 300 for safer avoiding
    templ_h = 83 # dimention of the drone at 5m with 50% bigger 55/100 * 150
    
    temp1 = np.array((templ_h, templ_w, cv2.CV_8U), cv2.Scalar(0,0,0))

    center_frame_tf = ((frame_w / 2) - (templ_w / 2), (frame_h / 2) - (templ_h / 2))
    # In Python, return the center rect as a tuple?
    # image = cv2.rectangle(image, start_point, end_point, color, thickness); C++
    # center_Rect = cv::Rect(center_frame_tf.x, center_frame_tf.y, templ_w, templ_h);

    img_binary_formatted = cv2.cvtColor(img_binary, cv2.COLOR_BGR2GRAY)

    # cv::Rect(x,y,w,h)
    # x = top-left corner; y = top-left corner; w = width of rect; h = height of rect
    # img_bianry_cropped = img_binary_formatted(cv::Rect(0, center_frame_tf.y, frame_w, templ_h)); // create a frame fit the templ, so that it can only slide left and right
    # cropped_frame_tf.x = 0;
    # In Python, roi = im[y1:y2, x1:x2]
    # img_binary_cropped = 


# Find all available spaces and returns them as a list
def finding_all_available_spaces(img_result_templ_matching):
    space_loc_tf = []
    rows_cols_,_ = img_result_templ_matching.shape
    for i in range(rows):
        for j in range(cols):
            if img_result_templ_matching[i, j] == 0:
                space_loc_tf.append((j, i))
    return space_loc_tf


# From all avaible space, find the closet space to the center then return the index of that space rectangle in the vector
def finding_closest_space_to_center():
    pass