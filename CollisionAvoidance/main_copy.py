import sys
import math
import numpy as np
import pyzed.sl as sl
import cv2

# https://github.com/huy23tran11/Collision_Avoidance/blob/flight_test_2/src/main.cpp


# Initialize colors
blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)
line_thickness = 2
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1/2
font_thickness = 2
text1_loc = (200, 50)
text2_loc = (50, 300)
templ_w = 200 # dimension of the drone at 5m with 50% bigger 133/100 * 150, recommend 300 for safer avoiding
templ_h = 83 # dimention of the drone at 5m with 50% bigger 55/100 * 150

# Find best space that closet to the center then return true, return false if there is no space at all
# img_binary = after converting depth_image_ocv to a binary image
# templ_rect = cv::Rect
# center_rect = cv::Rect
def finding_best_space(img_binary):

    img_binary_h, img_binary_w, img_binary_channels = img_binary.shape

    # Create a template that is all black
    # To use with the "matchTemplate" method, the dim should be only 2
    templ = np.zeros((templ_h, templ_w), np.uint8)

    # Top left point of the frame in the center
    center_frame_tf = (int((img_binary_w / 2) - (templ_w / 2)), int((img_binary_h / 2) - (templ_h / 2)))
    center_rect = (center_frame_tf[0], center_frame_tf[1], templ_w, templ_h)

    # binary img need to be formatted to be used for template matching algothsm
    # convert to a gray
    img_binary_formatted = cv2.cvtColor(img_binary, cv2.COLOR_BGR2GRAY)

    # Crop the center of the image with a size of (templ_h x original image width)
    center_frame_start_x = center_frame_tf[1]
    center_frame_end_x = center_frame_tf[1]+templ_h+1
    img_binary_cropped = img_binary_formatted[center_frame_start_x:center_frame_end_x , :]

    # Top left point of the cropped frame
    cropped_frame_tf = (0, center_frame_tf[1])

    # cropped_img_binary_h, cropped_img_binary_w = img_binary_cropped.shape
    # start_point = (236, 0)
    # end_point = (436 , 83)
    # cv2.rectangle(img_binary, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), blue, line_thickness)
    # cv2.rectangle(img_binary, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), blue, line_thickness)

    # Refer to here to learn about cv::matchTemplate()
    # https://docs.opencv.org/3.4/de/da9/tutorial_template_matching.html
    # https://stackoverflow.com/questions/61133437/template-matching-with-multiple-objects-opencv-python

    # New way of finding matched spaces
    res = cv2.matchTemplate(img_binary_cropped, templ, cv2.TM_SQDIFF)

    threshold = 0.0 # 
    yloc, xloc = np.where(res == threshold) # only get the ones that are completely black

    # If the len > 0, then there is an open space
    if len(yloc) > 0:
        # Initialize with the 1st open space
        closest_space = (xloc[0], yloc[0])
        closest_space_euclidean_distance = euclidean_distance(closest_space, center_frame_tf)

        # Go thru all the open spaces and find the closest one from the center frame
        for (x,y) in zip(xloc, yloc):
            cur_space_top_left= (x, y+center_frame_tf[1])
            cur_euclidean_distance = euclidean_distance(cur_space_top_left, center_frame_tf)
            if cur_euclidean_distance < closest_space_euclidean_distance:
                closest_space_euclidean_distance = cur_euclidean_distance
                closest_space = cur_space_top_left

        templ_rect = (closest_space[0], closest_space[1], templ_w, templ_h)
        return (templ_rect, center_rect)
    # Return templ_rect as False, if there is no open space
    else:
        return (False, center_rect)

    # Check if there is any open space
    # if len(res) >= 1:

    #     threshold = 1.0
    #     all_open_spaces = np.where(res >= threshold) # only get the ones that are completely black

    #     for open_space in all_open_spaces:
    #         # cv2.rectangle(img_binary, (open_space[0]+cropped_frame_tf[0], open_space[1]+cropped_frame_tf[1]), 
    #         #                             (open_space[0]+cropped_frame_tf[0]+templ_w, open_space[1]+cropped_frame_tf[1]+templ_h), red, line_thickness)
    #         cv2.rectangle(img_binary_cropped, (open_space[0], open_space[1]), (open_space[0]+templ_w, open_space[1]+templ_h), red, line_thickness)

    #     # # cv2.imshow("Binary", img_binary)
    #     cv2.imshow("Binary", img_binary_cropped)


        # min_dist_space = finding_closest_space_to_center_ver2(all_open_spaces, center_frame_tf)

        # Instead of returning a tuple of 2 Falses,
        # Return as it is? 
        # The new method of fining the closest open space will return the center frame
        # if there is no obstacle in the center
        # templ_rect = (min_dist_space[0], min_dist_space[1], templ_w, templ_h)
        # return (templ_rect, center_rect)
    
    # when there is no open space
    # else:
    #     templ_rect = (0, 0, templ_w, templ_h)
    #     return (templ_rect, center_rect)

    # --------------------------------------------------------------
    # ----Original method of finding the open spaces
    # img_result_templ_matching = cv2.matchTemplate(img_binary_cropped, templ, cv2.TM_SQDIFF)

    # space_loc_tf = finding_all_available_spaces(img_result_templ_matching)

    # if len(space_loc_tf) <= 0:
    #     return (False, False)
    # else:
    #     min_dis_index = finding_closest_space_to_center(space_loc_tf, center_frame_tf)
    #     top_left = (cropped_frame_tf[0] + space_loc_tf[min_dis_index][0], cropped_frame_tf[1] + space_loc_tf[min_dis_index][1])
    #     templ_rect = (top_left[0], top_left[1], templ_w, templ_h)
    #     return (templ_rect, center_rect)


# Find all available spaces and returns them as a list
# We dont need this method? we can 
def finding_all_available_spaces(img_result_templ_matching):
    space_loc_tf = []
    print(img_result_templ_matching.shape)
    rows, cols = img_result_templ_matching.shape
    for i in range(rows):
        for j in range(cols):
            if img_result_templ_matching[i, j] == 0:
                space_loc_tf.append((j, i))
    return space_loc_tf

# Calculates the Euclidean distance between 2 points
def euclidean_distance(point1, point2):
    return math.sqrt(math.pow(point1[0]-point2[0], 2) + math.pow(point1[1]-point2[1], 2))

# Finds the closest open space from the center frame
def finding_closest_space_to_center_ver2(all_possible_spaces, center_frame_tf):
    min_dist = 999999
    min_dist_space = ''

    for space in all_possible_spaces:
        cur_dist = euclidean_distance(space, center_frame_tf)
        if cur_dist < min_dist:
            min_dist = cur_dist
            min_dist_space = space
    
    return min_dist_space


# From all available spaces, find the closet space to the center then return the index of that space rectangle in the vector
def finding_closest_space_to_center(space_loc_tf, center_frame_tf):
    distances = []

    # Go thru each space location and calculate the euclidean distance between the center frame
    # It uses the top left points
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

# rect is a tuple in (top_left_x, top_left_y, width, height)
def get_rect_bottom_right(rectangle):
    return (rectangle[0] + rectangle[2], rectangle[1] + rectangle[3])

def check_rect_matched(templ_rect, center_rect):
    error = 5
    return templ_rect[0] < center_rect[0] + error and templ_rect[0] > center_rect[0] - error


def put_text(image, is_space, is_matched, templ_rect, center_rect):
    if not is_space:
        cv2.putText(image, "STOP", text1_loc, font, font_scale, red, font_thickness)
        cv2.putText(image, "Obstacle Detected", text2_loc, font, font_scale, red, font_thickness)
    elif is_matched:
        cv2.putText(image, "Go to Location", text1_loc, font, font_scale, blue, font_thickness)
    elif not is_matched:
        # Slide Right
        if templ_rect[0] - center_rect[0] > 0:
            cv2.putText(image, "Slide Right", text1_loc, font, font_scale, blue, font_thickness)
            cv2.putText(image, "Obstacle Detected", text2_loc, font, font_scale, red, font_thickness)
        # Sldie Left
        else:
            cv2.putText(image, "Slide Left", text1_loc, font, font_scale, blue, font_thickness)
            cv2.putText(image, "Obstacle Detected", text2_loc, font, font_scale, red, font_thickness)
    else:
        cv2.putText(image, "STOP, Cannot Figure What To Do", text1_loc, font, font_scale, red, font_thickness)


def main():
    # ----------------------------------------------------------------
    # Need to add Pymovement here?
    # Skip for now
    # ----------------------------------------------------------------

    # Initialize a ZED camera and set the initial parameters
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    # Need to vary depth distances depending on testing environments
    init_params.depth_minimum_distance = 0.5
    init_params.depth_maximum_distance = 1

    # For Video Playback?
    # Refer to https://github.com/stereolabs/zed-examples/blob/master/svo%20recording/playback/python/svo_playback.py
    # and https://www.stereolabs.com/docs/video/using-video/
    # if len(sys.argv) != 2:
    #     print("Plese specify path to .svo file.")
    #     exit()

    # filepath = sys.argv[1]
    # init_params.set_from_svo_file(filepath)

    # Open the camera
    err = zed.open(init_params)

    # Check the status of the camera
    if err != sl.ERROR_CODE.SUCCESS:
        print("Error {}, exit program".format(err)) # Display the error
        exit()
    
    # A bool var 
    # is_moving_to_target = False

    # Set runtime parameters after opening the camera
    # To change the depth sensing mode to FILL
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.FILL

    # Get the image size
    image_size = zed.get_camera_information().camera_resolution
    print("Image Width: ", image_size.width)
    print("Image Height: ",image_size.height)

    # Declare your sl.Mat metrics
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    # ----------------------------------------------------------------
    # To count FPS?
    fps = 0

    # For saving
    final_real_img = ""
    final_binary_img = ""
    final_depth_img = ""

    # For video capturing
    # Skip for now
    # ----------------------------------------------------------------


    # Start the collision avoidance
    terminated = False
    while not terminated:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            
            # Take a color image
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            # Take a depth image
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            image_ocv = image_zed.get_data()
            depth_image_ocv = depth_image_zed.get_data()

            # Convert the depth img to binary
            ret, img_binary = cv2.threshold(depth_image_ocv, 5, 255, cv2.THRESH_BINARY)
            img_binary = cv2.cvtColor(img_binary, cv2.COLOR_BGRA2BGR)

            # Find the best space to move
            # templ_rect, center_rect = finding_best_space(img_binary)
            templ_rect, center_rect = finding_best_space(img_binary)

            # Draw rects on the center always
            cv2.rectangle(image_ocv, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), blue, line_thickness)
            cv2.rectangle(img_binary, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), blue, line_thickness)
            cv2.rectangle(depth_image_ocv, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), blue, line_thickness)


            # If there is an open space, draw the rectangles
            if templ_rect:
                # Check if the open space is the center
                is_matched = check_rect_matched(templ_rect, center_rect)

                cv2.rectangle(image_ocv, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), red, line_thickness)
                cv2.rectangle(img_binary, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), red, line_thickness)
                cv2.rectangle(depth_image_ocv, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), red, line_thickness)
                put_text(img_binary, True, is_matched, templ_rect, center_rect)
                put_text(image_ocv, True, is_matched, templ_rect, center_rect)
            # If there is no open space, display no open space
            else:
                put_text(img_binary, False, is_matched, templ_rect, center_rect)
                put_text(image_ocv, False, is_matched, templ_rect, center_rect)

            # Check if there is a space
            # is_space = False
            # if templ_rect and center_rect:
            #     is_space = True

            # is_matched = check_rect_matched(templ_rect, center_rect)

            # Draw rects on the center always
            # cv2.rectangle(image_ocv, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), red, line_thickness)
            # cv2.rectangle(img_binary, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), red, line_thickness)
            # cv2.rectangle(depth_image_ocv, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), red, line_thickness)

            # Check the 'templ_rect' if there is any open space
            # If the x coordinate of the 'templ_rect' is 0, then there is no space
            # if templ_rect[0] != 0:
            #     cv2.rectangle(image_ocv, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), blue, line_thickness)
            #     cv2.rectangle(img_binary, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), blue, line_thickness)
            #     cv2.rectangle(depth_image_ocv, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), blue, line_thickness)
            #     put_text(img_binary, True, is_matched, templ_rect, center_rect)
            #     put_text(image_ocv, True, is_matched, templ_rect, center_rect)
            # else:
            #     put_text(img_binary, False, is_matched, templ_rect, center_rect)
            #     put_text(image_ocv, False, is_matched, templ_rect, center_rect)



            # Only draw rects and put texts if it found a space
            # It displays nothing when there is no obstacle in the center frame
            # if is_space:
            #     # Drawing rectangles
            #     # Template Rect = Blue
            #     # Center Rect = Red
            #     # OCV image
            #     cv2.rectangle(image_ocv, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), blue, line_thickness)
            #     cv2.rectangle(image_ocv, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), red, line_thickness)

            #     # Binary image
            #     cv2.rectangle(img_binary, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), blue, line_thickness)
            #     cv2.rectangle(img_binary, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), red, line_thickness)

            #     # OCV depth image
            #     cv2.rectangle(depth_image_ocv, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), blue, line_thickness)
            #     cv2.rectangle(depth_image_ocv, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), red, line_thickness)
                

            #     # Check if templ_rect and center_rect match within the error of margin
            #     # Do we need to check if templ_rect and center_rect match?
            #     # If there is no obstacle in the center, then there is no need to check 
            #     # and it's safe to move forward
            #     is_matched = check_rect_matched(templ_rect, center_rect)

            #     # Put text
            #     put_text(img_binary, is_space, is_matched, templ_rect, center_rect)
            #     put_text(image_ocv, is_space, is_matched, templ_rect, center_rect)

            # Show images
            cv2.imshow("Real", image_ocv)
            cv2.imshow("Binary", img_binary)
            cv2.imshow("Depth", depth_image_ocv)

            # ----------------------------------------------------------------
            # Calls the manuver method
            # Skip for now

            # Save image
            # Skip for now
            # ----------------------------------------------------------------

        # Need to check for the key
        # if pressed == 'q', terminate
        key = cv2.waitKey(1)
        if key == 81 or key == 113: # ASCII codes of 'q' and 'Q'
            terminated = True

    cv2.destroyAllWindows()
    zed.close()


if __name__ == "__main__":
    main()

    