import sys
import math
import numpy as np
import pyzed.sl as sl
import cv2
from PyMovement import *
from datetime import datetime

# https://github.com/huy23tran11/Collision_Avoidance/blob/flight_test_2/src/main.cpp


# Initialize colors
blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)
line_thickness = 2
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1/2 # for the open space detection
font_thickness = 2 # for the open space detection
font_scale_2 = 0.3 # for obj detection
font_thickness_2 = 1 # for obj detection
templ_w = 200 # dimension of the drone at 5m with 50% bigger 133/100 * 150, recommend 300 for safer avoiding
templ_h = 83 # dimention of the drone at 5m with 50% bigger 55/100 * 150
is_moving_to_target = False # A global var to track whether the drone is moving to the target loc

# Video Dir Paths
real_video_path = "Videos/Real/"
binary_video_path = "Videos/Binary/"
video_fps = 15


# Finds best space that closet to the center then return true, return false if there is no space at all
# img_binary = after converting depth_image_ocv to a binary image
# templ_rect = cv::Rect
# center_rect = cv::Rect
def finding_best_space(img_binary):
    # Get the shape of the binary image
    img_binary_h, img_binary_w, img_binary_channels = img_binary.shape

    # Create a template that is all black
    # To use with the "matchTemplate" method, the dim should be only 2
    templ = np.zeros((templ_h, templ_w), np.uint8)

    # Top left point of the frame in the center
    center_frame_tf = (int((img_binary_w / 2) - (templ_w / 2)), int((img_binary_h / 2) - (templ_h / 2)))
    center_rect = (center_frame_tf[0], center_frame_tf[1], templ_w, templ_h)

    # binary img need to be formatted to be used for template matching algorithm
    # convert to a gray
    img_binary_formatted = cv2.cvtColor(img_binary, cv2.COLOR_BGR2GRAY)

    # Crop the center of the image with a size of (templ_h x original image width)
    center_frame_start_y = center_frame_tf[1]
    center_frame_end_y = center_frame_tf[1]+templ_h+1
    img_binary_cropped = img_binary_formatted[center_frame_start_y:center_frame_end_y , :]

    # Do the template mathcing on the cropped image
    res = cv2.matchTemplate(img_binary_cropped, templ, cv2.TM_SQDIFF)

    threshold = 0.0 # Since we are using "TM_SQDIFF", we need to find the minimum
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

# Calculates the Euclidean distance between 2 points
def euclidean_distance(point1, point2):
    return math.sqrt(math.pow(point1[0]-point2[0], 2) + math.pow(point1[1]-point2[1], 2))

# rect is a tuple in (top_left_x, top_left_y, width, height)
def get_rect_bottom_right(rectangle):
    return (rectangle[0] + rectangle[2], rectangle[1] + rectangle[3])

# Checks if the found open space template rect is close/same to the center rect
def check_rect_matched(templ_rect, center_rect):
    error = 5
    return templ_rect[0] < center_rect[0] + error and templ_rect[0] > center_rect[0] - error

# Displays texts depending on a situation
def put_text(image, is_space, is_matched, templ_rect, center_rect):
    # text1_loc = (center_rect.x, center_rect.y)
    # text2_loc = (center_rect.x, center_rect.y - 20)
    text1_loc = (center_rect[0], center_rect[1])
    text2_loc = (center_rect[0], center_rect[1] - 20)
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

# Sends the appropriate command to the drone
def manuver(is_space, is_matched, templ_Rect, center_Rect):
    if not is_space:
        if is_moving_to_target:
            stop()
            is_moving_to_target = False
    elif is_matched:
        if not is_moving_to_target:
            goToTargetLoc()
            is_moving_to_target = True
    elif not is_matched:
        if temp_Rect[0] - center_Rect[0] > 0:
            slide_right()
            is_moving_to_target = False
        else:
            slide_left()
            is_moving_to_target = False
    else:
        if is_moving_to_target:
            stop()
            is_moving_to_target = False

# Convert the passed in image into the correct image format to save it
# Need to convert twice for some reason
def correct_real_img_format(real_image):
    correct_real_image = cv2.cvtColor(real_image, cv2.COLOR_RGB2BGR)
    return cv2.cvtColor(correct_real_image, cv2.COLOR_RGB2BGR)

def main():
    # --------------------------------------------------------------
    # PyMovement Functions
    # 1. connectionFunc
    # 2. setLocation
    # 3. arm_and_takeoff
    # 4. setTargetLoc

    # Call the connection function
    connectionFunc()

    # Set the home location
    setLocation()

    # Arm and takeoff
    arm_and_takeoff()

    # Set the target location
    setTargetLoc()
    # --------------------------------------------------------------

    # Initialize a ZED camera and set the initial parameters
    zed = sl.Camera()
    init_params = sl.InitParameters()
    # init_params.camera_resolution = sl.RESOLUTION.VGA # only with the openspace detection
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    # Need to vary depth distances depending on testing environments
    init_params.sdk_verbose = True
    init_params.depth_minimum_distance = 0.5
    init_params.depth_maximum_distance = 1

    # Open the camera
    err = zed.open(init_params)

    # Check the status of the camera
    if err != sl.ERROR_CODE.SUCCESS:
        print("Error {}, exit program".format(err)) # Display the error
        exit()
    
    # Define the Objects detection module parameters
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking=True
    obj_param.image_sync=True
    obj_param.enable_mask_output=True

    # Set runtime parameters after opening the camera
    # To change the depth sensing mode to FILL
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.FILL

    # Object tracking requires the positional tracking module
    camera_infos = zed.get_camera_information()
    if obj_param.enable_tracking :
        zed.enable_positional_tracking()

    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS :
        print (repr(err))
        zed.close()
        exit(1)
    
    # Detection Output
    objects = sl.Objects()
    # Detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

    # Get the image size
    image_size = zed.get_camera_information().camera_resolution
    print("Image Width: ", image_size.width)
    print("Image Height: ",image_size.height)

    # Declare your sl.Mat metrics
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    # For obj detection displays
    # obj_det_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    obj_det_depth_map = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    # obj_det_depth_for_display = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)


    # For video capturing
    cur_date_time = datetime.now()
    # Open Space Det
    dt_string = cur_date_time.strftime("%m_%d_%Y_%H_%M_%S")
    real_video = cv2.VideoWriter(real_video_path + dt_string + ".avi", 
                                            cv2.VideoWriter_fourcc(*'MJPG'), video_fps, 
                                            (image_size.width, image_size.height))
    binary_video = cv2.VideoWriter(binary_video_path + dt_string + ".avi", 
                                            cv2.VideoWriter_fourcc(*'MJPG'), video_fps, 
                                            (image_size.width, image_size.height))
    # ----------------------------------------------------------------


    # Start the collision avoidance
    terminated = False
    while not terminated:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            
            zed_error = zed.retrieve_objects(objects, obj_runtime_param)

            # Take a color image
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)

            # Take a depth image
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
            zed.retrieve_measure(obj_det_depth_map, sl.MEASURE.DEPTH)

            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            image_ocv = image_zed.get_data()
            depth_image_ocv = depth_image_zed.get_data()

            # Convert the depth img to binary
            ret, img_binary = cv2.threshold(depth_image_ocv, 5, 255, cv2.THRESH_BINARY)
            img_binary = cv2.cvtColor(img_binary, cv2.COLOR_BGRA2BGR)

            # ---------------------------OPEN SPACE DETECTION----------------------------
            # Find the best space to move
            templ_rect, center_rect = finding_best_space(img_binary)

            # Draw rects on the center always
            cv2.rectangle(image_ocv, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), blue, line_thickness)
            cv2.rectangle(img_binary, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), blue, line_thickness)
            cv2.rectangle(depth_image_ocv, (center_rect[0], center_rect[1]), get_rect_bottom_right(center_rect), blue, line_thickness)

            # If there is an open space, draw the rectangles
            if templ_rect:
                # Check if the open space is the center
                is_matched = check_rect_matched(templ_rect, center_rect)

                # Draw rects and put texts
                cv2.rectangle(image_ocv, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), red, line_thickness)
                cv2.rectangle(img_binary, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), red, line_thickness)
                # cv2.rectangle(depth_image_ocv, (templ_rect[0], templ_rect[1]), get_rect_bottom_right(templ_rect), red, line_thickness)
                put_text(img_binary, True, is_matched, templ_rect, center_rect)
                put_text(image_ocv, True, is_matched, templ_rect, center_rect)

                # Send the command
                manuver(True, is_matched, templ_rect, center_rect)

            # If there is no open space, display no open space
            else:
                # Put Texts
                put_text(img_binary, False, False, templ_rect, center_rect)
                put_text(image_ocv, False, False, templ_rect, center_rect)

                # Send the command
                manuver(False, is_matched, templ_rect, center_rect)

            # ---------------------------OPEN SPACE DETECTION----------------------------

            # ---------------------------OBJECT DETECTION----------------------------
            for object in objects.object_list:
                detected_label = str(object.label)
                if detected_label == 'Person' or detected_label == 'Vehicle':
                    top_left = (int(object.bounding_box_2d[0][0]), int(object.bounding_box_2d[0][1]))
                    bottom_right = (int(object.bounding_box_2d[2][0]), int(object.bounding_box_2d[2][1]))
                    cv2.rectangle(image_ocv, top_left, bottom_right, green, line_thickness)
                    cv2.rectangle(img_binary, top_left, bottom_right, green, line_thickness)
                    obj_center = ((top_left[0]+bottom_right[0])//2, (top_left[1]+bottom_right[1])//2)
                    err, depth_value = obj_det_depth_map.get_value(obj_center[0], obj_center[1])
                    label_and_depth = detected_label + " || " + str(round(depth_value, 3)) + " || OBJ Coordinate: Top Left = " + str(top_left) + ", Bottom Right = " + str(bottom_right) + ", Center Pixel = " + str(obj_center)
                    # if the thickness = -1, it fills the rectangle
                    # add a small rectangle behind the label text?
                    # cv2.rectangle(image_ocv, top_left, bottom_right, blue, line_thickness)
                    cv2.putText(image_ocv, label_and_depth, top_left, font, font_scale, red, font_thickness)
                    cv2.putText(img_binary, label_and_depth, top_left, font, font_scale, red, font_thickness)

            # ---------------------------OBJECT DETECTION----------------------------
            
            # ----------------------------------------------------------------
            # Display images
            cv2.imshow("Open Space Det Real", image_ocv)
            cv2.imshow("Open Space Det Binary", img_binary)
            # ----------------------------------------------------------------

            # ----------------------------------------------------------------
            # Save videos
            bgr_image_ocv = correct_real_img_format(image_ocv)
            bgr_img_binary = cv2.cvtColor(img_binary, cv2.COLOR_RGB2BGR)
            real_video.write(bgr_image_ocv)
            binary_video.write(bgr_img_binary)
            # ----------------------------------------------------------------

        # Need to check for the key
        # if pressed == 'q', terminate
        key = cv2.waitKey(1)
        if key == 81 or key == 113: # ASCII codes of 'q' and 'Q'
            terminated = True

    real_video.release()
    binary_video.release()
    cv2.destroyAllWindows()
    zed.close()


if __name__ == "__main__":
    main()

    