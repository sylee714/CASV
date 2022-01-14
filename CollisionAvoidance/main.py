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
    # 1. connectionFunc
    # 2. setLocation
    # 3. arm_and_takeoff
    # 4. setTargetLoc


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
                put_text(img_binary, False, False, templ_rect, center_rect)
                put_text(image_ocv, False, False, templ_rect, center_rect)

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

    