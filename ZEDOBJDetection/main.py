import sys
import math
import numpy as np
import pyzed.sl as sl
import cv2

# Initialize colors
blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)
black = (255, 255, 255)
line_thickness = 2
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1
font_thickness = 1

# https://www.stereolabs.com/docs/api/classsl_1_1Objects.html
# API for Object Detection
def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.sdk_verbose = True

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Define the Objects detection module parameters
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking=True
    obj_param.image_sync=True
    obj_param.enable_mask_output=True

    # Object tracking requires the positional tracking module
    camera_infos = zed.get_camera_information()
    if obj_param.enable_tracking :
        zed.enable_positional_tracking()

    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS :
        print (repr(err))
        zed.close()
        exit(1)
    
    # Detection Ouqqtput
    objects = sl.Objects()
    # Detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

    image_size = zed.get_camera_information().camera_resolution
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    terminated = False
    while not terminated:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed_error = zed.retrieve_objects(objects, obj_runtime_param)

            # Take a color image
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            image_ocv = image_zed.get_data()

            for object in objects.object_list:
                detected_label = str(object.label)
                if detected_label == 'Person' or detected_label == 'Vehicle':
                    top_left = (int(object.bounding_box_2d[0][0]), int(object.bounding_box_2d[0][1]))
                    bottom_right = (int(object.bounding_box_2d[2][0]), int(object.bounding_box_2d[2][1]))
                    cv2.rectangle(image_ocv, top_left, bottom_right, blue, line_thickness)
                    # if the thickness = -1, it fills the rectangle
                    # add a small rectangle behind the label text?
                    # cv2.rectangle(image_ocv, top_left, bottom_right, blue, line_thickness)
                    cv2.putText(image_ocv, detected_label, top_left, font, font_scale, red, font_thickness)


            cv2.imshow("Real", image_ocv)

        # Need to check for the key
        # if pressed == 'q', terminate
        key = cv2.waitKey(1)
        if key == 81 or key == 113: # ASCII codes of 'q' and 'Q'
            terminated = True

    # Disable object detection and close the camera
    zed.disable_object_detection()
    zed.close()
    return 0


if __name__== "__main__":
    main()

