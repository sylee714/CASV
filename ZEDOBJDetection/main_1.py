import sys
import math
import numpy as np
import pyzed.sl as sl
import cv2

blue = (255, 0, 0)
green = (0, 255, 0)
red = (0, 0, 255)
line_thickness = 2

def main():
    # Create the camera
    zed = sl.Camera()

    # Initialize the camera
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.sdk_verbose = True

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Define the Objects detection module parameters
    obj_param = sl.ObjectDetectionParameters()
    obj_param.enable_tracking = True
    obj_param.image_sync = True
    obj_param.enable_mask_output = True
    
    # Object tracking requires the positional tracking module
    camera_infos = zed.get_camera_information()
    if obj_param.enable_tracking:
        zed.enable_positional_tracking()

    err = zed.enable_object_detection(obj_param)
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(err))
        zed.close()
        exit(1)

    # Detection Output
    objects = sl.Objects()

    # Detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()

    image_size = zed.get_camera_information().camera_resolution
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    
    terminated = False
    while not terminated:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed_error = zed.retrieve_objects(objects, obj_runtime_param)

            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            image_ocv = image_zed.get_data()
            print(type(image_ocv))

            for object in objects.object_list:
                object_id = object.id # Get the object id
                object_position = object.position # Get the object position
                object_velocity = object.velocity # Get the object velocity
                object_tracking_state = object.tracking_state # Get the tracking state of the object
                object_2Dbbox = object.bounding_box_2d # Get the 2D bounding box of the object
                print("Object ID: ",  object_id)
                print("Object Position: ",  object_position)
                print("Object Velocity: ", object_velocity)
                print("Object Tracking State: ", object_tracking_state)
                cv2.rectangle(image_ocv, (int(object_2Dbbox[0][0]), int(object_2Dbbox[0][1])), (int(object_2Dbbox[2][0]), int(object_2Dbbox[2][1])), blue, line_thickness)
                # if object_tracking_state == sl.OBJECT_TRACK_STATE.OK :
                #     print("Object {0} is tracked\n".format(object_id))
            
            cv2.imshow("Real", image_ocv)

        # if objects.is_new:
        #     print(str(len(objects.object_list))+" Object(s) detected ("+str(zed.get_current_fps())+" FPS)")
        key = cv2.waitKey(1)
        if key == 81 or key == 113: # ASCII codes of 'q' and 'Q'
            terminated = True

    # Disable object detection and close the camera
    zed.disable_object_detection()
    zed.close()

    # ---------------------------------------------------------------------------
    # # Set initialization parameters
    # detection_parameters = sl.ObjectDetectionParameters()
    # detection_parameters.enable_tracking = True

    # # Set runtime parameters
    # detection_parameters_rt = sl.ObjectDectectionRuntimeParameters()
    # detection_parameters_rt.detection_confidence_threshold = 25

    # # Choose a detection model
    # detection_parameters.detection_model = sl.DETECTION_MODEL.MULTI_CLASS_BOX

    # # -----------------------
    # # Eneable positional tracking with default parameters
    # # tracking_parameters = sl.PositionalTrackingParameters()
    # # err = zed.enable_positional_tracking(tracking_parameters)
    # # -----------------------

    # if detection_parameters.enable_tracking:
    #     # Set positional tracking parameters
    #     positional_tracking_parameters = sl.PositionalTrackingParameters()
    #     # Enable positional tracking
    #     zed.enable_positional_tracking(positional_tracking_parameters)
    
    # # Enable object detection with initialization parameters
    # zed_error = zed.enable_object_detection(detection_parameters)
    # if zed_error != sl.ERROR_CODE.SUCCESS:
    #     print("enable_object_detection", zed_error, "\nExit program")
    #     zed.close()
    #     exit(-1)
    # ---------------------------------------------------------------------------

    



if __name__== "__main__":
    main()

