# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
import numpy as np
from sys import platform
import argparse

try:
    # Import Openpose (Windows/Ubuntu/OSX)
    dir_path = os.path.dirname(os.path.realpath(__file__))
    try:
        # Windows Import
        if platform == "win32":
            # Change these variables to point to the correct folder (Release/x64 etc.)
            sys.path.append(dir_path + '/../../python/openpose/Release')
            os.environ['PATH']  = os.environ['PATH'] + ';' + dir_path + '/../../x64/Release;' +  dir_path + '/../../bin;'
            import pyopenpose as op
        else:
            # Change these variables to point to the correct folder (Release/x64 etc.)
            # sys.path.append('/usr/local/python')
            # If you run `make install` (default path is `/usr/local/python` for Ubuntu), you can also access the OpenPose/python module from there. This will install OpenPose and the python library at your desired installation path. Ensure that this is in your python path in order to use it.
            sys.path.append('/usr/local/python')
            from openpose import pyopenpose as op
    except ImportError as e:
        print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
        raise e

    # Flags
    parser = argparse.ArgumentParser()
    parser.add_argument("--image_path", default="/home/s4626478/cheese.png", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
    args = parser.parse_known_args()

    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    params = dict()
    params["model_folder"] = "/home/s4626478/openpose/models/"
    #params["face"] = True
    #params["face_detector"] = 2

    # Add others in path?
    for i in range(0, len(args[1])):
        curr_item = args[1][i]
        if i != len(args[1])-1: next_item = args[1][i+1]
        else: next_item = "1"
        if "--" in curr_item and "--" in next_item:
            key = curr_item.replace('-','')
            if key not in params:  params[key] = "1"
        elif "--" in curr_item and "--" not in next_item:
            key = curr_item.replace('-','')
            if key not in params: params[key] = next_item

    # Construct it from system arguments
    # op.init_argv(args[1])
    # oppython = op.OpenposePython()

    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    # Process Image
    for i in range(10, 436):
        datum = op.Datum()
        string = '/home/s4626478/kinect_streamer/blah/color/' + str(i) + '.bin'
        imageToProcess = np.fromfile(string, dtype=np.uint8)
        imageToProcess = np.reshape(imageToProcess, (1080, 1920, 4))
        faceRectangles = [
            op.Rectangle(330.119385, 277.532715, 48.717274, 48.717274),
            op.Rectangle(24.036991, 267.918793, 65.175171, 65.175171),
            op.Rectangle(151.803436, 32.477852, 108.295761, 108.295761),
        ]

        datum.faceRectangles = faceRectangles
        
        imageToProcess = imageToProcess[:, :, :3].copy()
        datum.cvInputData = imageToProcess
        opWrapper.emplaceAndPop(op.VectorDatum([datum]))
        #print("Face keypoints: \n" + str(datum.faceKeypoints))
        #Display Image
        #print("Body keypoints: \n" + str(datum.poseKeypoints))
        cv2.imshow("OpenPose 1.7.0 - Tutorial Python API", datum.cvOutputData)
        cv2.waitKey(1)
except Exception as e:
    print(e)
    sys.exit(-1)
