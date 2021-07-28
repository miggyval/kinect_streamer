# From Python
# It requires OpenCV installed for Python
import sys
import cv2
import os
import numpy as np
import math
from sys import platform
import argparse

def main():
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
        args = parser.parse_known_args()

        # Custom Params (refer to include/openpose/flags.hpp for more parameters)
        params = dict()
        params["model_folder"] = "/home/medrobotics/openpose/models/"
        params["net_resolution"] = "-1x352"
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
        for i in range(10000):
            datum = op.Datum()
            string = '/home/medrobotics/kinect_ws/src/kinect_streamer/test/color/' + str(i) + '.bin'
            imageToProcess = np.fromfile(string, dtype=np.uint8)
            imageToProcess = np.reshape(imageToProcess, (1080, 1920, 4))
            imageToProcess = imageToProcess[:, :, :3].copy()
            datum.cvInputData = imageToProcess
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))
            out = datum.cvOutputData
            if datum.poseKeypoints is not None:
                for j in range(datum.poseKeypoints.shape[0]):
                    mean = np.zeros(shape=(2,))
                    variance = np.zeros(shape=(2,))
                    count = 0
                    for k in [0, 14, 15, 16, 17]:
                        if datum.poseKeypoints[j][k][2] != 0.0:
                            count += 1
                    if count == 0:
                        break
                    for k in [0, 14, 15, 16, 17]:
                        if datum.poseKeypoints[j][k][2] != 0.0:
                            mean += datum.poseKeypoints[j][k][:2] / count
                    for k in [0, 14, 15, 16, 17]:
                        if datum.poseKeypoints[j][k][2] != 0.0:
                            variance += (datum.poseKeypoints[j][k][:2] - mean) * (datum.poseKeypoints[j][k][:2] - mean) / count
                    std_dev = np.sqrt(variance)
                    x1 = min(max(int(mean[0] - 3 * std_dev[0]), 0), 1920)
                    x2 = min(max(int(mean[0] + 3 * std_dev[0]), 0), 1920)
                    y1 = min(max(int(mean[1] - 3 * std_dev[1]), 0), 1080)
                    y2 = min(max(int(mean[1] + 3 * std_dev[1]), 0), 1080)
                    face = out[y1:y2, x1:x2]
                    if face is not None and face.size != 0:
                        face = cv2.GaussianBlur(face, (101, 101), 100)
                    out[y1:y2, x1:x2] = face
            cv2.imshow("OpenPose 1.7.0 - Tutorial Python API", out)
            if cv2.waitKey(1) == ord('q'):
                sys.exit(1)
    except Exception as e:
        print(e)
        sys.exit(-1)

if __name__ == "__main__":
    main()
