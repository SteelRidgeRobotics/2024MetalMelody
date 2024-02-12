import cv2
import numpy as np

from cscore import CameraServer as CS

from constants import *


def main():
    CS.enableLogging()

    # Get the UsbCamera from CameraServer
    camera = CS.startAutomaticCapture()
    # Set the resolution
    camera.setResolution(LimelightConstants.RESOLUTIONX, LimelightConstants.RESOLUTIONY)

    # Get a CvSink. This will capture images from the camera
    cvSink = CS.getVideo()
    # Setup a CvSource. This will send images back to the Dashboard
    outputStream = CS.putVideo("limelight", LimelightConstants.RESOLUTIONX, LimelightConstants.RESOLUTIONY)

    # Allocating new images is very expensive, always try to preallocate
    mat = np.zeros(shape=(LimelightConstants.RESOLUTIONY, LimelightConstants.RESOLUTIONX, 3), dtype=np.uint8)

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, mat = cvSink.grabFrame(mat)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue

        # Put a rectangle on the image
        cv2.rectangle(mat, (100, 100), (400, 400), (255, 255, 255), 5)

        # Give the output stream a new image to display
        outputStream.putFrame(mat)
