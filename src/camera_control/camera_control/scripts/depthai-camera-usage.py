#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
pipeline = dai.Pipeline()

# Define source and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
xoutPreview = pipeline.create(dai.node.XLinkOut)

xoutPreview.setStreamName("preview")

# Properties
camRgb.setPreviewSize(600, 600)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(True)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Linking
camRgb.preview.link(xoutPreview.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    preview = device.getOutputQueue('preview')

    while True:
        previewFrame = preview.get()

        cv2.imshow("preview", previewFrame.getFrame())

        if cv2.waitKey(1) == ord('q'):
            break