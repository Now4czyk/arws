#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# this is how to import custom messages
from ur_custom_interfaces.msg import URCommand
import cv2
import argparse

from ultralytics import YOLO
import supervision as sv

import depthai as dai
import logging
import numpy as np

RESOLUTION_X = 1280
RESOLUTION_Y = 720
ACCURACY_X = 100
ACCURACY_Y = 80

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = True

# Create pipeline
pipeline = dai.Pipeline()

# Define source and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
xoutPreview = pipeline.create(dai.node.XLinkOut)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutPreview.setStreamName("preview")
xout.setStreamName("disparity")
xoutDepth.setStreamName("depth")

# Properties
camRgb.setPreviewSize(RESOLUTION_X, RESOLUTION_Y)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(True)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
# depth.initialConfig.setConfidenceThreshold(200)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(subpixel)

# Linking
camRgb.preview.link(xoutPreview.input)
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)
depth.depth.link(xoutDepth.input)

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 live")
    parser.add_argument(
        "--webcam-resolution", 
        default=[RESOLUTION_X, RESOLUTION_Y], 
        nargs=2, 
        type=int
    )
    args = parser.parse_args()
    return args


class CameraNode (Node):
    def __init__ (self):
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger ().info("Hello XXXXXXXXXXXX Camera Node")
        self.create_timer (0.5, self.timer_callback)
        self.publisher_ = self.create_publisher(URCommand, "custom_camera", 1)
        # self.prevX = 0
        # self.prevY = 0
    
    def timer_callback(self):
        self.counter_ += 1    
        self.get_logger().info("[TEST LOG]:" + str(self.counter_))

    def recognizing_apples(self):
        with dai.Device(pipeline) as device:
            logging.getLogger("ultralytics").setLevel(logging.ERROR)

            model = YOLO("yolov8n.pt")

            box_annotator = sv.BoxAnnotator(
                thickness=1,
                text_thickness=1,
                text_scale=1
            )

            previewQueue = device.getOutputQueue('preview')
            disparityQueue = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

            while True:
                previewFrame = previewQueue.get().getFrame()
                depthFrame = depthQueue.get().getFrame()
                disparityFrame = disparityQueue.get().getFrame() 
                disparityFrame = (disparityFrame * (255 / depth.initialConfig.getMaxDisparity())).astype(np.uint8)

                result = model(previewFrame, agnostic_nms=True)[0]
                detections = sv.Detections.from_yolov8(result)

                #Filter only apples
                detections = detections[detections.class_id == 47]

                labels = [
                    f"{model.model.names[class_id]} {confidence:0.2f}"
                    for _, confidence, class_id, _
                    in detections
                ]
                yoloFrame = box_annotator.annotate(
                    scene=previewFrame, 
                    detections=detections, 
                    labels=labels
                )

                objectBounds = detections.xyxy

                # print('SHARE yoloFrame', yoloFrame.shape)
                # print('SHARE depthFrame', depthFrame.shape)

                if len(objectBounds) > 0:
                    xLeft = objectBounds[0][0]
                    xRight = objectBounds[0][2]
                    yTop = objectBounds[0][1]
                    yBottom = objectBounds[0][3]
                    xPoint = int(xLeft + (xRight-xLeft)/2)
                    yPoint = int(yTop + (yBottom-yTop)/2)
                    # Przeliczenie punktu z obrazu rgb na rozdzielczość kamery głębi
                    # Rozdzielczość kamery głębi: 640 x 400
                    # Rozdzielczość kamery rgb: RESOLUTION_X x RESOLUTION_Y
                    xForDepth = int(xPoint * 640 / RESOLUTION_X)
                    yForDepth = int(yPoint * 400 / RESOLUTION_Y)
                    # print(f'xLeft: {xLeft}, xRight: {xRight}, yTop: {yTop}, yBottom: {yBottom}')
                    # print('Depth', depthFrame[yForDepth][xForDepth], f'xD: {xForDepth}, yD: {yForDepth}, xP: {xPoint}, yP: {yPoint}')
                    cv2.circle(yoloFrame, (xPoint, yPoint), 2, (0, 0, 255), 5)
                    cv2.putText(
                        yoloFrame, 
                        f"{depthFrame[yForDepth][xForDepth]/10}cm", 
                        (xPoint, yPoint), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, 
                        (255, 255, 255), 
                        1, 
                    )
                    cv2.putText(
                        yoloFrame, 
                        f"x:{self.get_factor(xPoint, RESOLUTION_X/2)} y:{self.get_factor(yPoint, RESOLUTION_Y/2)}", 
                        (xPoint, yPoint+20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, 
                        (255, 255, 255), 
                        1, 
                    )

                    # p1 = RESOLUTION_X/2 - ACCURACY_X, RESOLUTION_Y/2 - ACCURACY_Y
                    # p2 = RESOLUTION_X/2 + ACCURACY_X, RESOLUTION_Y/2 + ACCURACY_Y

                    # cv2.rectangle(
                    #     yoloFrame, 
                    #     p1, 
                    #     p2, 
                    #     (0, 255, 0), 
                    #     1
                    # )

                    # Publikacja do naszego tematu
                    # xFactCurr = self.get_factor(xPoint, RESOLUTION_X/2)
                    # yFactCurr = self.get_factor(yPoint, RESOLUTION_Y/2)
                    
                    # if xFactCurr == self.prevX and yFactCurr == self.prevY:
                    #     pass
                    # else:
                    self.publish_command(xPoint, yPoint, depthFrame[yForDepth][xForDepth])

                    # self.prevX = xFactCurr
                    # self.prevY = yFactCurr

                depthFrame = (depthFrame * (255 / 10000)).astype(np.uint8)


                # detections.xyxy [xLeft, yTop, xRight, yBottom]
                # x -> (min, max) = (0, RESOLUTION_X)
                # y -> (min, max) = (0, RESOLUTION_Y)
                cv2.imshow("disparity", disparityFrame)
                cv2.imshow("depthFrame", depthFrame)
                cv2.imshow("yolov8", yoloFrame)

                if cv2.waitKey(1) == ord('q'):
                    break

    def get_factor(self, position, boundary, accuracy = 30):
        factor = 0

        if position < boundary - accuracy:
            factor = -1
        elif position > boundary + accuracy:
            factor = 1
        else:
            factor = 0

        return factor

    def publish_command(self, x, y, depth):
        xFactor = self.get_factor(x, RESOLUTION_X/2, ACCURACY_X)
        yFactor = self.get_factor(y, RESOLUTION_Y/2, ACCURACY_Y)

        print('Publishing command with data:', xFactor, yFactor, depth)
        msg = URCommand()
        msg.x = str(xFactor)
        msg.y = str(yFactor)
        msg.depth = str(depth)
        self.publisher_.publish(msg)
        print('Published command')

def main (args=None):
    rclpy.init(args=args)
    node = CameraNode()

    node.recognizing_apples()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()