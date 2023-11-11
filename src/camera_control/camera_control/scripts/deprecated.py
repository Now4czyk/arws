import cv2
import argparse

from ultralytics import YOLO
import supervision as sv

import depthai as dai
import logging

# Create pipeline
pipeline = dai.Pipeline()

# Define source and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
xoutPreview = pipeline.create(dai.node.XLinkOut)

xoutPreview.setStreamName("preview")

# Properties
camRgb.setPreviewSize(1280, 720)
camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(True)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Linking
camRgb.preview.link(xoutPreview.input)

def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLOv8 live")
    parser.add_argument(
        "--webcam-resolution", 
        default=[1280, 720], 
        nargs=2, 
        type=int
    )
    args = parser.parse_args()
    return args

with dai.Device(pipeline) as device:
    args = parse_arguments()
    frame_width, frame_height = args.webcam_resolution
    logging.getLogger("ultralytics").setLevel(logging.ERROR)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

    model = YOLO("yolov8n.pt")

    box_annotator = sv.BoxAnnotator(
        thickness=1,
        text_thickness=1,
        text_scale=1
    )

    preview = device.getOutputQueue('preview')

    while True:
        previewFrame = preview.get().getFrame()

        result = model(previewFrame, agnostic_nms=True)[0]
        detections = sv.Detections.from_yolov8(result)

        #Filter only apples
        detections = detections[detections.class_id == 47]

        labels = [
            f"{model.model.names[class_id]} {confidence:0.2f}"
            for _, confidence, class_id, _
            in detections
        ]
        frame = box_annotator.annotate(
            scene=previewFrame, 
            detections=detections, 
            labels=labels
        )

        # detections.xyxy [xLeft, yTop, xRight, yBottom]
        # x -> (min, max) = (0, 1280)
        # y -> (min, max) = (0, 720)
        cv2.imshow("yolov8", frame)

        if cv2.waitKey(1) == ord('q'):
            break