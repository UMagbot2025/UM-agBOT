import os
import cv2
import argparse
import numpy as np
import onnxruntime as ort
from dataclasses import dataclass


@dataclass
class Constants:
    model_path: str = "./trained_yolo_agbot2025.onnx"
    confidence_threshold: float = 0.4
    min_box_area: float = 500.0
    max_eggs_in_frame: int = 20
    stop_after_first_detection: bool = True
    input_size: int = 640  # Change if your model uses a different size


def load_model(model_path):
    return ort.InferenceSession(model_path)


def letterbox(img, new_shape=(640, 640), color=(114, 114, 114)):
    shape = img.shape[:2]  # current shape [height, width]
    ratio = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = (int(round(shape[1] * ratio)), int(round(shape[0] * ratio)))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2
    img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)


def preprocess_image(frame, input_size=640):
    img, ratio, dwdh = letterbox(frame, (input_size, input_size))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))  # HWC to CHW
    img = np.expand_dims(img, axis=0)   # Add batch dimension
    return img, ratio, dwdh


def run_inference(session, img):
    ort_inputs = {session.get_inputs()[0].name: img}
    outputs = session.run(None, ort_inputs)
    return outputs[0]


def nms(boxes, scores, iou_threshold=0.5):
    boxes_xyxy = []
    for x, y, w, h in boxes:
        x1 = x - w / 2
        y1 = y - h / 2
        x2 = x + w / 2
        y2 = y + h / 2
        boxes_xyxy.append([x1, y1, x2, y2])
    indices = cv2.dnn.NMSBoxes(
        bboxes=boxes_xyxy,
        scores=scores,
        score_threshold=0.0,
        nms_threshold=iou_threshold
    )
    indices = indices.flatten() if len(indices) > 0 else []
    return indices


def postprocess_detections(output, ratio, dwdh, conf_thresh=0.4, min_box_area=500.0, iou_thresh=0.5):
    if output.shape[1] == 5:
        output = np.transpose(output, (0, 2, 1))
    detections = output[0]

    boxes = []
    scores = []
    for det in detections:
        x, y, w, h, conf = det[:5]
        if conf >= conf_thresh:
            # Undo letterbox and resize to get original image coordinates
            x = (x - dwdh[0]) / ratio
            y = (y - dwdh[1]) / ratio
            w = w / ratio
            h = h / ratio
            area = w * h
            if area >= min_box_area:
                boxes.append([x, y, w, h])
                scores.append(float(conf))

    if boxes:
        indices = nms(boxes, scores, iou_threshold=iou_thresh)
        boxes = [boxes[i] for i in indices]
        scores = [scores[i] for i in indices]
    else:
        boxes = []
        scores = []

    num_detections = len(boxes)
    return num_detections, boxes, scores


def get_detections(frame, session, conf_thresh, min_box_area, input_size):
    img, ratio, dwdh = preprocess_image(frame, input_size)
    output = run_inference(session, img)
    return postprocess_detections(output, ratio, dwdh, conf_thresh, min_box_area)


def send_signal():
    print("Sending signal to robot to pick up the egg.")
    return Constants.stop_after_first_detection


def run_webcam_inference(session, constants):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    print("Webcam started. Press 'q' to quit.")
    stop_inference = False
    while not stop_inference:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break
        num, xywh_list, scores = get_detections(
            frame, session, constants.confidence_threshold, constants.min_box_area, constants.input_size
        )
        print(f"Number of detections: {num}")
        for xywh, score in zip(xywh_list, scores):
            print("xywh:", xywh, "confidence:", score)
            x, y, w, h = map(int, xywh)
            x1, y1 = int(x - w/2), int(y - h/2)
            x2, y2 = int(x + w/2), int(y + h/2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.imshow("Detection", frame)
        if num > 0:
            stop_inference = send_signal()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Exiting.")
            stop_inference = True
    cap.release()
    cv2.destroyAllWindows()


def run_test_img_reference(session, constants, test_dir_path):
    for filename in os.listdir(test_dir_path):
        if filename.endswith('.jpg'):
            frame = cv2.imread(os.path.join(test_dir_path, filename))
            num, xywh_list, scores = get_detections(
                frame, session, constants.confidence_threshold, constants.min_box_area, constants.input_size
            )
            print(f"Image: {filename} | Number of detections: {num}")
            for xywh, score in zip(xywh_list, scores):
                print("xywh:", xywh, "confidence:", score)
                x, y, w, h = map(int, xywh)
                x1, y1 = int(x - w/2), int(y - h/2)
                x2, y2 = int(x + w/2), int(y + h/2)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.imshow("Detection", frame)
            cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_basler_cam(session, constants):
    '''
    this is the main function to run inference on Basler camera.
    It connects to the camera, grabs frames, and runs inference on each frame.
    It will display the detections in a window and can stop after the first detection and returns true if a detection is made.
    If no detections are made, it will continue to grab frames until the user stops it or the camera is disconnected.
    
    this will be used ot tell nvigation to stop on seeing eggs.
    
    i am commenting out the cv2.imshow and cv2.waitKey lines to avoid displaying the camera feed in the terminal.
    you can uncomment them if you want to see the camera feed.
    note enusre camera is connected properly with docker container run the shell script with --basler flag.
    example: ./docker_scripts/arm64.sh --basler
    or
    ./docker_scripts/x86.sh --basler
    or
    python onnx_inference.py --basler
    '''

    try:
        from pypylon import pylon
    except ImportError:
        print("pypylon is not installed. Please install it to use Basler camera input.")
        return

    # Connect to the first available Basler camera
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()

    # Set up the image converter for OpenCV
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    # Start grabbing images continuously
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    

    print("Basler camera started. Press 'Esc' to quit.")

    while camera.IsGrabbing() and not stop_inference:
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            # Convert to OpenCV BGR format
            image = converter.Convert(grabResult)
            img = image.GetArray()

            # Run inference on the current frame
            num, xywh_list, scores = get_detections(
                img, session, constants.confidence_threshold, constants.min_box_area, constants.input_size
            )
            print(f"Number of detections: {num}")
            for xywh, score in zip(xywh_list, scores):
                print("xywh:", xywh, "confidence:", score)
                x, y, w, h = map(int, xywh)
                x1, y1 = int(x - w/2), int(y - h/2)
                x2, y2 = int(x + w/2), int(y + h/2)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
            # cv2.imshow("Basler Camera Detection", img)

            # Stop after first detection if desired
            if num > 0 and constants.stop_after_first_detection:
                print("Sending signal to robot to pick up the egg.")
                return True

            # # Exit on 'Esc' key
            # if cv2.waitKey(1) == 27:
            #     print("Exiting Basler camera inference.")
            #     stop_inference = True

        grabResult.Release()

    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--webcam', action='store_true', help='Run inference on webcam')
    parser.add_argument('--images', action='store_true', help='Run inference on test images')
    parser.add_argument('--basler', action='store_true', help='Run inference on Basler camera')
    args = parser.parse_args()

    constants = Constants()
    session = load_model(constants.model_path)

    if args.webcam:
        run_webcam_inference(session, constants)
    elif args.images:
        run_test_img_reference(session, constants, './egg_dataset/test/images')
    elif args.basler:
        run_basler_cam(session, constants)
    else:
        print("No valid input source selected:")
        print("use --webcam or --images or --basler to specify input.")
