import argparse
import cv2
import os
import time
import random
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference


def map_range(value, in_min, in_max, out_min, out_max):
    return out_min + (float(value - in_min) / (in_max - in_min)) * (out_max - out_min)


def main():
    default_model_dir = "/home/pi/tflite_models"
    default_model = "ssd_mobilenet_v2_face_quant_postprocess_edgetpu.tflite"
    default_labels = "coco_labels.txt"
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        help=".tflite model path",
        default=os.path.join(default_model_dir, default_model),
    )
    parser.add_argument(
        "--labels",
        help="label file path",
        default=os.path.join(default_model_dir, default_labels),
    )
    parser.add_argument(
        "--top_k",
        type=int,
        default=3,
        help="number of categories with highest score to display",
    )
    parser.add_argument(
        "--camera_idx", type=int, help="Index of which video source to use.", default=0
    )
    parser.add_argument(
        "--threshold", type=float, default=0.1, help="classifier score threshold"
    )
    args = parser.parse_args()

    print(f"Loading {args.model} with {args.labels}.")
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)
    inference_size = input_size(interpreter)

    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Servo channel assignments
    blink_channels = [0, 1]  # Blink servo channels
    horizontal_channels = [2, 3]  # Horizontal movement servo channels
    vertical_channels = [4, 5]  # Vertical movement servo channels

    # Servo pulse range limits (in microseconds)
    blink_pulse_ranges = [(600, 2400), (700, 2300)]
    vertical_pulse_ranges = [(732, 1122), (800, 1100)]
    horizontal_pulse_ranges = [(1464, 2100), (1500, 2050)]

    # Create servo objects with custom pulse widths
    blink_servos = [
        servo.Servo(pca.channels[ch], min_pulse=pr[0], max_pulse=pr[1])
        for ch, pr in zip(blink_channels, blink_pulse_ranges)
    ]
    vertical_servos = [
        servo.Servo(pca.channels[ch], min_pulse=pr[0], max_pulse=pr[1])
        for ch, pr in zip(vertical_channels, vertical_pulse_ranges)
    ]
    horizontal_servos = [
        servo.Servo(pca.channels[ch], min_pulse=pr[0], max_pulse=pr[1])
        for ch, pr in zip(horizontal_channels, horizontal_pulse_ranges)
    ]

    # Initialize servo positions
    for servo in blink_servos:
        servo.angle = 0  # Eye open
    for servo in vertical_servos:
        servo.angle = 90  # Center position
    for servo in horizontal_servos:
        servo.angle = 90  # Center position

    # Variables to hold current servo angles
    blink_current_angle = 0
    vertical_current_angle = 90
    horizontal_current_angle = 90

    # Variables for blinking behavior
    last_blink_time = time.time()
    blink_interval = random.uniform(10, 20)
    blinking = False
    blink_stage = 0

    # Variables for tracking target presence
    last_target_time = time.time()
    target_timeout = 2.0

    # Variables for eye movement when no target
    moving_right = True
    last_move_time = time.time()
    move_interval = 3.0

    # Variables for smooth servo movements
    horizontal_target_angle = horizontal_current_angle
    vertical_target_angle = vertical_current_angle
    servo_move_speed = 200

    display_available = os.environ.get("DISPLAY") is not None

    cap = cv2.VideoCapture(args.camera_idx)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        exit()

    screen_height, screen_width, _ = frame.shape

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        cv2_im = frame

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        cv2_im_rgb = cv2.resize(cv2_im_rgb, inference_size)
        run_inference(interpreter, cv2_im_rgb.tobytes())
        objs = get_objects(interpreter, args.threshold)[: args.top_k]
        cv2_im = append_objs_to_img(cv2_im, inference_size, objs, labels)

        current_time = time.time()
        dt = current_time - last_move_time

        if objs:
            face = objs[0]
            x0, y0 = int(face.bbox.xmin), int(face.bbox.ymin)
            x1, y1 = int(face.bbox.xmax), int(face.bbox.ymax)
            center_x = (x0 + x1) * 2
            center_y = (y0 + y1) * 2

            horizontal_target_angle = int(
                map_range(center_x, 0, screen_width * 2, 0, 180)
            )
            vertical_target_angle = int(
                map_range(center_y, 0, screen_height * 2, 0, 180)
            )

            last_target_time = current_time
        else:
            if current_time - last_target_time > target_timeout:
                if current_time - last_move_time > move_interval:
                    horizontal_target_angle = 180 if moving_right else 0
                    moving_right = not moving_right
                    last_move_time = current_time
                vertical_target_angle = 90

        horizontal_current_angle = update_servo_angle(
            horizontal_servos,
            horizontal_current_angle,
            horizontal_target_angle,
            dt,
            servo_move_speed,
        )
        vertical_current_angle = update_servo_angle(
            vertical_servos,
            vertical_current_angle,
            vertical_target_angle,
            dt,
            servo_move_speed,
        )

        if blinking:
            blink_current_angle, blink_stage = handle_blinking(
                blink_servos, blink_current_angle, blink_stage, dt
            )
            if blink_stage == 0:
                blinking = False
                last_blink_time = current_time
                blink_interval = random.uniform(3, 7)
        elif current_time - last_blink_time > blink_interval:
            blinking = True
            blink_stage = 1
        if display_available:
            cv2.imshow("frame", cv2_im)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        last_move_time = current_time

    cap.release()
    cv2.destroyAllWindows()
    pca.deinit()


def update_servo_angle(servo_objs, current_angle, target_angle, dt, speed):
    max_delta = speed * dt
    delta = target_angle - current_angle
    if abs(delta) > max_delta:
        delta = max_delta if delta > 0 else -max_delta
    current_angle += delta
    current_angle = max(0, min(180, current_angle))
    for servo_obj in servo_objs:
        servo_obj.angle = current_angle
    return current_angle


def handle_blinking(blink_servos, blink_current_angle, blink_stage, dt):
    blink_speed = 360
    max_delta = blink_speed * dt
    if blink_stage == 1:
        delta = max_delta
        blink_current_angle += delta
        if blink_current_angle >= 180:
            blink_current_angle = 180
            blink_stage = 2
    elif blink_stage == 2:
        delta = max_delta
        blink_current_angle -= delta
        if blink_current_angle <= 0:
            blink_current_angle = 0
            blink_stage = 0
    for blink_servo in blink_servos:
        blink_servo.angle = blink_current_angle
    return blink_current_angle, blink_stage


def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        percent = int(100 * obj.score)
        label = "{}% {}".format(percent, labels.get(obj.id, obj.id))

        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(
            cv2_im, label, (x0, y0 + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2
        )
    return cv2_im


if __name__ == "__main__":
    main()
