physical_tracking = False
takeoff = False

takeoff_altitude = 10
DEBUG = False

import time
from math import radians

import RPi.GPIO as io
import cv2
from dronekit import connect, VehicleMode
from picamera2 import Picamera2
from pymavlink import mavutil

io.setmode(io.BCM)
jammer_relay_pin = 4
jammer_enabled = "Disabled"
io.setup(jammer_relay_pin, io.OUT)
io.output(jammer_relay_pin, False)


def init():
    global tracking_speed, tracking_sensivity, picam2, p1, p2, state, tracker, connection_to_drone, roi, iha, physical_tracking
    tracking_speed = 4.0
    tracking_sensivity = 5 / 100

    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (1280, 720)
    picam2.preview_configuration.main.format = 'RGB888'
    picam2.configure("preview")
    picam2.start()
    p1, p2 = None, None
    state = 0
    tracker = cv2.TrackerCSRT_create()
    connection_string = "/dev/ttyACM0"

    try:
        global iha
        iha = connect(connection_string, wait_ready=True)
        connection_to_drone = True
        print("Connection to drone is successful")
    except:
        connection_to_drone = False
        print("Connection to drone failed")


def velocity(velocity_x, velocity_y, yaw_rate, velocity_z, iha):
    """
    Give velocity command to vehicle
    """
    msg = iha.message_factory.set_position_target_local_ned_encode(0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                   0b0000011111000111, 0, 0, 0, velocity_x, velocity_y,
                                                                   velocity_z, 0, 0, 0, 0, radians(yaw_rate))
    iha.send_mavlink(msg)
    time.sleep(0.1)


def descend(target_alt):
    """
    Basic descend to target altitude
    """
    while iha.location.global_relative_frame.alt >= target_alt:
        velocity(0, 0, 0, 1, iha)
        time.sleep(0.3)
        print("Current altitude: {}".format(iha.location.global_relative_frame.alt))
    velocity(0, 0, 0, 0, iha)


def ascend(target_alt):
    """
    Basic ascend to target altitude
    """
    while iha.location.global_relative_frame.alt <= target_alt * 0.94:
        velocity(0, 0, 0, -1, iha)
        print("Current altitude: {}".format(iha.location.global_relative_frame.alt))
        time.sleep(0.5)
    velocity(0, 0, 0, 0, iha)


def arm_and_takeoff(altitude):
    global iha
    """
    Arm the vehicle and takeoff
    """
    while not iha.is_armable:
        print("waiting to be armable")
        time.sleep(1)
    iha_mode = iha.mode.name
    print("Arming motors")
    iha.mode = VehicleMode("GUIDED")
    iha.armed = True

    while not iha.armed:
        time.sleep(1)

    print("Taking Off")
    iha.simple_takeoff(altitude)

    while True:
        v_alt = iha.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(0.5)
    velocity(0, 0, 0, 0, iha)
    iha.mode = VehicleMode(iha_mode)
    while not iha.mode.name == iha_mode:  # Wait until mode has changed
        print(" Waiting for mode change ...")
        time.sleep(1)


def on_mouse(event, x, y, flags, userdata):
    global state, p1, p2, jammer_enabled
    if event == cv2.EVENT_LBUTTONDOWN:
        if state == 0:
            p1 = (x, y)
            p2 = (x, y)
            state = 1
        else:
            cv2.destroyWindow('roi')
            p1, p2 = None, None
            state = 0

    elif event == cv2.EVENT_MOUSEMOVE:
        if state == 1:
            p2 = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        # Select first point
        if state == 1:
            p2 = (x, y)
            state = 2

    # Right click (erase current ROI)


def telemetry_data(img, info_package):
    for i, key in enumerate(info_package):
        cv2.putText(img, f"{key}: {info_package[key]}", (img.shape[1] // 15, img.shape[1] // 15 + 30 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)


def scaleit(image, scale_percent):
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)
    image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
    return image


def video():
    info_package = {}
    global state, p1, p2, roi, picam2, tracking_sensivity, tracking_speed, iha, tracker, connection_to_drone, jammer_enabled, physical_tracking
    while True:
        frame = picam2.capture_array()
#         img = cv2.flip(frame, 1)
        img = frame.copy()
        output = img.copy()
        # img = scaleit(img, 30)
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', on_mouse)
        timer = cv2.getTickCount()

        if state == 1:
            cv2.rectangle(output, p1, p2, (255, 0, 0), 10)
        elif state == 2:
            cv2.rectangle(output, p1, p2, (255, 0, 0), 10)
            x1 = min([p1[0], p2[0]])
            x2 = max([p1[0], p2[0]])
            y1 = min([p1[1], p2[1]])
            y2 = max([p1[1], p2[1]])
            w = x2 - x1
            h = y2 - y1

            bbox = [x1, y1, w, h]
            tracker.init(img, bbox)
            roi = img[y1:y2, x1:x2]
            if DEBUG:
                cv2.imshow('roi', roi)
            if physical_tracking and connection_to_drone:
                try:
                    print("\nSet Vehicle.mode = GUIDED (currently: %s)" % iha.mode.name)
                    iha.mode = VehicleMode("GUIDED")
                    timeout = 0
                    while not iha.mode.name == 'GUIDED':  # Wait until mode has changed
                        print(" Waiting for mode change ...")
                        time.sleep(1)
                        timeout += 1
                        if timeout > 8:
                            raise Exception("Timeout")
                    print("Drone Tracking Activated")
                except:
                    physical_tracking = False
                    print("Drone Tracking activation failed!")

            state = 3
            # görüntü işleme ile p1 ve p2 koordinatları arasında seçilen görüntü 'roi' takip edilecek
        if state == 3:

            success, bbox = tracker.update(img)

            if success:

                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(output, p1, p2, (255, 0, 0), 2, 1)

                target_x, target_y = int(bbox[0] + bbox[2] / 2), int(bbox[1] + bbox[3] / 2)
                width, height = img.shape[1], img.shape[0]

                crosshair_center = (width // 2, height // 2)
                x_diff = width * tracking_sensivity
                y_diff = height * tracking_sensivity

                crosshair_right = int(crosshair_center[0] + x_diff)
                crosshair_left = int(crosshair_center[0] - x_diff)
                crosshair_top = int(crosshair_center[1] + y_diff)
                crosshair_bottom = int(crosshair_center[1] - y_diff)
                """
                Draws a rectangle crosshair
                """
                cv2.rectangle(output, (crosshair_left, crosshair_top), (crosshair_right, crosshair_bottom), (0, 0, 255),
                              3,
                              1)
                """
                Checks for if the target is centered
                """
                if crosshair_left <= target_x <= crosshair_right and crosshair_bottom <= target_y <= crosshair_top:
                    print("ortaladık yeeey")
                    if connection_to_drone and physical_tracking:
                        velocity(0, 0, 0, 0, iha)

                else:
                    """
                    Calculates the x and y axis vectors to follow the target at desired speed(velocity = 0.5)
                    """
                    cv2.arrowedLine(output, (crosshair_center[0], crosshair_center[1]), (target_x, target_y),
                                    (0, 0, 255), 2, tipLength=0.5)  # merkezden hedefe ok çizer
                    diff_x = target_x - width / 2
                    diff_y = -(target_y - height / 2)
                    image_diagonal = (width ** 2 + height ** 2) ** (1 / 2)
                    velocity_per_pixel = tracking_speed / image_diagonal
                    velocity_x = diff_x * velocity_per_pixel
                    velocity_y = diff_y * velocity_per_pixel
                    print(f"Velo x: {velocity_x:.2f}", f"Velo y: {velocity_y:.2f}")

                    if connection_to_drone and physical_tracking:
                        velocity(velocity_y, velocity_x, 0, 0, iha)

            else:
                cv2.putText(output, "Tracking failure detected", (100, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                state = 4
            cv2.putText(output, "CRST " + " Tracker", (100, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
            if state == 4:
                """
                tries to detect tracked object to init the tracker again
                """
                if DEBUG:
                    print("Template matching started")
                    imageGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    templateGray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    result = cv2.matchTemplate(imageGray, templateGray,
                                               cv2.TM_CCOEFF_NORMED)
                    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
                    (startX, startY) = maxLoc
                    bbox = [startX, startY, roi.shape[1], roi.shape[0]]
                    trackerresult = img[startY:startY + roi.shape[0], startX:startX + roi.shape[1]]
                    cv2.imshow('trackerresult', trackerresult)
                    tracker.init(img, bbox)
                    state = 3
                else:
                    print("Hedef kayboldu")

        fps = int(cv2.getTickFrequency() / (cv2.getTickCount() - timer))

        if connection_to_drone:
            info_package.update({"Mode": iha.mode.name})
            info_package.update({"Altitude": iha.location.global_relative_frame})
            info_package.update({"Ground Speed": iha.groundspeed})
            info_package.update({"GPS": iha.gps_0})
            info_package.update({"Battery": iha.battery})

        info_package.update({"FPS": fps})
        info_package.update({"Jammer": jammer_enabled})
        telemetry_data(output, info_package)
        cv2.imshow('image', output)

        key = cv2.waitKey(5)
        if key == ord('q'):
            io.output(jammer_relay_pin, False)
            break
        elif key == ord('j'):
            if jammer_enabled == "Disabled":
                io.output(jammer_relay_pin, True)
                jammer_enabled = "Enabled!"
            elif jammer_enabled == "Enabled!":
                io.output(jammer_relay_pin, False)
                jammer_enabled = "Disabled"
        elif key == ord('l'):
            for counter in range(3, 0, -1):
                print(f"Setting LAND mode in {counter}...")
                time.sleep(1)
            iha.mode = VehicleMode("LAND")


init()
if takeoff:
    arm_and_takeoff(takeoff_altitude)
video()
cv2.destroyAllWindows()
