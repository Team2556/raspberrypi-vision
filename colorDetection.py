#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys
import cv2
import numpy as np

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

configFile = "/boot/frc.json"

class CameraConfig: pass

team = 2556
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

COLOR_LOWER = np.array([15, 100, 100])
COLOR_UPPER = np.array([40, 255, 255])

MIN_CONTOUR_AREA = 10000


def processFrame(input_stream, output_stream, ntinst: NetworkTableInstance):
    sink = CameraServer.getVideo(input_stream)
    source = CameraServer.putVideo("Yellow_Highlight", 320, 240)

    nt = ntinst
    
    vision_table = nt.getTable("Vision")
    yellow_detected_entry = vision_table.getBooleanTopic("yellowDetected").publish()
    yellow_detected_entry.set(False)

    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    while True:
        frame_time, img = sink.grabFrame(img)

        if frame_time == 0:
            source.notifyError(sink.getError())
            yellow_detected_entry.set(False)
            continue

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, COLOR_LOWER, COLOR_UPPER)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]

        yellow_detected = len(valid_contours) > 0
        yellow_detected_entry.set(yellow_detected)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        inv_mask = cv2.bitwise_not(mask)
        yellow_part = cv2.bitwise_and(img, img, mask=mask)
        gray_part = cv2.bitwise_and(gray_bgr, gray_bgr, mask=inv_mask)
        result = cv2.add(yellow_part, gray_part)
        
        for contour in valid_contours:
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            cv2.circle(result, (int(x), int(y)), int(radius), (0, 255, 0), 2)

        source.putFrame(result)

def bawVison (input_stream, output_stream, ntinst: NetworkTableInstance):
    sink = CameraServer.getVideo(input_stream)
    source = CameraServer.putVideo("Yellow_Highlight", 320, 240)

    nt = ntinst
    
    vision_table = nt.getTable("Vision")
    yellow_detected_entry = vision_table.getBooleanTopic("yellowDetected").publish()
    yellow_detected_entry.set(False)

    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    while True:
        frame_time, img = sink.grabFrame(img)

        if frame_time == 0:
            source.notifyError(sink.getError())
            yellow_detected_entry.set(False)
            continue

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv, COLOR_LOWER, COLOR_UPPER)



        min_hue = 36/2 # opencv uses 180 degrees instead of 360 so divide by 2
        max_hue = 90/2
        min_sat = 100
        max_sat = 255
        min_val = 140
        max_val = 255



        binary_img = cv2.inRange(hsv, (min_hue, min_sat, min_val), (max_hue, max_sat, max_val))

        kernel = np.ones((10, 10), np.uint8)
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (0,255,0), 3)


        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # valid_contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]

        # yellow_detected = len(valid_contours) > 0
        # yellow_detected_entry.set(yellow_detected)

        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        # inv_mask = cv2.bitwise_not(mask)
        # yellow_part = cv2.bitwise_and(img, img, mask=mask)
        # gray_part = cv2.bitwise_and(gray_bgr, gray_bgr, mask=inv_mask)
        # result = cv2.add(yellow_part, gray_part)
        
        # for contour in valid_contours:
        #     ((x, y), radius) = cv2.minEnclosingCircle(contour)
        #     cv2.circle(result, (int(x), int(y)), int(radius), (0, 255, 0), 2)


        source.putFrame(binary_img)


def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    cam = CameraConfig()
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False
    cam.streamConfig = config.get("stream")
    cam.config = config
    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    cam = CameraConfig()
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False
    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    global team
    global server
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False
    return True

def startCamera(config):
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)
    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)
    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))
    return camera

def startSwitchedCamera(config):
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.addSwitchedCamera(config.name)
    def listener(event):
        data = event.data
        if data is not None:
            value = data.value.value()
            if isinstance(value, int):
                if value >= 0 and value < len(cameras):
                    server.setSource(cameras[value])
            elif isinstance(value, float):
                i = int(value)
                if i >= 0 and i < len(cameras):
                    server.setSource(cameras[i])
            elif isinstance(value, str):
                for i in range(len(cameraConfigs)):
                    if value == cameraConfigs[i].name:
                        server.setSource(cameras[i])
                        break
    NetworkTableInstance.getDefault().addListener(
        NetworkTableInstance.getDefault().getEntry(config.key),
        EventFlags.kImmediate | EventFlags.kValueAll,
        listener)
    return server

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    if not readConfig():
        sys.exit(1)

    ntinst = NetworkTableInstance.getDefault()
    
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("wpilibpi")
        ntinst.setServerTeam(team)
        ntinst.startDSClient()

    for config in cameraConfigs:
        cameras.append(startCamera(config))

    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    if cameras:
        import threading
        vision_thread = threading.Thread(
            target=bawVison,
            args=(cameras[0], None, ntinst),
            daemon=True
        )
        vision_thread.start()
        print("Color detection vision thread started.")
    else:
        print("No cameras found — vision thread not started.", file=sys.stderr)

    while True:
        time.sleep(10)