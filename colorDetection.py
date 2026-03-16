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
from ntcore import NetworkTableInstance, EventFlags, IntegerTopic

configFile = "/boot/frc.json"

class CameraConfig: pass

team = 2556
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []



def ballArea (input_stream, output_stream, ntinst: NetworkTableInstance):
    sink = CameraServer.getVideo(input_stream)
    source = CameraServer.putVideo("Video", 320, 240)

    nt = ntinst
    
    vision_table = nt.getTable("RpiVision")
    _min_hue = vision_table.getIntegerTopic("Min Hue").subscribe(0)
    _max_hue = vision_table.getIntegerTopic("Max Hue").subscribe(0)
    _min_sat = vision_table.getIntegerTopic("Min Sat").subscribe(0)
    _max_sat = vision_table.getIntegerTopic("Max Sat").subscribe(0)
    _min_val = vision_table.getIntegerTopic("Min Val").subscribe(0)
    _max_val = vision_table.getIntegerTopic("Max Val").subscribe(0)

    yellow_area_entry = vision_table.getDoubleTopic("YellowAreaInHopper").publish()
    yellow_area_entry.set(0.0)
    yellow_detected_entry = vision_table.getBooleanTopic("YellowDetectedInHopper").publish()
    yellow_detected_entry.set(False)

    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    while True:
        frame_time, img = sink.grabFrame(img)

        if frame_time == 0:
            source.notifyError(sink.getError())
            yellow_detected_entry.set(False)
            continue

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        min_hue = _min_hue.get()
        max_hue = _max_hue.get()
        min_sat = _min_sat.get()
        max_sat = _max_sat.get()
        min_val = _min_val.get()
        max_val = _max_val.get()

        binary_img = cv2.inRange(hsv, (min_hue, min_sat, min_val), (max_hue, max_sat, max_val))

        kernel = np.ones((5, 5), np.uint8)
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area = 0
        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) > 500:
                cv2.drawContours(img, contours, i, (0,255,0), 3)
                area += cv2.contourArea(contours[i])
        yellow_area_entry.set(area)
        source.putFrame(img)
        if area > 500.0:
            yellow_detected_entry.set(True)
        else:
            yellow_detected_entry.set(False)


def ballDetector (input_stream, output_stream, ntinst: NetworkTableInstance):
    sink = CameraServer.getVideo(input_stream)
    source = CameraServer.putVideo("Video", 320, 240)

    nt = ntinst
    
    vision_table = nt.getTable("RpiVision")

    _min_hue = vision_table.getIntegerTopic("Min Hue").subscribe(0)
    _max_hue = vision_table.getIntegerTopic("Max Hue").subscribe(0)
    _min_sat = vision_table.getIntegerTopic("Min Sat").subscribe(0)
    _max_sat = vision_table.getIntegerTopic("Max Sat").subscribe(0)
    _min_val = vision_table.getIntegerTopic("Min Val").subscribe(0)
    _max_val = vision_table.getIntegerTopic("Max Val").subscribe(0)

    x_entry = vision_table.getDoubleTopic("x").publish()
    x_entry.set(0.0)
    y_entry = vision_table.getDoubleTopic("y").publish()
    y_entry.set(0.0)

    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    r = 162
    g = 222
    b = 168
    while True:
        frame_time, img = sink.grabFrame(img)

        if frame_time == 0:
            source.notifyError(sink.getError())
            continue

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        min_hue = _min_hue.get()
        max_hue = _max_hue.get()
        min_sat = _min_sat.get()
        max_sat = _max_sat.get()
        min_val = _min_val.get()
        max_val = _max_val.get()

        temp_x = 0
        temp_y = 0

        binary_img = cv2.inRange(hsv, (min_hue, min_sat, min_val), (max_hue, max_sat, max_val))

        kernel = np.ones((15, 15), np.uint8)
        binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(img, contours, -1, (0,255,0), 3)
        biggest = 0
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            M = cv2.moments(contours[i])
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if area > 100:
                    if cv2.contourArea(contours[biggest]) < area: # ts was moved it might break
                        cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)
                        cv2.putText(img, "BALL HERE", (cx - 20, cy - 20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (r, g, b), 2)
                        biggest = i
                        temp_x = cx / 2
                        temp_y = cy / 2
        # the code highkey crashes when the camera moves so this ignores it
        try:
            cv2.drawContours(img, contours[biggest], -1, (255, 150, 0), 15) 
        except:
            print("ahhhh I can't see when you move too fast I'm gonna crash")

        x_entry.set((temp_x-160)/160)
        y_entry.set((temp_y-120)/120)

        source.putFrame(img)

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
            server = False # supposed to be true 
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

'''def startSwitchedCamera(config):
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
    return server '''

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
        # ntinst.setServerTeam(team) 
        ntinst.setServer("192.168.42.213") # use team or ip, not both
        ntinst.startDSClient()

    for config in cameraConfigs:
        cameras.append(startCamera(config))

    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    if cameras:
        import threading
        hopper_thread = threading.Thread(
            target=ballArea,
            args=(cameras[1], None, ntinst),
            daemon=True
        )
        pathing_thread = threading.Thread(
            target=ballDetector,
            args=(cameras[0], None, ntinst),
            daemon=True
        )
        hopper_thread.start()
        pathing_thread.start()
        print("Color detection vision thread started.")
    else:
        print("No cameras found — vision thread not started.", file=sys.stderr)

    while True:
        time.sleep(10)