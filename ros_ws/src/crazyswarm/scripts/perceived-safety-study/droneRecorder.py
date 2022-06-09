#!/usr/bin/env python

import sys
sys.path.append("..")

import rospy
from tf2_msgs.msg import TFMessage
import math
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import csv


class Timestamp:
    def __init__(self, starting_secs):
        self.current_timestamp = 0
        self.timestamp_subtractor = starting_secs

    def update_timestamp(self, new_timestamp_temp):
        new_timestamp = round(new_timestamp_temp -
                              self.timestamp_subtractor, 2)

        is_different = self.current_timestamp != new_timestamp

        self.current_timestamp = new_timestamp

        return new_timestamp, is_different


class Pose:
    def __init__(self, x, y, z, q):
        self.x = x
        self.y = y
        self.z = z
        
        self.roll = round(math.atan2(2.0*(q.y*q.z + q.w*q.x),
                                     q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z) * (180/math.pi), 2)
        self.pitch = round(
            math.asin(-2.0*(q.x*q.z - q.w*q.y)) * (180/math.pi), 2)
        yaw_temp = round(math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w *
                         q.w + q.x*q.x - q.y*q.y - q.z*q.z) * (180/math.pi), 2)

        self.yaw = 6.28319 - yaw_temp if yaw_temp < 0 else yaw_temp


class Recorder:
    def __init__(self):
        self.timeUnit = "seconds"
        self.time_stamps = []
        self.poses = []

    def addEntry(self, new_time_stamp, new_pose):
        self.time_stamps.append(new_time_stamp)
        self.poses.append(new_pose)

    def __str__(self):
        output = ""

        for idx, pose in enumerate(self.poses):
            time_stamp = self.time_stamps[idx]
            x, y, z, angle = pose.x, pose.y, pose.z, pose.angle
            output += f"{time_stamp}s: [x, y, z, angle]=[{x}cm, {y}cm, {z}cm, {angle}°]"

        return output

    def getPositionEntries(self):
        positionEntries = [["time_stamp", "x", "y", "z"]]
        for idx, pose in enumerate(self.poses):
            time_stamp = self.time_stamps[idx]
            x = pose.x
            y = pose.y
            z = pose.z
            positionEntries.append([time_stamp, x, y, z])

        return positionEntries

    def getYawEntries(self):
        yawEntries = [["time_stamp", "yaw"]]
        for idx, pose in enumerate(self.poses):
            time_stamp = self.time_stamps[idx]
            yawEntries.append([float(time_stamp), float(pose.yaw)])

        return yawEntries

    def getLatestEntryAsString(self):
        latest_time_stamp = self.time_stamps[-1]
        latest_pose = self.poses[-1]
        x, y, z, yaw, pitch, roll = latest_pose.x, latest_pose.y, latest_pose.z, latest_pose.yaw, latest_pose.pitch, latest_pose.roll
        return f"[{x}, {y}, {z}], {np.deg2rad(yaw)}"
        # return f"{latest_time_stamp.__str__()}s: [x, y, z, yaw]=[{x}cm, {y}cm, {z}cm, {yaw}°]"


class Listener:
    def __init__(self):
        self.record_data = True
        self.last_timestamp = None
        self.recorder = Recorder()

        rospy.init_node('drone_sub', anonymous=True)

        rospy.Subscriber("tf", TFMessage, self.tf_callback)

        rospy.spin()

    def activate_logging_callback(self, data):
        rospy.loginfo(data.data)

        if (data.data == "Start recording"):
            self.record_data = True
        else:
            self.record_data = False

    def tf_callback(self, data):
        transform = data.transforms[0]
        pose = transform.transform

        drone_z = pose.translation.z  # z-axis == height

        # if drone_z > 5:
        self.record_data = True

        time_stamp = transform.header.stamp
        secs = float(math.floor(float(str(time_stamp))) / 1000000000)

        first_iteration = self.last_timestamp is None

        if first_iteration:
            self.last_timestamp = Timestamp(
                starting_secs=secs)
            new_time_stamp = 0
            timestamp_has_changed = False
        else:
            new_time_stamp, timestamp_has_changed = self.last_timestamp.update_timestamp(
                secs)

        if timestamp_has_changed or first_iteration:
            drone_x = pose.translation.x
            drone_y = pose.translation.y
            
            new_pose = Pose(drone_x, drone_y, drone_z, pose.rotation)
            self.recorder.addEntry(new_time_stamp, new_pose)

            latestEntryAsString = self.recorder.getLatestEntryAsString()
            rospy.loginfo(latestEntryAsString)
            
        # elif self.record_data:
        #     self.record_data = False
        #     # List of [timestamp, x, y, z]
        #     position_entries = np.array(self.recorder.getPositionEntries())

        #     # List of [timestamp, yaw]
        #     yaw_entries = np.array(self.recorder.getYawEntries())

        #     file_id = str(sys.argv[1]) if len(
        #         sys.argv) > 1 else str(datetime.now())

        #     filename1 = "recorded_data/" + file_id + "_position_entries"
        #     filename2 = "recorded_data/" + file_id + "_yaw_entries"

        #     with open(f"{filename1}.csv", 'w', newline='') as file:
        #         mywriter = csv.writer(file, delimiter=',')
        #         mywriter.writerows(position_entries)

        #     with open(f"{filename2}.csv", 'w', newline='') as file:
        #         mywriter = csv.writer(file, delimiter=',')
        #         mywriter.writerows(yaw_entries)


if __name__ == '__main__':
    listener = Listener()
