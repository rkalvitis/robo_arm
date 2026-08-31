#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Continuous /joint_states recorder for robot<->mocap registration.

Run on the robot PC in its own terminal for the whole capture
session, alongside the mocap recording (optitrack repo,
`pixi run record-poses`) and the pose replay:

    rosrun panda_moveit_ctrl record_joint_states.py

Writes joint_states_<timestamp>.db next to this script (override
with _output_file:=path): an sqlite database with one table,

    joint_states(stamp, joint1, ..., joint7)
    events(stamp, event, pose)

where stamp is the message header stamp in Unix epoch seconds
(UTC+0 by definition - epoch time is timezone-free, so the robot
PC's timezone setting cannot shift it).

The events table flags the photo moments: panda_manual_poses.py
publishes markers on the /pose_events topic while it runs, and
this recorder timestamps them into the SAME db -
'pose_reached,k' / 'pose_left,k' rows bracket the interval during
which the photo at pose k was taken, plus init_reached /
init_confirmed / return_start / run_end markers. One recorder
kept running across MANY pose-script runs collects everything in
one file (each run starts with a fresh init_reached).
This recorder runs on the robot PC while the mocap bag records on
the Mac - the two files are matched purely by timestamps.
The franka driver stamps at the 1 kHz state capture, so like the
mocap capture-time stamps there is no network-lag bias; the
constant robot-PC<->Mac clock offset is solved by the registration
itself (link_robot_mocap.py --joints, via motion cross-
correlation), so the two machines do NOT need synchronized clocks.

Stop with Ctrl-C; rows are committed once a second, so a crash
keeps everything up to the failure. Copy the .db into the session
folder <dataset>/phone_poses_robot/ together with the
manual_poses_*.csv logs.
"""

from __future__ import print_function

import datetime
import os
import sqlite3
import sys

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

PANDA_JOINTS = ["panda_joint%d" % k for k in range(1, 8)]


class Recorder(object):

    def __init__(self, path):
        self.con = sqlite3.connect(path, check_same_thread=False)
        self.con.execute(
            "CREATE TABLE IF NOT EXISTS joint_states ("
            "stamp REAL, joint1 REAL, joint2 REAL, joint3 REAL, "
            "joint4 REAL, joint5 REAL, joint6 REAL, joint7 REAL)")
        self.con.execute(
            "CREATE INDEX IF NOT EXISTS idx_stamp "
            "ON joint_states(stamp)")
        self.con.execute(
            "CREATE TABLE IF NOT EXISTS events ("
            "stamp REAL, event TEXT, pose INTEGER)")
        self.con.execute(
            "CREATE TABLE IF NOT EXISTS meta (key TEXT, value TEXT)")
        started_utc = datetime.datetime.utcnow().strftime(
            "%Y-%m-%dT%H:%M:%SZ")
        self.con.execute(
            "INSERT INTO meta VALUES ('topic', '/joint_states'), "
            "('joints', '%s'), "
            "('stamp_format', 'unix epoch seconds, UTC+0'), "
            "('clock', 'robot PC, header.stamp'), "
            "('started_utc', '%s')"
            % (",".join(PANDA_JOINTS), started_utc))
        self.con.commit()
        self.pending = []
        self.pending_events = []
        self.count = 0
        self.last_flush = 0.0
        self.warned = False
        self.closed = False

    def mark(self, event, pose=None):
        """Timestamped event marker (same UTC epoch clock as the joint
        stamps). Appended only - written out by the next flush."""
        self.pending_events.append(
            (rospy.get_rostime().to_sec(), event, pose))

    def flush(self):
        if self.closed:
            return
        if self.pending:
            self.con.executemany(
                "INSERT INTO joint_states VALUES (?,?,?,?,?,?,?,?)",
                self.pending)
            self.pending = []
        if self.pending_events:
            self.con.executemany(
                "INSERT INTO events VALUES (?,?,?)",
                self.pending_events)
            self.pending_events = []
        self.con.commit()

    def close_final(self):
        """Idempotent final flush + close (safe to register via atexit)."""
        if self.closed:
            return
        try:
            self.flush()
        finally:
            self.closed = True
            self.con.close()

    def callback(self, msg):
        try:
            index = [msg.name.index(j) for j in PANDA_JOINTS]
        except ValueError:
            # Not an arm message (e.g. gripper-only publisher).
            if not self.warned:
                rospy.logwarn(
                    "Ignoring /joint_states without the 7 panda "
                    "joints (names: %s)", ", ".join(msg.name))
                self.warned = True
            return

        stamp = msg.header.stamp.to_sec()

        self.pending.append(
            (stamp,) + tuple(msg.position[i] for i in index))
        self.count += 1

        if stamp - self.last_flush > 1.0:
            self.flush()
            self.last_flush = stamp


def main():
    rospy.init_node("record_joint_states", anonymous=False)

    default_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "joint_states_%s.db"
        % datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S"))

    path = rospy.get_param("~output_file", "") or default_path
    path = os.path.abspath(os.path.expanduser(path))

    recorder = Recorder(path)

    sub = rospy.Subscriber(
        "/joint_states", JointState, recorder.callback,
        queue_size=1000)

    def on_event(msg):
        parts = [p.strip() for p in msg.data.split(",")]
        pose = None
        if len(parts) > 1 and parts[1].lstrip("-").isdigit():
            pose = int(parts[1])
        recorder.mark(parts[0], pose)
        rospy.loginfo("event: %s", msg.data)

    ev_sub = rospy.Subscriber(
        "/pose_events", String, on_event, queue_size=100)

    rospy.loginfo("Recording /joint_states to %s - Ctrl-C to stop.",
                  path)

    rospy.spin()

    sub.unregister()
    ev_sub.unregister()
    recorder.close_final()

    print("\n%d samples written to %s" % (recorder.count, path))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
