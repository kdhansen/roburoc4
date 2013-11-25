#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
from math import sqrt
from PySide.QtGui import *
from PySide.QtCore import *
from ui_mission_viewer import Ui_MainWindow
import csv
import std_msgs.msg as stdmsgs
import geometry_msgs.msg as geometrymsgs
import roburoc4.msg as r4_msg
import rospy


class MissionModel(QAbstractListModel):
    """Model for holding a mission."""

    waypoints = []
    _current_index = 0
    is_running = False
    last_known_position = geometrymsgs.Point()
    target_threshold = 1.0

    missionStarting = Signal()
    missionStopping = Signal()

    def __init__(self, parent=None):
        super(MissionModel, self).__init__()
        rospy.Subscriber("state_estimates",
                         r4_msg.States,
                         self.set_last_known_position)
        self.control_mode_pub = rospy.Publisher('control_mode', stdmsgs.UInt8)

    # Model-view stuff
    def rowCount(self, parent):
        return len(self.waypoints)

    def data(self, index, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            s = self.waypoints[index.row()]
            return str(s)

    def flags(self, index):
        if index.row() > self._current_index + 1:
            return Qt.ItemIsEnabled
        else:
            return Qt.NoItemFlags

    # RobuROC4 Stuff
    def set_last_known_position(self, pos):
        self.last_known_position.x = pos.x
        self.last_known_position.y = pos.y
        # Check if we hit the target
        if self.is_running:
            wp_x = self.waypoints[self._current_index+1].x
            wp_y = self.waypoints[self._current_index+1].y
            distance_to_target = sqrt((wp_x - pos.x)**2 + (wp_y - pos.y)**2)
            print 'Distance tt: ' + str(distance_to_target)
            if distance_to_target < self.target_threshold:
                self.advance()
                print 'Advance!'

    # Mission stuff
    def appendWaypoint(self, wp):
        new_row_idx = len(self.waypoints)
        self.beginInsertRows(QModelIndex(), new_row_idx, new_row_idx)
        self.waypoints.append(wp)
        self.endInsertRows()

    def openMissionFile(self, mission_file):
        with open(mission_file, 'r') as f:
            self.waypoints = []
            self.reset_mission()
            csvreader = csv.reader(f, delimiter=',')
            for line in csvreader:
                p = geometrymsgs.Point(float(line[0]), float(line[1]), 0.0)
                self.appendWaypoint(p)

    def saveMissionFile(self, mission_file):
        with open(mission_file, 'w') as f:
            csvwriter = csv.writer(f, delimiter=',')
            for wp in self.waypoints:
                csvwriter.writerow([wp.x, wp.y])

    def start_mission(self):
        if self._current_index + 1 <= len(self.waypoints) - 1:
            self.is_running = True
            self.control_mode_pub.publish(1)  # 1 == helmsman control
            self.missionStarting.emit()

    def stop_mission(self):
        self.is_running = False
        self.control_mode_pub.publish(0)  # 0 == control off
        self.missionStopping.emit()

    def reset_mission(self):
        self.stop_mission()
        self.dataChanged.emit(0, self._current_index + 1)
        self._current_index = 0

    def current_segment(self):
        if self._current_index + 1 <= len(self.waypoints) - 1:
            from_wp = self.waypoints[self._current_index]
            to_wp = self.waypoints[self._current_index + 1]
            l = r4_msg.LineSegment(from_wp, to_wp)
            return l
        else:
            return None

    def advance(self):
        self._current_index += 1
        self.dataChanged.emit(self._current_index - 1, self._current_index + 1)
        if self._current_index + 1 > len(self.waypoints) - 1:
            self.stop_mission()


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, model, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.actionAbout.triggered.connect(self.about)
        self.actionOpen.triggered.connect(self.open_file)
        self.actionSave_as.triggered.connect(self.save_as_file)
        self.actionSave.triggered.connect(self.save_file)
        self.startButton.clicked.connect(model.start_mission)
        self.stopButton.clicked.connect(model.stop_mission)

        self.model = model
        self.waypointsListView.setModel(self.model)
        model.missionStarting.connect(self.set_buttons_as_running)
        model.missionStopping.connect(self.set_buttons_as_stopped)
        self.advanceButton.clicked.connect(model.advance)
        self.resetMission.clicked.connect(model.stop_mission)
        self.resetMission.clicked.connect(model.reset_mission)

        # Set a timer to display the position once in a while (1 sec),
        # in order not to bug down the interface with position updates.
        self.position_msg = QLabel()
        self.statusbar.addWidget(self.position_msg)
        self.position_update_timer = QTimer(self)
        self.position_update_timer.timeout.connect(self.update_position)
        self.position_update_timer.start(1000)

        self.targetDial.valueChanged.connect(self.set_target_threshold)

        self.__last_file_opened = None

    def about(self):
        '''Popup a box with about message.'''
        QMessageBox.about(
            self,
            "About the RobuROC4 Mission Viewer",
            """
            Copyright (C) 2013 Karl D. Hansen

            This program is free software: you can redistribute it and/or modify
            it under the terms of the GNU General Public License as published by
            the Free Software Foundation, either version 3 of the License, or
            (at your option) any later version.

            This program is distributed in the hope that it will be useful,
            but WITHOUT ANY WARRANTY; without even the implied warranty of
            MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
            GNU General Public License for more details.

            You should have received a copy of the GNU General Public License
            along with this program.  If not, see <http://www.gnu.org/licenses/>.
            """)

    def open_file(self):
        mission_file = QFileDialog.getOpenFileName(self)
        if mission_file[0]:
            self.model.openMissionFile(mission_file[0])
            self.__last_file_opened = mission_file[0]

    def save_as_file(self):
        mission_file = QFileDialog.getSaveFileName(self)
        if mission_file[0]:
            self.model.saveMissionFile(mission_file[0])
            self.__last_file_opened = mission_file[0]

    def save_file(self):
        if self.__last_file_opened:
            self.model.saveMissionFile(self.__last_file_opened)

    def set_buttons_as_running(self):
        self.startButton.setDisabled(True)
        self.stopButton.setEnabled(True)
        self.advanceButton.setEnabled(True)

    def set_buttons_as_stopped(self):
        self.stopButton.setDisabled(True)
        self.startButton.setEnabled(True)
        self.advanceButton.setDisabled(True)

    def update_position(self):
        self.position_msg.setText(str(self.model.last_known_position))

    def set_target_threshold(self, val):
        self.model.target_threshold = val


class MissionViewer:
    """Top-level class for the Mission Viewer"""

    def __init__(self, arg):
        rospy.init_node('mission_viewer')
        self.segment_pub = rospy.Publisher('cmd_segment', r4_msg.LineSegment)
        self.segment_timer = rospy.Timer(rospy.Duration(1),
                                         self.publish_segment)

        self.my_mission = MissionModel()

        self.app = QApplication(arg)
        self.frame = MainWindow(self.my_mission)
        self.app.lastWindowClosed.connect(self.shutdown)
        self.frame.show()
        self.app.exec_()

    def publish_segment(self, event):
        seg = self.my_mission.current_segment()
        if self.my_mission.is_running and seg:
            self.segment_pub.publish(seg)

    def run(self):
        self.segment_timer.run()
        rospy.spin()

    def shutdown(self):
        rospy.signal_shutdown('Last window closed.')
