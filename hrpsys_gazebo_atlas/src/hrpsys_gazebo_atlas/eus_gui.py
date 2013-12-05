#!/usr/bin/env python

import os
import sys

import rospy
import roslib

roslib.load_manifest("hrpsys_gazebo_atlas")

from std_srvs.srv import Empty
from std_msgs.msg import Float64
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from python_qt_binding.QtGui import QLabel, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QCheckBox, QWidget, QToolBar, QLineEdit, QPushButton, QPixmap
from python_qt_binding.QtCore import Qt, QTimer

class EusGUI(Plugin):
    def __init__(self, context):
        super(EusGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('EusGUI')
        self.msg = None
        # Create a container widget and give it a layout
        self._toolbar = QToolBar()
        self._toolbar.addWidget(QLabel('EusGUI'))
        self._container = QWidget()
        self._layout    = QVBoxLayout()
        self._container.setLayout(self._layout)
        
        self._layout.addWidget(self._toolbar)
        
        self._prev_button = QPushButton('PREV')
        self._prev_button.clicked.connect(self._prev_cb)
        self._layout.addWidget(self._prev_button)
        
        self._refresh_button = QPushButton('DO IT AGAIN')
        self._refresh_button.clicked.connect(self._refresh_cb)
        self._layout.addWidget(self._refresh_button)

        self._next_button = QPushButton('NEXT')
        self._next_button.clicked.connect(self._next_cb)
        self._layout.addWidget(self._next_button)
        context.add_widget(self._container)
    def _prev_cb(self):
        func = rospy.ServiceProxy('prev', Empty)
        func()
    def _next_cb(self):
        func = rospy.ServiceProxy('next', Empty)
        func()
    def _refresh_cb(self):
        func = rospy.ServiceProxy('refresh', Empty)
        func()
    def shutdown_plugin(self):
        pass
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass

    

    

