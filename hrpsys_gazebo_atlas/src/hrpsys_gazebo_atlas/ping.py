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

class RedirectStdStreams(object):
    def __init__(self, stdout=None, stderr=None):
        self._stdout = stdout or sys.stdout
        self._stderr = stderr or sys.stderr
    def __enter__(self):
        self.old_stdout, self.old_stderr = sys.stdout, sys.stderr
        self.old_stdout.flush(); self.old_stderr.flush()
        sys.stdout, sys.stderr = self._stdout, self._stderr
    def __exit__(self, exc_type, exc_value, traceback):
        self._stdout.flush(); self._stderr.flush()
        sys.stdout = self.old_stdout
        sys.stderr = self.old_stderr
                                                                                                

class PingGUI(Plugin):
    def __init__(self, context):
        super(PingGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('PingGUI')
        self.msg = None
        # Create a container widget and give it a layout
        self._container = QWidget()
        self._layout    = QVBoxLayout()
        self._container.setLayout(self._layout)
        self._label = QLabel("xx ms latency")
        p = self._label.palette()
        p.setColor(self._label.backgroundRole(), Qt.red)
        self._label.setPalette(p)
        self._layout.addWidget(self._label)
        self.set_bg_color(100, 100, 100)
        
        rospy.Subscriber("/ping/delay", Float64, self.ping_cb)
        context.add_widget(self._container)
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_gui)
        self._update_plot_timer.start(1)
    def update_gui(self):
        if not self.msg:
            return
        msg = self.msg
        # msec 
        # 100 -> green, 1000 -> red
        # normalize within 1000 ~ 100
        orig_latency = msg.data
        if msg.data > 1000:
            msg.data = 1000
        elif msg.data < 100:
            msg.data = 100
        ratio = (msg.data - 100) / (1000 - 100)
        color_r = ratio * 255.0
        color_g = (1 - ratio) * 255.0
        # devnull = open(os.devnull, "w")
        # with RedirectStdStreams(stdout=devnull, stderr=devnull):
        self.set_bg_color(color_r, color_g, 0)
        self._label.setText("%d ms latency" % (orig_latency))
    def set_bg_color(self, r, g, b):
        self._label.setStyleSheet("QLabel { display: block; background-color: rgba(%d, %d, %d, 255); text-align: center; font-size: 30px;}" % (r, g, b))
    def ping_cb(self, msg):
        self.msg = msg
    def shutdown_plugin(self):
        pass
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass

    

    
