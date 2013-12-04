#!/usr/bin/env python

import os

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



class PingGUI(Plugin):
    def __init__(self, context):
        super(PingGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('PingGUI')
        
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        
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
        
        # self.label = QLabel()
        # myPixmap = QPixmap('image.jpg')
        # myScaledPixmap = myPixmap.scaled(self.label.size(), Qt.KeepAspectRatio)
        # self.label.setPixmap(myScaledPixmap)
        # self._layout.addWidget(self.label)

        
        
        # # Add a button for killing nodes
        # self._go_button = QPushButton('Go')
        # self._go_button.clicked.connect(self._go)
        # self._layout.addWidget(self._go_button)
        
        # self._clear_button = QPushButton('Clear')
        # self._clear_button.clicked.connect(self._clear)
        # self._layout.addWidget(self._clear_button)
        # self._step_run_button.setStyleSheet('QPushButton {color: black}')
        rospy.Subscriber("/ping/delay", Float64, self.ping_cb)
        context.add_widget(self._container)
    def set_bg_color(self, r, g, b):
        self._label.setStyleSheet("QLabel { display: block; background-color: rgba(%d, %d, %d, 255); text-align: center; font-size: 30px;}" % (r, g, b))
    def ping_cb(self, msg):
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
        self.set_bg_color(color_r, color_g, 0)
        self._label.setText("%d ms latency" % (orig_latency))
    def shutdown_plugin(self):
        pass
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass

    

    
