#!/usr/bin/env python

import os

import rospy
import roslib

roslib.load_manifest("hrpsys_gazebo_atlas")

from std_srvs.srv import Empty

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from python_qt_binding.QtGui import QLabel, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QCheckBox, QWidget, QToolBar, QLineEdit, QPushButton
from python_qt_binding.QtCore import Qt, QTimer


class TriangleGUI(Plugin):
    def __init__(self, context):
        super(TriangleGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('TriangleGUI')
        
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        self._toolbar = QToolBar()
        self._toolbar.addWidget(QLabel('Triangle'))
        
                                                                
        # Create a container widget and give it a layout
        self._container = QWidget()
        self._layout    = QVBoxLayout()
        self._container.setLayout(self._layout)
        
        self._layout.addWidget(self._toolbar)

        # Add a button for killing nodes
        self._go_button = QPushButton('Go')
        self._go_button.clicked.connect(self._go)
        self._layout.addWidget(self._go_button)
        
        self._clear_button = QPushButton('Clear')
        self._clear_button.clicked.connect(self._clear)
        self._layout.addWidget(self._clear_button)
        # self._step_run_button.setStyleSheet('QPushButton {color: black}')
        
        context.add_widget(self._container)
    def _go(self):
        go = rospy.ServiceProxy('/triangle_screenpoint/go', Empty)
        go()
    def _clear(self):
        clear = rospy.ServiceProxy('/triangle_screenpoint/cancel', Empty)
        clear()
    def shutdown_plugin(self):
        pass
    def save_settings(self, plugin_settings, instance_settings):
        pass
    def restore_settings(self, plugin_settings, instance_settings):
        pass

    

    
