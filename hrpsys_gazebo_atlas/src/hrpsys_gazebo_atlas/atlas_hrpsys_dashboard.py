try:
    from hrpsys_ros_bridge.hrpsys_dashboard import HrpsysDashboard
except:
    import roslib; roslib.load_manifest("hrpsys_gazebo_atlas")
    from hrpsys_ros_bridge.hrpsys_dashboard import HrpsysDashboard

from rqt_robot_dashboard.widgets import MenuDashWidget
from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QMessageBox
import random
import rospy

from atlas_msgs.msg import AtlasSimInterfaceCommand
from std_msgs.msg import String
from sandia_hand_msgs.msg import SimpleGrasp

class SimCommandMenu(MenuDashWidget):
    def __init__(self):
        icons = [['bg-green.svg', 'ic-runstop-off.svg'],]
        super(SimCommandMenu, self).__init__('SimCommand', icons)
        self.update_state(0)
        self.add_action('UserMode', self.send_user_command)
        self.add_action('StandMode', self.send_stand_command)
        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))
        self.sim_command_pub = rospy.Publisher('/atlas/atlas_sim_interface_command', AtlasSimInterfaceCommand, queue_size=10)
    def send_user_command(self):
        rospy.loginfo("set user mode")
        _sim_msg = AtlasSimInterfaceCommand(behavior = 1, k_effort = [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255])
        _sim_msg.header.stamp = rospy.Time.now()
        self.sim_command_pub.publish(_sim_msg)
    def send_stand_command(self):
        rospy.loginfo("set stand mode")
        _sim_msg = AtlasSimInterfaceCommand(behavior = 0, k_effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        _sim_msg.header.stamp = rospy.Time.now()
        self.sim_command_pub.publish(_sim_msg)

class SimModeMenu(MenuDashWidget):
    def __init__(self):
        icons = [['bg-green.svg', 'ic-steering-wheel.svg'],]
        super(SimModeMenu, self).__init__('SimMode', icons)
        self.update_state(0)
        self.add_action('Pinned', self.set_pinned_mode)
        self.add_action('Unpinned', self.set_unpinned_mode)
        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))
        self.sim_command_pub = rospy.Publisher('/atlas/mode', String, queue_size=10)
    def set_pinned_mode(self):
        rospy.loginfo("set pinned mode")
        self.sim_command_pub.publish("pinned")
    def set_unpinned_mode(self):
        rospy.loginfo("set unpinned mode")
        self.sim_command_pub.publish("nominal")

class HandCommandMenu(MenuDashWidget):
    def __init__(self):
        icons = [['bg-green.svg', 'ic-wrench.svg'],
                 ['bg-red.svg', 'ic-wrench.svg'],]
        super(HandCommandMenu, self).__init__('HandCommand', icons)
        self.update_state(0)
        self.add_action('GraspHand', self.grasp_hand)
        self.add_action('OpenHand', self.open_hand)
        self.setFixedSize(self._icons[0].actualSize(QSize(50, 30)))
        self.left_hand_command_pub = rospy.Publisher('/sandia_hands/l_hand/simple_grasp', SimpleGrasp, queue_size=10)
        self.right_hand_command_pub = rospy.Publisher('/sandia_hands/r_hand/simple_grasp', SimpleGrasp, queue_size=10)
    def grasp_hand(self):
        rospy.loginfo("grasp_hand")
        _hand_msg = SimpleGrasp(name="cylindrical", closed_amount=1)
        self.left_hand_command_pub.publish(_hand_msg)
        self.right_hand_command_pub.publish(_hand_msg)
        self.update_state(1)
    def open_hand(self):
        rospy.loginfo("open hand")
        _hand_msg = SimpleGrasp(name="cylindrical", closed_amount=0)
        self.left_hand_command_pub.publish(_hand_msg)
        self.right_hand_command_pub.publish(_hand_msg)
        self.update_state(0)


class AtlasHrpsysDashboard(HrpsysDashboard):
    def setup(self, context):
        super(AtlasHrpsysDashboard, self).setup(context)
        self.name = "atlas_hrpsys dashboard"
        self._sim_command_button = SimCommandMenu()
        self._sim_mode_button = SimModeMenu()
        self._hand_command_button = HandCommandMenu()
    def get_widgets(self):
        widgets = []
        atlas_widgets = []
        if self._rtcd:
            widgets.append(self._rtcd)
        if self._ros_bridge:
            widgets.append(self._ros_bridge)
        if self._power:
            widgets.append(self._power)
        if self._servo:
            widgets.append(self._servo)
        if self._sim_command_button:
            atlas_widgets.append(self._sim_command_button)
        if self._sim_mode_button:
            atlas_widgets.append(self._sim_mode_button)
        if self._hand_command_button:
            atlas_widgets.append(self._hand_command_button)
        return [[self._monitor, self._console, self._log], widgets, atlas_widgets]
    def dashboard_callback(self, msg):
        super(AtlasHrpsysDashboard, self).dashboard_callback(msg)

