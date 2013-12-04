#!/usr/bin/env python
import roslib; roslib.load_manifest('hrpsys')
import OpenRTM_aist.RTM_IDL # for catkin
import sys

import hrpsys
from hrpsys.hrpsys_config import *
import OpenHRP

class ATLASHrpsysConfigurator(HrpsysConfigurator):
    Groups = [['torso', ['back_bkz', 'back_bky', 'back_bkx']],
              ['head', ['neck_ry']],
              ['rarm', ['r_arm_shy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx']],
              ['larm', ['l_arm_shy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx']],
              ['lleg', ['l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx']],
              ['rleg', ['r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx']]]

    def connectComps(self):
        HrpsysConfigurator.connectComps(self)
        # connect for kf rtc dummy rpy
        s_acc=filter(lambda s : s.type == 'Acceleration', self.sensors)
        if (len(s_acc)>0) and self.rh.port(s_acc[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
            disconnectPorts(self.rh.port(s_acc[0].name), self.kf.port('acc'))
        s_rate=filter(lambda s : s.type == 'RateGyro', self.sensors)
        if (len(s_rate)>0) and self.rh.port(s_rate[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
            disconnectPorts(self.rh.port(s_rate[0].name), self.kf.port("rate"))
            connectPorts(self.rh.port(s_rate[0].name), self.kf.port("rpyIn"))

        # delete co, when use collisoin detector is not used
#        disconnectPorts(self.rh.port("q"), self.co.port("qCurrent"))
#        disconnectPorts(self.st.port("q"), self.co.port("qRef"))
#        disconnectPorts(self.co.port("q"), self.el.port("qRef"))

        # delete co, and input current angle, to use collisoin detector just for checking
        #disconnectPorts(self.rh.port("q"), self.co.port("qCurrent"))
        disconnectPorts(self.st.port("q"), self.co.port("qRef"))
        disconnectPorts(self.co.port("q"), self.el.port("qRef"))
        connectPorts(self.rh.port("q"), self.co.port("qRef"))
        #
        connectPorts(self.st.port("q"), self.el.port("qRef"))

    def setSelfGroups(self):
        '''
        Set elements of body groups and joing groups that are statically
        defined as member variables within this class.
        '''
        for item in self.Groups:
            self.seq_svc.addJointGroup(item[0], item[1])

    def resetJointGroup(self):
        # remove jointGroups
        self.seq_svc.removeJointGroup("torso")
        self.seq_svc.removeJointGroup("head")
        self.seq_svc.removeJointGroup("larm")
        self.seq_svc.removeJointGroup("rarm")
        self.seq_svc.removeJointGroup("lleg")
        self.seq_svc.removeJointGroup("rleg")
        # setup jointGroups
        self.setSelfGroups() # restart groups

    def setupLogger(self):
        HrpsysConfigurator.setupLogger(self)
        self.connectLoggerPort(self.rh, 'q', 'command')

#    # delete co
#    def getRTCList(self):
#        return [self.rh, self.seq, self.sh, self.tf, self.kf, self.vs, self.ic, self.abc, self.st, self.el, self.log]

#    def activateComps(self):
#        HrpsysConfigurator.activateComps(self)
#        # stop co
#        self.co.stop()

    def init(self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)

if __name__ == '__main__':
    shcf=ATLASHrpsysConfigurator()
    shcf.init(sys.argv[1], sys.argv[2])
