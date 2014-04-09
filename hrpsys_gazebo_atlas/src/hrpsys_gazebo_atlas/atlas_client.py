#!/usr/bin/env python

import imp  ## for rosbuild
try:
    imp.find_module("hrpsys")
except:
    import roslib; roslib.load_manifest("hrpsys")

import hrpsys
from hrpsys.hrpsys_config import *
import OpenHRP

class ATLASHrpsysConfigurator(HrpsysConfigurator):
#    tjc = None

    def connectComps(self):
        HrpsysConfigurator.connectComps(self)
        #
        #disconnectPorts(self.st.port("q"),  self.co.port("qRef"))
        #disconnectPorts(self.el.port("q"),  self.rh.port("qRef"))
        #connectPorts(self.st.port("q"),  self.rh.port("qRef"))
        #connectPorts(self.st.port("q"), self.rh.port("qRef"))
        #self.tjc = self.createComp("TendonJointController", "tjc")
        #connectPorts(self.st.port("q"),  self.tjc.port("qRef"))
        #connectPorts(self.tjc.port("q"),  self.rh.port("qRef"))

    def getRTCList(self):
#        return [self.rh, self.seq, self.sh, self.tf, self.kf, self.afs, self.ic, self.abc, self.log]
#        return [self.rh, self.seq, self.sh, self.fk, self.tf, self.kf, self.vs, self.afs, self.ic, self.abc, self.st, self.log]
        #return [self.rh, self.seq, self.sh, self.fk, self.tf, self.kf, self.vs, self.afs, self.ic, self.abc, self.st, self.tjc, self.log]
        #return [self.rh, self.seq, self.sh, self.fk, self.tf, self.kf, self.vs, self.afs, self.ic, self.abc, self.st, self.tjc, self.log]
        #return [self.rh, self.seq, self.sh, self.fk, self.tf, self.kf, self.vs, self.afs, self.ic, self.abc, self.st, self.log]
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            ['vs', "VirtualForceSensor"],
            ['afs', "AbsoluteForceSensor"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            #['co', "CollisionDetector"],
            #['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]
        

    def init(self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        time.sleep(6)
        rh_svc = narrow(self.rh.service("service0"), "RobotHardwareService")
        rh_svc.setServoErrorLimit("all", 0); # disable RobotHardware Joint Error check

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
        #default = 4000 too shoort!
        self.log_svc.maxLength(400000)

if __name__ == '__main__':
    ATLASHrpsysConfigurator().init(sys.argv[1], sys.argv[2])
