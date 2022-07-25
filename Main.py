from threading import Thread
import serial
import time
#import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import copy
import pandas as pd
import mouse
import subprocess
import sys
import pybullet as p
import pybullet_data
import pygame as pg
import math
import os
import numpy as np
import datetime
from scipy.spatial.transform import Rotation as R
import cursorUITest as UI

class IMUMouse:

    #Initialize all values
    def __init__(self, serialPort='COM16', serialBaud=250000, plotLength=100, dataNumBytes=4, numPlots=3):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = plotLength
        self.dataNumBytes = dataNumBytes
        self.numPlots = numPlots
        self.rawData = bytearray(numPlots * dataNumBytes)
        self.dataType = None
        if dataNumBytes == 2:
            self.dataType = 'h'     # 2 byte integer
        elif dataNumBytes == 4:
            self.dataType = 'f'     # 4 byte float
        elif dataNumBytes == 8:
            self.dataTypes = 'd'   # 8 byte double
        self.data = []
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.previousTimer = 0

        #Set the intial position to the center of the screen
        self.xPos = 910
        self.yPos = 540
        self.zPos = 540
        self.lastSampleTime = time.time()
        self.thisSampleTime = 0

        self.counter = 0
        self.max_acceleration_x = 0.707
        self.max_acceleration_y = 0.707
        self.x_bound = 1920
        self.y_bound = 1080
        self.gravity = 9.81
        self.z_move = 0
        self.is_trans = 0
        self.is_orn = 1

        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')




        #Initialization for pybullet
        self.use2D   = 0
        self.logData = 0

        clid = p.connect(p.SHARED_MEMORY)
        if (clid<0):
            p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

        p.loadURDF("plane.urdf",[0,0,-.65])
        p.loadURDF("table/table.urdf", basePosition=[-0.4,0.0,-0.65])
        # p.loadURDF("tray/tray.urdf",[-0.8,-0.0,0.0])
        #p.loadURDF("dinnerware/cup/cup_small.urdf",[-0.4,-0.35,0.0])
        #p.loadURDF("dinnerware/plate.urdf",[-0.3,0,0.0])
        p.loadURDF("cube_small.urdf",[-0.4,0.35,0.0])
        p.loadURDF("sphere_small.urdf",[-0.2,-0.35,0.0])
        p.loadURDF("duck_vhacd.urdf",[0,-0.45,0.0])
        p.loadURDF("teddy_vhacd.urdf",[0.1,-0.35,0.0])

        p.loadURDF("block.urdf",[-0.7,0.0,0.0])

        cube1Id = p.loadURDF("cube_small.urdf",[-0.4,-0.4,0.0])
        # p.loadURDF("cube_small.urdf",[-0.3,-0.15,0.0])
        # p.loadURDF("cube_small.urdf",[-0.2,0.2,0.0])


        self.jacoId = p.loadURDF("jaco/j2n6s300.urdf", [0,0,0],  useFixedBase=True)

        self.basePos = [0,0,0]
        p.resetBasePositionAndOrientation(self.jacoId,self.basePos,[0,0,0,1])
        #p.resetDebugVisualizerCamera(cameraDistance=0.20, cameraYaw=10, cameraPitch=-30, cameraTargetPosition=[-0.4,-0.35,0.0])
        p.resetDebugVisualizerCamera(cameraDistance=1.20, cameraYaw=30, cameraPitch=-90, cameraTargetPosition=[-0.6,0.0,0.0])
        #p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=20, cameraPitch=-30, cameraTargetPosition=[-0.6,0.0,0.0])
        self.jacoEndEffectorIndex = 8
        self.numJoints = 10
        self.jacoArmJoints = [2, 3, 4, 5, 6, 7]
        self.jacoFingerJoints = [9, 11, 13]

        #joint damping coefficents
        self.jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        self.rp = [0,math.pi/4,math.pi,1.0*math.pi, 1.8*math.pi, 0*math.pi, 1.75*math.pi, 0.5*math.pi]


        # These are the upper and lower limits for movement
        self.wu = [-0.1, 0.5, 0.5]
        self.wl = [-.66, -.5, 0.00]

        for i in range(8):
            p.resetJointState(self.jacoId,i, self.rp[i])

        self.ls = p.getLinkState(self.jacoId, self.jacoEndEffectorIndex)
        p.setGravity(0,0,-10)

        self.t = 0.
        self.prevPose = [0, 0, 0]
        self.prevPose1 = [0, 0, 0]
        self.hasPrevPose = 0
        self.useNullSpace = 0

        self.useOrientation = 1
        self.useSimulation = 1
        self.useRealTimeSimulation = 1
        self.ikSolver = 0
        p.setRealTimeSimulation(self.useRealTimeSimulation)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) 
        self.trailDuration = 5

        pg.init()
        os.environ['SDL_VIDEO_WINDOW_POS'] = "%d, %d" % (0,0)
        pg.display.set_mode((500,500))
        pg.display.set_caption("Control Interface")
        #
        # runUI = UI(logData)

        self.pos = list(self.ls[4])
        self.pos = [-0.3, 0, 0.2]
        self.orn = list(self.ls[5])

        i=0

        self.JP = list(self.rp[2:9])
        self.fing = 0
        self.wri = 0

        self.newPosInput = 1
        self.keyT = time.time()
        self.kTotal = np.zeros([9,], dtype = int)

        pg.key.set_repeat()
        kp5_up = 1
        add_KP5 = 1

        self.dist = .002
        self.ang = .005
        self.rot_theta = .008
        self.inputRate = .05

        self.Rx = np.array([[1., 0., 0.],[0., np.cos(self.rot_theta), -np.sin(self.rot_theta)], [0., np.sin(self.rot_theta), np.cos(self.rot_theta)]])
        self.Ry = np.array([[np.cos(self.rot_theta), 0., np.sin(self.rot_theta)], [0., 1., 0.], [-np.sin(self.rot_theta), 0., np.cos(self.rot_theta)]])
        self.Rz = np.array([[np.cos(self.rot_theta), -np.sin(self.rot_theta), 0.], [np.sin(self.rot_theta), np.cos(self.rot_theta), 0.], [0., 0., 1.]])

        self.Rxm = np.array([[1., 0., 0.],[0., np.cos(-self.rot_theta), -np.sin(-self.rot_theta)], [0., np.sin(-self.rot_theta), np.cos(-self.rot_theta)]])
        self.Rym = np.array([[np.cos(-self.rot_theta), 0., np.sin(-self.rot_theta)], [0., 1., 0.], [-np.sin(-self.rot_theta), 0., np.cos(-self.rot_theta)]])
        self.Rzm = np.array([[np.cos(-self.rot_theta), -np.sin(-self.rot_theta), 0.], [np.sin(-self.rot_theta), np.cos(-self.rot_theta), 0.], [0., 0., 1.]])

        self.updateT = time.time()

    #Function to initialize the data collection thread
    def IMUMouseStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

    #Function to move the mouse called from backgroundThread()
    def mouseMoveGyro(self,xg,yg,zg):
        self.thisSampleTime = time.time()
        deltaTime = (self.thisSampleTime - self.lastSampleTime)*1000
        self.lastSampleTime = self.thisSampleTime
        print("DELTA: " + str(deltaTime))
        self.xPos += round(1.2*xg,1)
        self.yPos += round(0.8*yg,1)
        print("XPos: " + str(self.xPos) + " YPos: " + str(self.yPos))
        mouse.move(self.xPos,self.yPos,absolute=True)

    #Function to receive data continuously
    def backgroundThread(self):    # retrieve data                                              
        #time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.serialConnection.readinto(self.rawData)
            move = []
            for i in range(self.numPlots):
                data = self.rawData[(i*self.dataNumBytes):(self.dataNumBytes + i*self.dataNumBytes)]
                value, = struct.unpack(self.dataType, data)
                move.append(value)





            i+=1

            if (self.useSimulation and self.useRealTimeSimulation == 0):
                p.stepSimulation()

            delta = time.time() - self.updateT 

            if delta > self.inputRate:
                # updateT= time.time()

                # eulOrn = p.getEulerFromQuaternion()
                Rrm = R.from_quat(self.orn)

                # rx = eulOrn[0]
                # ry = eulOrn[1]
                # rz = eulOrn[2]

                #runUI.update()
                #inputMode = runUI.mode
                # inputMode = 0
                #inputKey  = runUI.state
                # inputKey = 2

                # baseTheta = JP[0]
                # s = math.sin(baseTheta)
                # c = math.cos(baseTheta)

                # c1 = math.cos(self.ang)
                # s1 = math.sin(self.ang)

                # c2 = math.cos(-self.ang)
                # s2 = math.sin(-self.ang)

                # n = np.sqrt(self.pos[0]*self.pos[0] + self.pos[1]*self.pos[1])
                # dx = -self.pos[1]/n
                # dy = self.pos[0]/n


                Rnew =  Rrm.as_euler('xyz')   
                print(Rnew)         


                if move[0] > 0.2 or move[0] < -0.2:
                    if self.is_trans:
                        self.pos[0] += 0.0001*move[0]
                    if self.is_orn:
                        Rnew[0] += move[0]*0.001

                if move[1] > 0.2 or move[1] < -0.2:
                    if self.is_trans:
                        self.pos[1] -= 0.0001*move[1]
                    if self.is_orn:
                        Rnew[1] += move[1]*0.001
                
                #self.z_move += move[2]
                #self.pos[2] += 0.000001*self.z_move


                Rn = R.from_rotvec(Rnew)
                self.orn = Rn.as_quat()

                if self.pos[0] > self.wu[0]:
                    self.pos[0] =  self.wu[0]
                if self.pos[0] < self.wl[0]:
                    self.pos[0] =  self.wl[0]
                if self.pos[1] > self.wu[1]:
                    self.pos[1] =  self.wu[1]
                if self.pos[1] < self.wl[1]:
                    self.pos[1] =  self.wl[1]
                if self.pos[2] > self.wu[2]:
                    self.pos[2] =  self.wu[2]
                if self.pos[2] < self.wl[2]:
                    self.pos[2] =  self.wl[2]

                if self.fing > 1.35:
                    self.fing = 1.35
                if self.fing < 0:
                    self.fing = 0

            if (self.newPosInput == 1):
                if (self.useNullSpace == 1):
                    if (self.useOrientation == 1):
                        jointPoses = p.calculateInverseKinematics(self.jacoId, self.jacoEndEffectorIndex, self.pos, self.orn, ll, ul,
                                                                jr, rp)
                    else:
                        jointPoses = p.calculateInverseKinematics(self.jacoId,
                                                                self.jacoEndEffectorIndex,
                                                                self.pos,
                                                                lowerLimits=ll,
                                                                upperLimits=ul,
                                                                jointRanges=jr,
                                                                restPoses=rp)
                else:
                    if (self.useOrientation == 1):
                        jointPoses = p.calculateInverseKinematics(self.jacoId,
                                                                self.jacoEndEffectorIndex,
                                                                self.pos,
                                                                self.orn,
                                                                jointDamping=self.jd,
                                                                solver=self.ikSolver,
                                                                maxNumIterations=100,
                                                                residualThreshold=.01)
                        (jointPoses)
                        JP = list(jointPoses)
                    else:
                        jointPoses = p.calculateInverseKinematics(self.jacoId,
                                                                self.jacoEndEffectorIndex,
                                                                self.pos,
                                                                solver=self.ikSolver)
                        JP = list(jointPoses)

            if (self.useSimulation):
                JS = p.getJointStates(self.jacoId, [1, 2, 3, 4, 5, 6, 7, 9, 11, 13])
                j = 0
                for i in [2,3,4,5,6,7]:
                    p.setJointMotorControl2(self.jacoId, i, p.POSITION_CONTROL, JP[j])
                    j = j+1
                
                for i in  [9, 11, 13]:
                    p.setJointMotorControl2(self.jacoId, i, p.POSITION_CONTROL, self.fing)

            else:
                j = 0
                for i in self.jacoArmJoints:
                    p.resetJointState(self.jacoId, i, self.jointPoses[j])
                    j = j+1

            ls = p.getLinkState(self.jacoId, self.jacoEndEffectorIndex)

            prevPose = tuple(self.pos)
            prevPose1 = ls[4]
            hasPrevPose = 1
            newPosInput = 0






            self.isReceiving = True


    #Ends the serial connection
    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        


def main():

    subprocess.Popen("PyGame_UI.py")

    portName = 'COM16'
    baudRate = 250000
    maxPlotLength = 100     # number of points in x-axis of real time plot
    dataNumBytes = 4        # number of bytes of 1 data point
    numPlots = 3            # number of plots in 1 graph
    s = IMUMouse(portName, baudRate, maxPlotLength, dataNumBytes, numPlots)   # initializes all required variables
    s.IMUMouseStart()                                               # starts background thread

    # #s.close()


if __name__ == '__main__':
    main()