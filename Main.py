from threading import Thread
import serial
import time
import collections
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
            self.dataTypes = 'd'    # 8 byte double
        self.data = []
        for i in range(numPlots):   # give an array for each type of data and store them in a list
            self.data.append(collections.deque([0] * plotLength, maxlen=plotLength))
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        # self.csvData = []

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

        self.wu = [0.1, 0.5, 0.5]
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
        #runUI = UI(logData)

        self.pos = list(self.ls[4])
        self.orn = list(self.ls[5])

        # print(ls)
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

    #Compiles data for plotting, not data for mouse movement
    def compilePlotData(self, frame, lines, lineValueText, lineLabel, timeText):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
        self.previousTimer = currentTimer
        timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')
        privateData = copy.deepcopy(self.rawData[:])    # so that the 2 values in our plots will be synchronized to the same sample time
        for i in range(self.numPlots):
            data = privateData[(i*self.dataNumBytes):(self.dataNumBytes + i*self.dataNumBytes)]
            value,  = struct.unpack(self.dataType, data)
            self.data[i].append(value)    # we get the latest data point and append it to our array
            lines[i].set_data(range(self.plotMaxLength), self.data[i])
            lineValueText[i].set_text('[' + lineLabel[i] + '] = ' + str(value))
        # self.csvData.append([self.data[0][-1], self.data[1][-1], self.data[2][-1]])
        print(self.data[1][-1])
        #mouse.move(40*self.data[0][-1],30*self.data[1][-1],absolute=False)

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

    #Function to move the mouse called from backgroundThread()
    #Uses gravitational acceleration in x and y and gyro integration in z
    def mouseMoveAcc1(self,xa,ya,zg):
        self.thisSampleTime = time.time()
        deltaTime = (self.thisSampleTime - self.lastSampleTime)*1000
        self.lastSampleTime = self.thisSampleTime
        #print("DELTA: " + str(deltaTime))

        self.zPos += zg*deltaTime
        x = xa*3
        y = self.zPos
        if self.counter == 20:
            print("XPos: " + str(x) + " YPos: " + str(y))
            self.counter = 0
        mouse.move(x,0,absolute=False)
        mouse.move(mouse.get_position()[0],y,absolute=True)
        self.counter += 1

    #Function to move the mouse called from backgroundThread()
    def mouseMoveAcc2(self,xa,ya,zg):
        self.thisSampleTime = time.time()
        deltaTime = (self.thisSampleTime - self.lastSampleTime)*1000
        self.lastSampleTime = self.thisSampleTime
        #print("DELTA: " + str(deltaTime))

        self.xPos = (xa * (self.x_bound/2)/(self.max_acceleration_x*self.gravity)) + self.x_bound/2
        self.yPos = 1080-((ya * (self.y_bound/2)/(self.max_acceleration_y*self.gravity)) + self.y_bound/2)
        if self.counter == 20:
            print("XPos: " + str(self.xPos) + " YPos: " + str(self.yPos))
            self.counter = 0
        mouse.move(self.xPos,self.yPos,absolute=True)
        self.counter += 1

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
            #print("X: " + str(round(move[0],4)) + " Y: " + str(round(move[1],4)))

            #self.mouseMoveAcc1(round(move[0],4),round(move[1],4),round(move[2],4))
            #self.mouseMoveGyro(round(move[0],4), round(move[1],4),round(move[2],4))






            i+=1
            # if (useRealTimeSimulation):
            #     #dt = datetime.now()
            #     #t = (dt.second / 60.) * 2. * math.pi
            # else:
            #     t = t + 0.01

            if (self.useSimulation and self.useRealTimeSimulation == 0):
                p.stepSimulation()

            delta = time.time() - self.updateT 

            if delta > self.inputRate:
                updateT= time.time()

                eulOrn = p.getEulerFromQuaternion(self.orn)
                Rrm = R.from_quat(self.orn)

                rx = eulOrn[0]
                ry = eulOrn[1]
                rz = eulOrn[2]

                #runUI.update()
                #inputMode = runUI.mode
                inputMode = 0
                #inputKey  = runUI.state
                inputKey = 2

                baseTheta = JP[0]
                s = math.sin(baseTheta)
                c = math.cos(baseTheta)

                c1 = math.cos(self.ang)
                s1 = math.sin(self.ang)

                c2 = math.cos(-self.ang)
                s2 = math.sin(-self.ang)

                n = np.sqrt(self.pos[0]*self.pos[0] + self.pos[1]*self.pos[1])
                dx = -self.pos[1]/n
                dy = self.pos[0]/n

                Rnew =  Rrm.as_matrix() 

                if self.use2D:
                    if inputMode == 0:
                        if inputKey == 4:
                            self.pos[0] = self.pos[0] + self.dist*dx
                            self.pos[1] = self.pos[1] + self.dist*dy
                            newPosInput = 1
                        if inputKey == 6:
                            self.pos[0] = self.pos[0] - self.dist*dx
                            self.pos[1] = self.pos[1] - self.dist*dy
                            newPosInput = 1
                        if inputKey == 8:
                            self.pos[0] = self.pos[0] + self.dist*c
                            self.pos[1] = self.pos[1] - self.dist*s
                            newPosInput = 1
                        if inputKey == 2:
                            self.pos[0] = self.pos[0] - self.dist*c
                            self.pos[1] = self.pos[1] + self.dist *s
                            newPosInput = 1

                    if inputMode ==1:
                        if inputKey == 8:
                            self.pos[2] = self.pos[2] + self.dist 
                            newPosInput = 1
                        if inputKey == 2:
                            self.pos[2] = self.pos[2] - self.dist 
                            newPosInput = 1
                        if inputKey == 4:
                            Rnew = Rrm.as_matrix() @ self.Rz
                            newPosInput = 1
                        if inputKey == 6:
                            Rnew = Rrm.as_matrix() @ self.Rzm
                            newPosInput = 1

                    if inputMode == 2:
                        if inputKey == 8:
                            Rnew = Rrm.as_matrix() @ self.Rx
                            newPosInput = 1
                        if inputKey == 2:
                            Rnew = Rrm.as_matrix() @ self.Rxm
                            newPosInput = 1
                        if inputKey == 6:
                            Rnew = Rrm.as_matrix() @ self.Ry
                            newPosInput = 1
                        if inputKey == 4:
                            Rnew = Rrm.as_matrix() @ self.Rym
                            newPosInput = 1

                    if inputMode == 3:
                        if inputKey == 8:
                            fing = fing - self.dist*5 
                        if inputKey == 2:
                            fing = fing + self.dist*5

                # else:
                #     if inputMode == 0:
                #         if inputKey == 4:
                #             self.pos[0] = self.pos[0] + self.dist*dx
                #             self.pos[1] = self.pos[1] + self.dist*dy
                #             newPosInput = 1
                #         if inputKey == 6:
                #             self.pos[0] = self.pos[0] - self.dist*dx
                #             self.pos[1] = self.pos[1] - self.dist*dy
                #             newPosInput = 1
                #         if inputKey == 8:
                #             self.pos[0] = self.pos[0] + self.dist*c
                #             self.pos[1] = self.pos[1] - self.dist*s
                #             newPosInput = 1
                #         if inputKey == 2:
                #             pos[0] = pos[0] - dist*c
                #             pos[1] = pos[1] + dist*s
                #             newPosInput = 1
                #         if inputKey == 7:
                #             pos[2] = pos[2] + dist 
                #             newPosInput = 1
                #         if inputKey == 1:
                #             pos[2] = pos[2] - dist 
                #             newPosInput = 1


                #     if inputMode == 1:
                #         if inputKey == 4:
                #             Rnew = Rrm.as_matrix() @ Rz
                #             newPosInput = 1
                #         if inputKey == 6:
                #             Rnew = Rrm.as_matrix() @ Rzm
                #             newPosInput = 1
                #         if inputKey == 8:
                #             Rnew = Rrm.as_matrix() @ Rx
                #             newPosInput = 1
                #         if inputKey == 2:
                #             Rnew = Rrm.as_matrix() @ Rxm
                #             newPosInput = 1
                #         if inputKey == 7:
                #             Rnew = Rrm.as_matrix() @ Ry
                #             newPosInput = 1
                #         if inputKey == 1:
                #             Rnew = Rrm.as_matrix() @ Rym
                #             newPosInput = 1

                #     if inputMode == 2:
                #         if inputKey == 8:
                #             fing = fing - dist*5 
                #         if inputKey == 2:
                #             fing = fing + dist*5

                Rn = R.from_matrix(Rnew)
                orn = Rn.as_quat()

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
                        print(jointPoses)
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
            #print(self.rawData)

    #Ends the serial connection
    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')
        


def main():
    #Call the practice UI
    #subprocess.Popen([sys.executable, "ui.py"])

    # portName = 'COM5'
    portName = 'COM16'
    baudRate = 250000
    maxPlotLength = 100     # number of points in x-axis of real time plot
    dataNumBytes = 4        # number of bytes of 1 data point
    numPlots = 3            # number of plots in 1 graph
    s = IMUMouse(portName, baudRate, maxPlotLength, dataNumBytes, numPlots)   # initializes all required variables
    s.IMUMouseStart()                                               # starts background thread

    # # plotting starts below
    # pltInterval = 10    # Period at which the plot animation updates [ms]
    # xmin = 0
    # xmax = maxPlotLength
    # ymin = -(10)
    # ymax = 10
    # fig = plt.figure(figsize=(10, 8))
    # ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    # ax.set_title('Angular Velocity over Time')
    # ax.set_xlabel("Time (s)")
    # ax.set_ylabel("Angular Velocity (rad/s)")

    # lineLabel = ['X', 'Y','Z']
    # style = ['r-', 'c-','r-']  # linestyles for the different plots
    # timeText = ax.text(0.70, 0.95, '', transform=ax.transAxes)
    # lines = []
    # lineValueText = []
    # for i in range(numPlots):
    #     lines.append(ax.plot([], [], style[i], label=lineLabel[i])[0])
    #     lineValueText.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))
    # #anim = animation.FuncAnimation(fig, s.compilePlotData, fargs=(lines, lineValueText, lineLabel, timeText), interval=pltInterval)    # fargs has to be a tuple

    # plt.legend(loc="upper left")
    # #plt.show()

    # #s.close()


if __name__ == '__main__':
    main()