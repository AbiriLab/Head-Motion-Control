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

            self.mouseMoveAcc1(round(move[0],4),round(move[1],4),round(move[2],4))
            #self.mouseMoveGyro(round(move[0],4), round(move[1],4),round(move[2],4))

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
    subprocess.Popen([sys.executable, "ui.py"])

    # portName = 'COM5'
    portName = 'COM16'
    baudRate = 250000
    maxPlotLength = 100     # number of points in x-axis of real time plot
    dataNumBytes = 4        # number of bytes of 1 data point
    numPlots = 3            # number of plots in 1 graph
    s = IMUMouse(portName, baudRate, maxPlotLength, dataNumBytes, numPlots)   # initializes all required variables
    s.IMUMouseStart()                                               # starts background thread

    # plotting starts below
    pltInterval = 10    # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin = -(10)
    ymax = 10
    fig = plt.figure(figsize=(10, 8))
    ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Angular Velocity over Time')
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Velocity (rad/s)")

    lineLabel = ['X', 'Y','Z']
    style = ['r-', 'c-','r-']  # linestyles for the different plots
    timeText = ax.text(0.70, 0.95, '', transform=ax.transAxes)
    lines = []
    lineValueText = []
    for i in range(numPlots):
        lines.append(ax.plot([], [], style[i], label=lineLabel[i])[0])
        lineValueText.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))
    #anim = animation.FuncAnimation(fig, s.compilePlotData, fargs=(lines, lineValueText, lineLabel, timeText), interval=pltInterval)    # fargs has to be a tuple

    plt.legend(loc="upper left")
    #plt.show()

    #s.close()


if __name__ == '__main__':
    main()