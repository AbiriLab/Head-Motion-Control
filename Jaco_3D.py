import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data
import pygame as pg
import random
import numpy as np
import os
import csv
from scipy.spatial.transform import Rotation as R

use2D   = 0
logData = 0

if use2D == 1:
  from cursorUITest_2D import UI
else:
  from cursorUITest import UI

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


jacoId = p.loadURDF("jaco/j2n6s300.urdf", [0,0,0],  useFixedBase=True)

basePos = [0,0,0]
p.resetBasePositionAndOrientation(jacoId,basePos,[0,0,0,1])
#p.resetDebugVisualizerCamera(cameraDistance=0.20, cameraYaw=10, cameraPitch=-30, cameraTargetPosition=[-0.4,-0.35,0.0])
p.resetDebugVisualizerCamera(cameraDistance=1.20, cameraYaw=30, cameraPitch=-90, cameraTargetPosition=[-0.6,0.0,0.0])
#p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=20, cameraPitch=-30, cameraTargetPosition=[-0.6,0.0,0.0])
jacoEndEffectorIndex = 8
numJoints = 10
jacoArmJoints = [2, 3, 4, 5, 6, 7]
jacoFingerJoints = [9, 11, 13]

#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
rp = [0,math.pi/4,math.pi,1.0*math.pi, 1.8*math.pi, 0*math.pi, 1.75*math.pi, 0.5*math.pi]

wu = [0.1, 0.5, 0.5]
wl = [-.66, -.5, 0.00]

for i in range(8):
  p.resetJointState(jacoId,i, rp[i])

ls = p.getLinkState(jacoId, jacoEndEffectorIndex)
p.setGravity(0,0,-10)

t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0

useOrientation = 1
useSimulation = 1
useRealTimeSimulation = 1
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) 
trailDuration = 5

pg.init()
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d, %d" % (0,0)
pg.display.set_mode((500,500))
pg.display.set_caption("Control Interface")
runUI = UI(logData)

pos = list(ls[4])
orn = list(ls[5])

# print(ls)
i=0

JP = list(rp[2:9])
fing = 0
wri = 0

newPosInput = 1
keyT = time.time()
kTotal = np.zeros([9,], dtype = int)

pg.key.set_repeat()
kp5_up = 1
add_KP5 = 1

dist = .002
ang = .005
rot_theta = .008
inputRate = .05

Rx = np.array([[1., 0., 0.],[0., np.cos(rot_theta), -np.sin(rot_theta)], [0., np.sin(rot_theta), np.cos(rot_theta)]])
Ry = np.array([[np.cos(rot_theta), 0., np.sin(rot_theta)], [0., 1., 0.], [-np.sin(rot_theta), 0., np.cos(rot_theta)]])
Rz = np.array([[np.cos(rot_theta), -np.sin(rot_theta), 0.], [np.sin(rot_theta), np.cos(rot_theta), 0.], [0., 0., 1.]])

Rxm = np.array([[1., 0., 0.],[0., np.cos(-rot_theta), -np.sin(-rot_theta)], [0., np.sin(-rot_theta), np.cos(-rot_theta)]])
Rym = np.array([[np.cos(-rot_theta), 0., np.sin(-rot_theta)], [0., 1., 0.], [-np.sin(-rot_theta), 0., np.cos(-rot_theta)]])
Rzm = np.array([[np.cos(-rot_theta), -np.sin(-rot_theta), 0.], [np.sin(-rot_theta), np.cos(-rot_theta), 0.], [0., 0., 1.]])

updateT = time.time()

if logData:
  dataDir = time.strftime("%Y%m%d")
  if not os.path.exists(dataDir):
    os.makedirs(dataDir)
  trialInd = len(os.listdir(dataDir))
  fn = dataDir + "/simdata00" + str(trialInd) + ".csv"
      
  logFile = open(fn, 'w', newline = '')
  fileObj = csv.writer(logFile)

while 1:

  i+=1
  if (useRealTimeSimulation):
    dt = datetime.now()
    t = (dt.second / 60.) * 2. * math.pi
  else:
    t = t + 0.01

  if (useSimulation and useRealTimeSimulation == 0):
    p.stepSimulation()

  delta = time.time() - updateT 

  if delta > inputRate:
    updateT= time.time()

    if logData:
      lsr = p.getLinkState(jacoId, jacoEndEffectorIndex)
      lsc = p.getBasePositionAndOrientation(cube1Id)

      ln = [updateT, lsr[4][0],lsr[4][1],lsr[4][2], lsr[5][0],lsr[5][1], lsr[5][2], lsr[5][3], fing, lsc[0][0],lsc[0][1],lsc[0][2], lsc[1][0],lsc[1][1], lsc[1][2], lsc[1][3]] 
      ln_rnd = [round(num, 4) for num in ln]
      fileObj.writerow(ln_rnd)

    eulOrn = p.getEulerFromQuaternion(orn)
    Rrm = R.from_quat(orn)

    rx = eulOrn[0]
    ry = eulOrn[1]
    rz = eulOrn[2]

    runUI.update()
    inputMode = runUI.mode
    inputKey  = runUI.state

    baseTheta = JP[0]
    s = math.sin(baseTheta)
    c = math.cos(baseTheta)

    c1 = math.cos(ang)
    s1 = math.sin(ang)

    c2 = math.cos(-ang)
    s2 = math.sin(-ang)

    n = np.sqrt(pos[0]*pos[0] + pos[1]*pos[1])
    dx = -pos[1]/n
    dy = pos[0]/n

    Rnew =  Rrm.as_matrix() 

    if use2D:
      if inputMode == 0:
        if inputKey == 4:
          pos[0] = pos[0] + dist*dx
          pos[1] = pos[1] + dist*dy
          newPosInput = 1
        if inputKey == 6:
          pos[0] = pos[0] - dist*dx
          pos[1] = pos[1] - dist*dy
          newPosInput = 1
        if inputKey == 8:
          pos[0] = pos[0] + dist*c
          pos[1] = pos[1] - dist*s
          newPosInput = 1
        if inputKey == 2:
          pos[0] = pos[0] - dist*c
          pos[1] = pos[1] + dist *s
          newPosInput = 1

      if inputMode ==1:
        if inputKey == 8:
          pos[2] = pos[2] + dist 
          newPosInput = 1
        if inputKey == 2:
          pos[2] = pos[2] - dist 
          newPosInput = 1
        if inputKey == 4:
          Rnew = Rrm.as_matrix() @ Rz
          newPosInput = 1
        if inputKey == 6:
          Rnew = Rrm.as_matrix() @ Rzm
          newPosInput = 1

      if inputMode == 2:
        if inputKey == 8:
          Rnew = Rrm.as_matrix() @ Rx
          newPosInput = 1
        if inputKey == 2:
          Rnew = Rrm.as_matrix() @ Rxm
          newPosInput = 1
        if inputKey == 6:
          Rnew = Rrm.as_matrix() @ Ry
          newPosInput = 1
        if inputKey == 4:
          Rnew = Rrm.as_matrix() @ Rym
          newPosInput = 1

      if inputMode == 3:
        if inputKey == 8:
          fing = fing - dist*5 
        if inputKey == 2:
          fing = fing + dist*5

    else:
      if inputMode == 0:
        if inputKey == 4:
          pos[0] = pos[0] + dist*dx
          pos[1] = pos[1] + dist*dy
          newPosInput = 1
        if inputKey == 6:
          pos[0] = pos[0] - dist*dx
          pos[1] = pos[1] - dist*dy
          newPosInput = 1
        if inputKey == 8:
          pos[0] = pos[0] + dist*c
          pos[1] = pos[1] - dist*s
          newPosInput = 1
        if inputKey == 2:
          pos[0] = pos[0] - dist*c
          pos[1] = pos[1] + dist*s
          newPosInput = 1
        if inputKey == 7:
          pos[2] = pos[2] + dist 
          newPosInput = 1
        if inputKey == 1:
          pos[2] = pos[2] - dist 
          newPosInput = 1


      if inputMode == 1:
        if inputKey == 4:
          Rnew = Rrm.as_matrix() @ Rz
          newPosInput = 1
        if inputKey == 6:
          Rnew = Rrm.as_matrix() @ Rzm
          newPosInput = 1
        if inputKey == 8:
          Rnew = Rrm.as_matrix() @ Rx
          newPosInput = 1
        if inputKey == 2:
          Rnew = Rrm.as_matrix() @ Rxm
          newPosInput = 1
        if inputKey == 7:
          Rnew = Rrm.as_matrix() @ Ry
          newPosInput = 1
        if inputKey == 1:
          Rnew = Rrm.as_matrix() @ Rym
          newPosInput = 1

      if inputMode == 2:
        if inputKey == 8:
          fing = fing - dist*5 
        if inputKey == 2:
          fing = fing + dist*5

    Rn = R.from_matrix(Rnew)
    orn = Rn.as_quat()

    if pos[0] > wu[0]:
      pos[0] =  wu[0]
    if pos[0] < wl[0]:
      pos[0] =  wl[0]
    if pos[1] > wu[1]:
      pos[1] =  wu[1]
    if pos[1] < wl[1]:
      pos[1] =  wl[1]
    if pos[2] > wu[2]:
      pos[2] =  wu[2]
    if pos[2] < wl[2]:
      pos[2] =  wl[2]

    if fing > 1.35:
      fing = 1.35
    if fing < 0:
      fing = 0

  if (newPosInput == 1):
    if (useNullSpace == 1):
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(jacoId, jacoEndEffectorIndex, pos, orn, ll, ul,
                                                  jr, rp)
      else:
        jointPoses = p.calculateInverseKinematics(jacoId,
                                                  jacoEndEffectorIndex,
                                                  pos,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)
    else:
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(jacoId,
                                                  jacoEndEffectorIndex,
                                                  pos,
                                                  orn,
                                                  jointDamping=jd,
                                                  solver=ikSolver,
                                                  maxNumIterations=100,
                                                  residualThreshold=.01)
        JP = list(jointPoses)
      else:
        jointPoses = p.calculateInverseKinematics(jacoId,
                                                  jacoEndEffectorIndex,
                                                  pos,
                                                  solver=ikSolver)
        JP = list(jointPoses)

  if (useSimulation):
    JS = p.getJointStates(jacoId, [1, 2, 3, 4, 5, 6, 7, 9, 11, 13])
    j = 0
    for i in [2,3,4,5,6,7]:
      p.setJointMotorControl2(jacoId, i, p.POSITION_CONTROL, JP[j])
      j = j+1
    
    for i in  [9, 11, 13]:
      p.setJointMotorControl2(jacoId, i, p.POSITION_CONTROL, fing)

  else:
    j = 0
    for i in jacoArmJoints:
      p.resetJointState(jacoId, i, jointPoses[j])
      j = j+1

  ls = p.getLinkState(jacoId, jacoEndEffectorIndex)

  prevPose = tuple(pos)
  prevPose1 = ls[4]
  hasPrevPose = 1
  newPosInput = 0


file.close()
p.disconnect()