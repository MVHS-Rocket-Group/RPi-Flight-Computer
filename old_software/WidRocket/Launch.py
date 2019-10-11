import time
import sys
sys.path.append('/home/pi/Desktop/WidRocket/Library/IMUlib')
import IMU

IMU.detectIMU()
IMU.initIMU()
startTime = time.time()
state = "Idle"
timeState = time.time()

#Returns different G forces
def timeInState():
	return time.time()-timeState
def timeRel():
	return time.time()-startTime
def accX():
	return IMU.readACCx()*(0.244)/1000
def accY():
	return IMU.readACCy()*(0.244)/1000
def accZ():
	return IMU.readACCz()*(0.244)/1000
def printLog():
	#printing ACC values
	print("Time=%.3f, "%timeRel()),
	print("State=%s"%state),
	print("X=%.3f, " % accX()),
	print("Y=%.3f, " %accY()),
	print("Z=%.3f" %accZ())
	return
def newState(str):
	global state
	global timeState
	state = str
	timeState = time.time()
	return
##################################################
#Edit here
while True:
	if state == "Idle":
		if accZ()<-2.0:
			newState("Thrust")
	elif state == "Thrust":
		if timeInState()>15:
			newState("PostThrust")
	elif state == "PostThrust":
		#hold tight
		state = "PostThrust"
	else:
		state = "uh oh"
	printLog()
	
	time.sleep(0.04)