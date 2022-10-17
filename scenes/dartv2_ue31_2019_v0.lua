-- Following function writes data to the socket (only single packet data for simplicity sake):
writeSocketData=function(client,data)
    local header=string.char(59,57,math.mod(#data,256),math.floor(#data/256),0,0)
    -- Packet header is (in this case): headerID (59,57), dataSize (WORD), packetsLeft (WORD) but not used here
    data_all = header..data
    --print ("send",string.len(data_all),"bytes ...")
    client:send(data_all)
end

-- Following function reads data from the socket (only single packet data for simplicity sake):
readSocketData=function(client)
    -- Packet header is: headerID (59,57), dataSize (WORD), packetsLeft (WORD) but not used here
    local header=client:receive(6)
    if (header==nil) then
        return(nil) -- error
    end
    if (header:byte(1)==59)and(header:byte(2)==57) then
        local l=header:byte(3)+header:byte(4)*256
        return(client:receive(l))
    else
        return(nil) -- error
    end
end

-- Use sleeping function of socket library
function sleep(sec)
    socket.select(nil,nil,sec)
end 

-- Get cuurent time (in sec) 
function gettime()
   return socket.gettime()
end

-- Get wheel angular position (in degrees)
function getWheelAngularPosition (handle)
    angles=simGetObjectOrientation(handle,sim_handle_parent)
    angPos = angles[3]*180.0/math.pi
    angPos = angPos + 180.0
    return angPos
end 
-- Compute increment of odemeter (
function deltaWheelAngularPosition(curPos,lastPos,wSpeed)
    if wSpeed == 0.0 then
        deltaPos = 0.0
    else
        deltaPos = curPos - lastPos
    end
    if deltaPos < 0.0 then
        if wSpeed > 0.0 then
            deltaPos = deltaPos + 360.0
        end
    else
        if wSpeed < 0.0 then
            deltaPos = deltaPos - 360.0
        end
    end
    local nTicks=300.0
    deltaPos = deltaPos*nTicks/360.0
    --print ("delta pos ",curPos,lastPos,wSpeed,deltaPos)
    return deltaPos
end

-- Take speed command and convert into V-REP motor command

function motor_curve (vin)
   vout = 0.0
   if vin >= 70.0 then
      a = 60.0
      b = 1.0
      v = vin
      v = math.log(v-a+b)
      vmx = math.log(255.0-a+b)
      vmn = math.log(70.0-a+b)
      vout = 70.0+(v-vmn)*(255.0-70.0)/(vmx-vmn)
   end
   return vout
end

function defineLeftSpeed (cmd)
    cmd1=cmd
    if cmd1 < 0 then
        cmd1 = -cmd1
    end
    if cmd1 < 70 then
        cmd1 = 0
    end
    if cmd1 > 255 then 
        cmd1 = 255
    end
    cmd1 = motor_curve(cmd1)
    vrepCmd = cmd1*cvtCmdSpeed
    if cmd < 0 then
        vrepCmd = -vrepCmd
    end
    return vrepCmd
end
function defineRightSpeed (cmd)
    cmd1=cmd
    if cmd1 < 0 then
        cmd1 = -cmd1
    end
    if cmd1 < 70 then
        cmd1 = 0
    end
    if cmd1 > 255 then 
        cmd1 = 255
    end
    cmd1 = motor_curve(cmd1)
    vrepCmd = cmd1*cvtCmdSpeed
    if cmd < 0 then
        vrepCmd = -vrepCmd
    end
    return vrepCmd
end


threadFunction=function()
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
        -- This is executed at each simulation step
      local t0 = gettime()
      local simTime = simGetSimulationTime()

      -- data sent to python control program dartv2b.py
      -- simulationTime
      -- distFront, distRear,distLeft, distRight
      -- distFrontLeft,distFrontRight
      -- encoderFrontLeft, encoderFrontRight, encoderRearLeft, encoderRearRight
      -- xRob,yRob,zRob,xAcc,yAcc,zAcc,xGyro,yGyro,zGyro
      local rb = simGetObjectPosition(dart,-1) -- -1 , location in world coordinates
      -- get accelerometer data
      tmpData=simTubeRead(accelCommunicationTube)
      if (tmpData) then
	 accel=simUnpackFloats(tmpData)
      end
      -- get gyroscope data
      tmpData=simTubeRead(gyroCommunicationTube)
      if (tmpData) then
	 gyro=simUnpackFloats(tmpData)
      end
      --local stOut = "Dart Accel x="..accel[1]..", y="..accel[2]..", z="..accel[3]
      --stOut = stOut.." , Gyro x="..gyro[1]..", y="..gyro[2]..", z="..gyro[3]
      --print (stOut)

      
      local dataOut={simTime,distFront,distRear,distLeft,distRight,distFrontLeft,distFrontRight,frontLeftEncoder,frontRightEncoder,rearLeftEncoder,rearRightEncoder,heading,rb[1],rb[2],rb[3],accel[1],accel[2],accel[3],gyro[1],gyro[2],gyro[3],rearEncoderLeftReset,rearEncoderRightReset}
      for iv=1,3 do
	 dataOut[13+iv-1] = rb[iv]
      end
      for iv=1,3 do
	 dataOut[16+iv-1] = accel[iv]
      end
      for iv=1,3 do
	 dataOut[19+iv-1] = gyro[iv]
      end
      
      if (cntSonar == cntActiveSonar) then
	 cntSonar = 0
	 if (numActiveSonar == 1) then
	    local result,distFront1,dtPoint,dtObjHandle,dtSurfNorm = simHandleProximitySensor(sonarFront)
	    distFront = distFront1
	    if not (result>0) then
	       distFront= distMax
	    end
	 end
	 if (numActiveSonar == 5) then
	    local result,distFrontLeft1,dtPoint,dtObjHandle,dtSurfNorm = simHandleProximitySensor(sonarFrontLeft)
	    distFrontLeft = distFrontLeft1
	    if not (result>0) then
	       distFrontLeft= distMax
	    end
	 end
	 if (numActiveSonar == 6) then
	    local result,distFrontRight1,dtPoint,dtObjHandle,dtSurfNorm = simHandleProximitySensor(sonarFrontRight)
	    distFrontRight = distFrontRight1
	    if not (result>0) then
	       distFrontRight= distMax
	    end
	 end
	 if (numActiveSonar == 2) then
	    local result,distRear1,dtPoint,dtObjHandle,dtSurfNorm = simHandleProximitySensor(sonarRear)
	    distRear = distRear1
	    if not (result>0) then
	       distRear = distMax
	    end
	 end
	 if (numActiveSonar == 3) then
	    local result,distLeft1,dtPoint,dtObjHandle,dtSurfNorm = simHandleProximitySensor(sonarLeft)
	    distLeft = distLeft1
	    if not (result>0) then
	       distLeft = distMax
	    end
	 end
	 if (numActiveSonar == 4) then
	    local result,distRight1,dtPoint,dtObjHandle,dtSurfNorm = simHandleProximitySensor(sonarRight)
	    distRight = distRight1
	    if not(result>0) then
	       distRight = distMax
	    end
	 end
	 numActiveSonar = numActiveSonar+1
	 if (numActiveSonar == 7) then
	    numActiveSonar = 1
	 end
      end
      cntSonar = cntSonar + 1
      
      --print ("Sonars : "..distFront..", "..distRear..", "..distLeft..", "..distRight)
      dataOut[2]=distFront
      dataOut[3]=distRear
      dataOut[4]=distLeft
      dataOut[5]=distRight
      dataOut[6]=distFrontLeft
      dataOut[7]=distFrontRight

      -- update encoders with relative orientation of the wheel (wrt joint axis)
      for iw=1,#wheelId do
	 handle=wheelId[iw]
	 --print ("wheel",iw,handle,lastRelativeOrientation[iw])
	 relativeOrientation=getWheelAngularPosition(handle)
	 currentOrientationTime=simGetSimulationTime()
	 --print (iw,relativeOrientation,lastRelativeOrientation[iw])
	 if iw == 1 or iw == 3 then  -- left wheels
	    wheelSpeed = leftWheelSpeed
	 end
	 if iw == 2 or iw == 4 then  -- right wheels
	    wheelSpeed = rightWheelSpeed
	 end
	 --print (iw,wheelCnt[iw])
	 if iw == 1 or iw == 2 then -- front wheels (count ++)
	    dCnt = deltaWheelAngularPosition(relativeOrientation,lastRelativeOrientation[iw],wheelSpeed)
	    --print (iw,"dcnt",dCnt)
	    wheelCnt[iw] = wheelCnt[iw]+dCnt
	 end
	 if iw == 3 or iw == 4 then -- rear wheels (count --)
	    dCnt = deltaWheelAngularPosition(relativeOrientation,lastRelativeOrientation[iw],wheelSpeed)
	    --print (iw,"dcnt",dCnt)
	    wheelCnt[iw] = wheelCnt[iw]-dCnt
	 end
	 --print (iw,wheelCnt[iw])
	 lastRelativeOrientation[iw]=relativeOrientation
	 lastOrientationTime[iw]=currentOrientationTime   
      end
      frontLeftEncoder = wheelCnt[1]
      frontRightEncoder = wheelCnt[2]
      rearLeftEncoder = wheelCnt[3]
      rearRightEncoder = wheelCnt[4]
      --print ("wheel cnt ",wheelCnt[1],wheelCnt[2],wheelCnt[3],wheelCnt[4])
      -- transformation to 16 bits integers is now done in python 
      dataOut[8]=frontLeftEncoder
      dataOut[9]=frontRightEncoder
      dataOut[10]=rearLeftEncoder
      dataOut[11]=rearRightEncoder

      -- update heading
      local handle = simGetObjectHandle("Dart")
      local angles = simGetObjectOrientation(handle,-1)
      for i=1,#angles do
	 angles[i] = angles[i]*180.0/math.pi
      end
      --print ("heading",angles[1],angles[2],angles[3])
      heading = -angles[3]   -- north along X > 0
      dataOut[12]= heading
            
      if not serverOn then
         print ("not connected")
         srv = assert(socket.bind('127.0.0.1',portNb))
         if (srv==nil) then
            print ("bad connect")
         else
            ip, port = srv:getsockname()
            print ("server ok at "..ip.." on port "..port)
	    simAddStatusbarMessage("server ok at "..ip.." on port "..port)
	    serverOn = true
            --srv:settimeout(connexionTimeout)
            print ("connexion granted !!! ")
         end
      end
      --print (serverOn)
      if serverOn then
         srv:settimeout(connexionTimeout)
         clt1 = srv:accept()
         if clt1 == nil then
            cntTimeout = cntTimeout + 1
             --print ("accept timeout")
             --serverOn = false
             --srv:close()
         else
            clt1:settimeout(connexionTimeout)
            dataIn = readSocketData(clt1)
            if dataIn ~= nil then
               --print (dataIn)
               targetCmd=simUnpackFloats(dataIn)
	       if targetCmd[5] ~= 0.0 or targetCmd[6] ~= 0.0 then
		  print ("received",targetCmd[1],targetCmd[2],targetCmd[3],targetCmd[4],targetCmd[5],targetCmd[6])
	       end
               local doLog = targetCmd[1]
               if doLog == 1.0 and not logOn then  
                  -- open new log file and start to log data
                  file = io.open(logFile,"w")
                  file:write("Hello!","\n")
                  file:close()
                  logOn = true
               end
               if doLog == 0.0 and logOn then
                  -- stop data log in file
                  logOn = false
               end
               leftWheelSpeed = defineLeftSpeed(targetCmd[2]) 
               rightWheelSpeed = defineRightSpeed(targetCmd[3])
	       speedCmdNew = targetCmd[4]
	       if speedCmdNew == 1 then
		  print ("set cmd speed",leftWheelSpeed,rightWheelSpeed)
		  simSetJointTargetVelocity(frontRightMotor,rightWheelSpeed)
		  simSetJointTargetVelocity(frontLeftMotor,leftWheelSpeed)
		  simSetJointTargetVelocity(rearRightMotor,rightWheelSpeed)
		  simSetJointTargetVelocity(rearLeftMotor,leftWheelSpeed)
               end
	       rearEncoderLeftResetRx = targetCmd[5]
	       rearEncoderRightResetRx = targetCmd[6]
	       if rearEncoderLeftResetRx == 1.0 then
		  wheelCnt[3] = 0
		  rearEncoderLeftReset = -1.0
	       else
		  rearEncoderLeftReset = 0.0
	       end
	       dataOut[22] = rearEncoderLeftReset
	       if rearEncoderRightResetRx == 1.0 then
		  wheelCnt[4] = 0
		  rearEncoderRightReset = -1.0
	       else
		  rearEncoderRightReset = 0.0
	       end
	       dataOut[23] = rearEncoderRightReset
	       --print ("sz",table.getn(dataOut))
               -- Pack the data as a string:srv:close()
               dataPacked=simPackFloats(dataOut)
               -- Send the data:
               writeSocketData(clt1,dataPacked)
               clt1:send(dataIn)
	       --print (string.len(dataPacked)," bytes")
	       --print ("dataOut sent ... ")
	       
	       --rearEncoderLeftReset = 0.0
	       --rearEncoderRightReset = 0.0     
            else
               print ("no data")
            end
            clt1:close()
         end
      end

      if cnt == cntDispl then
	 cnt = 0
	 local stOut = "Sonars F:"..distFront.." R:"..distRear.." L:"..distLeft.." R:"..distRight.." FL:"..distFrontLeft.." FR:"..distFrontRight
	 stOut = stOut..", Front Odos L:"..dataOut[8].." R:"..dataOut[9]
	 stOut = stOut..", Rear Odos L:"..dataOut[10].." R:"..dataOut[11]

	 stOut = stOut..", Head:"..heading..", Motors L:"..leftWheelSpeed.." R:"..rightWheelSpeed..", Reset L:"..rearEncoderLeftReset.." R:"..rearEncoderRightReset
	 stOut = string.format("Sonar Fr:%.2f Rr=%.2f Le:%.2f Ri:%.2f FL:%.2f FR:%.2f Front Odos L:%d R:%d Rear Odos L:%d R:%d Heading: %.2f Motors L:%d R:%d",distFront,distRear,distLeft,distRight,distFrontLeft,distFrontRight,dataOut[8],dataOut[9],dataOut[10],dataOut[11],heading,leftWheelSpeed,rightWheelSpeed) 
	 print (stOut)
	 simAddStatusbarMessage(stOut)
      end
      cnt = cnt+1

      -- Now don't waste time in this loop if the simulation time hasn't changed! This also synchronizes this thread with the main script
      simSwitchThread() -- This thread will resume just before the main script is called again
		
    end
end

-- Initialization:
simSetThreadSwitchTiming(5) -- We wanna manually switch for synchronization purpose (and also not to waste processing time!)

leftWheelSpeed = 0.0
rightWheelSpeed = 0.0
cvtCmdSpeed = 1./20. -- 1./7.8 before
  
-- get Main robot
dart = simGetObjectHandle("Dart")

-- get handles of sonar sensor
sonarFrontLeft = simGetObjectHandle("SonarFrontLeft")
sonarFrontRight = simGetObjectHandle("SonarFrontRight")
sonarLeft = simGetObjectHandle("SonarLeft")
sonarRight = simGetObjectHandle("SonarRight")
sonarFront = simGetObjectHandle("SonarFront")
sonarRear = simGetObjectHandle("SonarRear")
distMax = 99.9
distFront = distMax
distFrontLeft = distMax
distFrontRight = distMax
distRear = distMax
distLeft = distMax
distRight = distMax

frontRightMotor = simGetObjectHandle("MotorFR")
frontLeftMotor = simGetObjectHandle("MotorFL")
rearRightMotor = simGetObjectHandle("MotorRR")
rearLeftMotor = simGetObjectHandle("MotorRL")

-- initialize communication tube with accelerometers and gyroscopes
accelCommunicationTube=simTubeOpen(0,'accelerometerData'..simGetNameSuffix(nil),1) 
accel={0.0, 0.0, 0.0}
gyroCommunicationTube=simTubeOpen(0,'gyroData'..simGetNameSuffix(nil),1) -- put this in the initialization phase
gyro={0.0, 0.0, 0.0}

frontLeftEncoder=0.0
frontRightEncoder=0.0
rearLeftEncoder=65535.0
rearRightEncoder=65535.0
rearEncoderLeftReset = 0.0
rearEncoderRightReset = 0.0
--rearWheelPosLastLeft = simGetJointPosition(rearLeftMotor)
--rearWheelPosLastRight = simGetJointPosition(rearRightMotor)
--frontWheelPosLastLeft = simGetJointPosition(frontLeftMotor)
--frontWheelPosLastRight = simGetJointPosition(frontRightMotor)

heading=0.0

-- get orientation of Dart's body
lastRelativeOrientation={}
lastOrientationTime={}
wheelCnt={}
wheelId={}
wheels={"WheelFL","WheelFR","WheelRL","WheelRR"}
for i=1,#wheels do
    handle=simGetObjectHandle(wheels[i])
    wheelId[#wheelId+1]=handle
    lastRelativeOrientation[#lastRelativeOrientation+1]=getWheelAngularPosition(handle)
    lastOrientationTime[#lastOrientationTime+1]=simGetSimulationTime()
    wheelCnt[#wheelCnt+1]=0.0
end

cnt=0
cntDispl=1000
cntSonar=0
numActiveSonar=1
cntActiveSonar=5

logOn = false
logFile = "../dartv2b.log"

-- Socket Port number
portNb = 30100
serverOn = false
connexionTimeout = 0.005
cntTimeout = 0
socket=require("socket")
srv = nil
clt1 = nil

simAddStatusbarMessage('start virtual DartV2 at port '..portNb)

-- Execute the thread function:
res=false
err=" not launched delibarately "
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
   simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Clean-up:


