from FPS import *
from OSC import *
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from getch import *

import json
import math
import pprint

runningMainLoop = True
dynamixelSettings = None
motorDictionary = {}
connectInfos = {}
OSCInfo = None

def Clamp(value, min, max):
    return max(min(value, max), min)

def GetXL320ValueFromRad(radian):
    digree = -(radian * 180.0 / math.pi)
    x = digree / 150.0
    return Clamp(x, -1.0, 1.0) * 512.0 + 512.0

def GetXM430ValueFromRad(radian):
    return (-radian + math.pi) * 4096.0 / (math.pi * 2)

def GetXL330ValueFromRad(radian):
    return (-radian + math.pi) * 4096.0 / (math.pi * 2)

def GetRadianFromXL320Value(value):
    return (150.0 * math.pi / 180.0) * (1.0 - value / 512.0)

def GetRadianFromXM430Value(value):
    return math.pi * (1.0 - (2.0 * value / 4096.0))

def GetRadianFromXL330Value(value):
    return math.pi * (1.0 - (2.0 * value / 4096.0))

def loadJson(fileName):
    return json.load(open(fileName, 'r'))

def printJson(jsonData):
    print(json.dumps(jsonData, indent=2))

def loopCheckFunction():    
    return runningMainLoop



dynamixelValueRadianToMotorConversionTable = {
    "Dynamixel.XM430": GetXM430ValueFromRad, 
    "Dynamixel.XL320": GetXL320ValueFromRad,  
    "Dynamixel.XL330": GetXL330ValueFromRad}

dynamixelValueMotorToRadianConversionTable = {
"Dynamixel.XM430": GetRadianFromXM430Value, 
"Dynamixel.XL320": GetRadianFromXL320Value,  
"Dynamixel.XL330": GetRadianFromXL330Value}


def idleFunction():
    pass

def sendFunction():
    sendStockedValue()

def readFunction():
    readMotorInformations()

loopStateFunctions = {
    "Idle" : idleFunction,
    "Send" : sendFunction,
    "Read" : readFunction,
}

currentState = "Idle"

class ConnectInfo:
    def __init__(self):
        pass

class OSCData:
    def __init__(self):
        pass


def createMotorDictionary(motorSettings, connectInfos):
    motorDictionary = {}
    for motorInfo in motorSettings["MotorInfos"]:
        motor = {}
        motor["ID"] = int(motorInfo["ID"])
        motor["Type"] = motorInfo["Type"]
        motorDictionary[motorInfo["Name"]] = motor
        
    for connectInfoKey in connectInfos:        
            for motorName in connectInfos[connectInfoKey].motorNames:
                motorDictionary[motorName]["PortName"] = connectInfoKey

    return motorDictionary


def createPortInformation(motorSettings):
    connectInfos = {}

    for tConnectInfo in motorSettings["MotorConnectInformations"]:

        connectInfo = ConnectInfo()
        connectInfo.servoPortName = tConnectInfo["ServoPortName"]
        connectInfo.portHandler = PortHandler(tConnectInfo["ServoPortName"])
        connectInfo.packetHandler = PacketHandler(2.0)        
        connectInfo.groupBulkWrite = GroupBulkWrite(connectInfo.portHandler, connectInfo.packetHandler)
        connectInfo.groupBulkRead = GroupBulkRead(connectInfo.portHandler, connectInfo.packetHandler)
        connectInfo.label = tConnectInfo["Label"]
        connectInfo.baudRate = tConnectInfo["BaudRate"]
        connectInfo.motorNames = tConnectInfo["MotorNames"]

        connectInfos[tConnectInfo["Label"]] = connectInfo
    
    print(connectInfos)
    return connectInfos


def connectPorts(connectInfos):

    for connectInfoKey in connectInfos:
        connectInfo = connectInfos[connectInfoKey]

        if connectInfo.portHandler.openPort():
            print("Succeeded to open the port: " +
                  connectInfo.servoPortName )
        else:
            print("Failed to open the port: " + connectInfo.servoPortName )
            print("Press any key to terminate...")
            getch()
            quit()

            # Set port baudrate
        if connectInfo.portHandler.setBaudRate(int(connectInfo.baudRate)):
            print("Succeeded to change the baudrate: " + connectInfo.baudRate)
        else:
            print("Failed to change the baudrate: " + connectInfo.baudRate)
            print("Press any key to terminate...")
            getch()
            quit()



def closeSerialPorts(connectInfos):
    for connectInfoKey in connectInfos:
        connectInfos[connectInfoKey].groupBulkRead.clearParam()
        connectInfos[connectInfoKey].portHandler.closePort()

def to4ByteArray(value):
    valueInt = int(value)
    return [DXL_LOBYTE(DXL_LOWORD(valueInt)), DXL_HIBYTE(DXL_LOWORD(valueInt)), DXL_LOBYTE(DXL_HIWORD(valueInt)), DXL_HIBYTE(DXL_HIWORD(valueInt))]

def to2ByteArray(value):
    byteArrayData = int(value).to_bytes(2,'little')
    return [byteArrayData[0], byteArrayData[1]]

def sendStockedValue():
    for connectInfoKey in connectInfos:
        groupBulkWrite = connectInfos[connectInfoKey].groupBulkWrite

        if len(groupBulkWrite.data_list) == 0:
            return
        
        dxl_comm_result = groupBulkWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        groupBulkWrite.clearParam()
        #print("---")

def addWriteParam(id, address, length, param):
    dxl_addparam_result = groupBulkWrite.addParam(id, address, length, param)

def addParameter(motorName, commandName, data):
    id = motorDictionary[motorName]["ID"]
    motorType = motorDictionary[motorName]["Type"]
    address = dynamixelSettings[motorType][commandName]["Address"]
    length = dynamixelSettings[motorType][commandName]["Length"]
    groupBulkWrite = connectInfos[motorDictionary[motorName]["PortName"]].groupBulkWrite

    return groupBulkWrite.addParam(id, address, length, data)

def changeMotorPosition(address, *value):
    name = value[0]
    angleRad = value[1]

    commandName = "GoalPosition"  
    motorType = motorDictionary[name]["Type"]
    angleValue = dynamixelValueRadianToMotorConversionTable[motorType](angleRad)
    data = to4ByteArray(angleValue)

    result = addParameter(name, commandName, data)#groupBulkWrite.addParam(id, address, length, data)

    if result != True:
        print("[ID:%03d] groupBulkWrite addparam failed" % id)
    else:
        pass
        #print("[Motor: {0}] ~ {1}".format(value[0], value[1]))

def enableAllTorques(flag):
    for name in motorDictionary:
        commandName = "TorqueEnable"
        data = [flag]        

        result = addParameter(name, commandName, data)#groupBulkWrite.addParam(id, address, length, data)

        if result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % id)
        else:
            pass
            #print("name: " + format(name) + ",id: " + format(id) + ",address: " + format(address) + ",length: " + format(length) + ",flag: " + format(flag))
    
    sendStockedValue()

def changeAllPValue(value):
    for name in motorDictionary:
        commandName = "PositionPGain"
        data = to2ByteArray(value)        

        result = addParameter(name, commandName, data)#groupBulkWrite.addParam(id, address, length, data)

        if result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % id)
        else:
            pass
            #print("name: " + format(name) + ",id: " + format(id) + ",address: " + format(address) + ",length: " + format(length) + ",flag: " + format(flag))
    
    sendStockedValue()


def shutdownCommand(address, *value):
    global runningMainLoop
    runningMainLoop = False

def enableAllTorquesCommand(address, *value):
    enableAllTorques(1)
    #changeAllPValue(200)

def disableAllTorquesCommand(address, *value):
    enableAllTorques(0)

def toSendStateCommand(address, *value):
    global currentState
    currentState = "Send"

def toIdleStateCommand(address, *value):
    global currentState
    currentState = "Idle"

def toReadStateCommand(address, *value):
    global currentState
    currentState = "Read"

def setReadInformations():
    
    for name in motorDictionary:
        id = motorDictionary[name]["ID"]
        motorType = motorDictionary[name]["Type"]
        address = dynamixelSettings[motorType]["PresentPosition"]["Address"]
        length = dynamixelSettings[motorType]["PresentPosition"]["Length"]
        groupBulkRead = connectInfos[motorDictionary[name]["PortName"]].groupBulkRead
        #print("name: " + format(name) + ",id: " + format(id) + ",address: " + format(address) + ",length: " + format(length))
        result = groupBulkRead.addParam(id, address, length)

        if result != True:
            print("[ID:%03d] groupBulkRead addparam failed" % id)
            quit()

def readMotorInformations():

    for connectInfoKey in connectInfos:
        connectInfo = connectInfos[connectInfoKey]
        result = connectInfo.groupBulkRead.txRxPacket()

        if result != COMM_SUCCESS:
            print("%s" % connectInfo.packetHandler.getTxRxResult(result))


    for name in motorDictionary:
        id = motorDictionary[name]["ID"]
        motorType = motorDictionary[name]["Type"]
        address = dynamixelSettings[motorType]["PresentPosition"]["Address"]
        length = dynamixelSettings[motorType]["PresentPosition"]["Length"]
        groupBulkRead = connectInfos[motorDictionary[name]["PortName"]].groupBulkRead
        #print("name: " + format(name) + ",id: " + format(id) + ",address: " + format(address) + ",length: " + format(length))

        result = groupBulkRead.isAvailable(id, address, length)

        if result != True:
            print("[ID:%03d] groupBulkRead getdata failed" % id)
        else:
            value = groupBulkRead.getData(id, address, length)
            motorDictionary[name]["PresentPosition"] = value
            #print("[ID:%03d] Present Position : %d" % (id, value))
    
    for name in motorDictionary:
        motorType = motorDictionary[name]["Type"]
        angleInt = int(motorDictionary[name]["PresentPosition"])
        angleRad = dynamixelValueMotorToRadianConversionTable[motorType](angleInt)
        OSCInfo.client.send_message("/" + name, angleRad)
        
def loopFunction():
    loopStateFunctions[currentState]()

async def OSCMainLoop(duration):
    try:
        await scheduler(duration, loopFunction, loopCheckFunction, False)
    except KeyboardInterrupt:
        print('!!FINISH!!')

def createDispatcher():
    dispatcher = Dispatcher()
    dispatcher.map("/motor", changeMotorPosition)
    dispatcher.map("/shutdown", shutdownCommand)
    dispatcher.map("/torqueOn", enableAllTorquesCommand)
    dispatcher.map("/torqueOff", disableAllTorquesCommand)
    dispatcher.map("/idle", toIdleStateCommand)
    dispatcher.map("/send", toSendStateCommand)
    dispatcher.map("/read", toReadStateCommand)
    return dispatcher

async def OSCInit_Main(oscSettings):
    ip = oscSettings["OSCSendHost"]
    receivePort = oscSettings["OSCReceivePortNum"]
    sendPort = oscSettings["OSCSendPortNum"]

    global OSCInfo
    OSCInfo = OSCData()
    OSCInfo.dispatcher = createDispatcher()    
    OSCInfo.server = AsyncIOOSCUDPServer((ip, receivePort), OSCInfo.dispatcher, asyncio.get_event_loop())
    OSCInfo.client = SimpleUDPClient(ip, sendPort)

    transport, protocol = await OSCInfo.server.create_serve_endpoint() 

    print("Succeeded to open OSC port")   
    print("OSC Init start")

    await OSCMainLoop(0.02)

    transport.close()  # Clean up serve endpoint


def connectOSC(oscSettings):
    print("connectOSC")
    asyncio.run(OSCInit_Main(oscSettings))


def main():
    global dynamixelSettings
    global motorDictionary
    global connectInfos

    #load dynamixel definition
    dynamixelSettings = loadJson("DynamixelDefinitions.json")
    printJson(dynamixelSettings)

    #load motor settings
    motorSettings = loadJson("../settings/MotorSettings.json")
    printJson(motorSettings)

    #load osc settings
    oscSettings = loadJson("../settings/OSCSettings.json")
    printJson(oscSettings)

    #create connectInfors
    connectInfos = createPortInformation(motorSettings)
    pprint.pprint(connectInfos)

    #create motor dictionary
    motorDictionary = createMotorDictionary(motorSettings, connectInfos)
    pprint.pprint(motorDictionary)

    #set read informations
    setReadInformations()

    #connect ports
    connectPorts(connectInfos)

    #connect osc
    connectOSC(oscSettings)

    #close ports
    closeSerialPorts(connectInfos)

if __name__ == "__main__":
    main()
