from HIMUServer import HIMUServer

# example of listener implementation.
class SimplePrintListener:
    def __init__(self, serverInstance):
        self.__server = serverInstance

    def notify (self, sensorData):
		# Customize the notify method in order to elaborate data
		# sensorData contains String values (see HIMUServer.__extractSensorData())
        HIMUServer.printSensorsData(sensorData)
		# for a string-to-float conversion, try HIMUServer.strings2Floats()

myHIMUServer = HIMUServer()
#Creating listener and adding it to the server instance:
myListener = SimplePrintListener(myHIMUServer)
myHIMUServer.addListener(myListener)

#Change the timeout (in seconds) :
myHIMUServer.timeout = 2
#Launch acquisition via TCP on port 2055:
myHIMUServer.start("TCP", 2055)

