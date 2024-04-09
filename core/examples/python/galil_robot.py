# Start Galil Controller

import os, sys, ctypes

# Set RTLD_GLOBAL flag for dynamic loading (on Linux)
try:
   flags = sys.getdlopenflags()
   sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)
except AttributeError, e:
    print 'Skipping dlopen flags, ', e

import cisstCommonPython as cisstCommon

# Set up cisst logging system to print errors, warnings, and verbose (but not debug)
cisstCommon.cmnLogger.SetMask(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskFunction(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.SetMaskDefaultLog(cisstCommon.CMN_LOG_ALLOW_VERBOSE)
cisstCommon.cmnLogger.AddChannelToStdOut(cisstCommon.CMN_LOG_ALLOW_ERRORS_AND_WARNINGS)

def log():
   os.system('tail cisstLog.txt')

import cisstMultiTaskPython as cisstMultiTask
import cisstParameterTypesPython as cisstParameterTypes
import numpy

LCM = cisstMultiTask.mtsManagerLocal.GetInstance()
print 'Creating Galil client'
GalilClient = cisstMultiTask.mtsComponentWithManagement('GalilClient')
LCM.AddComponent(GalilClient)
LCM.CreateAll()
LCM.StartAll()

Manager = GalilClient.GetManagerComponentServices()
print 'Loading sawGalilController'
if not Manager.Load('sawGalilController'):
    print 'Failed to load sawGalilController (see cisstLog.txt)'

print 'Creating Galil server (mtsGalilController)'
GalilServer = LCM.CreateComponentDynamically('mtsGalilController', 'GalilServer', '')
if GalilServer:
   print 'Component created'
   LCM.AddComponent(GalilServer)
   print 'Configuring Galil server.'
   configFile = raw_input('Enter config filename (JSON): ')
   GalilServer.Configure(configFile)
   GalilServer.Create()

   print 'Connecting Galil client to Galil server'
   # robot is the required interface
   robot = GalilClient.AddInterfaceRequiredAndConnect(('GalilServer', 'control'))

   print 'Starting Galil server'
   GalilServer.Start()

   print 'System ready. Type dir(robot) to see available commands.'
