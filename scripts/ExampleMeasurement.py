import sys
import time
import os
# from ..API import oxapi

# path to .NET OXApi.Dll and python wrapper
# sys.path.append(r"../API")
sys.path.append(os.path.join(os.path.dirname(sys.path[0]),'OxSDK_V2_0_0','CSharp', 'API')) # add path to API folder

import oxapi


# creates a OX object
ox = oxapi.ox("192.168.0.250")

# establish a connection to the OXP
ox.Connect()

# login is required to access some sensor resources, e.g. the raw image.
ox.Login("admin", "")    # the default password is empty, but can be changed on the web interface

try:
    
    qualityId, config, alarm, digitalouts, encoder, timeStamp, rate, values = ox.GetMeasurement()
    print("### Measurement ###")
    print(qualityId)
    print(config)
    print(alarm)
    print(digitalouts)
    print(encoder)
    print(timeStamp)
    print(rate)
    print(values)

except:
    print(sys.exc_info()[0])
    print(sys.exc_info()[1])


# always close the connection
ox.Disconnect()



