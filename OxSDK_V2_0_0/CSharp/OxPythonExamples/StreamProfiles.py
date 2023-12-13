import sys
import time

# path to .NET OXApi.Dll and python wrapper
sys.path.append(r"..\API")

import oxapi

# creates a Ox object
ox = oxapi.ox("192.168.0.250")

stream = ox.CreateStream()

stream.Start()

stream.ClearMeasurementQueue()

for i in range(0, 100):
    time.sleep(0.01)
    if stream.GetProfileCount() > 0:
        blockId, confiMode, ntpSync, valid, alarm, quality, timestamp, length, encoder, x, z, i = stream.ReadProfile()
        print(timestamp)

stream.Stop()






