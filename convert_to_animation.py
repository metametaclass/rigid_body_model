#!/usr/bin/env python

import json
import sys

with open(sys.argv[1], encoding='utf-8') as fh:
   times = []
   position_track = []
   rotation_track = []
   max_time = 0.0
   for i in fh:
     record = i.split()
     time = float(record[0])
     max_time = max(max_time, time)
     times.append(time)

     #Xthree(east) = Yned (east)
     #Ythree(up) = -Zned (-down)
     #Zthree(south) = -Xned (-north)
     Xned = float(record[1])
     Yned = float(record[2])
     Zned = float(record[3])

     #position_track.append(Yned)
     #position_track.append(-Zned)
     #position_track.append(-Xned)

     position_track.append(Xned)
     position_track.append(Yned)
     position_track.append(Zned)

     QW = float(record[4])
     QX = float(record[5])
     QY = float(record[6])
     QZ = float(record[7])

     #rotation_track.append(QY)
     #rotation_track.append(-QZ)
     #rotation_track.append(-QX)
     #rotation_track.append(QW)

     rotation_track.append(QX)
     rotation_track.append(QY)
     rotation_track.append(QZ)
     rotation_track.append(QW)



   result=[{
     "name": "Action",
     "duration": max_time,
     "tracks":[
       {
         "name": ".position",
         "type": "vector",
         "times": times,
         "values": position_track

       },
       {
         "name": ".quaternion",
         "type": "quaternion",
         "times": times,
         "values": rotation_track
       }
     ]
   }]
   print(json.dumps(result, indent=4, ensure_ascii=True))
