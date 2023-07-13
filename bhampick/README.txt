Node used :
- pick : main
- broadcast : broadcast tf
- testclient : node for publishing service (rosservice call /boxpose doesn't work with negative values)

Rosrun :
- rosrun bhampick pick
- rosrun bhampick broadcast
- rosrun bhampick testclient

Roslaunch :
- roslaunch bhampick pick
- Change /boxpose (x,y,z,roll,pitch,yaw) in testclient.cpp
