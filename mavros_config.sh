#!/bin/bash
sleep 15
rosrun mavros mavparam set SYSID_MYGCS 1
rosrun mavros mavsys rate --all 10
sleep 180
rosrun mavros mavsafety arm

