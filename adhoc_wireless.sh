#!/bin/bash

COUNT=`expr $1 - 100`
echo $COUNT

echo "Starting up adhoc wireless"
echo "Make sure to use sudo when executing"
echo ""

stop network-manager
ifconfig wlan0 down
sleep 2
iwconfig wlan0 mode ad-hoc
iwconfig wlan0 channel 1
iwconfig wlan0 key aaaaa11111
iwconfig wlan0 essid RPi
ifconfig wlan0 10.0.0.$COUNT netmask 255.255.255.0 up

ping 10.0.0.$1
