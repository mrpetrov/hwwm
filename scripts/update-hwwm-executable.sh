#!/bin/bash

daemon=hwwm
daemon_pid=/run/$daemon.pid

echo "Trying to kill running daemon $daemon..."
if [ ! -e $daemon_pid ]
then
    killall -e $daemon
else
    kill `cat $daemon_pid`
    rm -f $daemon_pid
fi
sleep 1
echo "Replacing /usr/sbin/$daemon with the one from /home/pi/$daemon..."
cp /home/pi/$daemon/$daemon /usr/sbin
sleep 3
echo "Starting $daemon again..."
/usr/sbin/$daemon
