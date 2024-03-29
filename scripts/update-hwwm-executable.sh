#!/bin/bash

daemon=hwwm
daemon_pid=/run/$daemon.pid
log_file=/var/log/$daemon.log

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
echo "Replacing $daemon-reload and $daemon-restart in /usr/sbin with ones from /home/pi/$daemon/scripts/..."
cp /home/pi/$daemon/scripts/$daemon-reload /usr/sbin
cp /home/pi/$daemon/scripts/$daemon-restart /usr/sbin
cp /home/pi/$daemon/scripts/$daemon-stop /usr/sbin
cp /home/pi/$daemon/scripts/hwwm_backup-cfg /usr/sbin
chmod +x /usr/sbin/$daemon-reload
chmod +x /usr/sbin/$daemon-restart
chmod +x /usr/sbin/$daemon-stop
chmod +x /usr/sbin/hwwm_backup-cfg
sleep 3
echo "Starting $daemon again..."
/usr/sbin/$daemon
echo "Here is the log:"
tail -n 30 $log_file
