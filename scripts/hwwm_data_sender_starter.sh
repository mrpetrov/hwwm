#!/bin/bash
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

# Print the IP address
_IP=$(hostname -I) || true
if [ "$_IP" ]; then
  printf "My IP address is %s\n" "$_IP"
fi

################################
#
# Custom part from hwwm package to complement it by starting another
# custom script tasked with exporting solard data to emoncms from the
# openenergymonitor.org package
#

# wait 15 seconds
sleep 15

# start main daemon
/usr/sbin/hwwm

# wait 8 seconds
sleep 8

# start emoncms sender
/etc/rc.hwwm_sender >>/run/shm/hwwm_sender_log &

# start hwwm Home Assistant interfacer script
/etc/rc.hwwm_ha_interfacer >>/run/shm/hwwm_ha_interfacer_log &

exit 0

#EOF
