#!/usr/bin/env bash

USAGE="USAGE: ${0} <sd-disk> <ip-address>"
HELP="An SD card to do the things we want

sd-disk     :    A disk with the RPi boot image loaded onto it.
ip-address  :    The static IP address to assign to this Pi.
"

echo "${USAGE}"
echo "${HELP}"

DISK=${1}
if [ "$#" -ne 2 ]; then
    IP_ADDRESS="192.168.2.240"
    echo "WARNING: setting defualt ip address ${IP_ADDRESS}"
else
    IP_ADDRESS=${2}
fi 

GATEWAY_ADDRESS="192.168.2.1"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SD_VOLUME="/Volumes/boot"

sudo diskutil unmountDisk ${DISK}
sudo diskutil mountDisk ${DISK}

if [ ! -d "${SD_VOLUME}" ]; then
    echo "Boot volume ${SD_VOLUME} not found; exiting."
    exit -1
fi

# Enable ssh
sudo touch ${SD_VOLUME}/ssh

# Configure a static IP
export IP_ADDRESS
export GATEWAY_ADDRESS
perl -p -e 's/\$\{([^}]+)\}/defined $ENV{$1} ? $ENV{$1} : $&/eg' \
  < cmdline.txt.template \
  > ${SD_VOLUME}/cmdline.txt

# Eject the disk
sudo diskutil eject ${DISK}
