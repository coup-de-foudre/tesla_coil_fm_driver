#!/usr/bin/env bash

USAGE="USAGE: ${0} <sd-disk> <ip-address>"

HELP="Configure SD card set up with SSH and a static IP

sd-disk     :    A disk with the RPi boot image loaded onto it.
ip-address  :    The static IP address to assign to this Pi, e.g., 192.168.2.240

"

echo "${USAGE}"
echo "${HELP}"

if [ "$#" -ne 2 ]; then
    echo "ERROR: Missing arguments"
    exit -1
fi
DISK=${1}
IP_ADDRESS=${2}

FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TEMPLATE_DIR=${FILE_DIR}

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
CMDLINE_APPEND="ip=${IP_ADDRESS}::${GATEWAY_ADDRESS}:255.255.255.0:rpi:eth0:off"
CMDLINE_FILE=${SD_VOLUME}/cmdline.txt

if grep -q "ip="  "${CMDLINE_FILE}"; then
    echo "IP address already configured in ${CMDLINE_FILE}"
    exit -1;
else
    CMDLINE="$(cat ${CMDLINE_FILE} | tr -d '\n')"
    echo "${CMDLINE} ${CMDLINE_APPEND}" > ${CMDLINE_FILE}
fi
