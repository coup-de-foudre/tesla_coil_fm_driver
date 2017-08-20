#!/usr/bin/env bash
set -e 

USAGE="USAGE: ${0} <image> <sd-disk> <ip-address>"
HELP="Load an image onto an SD card

image   :    The image (e.g., ~/Downloads/2017-07-05-raspbian-jessie-lite.img)
sd-disk :    The disk to flash (e.g., /dev/disk2, from 'diskutil list')
ip-address  :    The static IP address to assign to this Pi, e.g., 192.168.2.240

NOTE: Use Ctrl-T to check the status, Ctrl-C to abort.
"

echo "${USAGE}"
echo "${HELP}"

if [ "$#" -ne 3 ]; then
    echo "ERROR: Missing arguments"
    exit -1
fi

IMAGE=${1}
DISK=${2}
IP_ADDRESS=${3}

FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SD_VOLUME="/Volumes/boot"

if sudo diskutil unmountDisk ${DISK}; then
    echo "Disk ${DISK} not mounted";
fi

# Copy image
sudo dd if=${IMAGE} of=${DISK} bs=2m conv=sync

sudo diskutil unmountDisk ${DISK}
sudo diskutil mountDisk ${DISK}

${FILE_DIR}/configure_sd_card.sh ${DISK} ${IP_ADDRESS} ${SD_VOLUME}


# Eject the disk
sudo diskutil eject ${DISK}

read -p "
   Transfer SD card to Pi. When it's booted, press enter to continue setup process.
"

ssh -A pi@${IP_ADDRESS} "bash -s" < ${FILE_DIR}/setup_pi.sh  
