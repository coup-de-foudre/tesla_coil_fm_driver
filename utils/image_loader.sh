#!/usr/bin/env bash
set -e 

USAGE="USAGE: ${0} <image> <sd-disk>"
HELP="Load an image onto an SD card

image   :    The image (e.g., ~/Downloads/2017-07-05-raspbian-jessie-lite.img)
sd-disk :    The disk to flash (e.g., /dev/disk2, from 'diskutil list')

NOTE: Use Ctrl-T to check the status, Ctrl-C to abort.
"

echo "${USAGE}"
echo "${HELP}"

IMAGE="${1}"
DISK="${2}"

if sudo diskutil unmountDisk ${DISK}; then
    echo "Disk ${DISK} not mounted";
fi

# Copy image
sudo dd if=${IMAGE} of=${DISK} bs=2m conv=sync
sudo diskutil eject ${DISK}