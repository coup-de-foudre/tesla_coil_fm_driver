#!/usr/bin/env bash

# Meant to be run remotely using `ssh -A pi@<address> "bash -s" < setup_pi.sh`
set -e

HOME_DIR=/home/pi
BASE_DIR=${HOME_DIR}
CODE_DIR=${BASE_DIR}/FM_transmitter

AUTHORIZED_KEYS="
ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAACAQDAIIYSTOoebOomqiv6Mkv3iXK02ftCtyBlA789wqvyXhGkSEfXWqXfiwucegJm4277cs/M7fBzsHs/OANr21adRfUd3OYpsCfne6Tj8InNj/4EoZkcEfNxyDUB4tNT039LliVyAAmrG7U2MPsHS51JLrz5h+Dxwj1kqH4vAmBCRAISflf2kRzaV8xXoBRaXB8XJYuRfoGBMJlyUwXyjrErt1nuox7shSfRUNDsbdU0faNilcx0cx/yWYNAWKNYQ64TrMxjiAW70daVaqZxTNJx3bDO5kW6XZ6d9UehtQGDrHobUIY9NpwS538t0cF7RwwWzTNGdWYNAHao6iYQHR4Y46SClowOiFCTSACQmQHV3ABEFSHVNZxCmvDv/Trwr5Vpj9ZjhDMvhYT7Cm1BKWQvNMzeR66zyVu+4jI7l8ELdBAoBT7ztwds0FIM2XHhxhJuQKfeT7T7p6D95ZGPzKCvEXhlX8Gl+IcdAfYNg3gjjYrW+aCqTltIhe1u+a7HlJQOTEmWgI8vBsE29o4CW98gGmFHhcRHTOehHOMeo/W0QjcoP5IZjxrqsodFjVsjkpgRZUo+DV8+3jDURgAWxRKu26sQtIGGQAKWtx8Z0Tey6v1lHHSCvNSVqCZ6xLB7Fxix2wdlns12W1AmnuhQ7cHQIcWpTT3ydy0A1ZX+zepRpw==

ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQDUo+gdMTLPTduwJgEfPOWl3vgs6qD/WJwKNp2w9kLZMpjZ/O+JrHpwuE3NTOTv8Gd4xvdRGfuzWTZ5Y+ugEQ/W1ZHeoYYVBUYCiQdBOVYmk/FH+76VP+KjD3PvmZKLzVGRgUzamgxY43RyTAnmUftnK15BFlyurM/BpWMIqo8MjoPy5IkC0GZqJkVQbaSBhHbPQeFVwIQMvv3LrT9+sXwiiKnJikovVTDm1xQ+SaEAgPHZLhMhVZXVw/r8vrxuKZkW02bvMeKg+4lF3RWokh0jwp2Q6lufa/5SSrhQ/1AArhAQgg2qNX2KckL5Qq0WUGEDoCpuakhjSedrttJdGqCx
"

mkdir -p ${HOME_DIR}/.ssh && touch ${HOME_DIR}/.ssh/authorized_keys
echo "${AUTHORIZED_KEYS}" >> ${HOME_DIR}/.ssh/authorized_keys

# TODO: Change password?


# Update apt and install minimum requirements for this script
sudo apt-get update && sudo apt-get install -y \
    git \
    make \
    gcc \
    g++

if [ -d ${CODE_DIR} ]; then
   echo "Already checked out code?"
   cd ${CODE_DIR} 
   git checkout master && git pull
else
   echo "Cloning repo"
   git clone git@github.com:coup-de-foudre/FM_transmitter.git ${CODE_DIR}
   #git clone https://github.com/coup-de-foudre/FM_transmitter
fi

cd ${CODE_DIR}
make packages && make && make daemon
