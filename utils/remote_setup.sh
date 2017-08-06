#!/usr/bin/env bash
set -e

DIR=/home/pi

sudo apt-get install git make gcc g++

cd ${DIR}
git checkout https://github.com/coup-de-foudre/FM_transmitter
cd FM_transmitter/
make packages && make
