#!/bin/bash

SUDO=""

if [[ $(id -u) == "0" ]]; then
    SUDO="sudo"
fi

cd /usr/src
${SUDO} git clone https://github.com/shikishima-TasakiLab/new-lg4ff.git
${SUDO} dkms install /usr/src/new-lg4ff
