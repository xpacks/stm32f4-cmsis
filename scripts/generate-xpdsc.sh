#! /bin/bash
set -euo pipefail
IFS=$'\n\t'

if [ ! -f xpack.json ]
then
	echo "Must be started in a package root folder."
	exit 1
fi

PDSC_FILE="Keil.STM32F4xx_DFP.pdsc"
xcdl pdsc-convert --file ${PDSC_FILE} --output xpdsc.json

