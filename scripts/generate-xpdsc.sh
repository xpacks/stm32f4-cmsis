#! /bin/bash
set -euo pipefail
IFS=$'\n\t'

if [ ! -f xpack.json ]
then
	echo "Must be started in a package root folder."
	exit 1
fi

PDSC_FILE="Keil.STM32F4xx_DFP.pdsc"
${HOME}/node_modules/xcdl/bin/xcdl-js generate-xpdsc -i ${PDSC_FILE} -o xpdsc.json

