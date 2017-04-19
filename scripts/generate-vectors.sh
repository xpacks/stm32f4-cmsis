#! /bin/bash
set -euo pipefail
IFS=$'\n\t'

if [ ! -f xpack.json ]
then
	echo "Must be started in a package root folder."
	exit 1
fi

CMSIS_FOLDER="Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates"
SRC_FOLDER="${CMSIS_FOLDER}/arm"
DEST_FOLDER="${CMSIS_FOLDER}/gcc"

bash ../scripts.git/generate-vectors-from-arm-startup.sh "${SRC_FOLDER}" "${DEST_FOLDER}"
