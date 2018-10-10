#!/bin/bash

#### CONFIGURACION ####
DEMPS_PATH=./demps 

DEMPS_CONFIG_FILE="demps-iquique.config"
#DEMPS_CONFIG_FILE="demps-curaumaPUCV.config"
#DEMPS_CONFIG_FILE="demps-vdm.config"

# command-line JSON processor
# https://stedolan.github.io/jq/
JQ_PATH=/usr/bin/jq

if [[ ! -e $JQ_PATH ]]; then
	echo "JQ no está instalado en $JQ_PATH."
	echo "https://stedolan.github.io/jq/"
	exit
fi

DEMPS_OPTS="-s $DEMPS_CONFIG_FILE"
RESULTS_DIR=$(cat $DEMPS_CONFIG_FILE | $JQ_PATH -r '.output."filesim-path"')
RESULTS_FILES="$RESULTS_DIR/*"

RM_CMD="$(which rm) -f"

#### MAIN ####
printf "DEMPS_CONFIG_FILE=%s\n" "$DEMPS_CONFIG_FILE"

echo "Eliminando resultados anteriores..."
$RM_CMD $RESULTS_FILES

echo "Ejecutando DEMPS..."
time $DEMPS_PATH $DEMPS_OPTS
