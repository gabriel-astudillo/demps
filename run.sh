#!/bin/bash

BASEDIR=$(readlink -f $0)
BASEDIR=$(dirname $BASEDIR)

ulimit -c unlimited
rm -f core

DEMPS_BIN="./demps"
MAKE_MAPS_SCRIPT="./scripts/visualizador-offline/run-make_maps.sh"

#### CONFIGURACION ####
DEMPS_PATH=$BASEDIR/$DEMPS_BIN 

if [[ ! -e $DEMPS_PATH  ]]; then
	echo "Eror: $DEMPS_PATH no existe"
	exit 1
fi

MAKE_MAPS_SCRIPT_PATH=$BASEDIR/$MAKE_MAPS_SCRIPT

if [[ ! -e $MAKE_MAPS_SCRIPT_PATH  ]]; then
	echo "Eror: Script $MAKE_MAPS_SCRIPT_PATH no existe. DISABILITADO."
	MAKE_MAPS_DISABLE=1
else
	MAKE_MAPS_DISABLE=0
fi

DEMPS_CONFIG_FILE="$BASEDIR/demps-iquique.config"
#DEMPS_CONFIG_FILE="$BASEDIR/demps-curaumaPUCV.config"
#DEMPS_CONFIG_FILE="$BASEDIR/demps-vdm.config"

if [[ ! -e $DEMPS_CONFIG_FILE ]]; then
	echo "El archivo de configuración $DEMPS_CONFIG_FILE no exite."
	exit
fi

# command-line JSON processor
# https://stedolan.github.io/jq/
JQ_PATH=$(which jq)

if [[ ! -e $JQ_PATH ]]; then
	echo "JQ no está instalado en $PATH."
	echo "https://stedolan.github.io/jq/"
	exit
fi

DEMPS_OPTS="-s $DEMPS_CONFIG_FILE"
RESULTS_DIR=$(cat $DEMPS_CONFIG_FILE | $JQ_PATH -r '.output."filesim-path"')
RESULTS_DIR_PATH=$BASEDIR/$RESULTS_DIR

STATS_DIR=$(cat $DEMPS_CONFIG_FILE | $JQ_PATH -r '.output."stats-path"')
STATS_DIR_PATH=$BASEDIR/$STATS_DIR

if [[ ! -e $RESULTS_DIR_PATH ]]; then
	mkdir -p $RESULTS_DIR_PATH
fi

if [[ ! -e $STATS_DIR_PATH ]]; then
	mkdir -p $STATS_DIR_PATH
fi

RESULTS_FILES="$RESULTS_DIR_PATH/* $STATS_DIR_PATH/*"

FILESIM_OUT=$(cat $DEMPS_CONFIG_FILE | $JQ_PATH -r '.output."filesim-out"')
CREATE_GIF=$(cat $DEMPS_CONFIG_FILE | $JQ_PATH -r '.output."create-gif"')
THREADS=$(cat $DEMPS_CONFIG_FILE | $JQ_PATH -r '.threads')

RM_CMD="$(which rm) -f"

#### MAIN ####
printf "DEMPS_CONFIG_FILE=%s\n" "$DEMPS_CONFIG_FILE"

echo "Eliminando resultados anteriores..."
$RM_CMD $RESULTS_FILES

echo "Ejecutando DEMPS..."
export OMP_NUM_THREADS=$THREADS
#export OMP_SCHEDULE='schedule(guided,8)'
$DEMPS_PATH $DEMPS_OPTS

#SI FILESIM_OUT==true AND CREATE_GIF==true ==> crear gif animado
# $BASEDIR/scripts/visualizador-offline/run-make_maps.sh

if [[ $? -eq 0 && $FILESIM_OUT == "true" &&  $CREATE_GIF == "true" && ($MAKE_MAPS_DISABLE -eq 0) ]]; then
	echo "Creando gif animado de la simulación..."
	$MAKE_MAPS_SCRIPT_PATH $RESULTS_DIR_PATH
fi