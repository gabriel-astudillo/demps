#!/bin/bash

BASEDIR=$(readlink -f $0)
BASEDIR=$(dirname $BASEDIR)

ulimit -c unlimited
rm -f core

DEMPS_BIN="./demps"
DEMPS_CONFIG="./iquique.config"
MAKE_MAPS_SCRIPT="./scripts/visualizador-offline/run-make_maps.sh"

#### CONFIGURACION ####
DEMPS_PATH=$BASEDIR/$DEMPS_BIN 

if [[ ! -e $DEMPS_PATH  ]]; then
	echo "Error: $DEMPS_PATH no existe"
	exit 1
fi

MAKE_MAPS_SCRIPT_PATH=$BASEDIR/$MAKE_MAPS_SCRIPT

if [[ ! -e $MAKE_MAPS_SCRIPT_PATH  ]]; then
	echo "Error: Script $MAKE_MAPS_SCRIPT_PATH no existe. DISABILITADO."
	MAKE_MAPS_DISABLE=1
else
	MAKE_MAPS_DISABLE=0
fi

DEMPS_CONFIG_PATH="$BASEDIR/$DEMPS_CONFIG"

if [[ ! -e $DEMPS_CONFIG_PATH ]]; then
	echo "El archivo de configuración $DEMPS_CONFIG_PATH no exite."
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

TEST_CONFIG_OK=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."filesim-path"' )
if [[  $? -ne 0  ]]; then
	echo "El archivo $DEMPS_CONFIG_PATH tiene errores."
	exit
fi


DEMPS_OPTS="-s $DEMPS_CONFIG_PATH $*"
RESULTS_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."filesim-path"')
RESULTS_DIR_PATH=$BASEDIR/$RESULTS_DIR

STATS_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."stats-path"')
STATS_DIR_PATH=$BASEDIR/$STATS_DIR

if [[ ! -e $RESULTS_DIR_PATH ]]; then
	mkdir -p $RESULTS_DIR_PATH
fi

if [[ ! -e $STATS_DIR_PATH ]]; then
	mkdir -p $STATS_DIR_PATH
fi

RESULTS_FILES="$RESULTS_DIR_PATH/* $STATS_DIR_PATH/*"

FILESIM_OUT=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."filesim-out"')
CREATE_GIF=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."create-gif"')
THREADS=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.threads')


RM_CMD="$(which rm) -f"

#### MAIN ####
printf "DEMPS_CONFIG_PATH=%s\n" "$DEMPS_CONFIG_PATH"

echo "Eliminando resultados anteriores..."
$RM_CMD $RESULTS_FILES

echo "Ejecutando DEMPS..."
#export OMP_NUM_THREADS=$THREADS
#export OMP_SCHEDULE='schedule(guided,8)'
$DEMPS_PATH $DEMPS_OPTS

#SI FILESIM_OUT==true AND CREATE_GIF==true ==> crear gif animado
# $BASEDIR/scripts/visualizador-offline/run-make_maps.sh

if [[ $? -eq 0 && $FILESIM_OUT == "true" &&  $CREATE_GIF == "true" && ($MAKE_MAPS_DISABLE -eq 0) ]]; then
	echo "Creando gif animado de la simulación..."
	echo $MAKE_MAPS_SCRIPT_PATH $RESULTS_DIR_PATH
	$MAKE_MAPS_SCRIPT_PATH $RESULTS_DIR_PATH
fi
