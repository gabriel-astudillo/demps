#!/bin/bash

BASEDIR=$(readlink -f $0)
BASEDIR=$(dirname $BASEDIR)

RM_CMD="$(which rm) -f"
CP_CMD="$(which cp)"
CHMOD_CMD="$(which chmod)"
MKDIR_CMD="$(which mkdir)"

ulimit -c unlimited
rm -f core

DEMPS_BIN="./demps"

DEMPS_CONFIG="./iquique.config"
#DEMPS_CONFIG="./valpo.config"

#### CONFIGURACION ####
DEMPS_PATH=$BASEDIR/$DEMPS_BIN 

if [[ ! -e $DEMPS_PATH  ]]; then
	echo "Error: $DEMPS_PATH no existe"
	exit 1
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

TEST_CONFIG_OK=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."agents-path"' )
if [[  $? -ne 0  ]]; then
	echo "El archivo $DEMPS_CONFIG_PATH tiene errores."
	exit
fi


DEMPS_OPTS="-s $DEMPS_CONFIG_PATH $*"

RESULTS_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output.directory')
RESULTS_DIR_PATH=$BASEDIR/$RESULTS_DIR

AGENTS_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."agents-path"')
AGENTS_DIR_PATH=$RESULTS_DIR_PATH/$AGENTS_DIR

STATS_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."stats-path"')
STATS_DIR_PATH=$RESULTS_DIR_PATH/$STATS_DIR

if [[ ! -e $AGENTS_DIR_PATH ]]; then
	mkdir -p $AGENTS_DIR_PATH
fi

if [[ ! -e $STATS_DIR_PATH ]]; then
	mkdir -p $STATS_DIR_PATH
fi

#Copiar los archivos input/*geojosn a RESULTS_DIR
#(necesarios para la visualizacion)
INPUT_PATH=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.input.directory')

$MKDIR_CMD -p $RESULTS_DIR/input/
$CP_CMD $INPUT_PATH/*.geojson $RESULTS_DIR/input/

$CP_CMD input/animacion*.html $RESULTS_DIR/

$CHMOD_CMD -R +r $RESULTS_DIR/input


RESULTS_FILES="$AGENTS_DIR_PATH/* $STATS_DIR_PATH/*"

#FILESIM_OUT=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."agents-out"')
#CREATE_GIF=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."create-gif"')
THREADS=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.threads')



#### MAIN ####
printf "DEMPS_CONFIG_PATH=%s\n" "$DEMPS_CONFIG_PATH"

echo "Eliminando resultados anteriores..."
$RM_CMD $RESULTS_FILES

echo "Ejecutando DEMPS..."
#export OMP_NUM_THREADS=$THREADS
#export OMP_SCHEDULE='schedule(guided,8)'
$DEMPS_PATH $DEMPS_OPTS

