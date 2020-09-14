#!/bin/bash

BASEDIR=$(readlink -f $0)
BASEDIR=$(dirname $BASEDIR)

RM_CMD="$(which rm) "
CP_CMD="$(which cp)"
CHMOD_CMD="$(which chmod)"
MKDIR_CMD="$(which mkdir)"

ulimit -c unlimited
rm -f core

DEMPS_BIN="./demps"

EXPERIMENT_NUMBER=-1
while getopts "c:o:e:" opt; do
  case ${opt} in
    o ) 
		RESULTS_DIR_OPT=$OPTARG
		;;
	e )
		EXPERIMENT_NUMBER=$OPTARG
		;;
	c )
		DEMPS_CONFIG=$OPTARG
		;;
	\? )
      ;;
  esac
done
shift $((OPTIND -1))

#### CONFIGURACION ####
DEMPS_PATH=$BASEDIR/$DEMPS_BIN 

if [[ ! -f $DEMPS_PATH  ]]; then
	echo "Error: $DEMPS_PATH no existe"
	exit 1
fi

DEMPS_CONFIG_PATH="$BASEDIR/$DEMPS_CONFIG"

if [[ ! -f $DEMPS_CONFIG_PATH ]]; then
	echo "El archivo de configuración $DEMPS_CONFIG_PATH no exite."
	echo "USO: $0 -c config_file.json"
	exit 1
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
	exit 1
fi


RESULTS_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output.directory')

#Verificar si hay que reemplazar el valor de $RESULTS_DIR
#del archivo de configuración con $RESULTS_DIR_OPT (parámetro -o)
if [[ -z $RESULTS_DIR_OPT ]]; then
	RESULTS_DIR_PATH=$BASEDIR/$RESULTS_DIR
else
	RESULTS_DIR_PATH=$BASEDIR/$RESULTS_DIR_OPT
	RESULTS_DIR=$RESULTS_DIR_OPT
fi
#RESULTS_DIR_PATH=$BASEDIR/$RESULTS_DIR


echo "Eliminando resultados anteriores... $RESULTS_DIR_PATH"
$RM_CMD -rf $RESULTS_DIR_PATH


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

$MKDIR_CMD -p $RESULTS_DIR_PATH/input/
echo $CP_CMD $INPUT_PATH/*.geojson $RESULTS_DIR_PATH/input/
$CP_CMD $INPUT_PATH/*.geojson $RESULTS_DIR_PATH/input/

$CP_CMD input/animacion*.html $RESULTS_DIR_PATH/

$CHMOD_CMD -R +r $RESULTS_DIR_PATH/input


RESULTS_FILES="$AGENTS_DIR_PATH/* $STATS_DIR_PATH/*"

#THREADS=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.threads')



#### MAIN ####
printf "DEMPS_CONFIG_PATH=%s\n" "$DEMPS_CONFIG_PATH"



echo "Ejecutando DEMPS..."
#export OMP_NUM_THREADS=$THREADS
#export OMP_SCHEDULE='schedule(guided,8)'

DEMPS_OPTS="-c $DEMPS_CONFIG_PATH -o $RESULTS_DIR -e $EXPERIMENT_NUMBER"

echo $DEMPS_PATH $DEMPS_OPTS
$DEMPS_PATH $DEMPS_OPTS

