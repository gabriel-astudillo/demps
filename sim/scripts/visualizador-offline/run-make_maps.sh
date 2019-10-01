#!/bin/bash
#BASEDIR=$(dirname "$0")

BASEDIR=$(readlink -f $0)
BASEDIR=$(dirname $BASEDIR)

#Numero de threads para la conversion ARCHIVOS LOGS-> ARCHIVOS PNG
NUM_THREADS=4

#Ruta donde están los logs de la simulación
#Si no se especifican a través de $1, se asume este valor.
#La ruta espedicificada a través de $1 DEBE SER ABSOLUTA
#RESULTS_DIR DEBE SER RELATIVA
RESULTS_DIR="../../results/agents"

if [[ $1 ]]; then
	RESULTS_DIR=$1
	
	if [[ ! -e $RESULTS_DIR ]]; then
		echo "El directorio de logs del simulador $RESULTS_DIR no existe."
		exit
	fi
	rm -f $RESULTS_DIR/._*
else
	if [[ ! -e $BASEDIR/$RESULTS_DIR ]]; then
		echo "El directorio de logs del simulador $BASEDIR/$RESULTS_DIR no existe."
		exit
	fi
	rm -f $RESULTS_DIR/._*
fi


echo "Eliminando imágenes anteriores..."
rm -f $BASEDIR/mapimages_sim/*

echo "Creando imágenes a partir de los logs de la simulación..."
$BASEDIR/make_maps.pl -r $RESULTS_DIR -p $NUM_THREADS

if [ $? -ne 0 ]; then
	echo "No se puede continuar debido a un error del script make_maps.pl"
	exit 1;
fi

$BASEDIR/anim_gif.sh




