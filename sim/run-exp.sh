#!/bin/bash

#Total de experimentos a realizar
TOTAL_EXP=10

#Archivo de configuración de los experimentos
CONFIG_FILE="iquique_test.config"

#Directorio donde se dejan las simulaciones
DIR_OUTPUT="output/iquique_test_exp"

for i in $(seq 1 $TOTAL_EXP)
do
	DIROUTPUT=$(printf "%s/%05d" $DIR_OUTPUT $i)
	#DIROUTPUT es de la forma DIR_OUTPUT/N, siendo N un número de 5 digitos
	#Por ejemplo: output/iquique_test_exp/00005
	
	CMD="./run.sh -c $CONFIG_FILE -o $DIROUTPUT -e $i"
	echo "======"
	echo $CMD
	echo "======"
	$CMD
done
