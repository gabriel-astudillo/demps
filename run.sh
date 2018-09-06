#!/bin/bash

#### CONFIGURACION ####
DEMPS_PATH=./demps 
DEMPS_OPTS="-s demps.config"
RESULTS_FILES="./results/*"

RM_CMD="$(which rm) -f"

#### MAIN ####
echo "Eliminando resultados anteriores..."
$RM_CMD $RESULTS_FILES
echo "Ejecutando DEMPS..."
time $DEMPS_PATH $DEMPS_OPTS