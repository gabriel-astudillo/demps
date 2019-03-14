#!/bin/bash

BASEDIR=$(readlink -f $0)
BASEDIR=$(dirname $BASEDIR)

DEMPS_BIN="$BASEDIR/demps"
CONFIG="$BASEDIR/demps-iquique.config"

NUM_THREADS="1 2 4"
NUM_AGENTS=$(seq 10000 10000 120000)
#NUM_AGENTS="3000 4000 5000"
T_SIM=$(seq 2000 2000 10000)
T_SIM="100"

ROOT_DIR_LOGS="$BASEDIR/logs-exp"
mkdir -p $ROOT_DIR_LOGS
rm -f $ROOT_DIR_LOGS/*


OUT_FILE="$ROOT_DIR_LOGS/timeExec.txt"
OUT_FILE_TH="$ROOT_DIR_LOGS/timeExec"



#> $OUT_FILE

for agents in $NUM_AGENTS
do
	printf "AGENTS=%d\n" $agents
	for tsim in $T_SIM
	do
		printf ">TSIM=%d\n" $tsim
		for num_th in $NUM_THREADS
		do
			printf ">>THREADS=%d\n" $num_th
			BASE_FILE="$ROOT_DIR_LOGS/exp-$agents-$tsim-$num_th"
			LOG_FILE="$BASE_FILE.log"
			RES_FILE="$BASE_FILE.res"
			> $LOG_FILE
	
			for repeticion in {1..3}
			do		
				#echo "."
				$DEMPS_BIN -s $CONFIG -t $num_th -d $tsim -n $agents  | tail -1  >> $LOG_FILE
				#echo "-"
			done

	
			T_CREACION=$(cat $LOG_FILE |cut -d ':' -f 6 \
				| awk '{if(min==""){min=max=$1}; if($1>max) {max=$1}; if($1<min) {min=$1}; total+=$1; count+=1} END \
					{print total/count, max, min}')
			T_CALIBRACION=$(cat $LOG_FILE |cut -d ':' -f 7 \
				| awk '{if(min==""){min=max=$1}; if($1>max) {max=$1}; if($1<min) {min=$1}; total+=$1; count+=1} END \
					{print total/count, max, min}')
			T_SIMULACION=$(cat $LOG_FILE |cut -d ':' -f 8 \
				| awk '{if(min==""){min=max=$1}; if($1>max) {max=$1}; if($1<min) {min=$1}; total+=$1; count+=1} END \
					{print total/count, max, min}')
			MEMORY=$(cat $LOG_FILE |cut -d ':' -f 10 \
				| awk '{if(min==""){min=max=$1}; if($1>max) {max=$1}; if($1<min) {min=$1}; total+=$1; count+=1} END \
					{print total/count, max, min}')
						
			AG_MEMORY=$(cat $LOG_FILE |cut -d ':' -f 11 \
				| awk '{if(min==""){min=max=$1}; if($1>max) {max=$1}; if($1<min) {min=$1}; total+=$1; count+=1} END \
					{print total/count, max, min}')
	
			printf "%d %d %d %.2f %d %d %.2f %d %d %.2f %d %d %.2f %d %d %.2f %d %d\n"    \
				$agents $tsim $num_th $T_CREACION $T_CALIBRACION $T_SIMULACION $MEMORY $AG_MEMORY >> \
				$OUT_FILE_TH-th-$num_th-$agents.txt
			
			printf "%d %d %d %.2f %d %d %.2f %d %d %.2f %d %d %.2f %d %d\ %.2f %d %dn"\
				 $agents $tsim $num_th $T_CREACION $T_CALIBRACION $T_SIMULACION $MEMORY $AG_MEMORY >> \
					 $OUT_FILE_TH-ag-$agents-$tsim.txt
			
			printf "%d %d %d %.2f %d %d %.2f %d %d %.2f %d %d %.2f %d %d %.2f %d %d\n"\
				 $agents $tsim $num_th $T_CREACION $T_CALIBRACION $T_SIMULACION $MEMORY $AG_MEMORY >> \
					 $OUT_FILE_TH-tsim-$tsim-$num_th.txt
		done
		
	done
done
