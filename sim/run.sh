#!/bin/bash
#================================================================
#% SYNOPSIS
#+    ${SCRIPT_NAME} -c config_file [options] [-h|--help]
#%
#% DESCRIPTION
#%    Script to call the execution of DEMPS
#%
#% OPTIONS
#%    -c, --config                  config file (JSON)
#%    -t, --threads                 threads to se
#%    -T, --timesim                 total simulation time
#%    -s, --samplinglevel            sampling level 0..1
#%    -o, --outdir                  output directory (overwrite config file)
#%    -n, --residents               residents agents
#%    -N, --visitors                visitors agents
#%    -D, --description             simulation description, e.g: --description "test city"
#%    -d, --densitymodel            enable density model (1:true, 0:false, -1:config)
#%    -p, --panicmodel              enable panic model (1:true, 0:false, -1:config)
#%    -P, --emotionthreshold        emotion threshlod for enter to panic state
#%    -f, --floodmodel              enable flood model (1:true, 0:false, -1:config)
#%    -b, --debrismodel             enable debris model (1:true, 0:false, -1:config)
#%    -B, --debrisratio             ratio of patchs with debris (1..100)%
#%    -v, --elevationmodel          enable elevation model (1:true, 0:false, -1:config)
#%    -V, --elevationfile           file with data elevation. Default: 'elevationPatchData.txt'.
#%    -E, --patchcoords             save in file 'elevationPatch-<city>.txt the coords (lat,lon) of each patch and end.
#%    -e, --experiment              experiment number
#%    
#-
#- IMPLEMENTATION
#-    version         ${SCRIPT_NAME} 1.0.0
#-    author          Gabriel Astudillo
#-    license         GNU General Public License
#================================================================


#    -F [name], --foobar=[name] set foobar and foobar_name
#    -h, --help                 print this help

usage() { printf "Usage: "; head -50 ${0} | grep "^#+" | sed -e "s/^#+[ ]*//g" -e "s/\${SCRIPT_NAME}/${SCRIPT_NAME}/g" ; }
usagefull() { head -50 ${0} | grep -e "^#[%+-]" | sed -e "s/^#[%+-]//g" -e "s/\${SCRIPT_NAME}/${SCRIPT_NAME}/g" ; }
 
#============================
#  SET VARIABLES
#============================
unset SCRIPT_NAME SCRIPT_OPTS ARRAY_OPTS
 
#== general variables ==#
SCRIPT_NAME="$(basename ${0})"
OptFull=$@
OptNum=$#
 

BASEDIR=$(readlink -f $0)
BASEDIR=$(dirname $BASEDIR)

RM_CMD="$(which rm) "
CP_CMD="$(which cp)"
CHMOD_CMD="$(which chmod)"
MKDIR_CMD="$(which mkdir)"

ulimit -c unlimited

DEMPS_BIN="./demps"

EXPERIMENT_NUMBER=-1
SAMPLING_LEVEL=-1
DURATION=0
AGENTS_RESIDENTS_NUMBER=0
AGENTS_VISITORS_NUMER=0
DESCRIPTION="null"
FLOOD_MODEL=-1
PANIC_MODEL=-1
EMOTION_THRESHOLD=0
DENSITY_MODEL=-1
DEBRIS_MODEL=-1
DEBRIS_RATIO=1
ELEVATION_MODEL=-1
ELEVATION_FILE=""
CALC_ELEVATION=""
DEMPS_CONFIG="null"
THREADS=0


SCRIPT_OPTS="c:o:d:e:s:n:N:D:f:p:P:b:B:T:t:v:V:-:Eh"
typeset -A ARRAY_OPTS
ARRAY_OPTS=(
[config]=c
[outdir]=o
[densitymodel]=d
[experiment]=e
[samplinglevel]=s
[residents]=n
[visitors]=N
[description]=D
[floodmodel]=f
[panicmodel]=p
[emotionthreshold]=P
[debrismodel]=b
[debrisratio]=B
[elevationmodel]=v
[elevationfile]=V
[timesim]=T
[threads]=t
[patchcoords]=E
[help]=h
)

#while getopts $SCRIPT_OPTS opt; do
#  case ${opt} in
#   o ) 
#		RESULTS_DIR_OPT=$OPTARG
#		;;
#	T )
#		DURATION=$OPTARG
#		;;
#	e )
#		EXPERIMENT_NUMBER=$OPTARG
#		;;
#	E )
#		CALC_ELEVATION="-E"
#		;;
#	c )
#		DEMPS_CONFIG=$OPTARG
#		;;
#	s )
#		SAMPLING_LEVEL=$OPTARG
#		;;
#	n )
#		AGENTS_RESIDENTS_NUMBER=$OPTARG
#		;;
#	N )
#		AGENTS_VISITORS_NUMER=$OPTARG
#		;;
#	D )
#		DESCRIPTION=$OPTARG
#		;;
#	f )
#		FLOOD_MODEL=$OPTARG
#		;;
#	p)
#		PANIC_MODEL=$OPTARG
#		;;
#	d)
#		DENSITY_MODEL=$OPTARG
#		;;
#	\? )
#      ;;
#  esac
#done
#shift $((OPTIND -1))

while getopts ${SCRIPT_OPTS} OPTION ; do
	#== translate long options to short ==#
	if [[ "x$OPTION" == "x-" ]]; then
		LONG_OPTION=$OPTARG
		LONG_OPTARG=$(echo $LONG_OPTION | grep "=" | cut -d'=' -f2)
		LONG_OPTIND=-1
		[[ "x$LONG_OPTARG" = "x" ]] && LONG_OPTIND=$OPTIND || LONG_OPTION=$(echo $OPTARG | cut -d'=' -f1)
		[[ $LONG_OPTIND -ne -1 ]] && eval LONG_OPTARG="\$$LONG_OPTIND"
		OPTION=${ARRAY_OPTS[$LONG_OPTION]}
		[[ "x$OPTION" = "x" ]] &&  OPTION="?" OPTARG="-$LONG_OPTION"
 
		if [[ $( echo "${SCRIPT_OPTS}" | grep -c "${OPTION}:" ) -eq 1 ]]; then
			if [[ "x${LONG_OPTARG}" = "x" ]] || [[ "${LONG_OPTARG}" = -* ]]; then
				OPTION=":" OPTARG="-$LONG_OPTION"
			else
				OPTARG="$LONG_OPTARG";
				if [[ $LONG_OPTIND -ne -1 ]]; then
					[[ $OPTIND -le $Optnum ]] && OPTIND=$(( $OPTIND+1 ))
					shift $OPTIND
					OPTIND=1
				fi
			fi
		fi
	fi
 
	#== options follow by another option instead of argument ==#
	if [[ "x${OPTION}" != "x:" ]] && [[ "x${OPTION}" != "x?" ]] && [[ "${OPTARG}" = -* ]]; then
		OPTARG="$OPTION" OPTION=":"
	fi
 
	#== manage options ==#
	case "$OPTION" in
		o ) 
			RESULTS_DIR_OPT=$OPTARG
			;;
		T )
			DURATION=$OPTARG
			;;
		t )
			THREADS=$OPTARG
			;;
		e )
			EXPERIMENT_NUMBER=$OPTARG
			;;
		E )
			CALC_ELEVATION="-E"
			;;
		c )
			DEMPS_CONFIG=$OPTARG
			;;
		s )
			SAMPLING_LEVEL=$OPTARG
			;;
		n )
			AGENTS_RESIDENTS_NUMBER=$OPTARG
			;;
		N )
			AGENTS_VISITORS_NUMER=$OPTARG
			;;
		D )
			DESCRIPTION=$OPTARG
			;;
		f )
			FLOOD_MODEL=$OPTARG
			;;
		p )
			PANIC_MODEL=$OPTARG
			;;
		P )
			EMOTION_THRESHOLD=$OPTARG
			;;
		b )
			DEBRIS_MODEL=$OPTARG
			;;
		B )
			DEBRIS_RATIO=$OPTARG
			;;
		d )
			DENSITY_MODEL=$OPTARG
			;;
			
		v )
			ELEVATION_MODEL=$OPTARG
			;;
		V )
			ELEVATION_FILE="--elevationfile $OPTARG"
			;;

		h ) usagefull && exit 0 ;;
		: ) echo "${SCRIPT_NAME}: -$OPTARG: option requires an argument" >&2 && usage >&2 && exit 99 ;;
		? ) echo "${SCRIPT_NAME}: -$OPTARG: unknown option" >&2 && usage >&2 && exit 99 ;;
	esac
done
shift $((${OPTIND} - 1))

#### CONFIGURACION ####
if [[  $DEMPS_CONFIG == "null" ]]; then
	usage >&2 && exit 99
fi

DEMPS_PATH=$DEMPS_BIN 

if [[ ! -f $DEMPS_PATH  ]]; then
	echo "Error: $DEMPS_PATH no existe"
	usage >&2 && exit 99
fi

DEMPS_CONFIG_PATH="$BASEDIR/$DEMPS_CONFIG"

if [[ ! -f $DEMPS_CONFIG_PATH ]]; then
	echo "El archivo de configuración $DEMPS_CONFIG_PATH no existe."
	usage >&2 && exit 99
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

#BASEDIR_SIM=$(dirname $DEMPS_CONFIG_PATH)
BASEDIR_SIM=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.baseDirSim')
if [[ ! -d $BASEDIR_SIM ]]; then
	echo "Error en la entrda [\"baseDirSim\"]"
	echo "El directorio \"$BASEDIR_SIM\" no existe."
	exit 1
fi

RESULTS_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output.directory')

#Verificar si hay que reemplazar el valor de $RESULTS_DIR
#del archivo de configuración con $RESULTS_DIR_OPT (parámetro -o)
if [[ -z $RESULTS_DIR_OPT ]]; then
	RESULTS_DIR_PATH=$BASEDIR_SIM/$RESULTS_DIR
else
	RESULTS_DIR_PATH=$BASEDIR_SIM/$RESULTS_DIR_OPT
	RESULTS_DIR=$RESULTS_DIR_OPT
fi
#RESULTS_DIR_PATH=$BASEDIR/$RESULTS_DIR


echo "Eliminando resultados anteriores... $RESULTS_DIR_PATH"
$RM_CMD -rf $RESULTS_DIR_PATH


AGENTS_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."agents-path"')
AGENTS_DIR_PATH=$RESULTS_DIR_PATH/$AGENTS_DIR

STATS_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."stats-path"')
STATS_DIR_PATH=$RESULTS_DIR_PATH/$STATS_DIR

HEATMAP_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.output."heatMap-path"')
HEATMAP_DIR_PATH=$RESULTS_DIR_PATH/$HEATMAP_DIR

if [[ ! -e $AGENTS_DIR_PATH ]]; then
	mkdir -p $AGENTS_DIR_PATH
fi

if [[ ! -e $STATS_DIR_PATH ]]; then
	mkdir -p $STATS_DIR_PATH
fi

if [[ ! -e $HEATMAP_DIR_PATH ]]; then
	mkdir -p $HEATMAP_DIR_PATH
fi

#Copiar los archivos input/*geojson a RESULTS_DIR
#(necesarios para la visualizacion)
INPUT_DIR=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.input.directory')
INPUT_PATH=$BASEDIR_SIM/$INPUT_DIR

if [[ ! -d $INPUT_PATH ]]; then
	echo "Error en la entrada [\"input\"][\"directory\"]"
	echo "El directorio \"$INPUT_DIR\" no existe."
	exit 1
fi


$MKDIR_CMD -p $RESULTS_DIR_PATH/input/
echo $CP_CMD $INPUT_PATH/*.geojson $RESULTS_DIR_PATH/input/
$CP_CMD $INPUT_PATH/*.geojson $RESULTS_DIR_PATH/input/
$CP_CMD $INPUT_PATH/*.png $RESULTS_DIR_PATH/input/ 2> /dev/null
$CP_CMD input/animacion*.html $RESULTS_DIR_PATH/

$CHMOD_CMD -R +r $RESULTS_DIR_PATH/input


RESULTS_FILES="$AGENTS_DIR_PATH/* $STATS_DIR_PATH/*"

#THREADS=$(cat $DEMPS_CONFIG_PATH | $JQ_PATH -r '.threads')



#### MAIN ####
printf "DEMPS_CONFIG_PATH=%s\n" "$DEMPS_CONFIG_PATH"



echo "Ejecutando DEMPS..."
#export OMP_NUM_THREADS=$THREADS
#export OMP_SCHEDULE='schedule(guided,8)'

DEMPS_OPTS="--config $DEMPS_CONFIG_PATH "
DEMPS_OPTS+="--sampliglevel $SAMPLING_LEVEL "
DEMPS_OPTS+="--timesim $DURATION "
DEMPS_OPTS+="--threads $THREADS "
DEMPS_OPTS+="--floodmodel $FLOOD_MODEL "
DEMPS_OPTS+="--panicmodel $PANIC_MODEL "
DEMPS_OPTS+="--emotionthreshold $EMOTION_THRESHOLD "
DEMPS_OPTS+="--debrismodel $DEBRIS_MODEL --debrisratio $DEBRIS_RATIO "
DEMPS_OPTS+="--densitymodel $DENSITY_MODEL "
DEMPS_OPTS+="--residents $AGENTS_RESIDENTS_NUMBER --visitors $AGENTS_VISITORS_NUMER "
DEMPS_OPTS+="--outdir $RESULTS_DIR "
DEMPS_OPTS+="--experiment $EXPERIMENT_NUMBER "
DEMPS_OPTS+="--elevationmodel $ELEVATION_MODEL "
DEMPS_OPTS+="$ELEVATION_FILE "
DEMPS_OPTS+="$CALC_ELEVATION " 

#echo $DEMPS_OPTS
echo $DEMPS_PATH $DEMPS_OPTS -D "$DESCRIPTION"
$DEMPS_PATH $DEMPS_OPTS -D "$DESCRIPTION"

