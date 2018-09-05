#!/bin/bash

echo "Creando imágenes a partir de los logs de la simulación"
./make_maps.pl -r ../../results

./anim_gif.sh

