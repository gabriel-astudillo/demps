#!/bin/bash
echo "Eliminando imágenes anteriores..."
rm -f mapimages_sim/*

echo "Creando imágenes a partir de los logs de la simulación..."
./make_maps.pl -r ../../results

./anim_gif.sh

