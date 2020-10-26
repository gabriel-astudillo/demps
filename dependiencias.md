# Dependencias


## JQ

Utilitario de línea de comando para procesar archivos JSON. Ver https://stedolan.github.io/jq/.

* Linux: ```apt-get install jq```

## Geographic lib

"GeographicLib is a small set of C++ classes for performing conversions between geographic, UTM, UPS, MGRS, geocentric, and local cartesian coordinates", https://geographiclib.sourceforge.io

1) Bajar de https://geographiclib.sourceforge.io, descomprimir y entrar al directorio respectivo.
2) Compilar e instalar la librería en el directorio por omisión ```/usr/local/```:
```
mkdir build
cd build
../configure
make
make install (!)
```
(!) Necesita permisos de superusario.

## CGAL

The Computational Geometry Algorithms Library, https://www.cgal.org. Utilizar versión ≥ 5.0

* Linux: ```apt-get install libcgal-dev```


## OSRM backend

Open Source Routing Machine: The OpenStreetMap Data Routing Engine, http://project-osrm.org. Dependencias: ```libtbb-dev```, ```libbz2-dev```, ```liblua5.3-dev```, ```libexpat1-dev```. Sitio original https://github.com/Project-OSRM/osrm-backend/wiki/Building-OSRM

1) Bajar de https://github.com/Project-OSRM/osrm-backend/releases, descomprimir y entrar al directorio respectivo. Los mapas están procesados en base a la versión 5.18.0. Si instala una más reciente, debe procesar los mapas nuevamente a través del script ```download.py``` disponible en la carpeta ```sim/input/CiudadX```.
2) Compilar e instalar. Por omisión, ```--prefix=/usr/local```:
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release  (!)
cmake --build .                      (!!)
cmake --build . --target install     (!!!)
```
(!)  ```cmake .. -DCMAKE_INSTALL_PREFIX=/ruta/instalacion  -DCMAKE_BUILD_TYPE=Release```

(!!) Este proceso necesita al menos 3GB en RAM.

(!!!) Necesita permisos de superusario.




