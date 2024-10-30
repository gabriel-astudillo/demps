# Dependencias

## Make, g++, cmake, libUuid C++

 Ubuntu 20.04: ```sudo apt-get install make g++ cmake uuid-dev```

## LibBoost C++

Ubuntu 20.04: ```sudo apt install libboost-all-dev```

## JQ

Utilitario de línea de comando para procesar archivos JSON. Ver https://stedolan.github.io/jq/.

* Ubuntu 20.04: ```sudo apt-get install jq```


## OSMCTOOLS

Utilidades de línea de comandos para manipular archivos de OpenStreetMaps.

* Ubuntu 20.04: ```sudo apt-get install osmctools```

## Geographic lib

"GeographicLib is a small set of C++ classes for performing conversions between geographic, UTM, UPS, MGRS, geocentric, and local cartesian coordinates", https://geographiclib.sourceforge.io

1) Bajar de https://geographiclib.sourceforge.io, descomprimir y entrar al directorio respectivo.
2) Compilar e instalar la librería en el directorio por omisión ```/usr/local/```:
```
mkdir build
cd build
../configure
make
sudo make install
```

## CGAL

The Computational Geometry Algorithms Library, https://www.cgal.org. Utilizar versión ≥ 5.0

* Ubuntu 20.04: ```sudo apt-get install libcgal-dev```


## Restclient-cpp

```
$ sudo apt install libcurlpp-dev
$ sudo apt install libcurl4-openssl-dev
$ git clone https://github.com/mrtazz/restclient-cpp.git
$ cd restclient-cpp
$ ./autogen.sh
$ ./configure --prefix=/usr/local/restclient-cpp
$ sudo make install
```

Las librerías quedan instaladas en ```/usr/local/restclient-cpp/lib``` y las bibliotecas en ```/usr/local/restclient-cpp/include```


## OSRM backend

Open Source Routing Machine: The OpenStreetMap Data Routing Engine, http://project-osrm.org. Dependencias: ```libtbb-dev```, ```libbz2-dev```, ```liblua5.3-dev```, ```libexpat1-dev```. Sitio original https://github.com/Project-OSRM/osrm-backend/wiki/Building-OSRM

1) Bajar de https://github.com/Project-OSRM/osrm-backend/releases, descomprimir y entrar al directorio respectivo. Los mapas están procesados en base a la versión 5.18.0. Si instala una más reciente, debe procesar los mapas nuevamente a través del script ```demps-map-download.py``` disponible en el directorio ```/usr/local/bin```.
2) Compilar e instalar. Por omisión, ```--prefix=/usr/local```:
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release  (!)
cmake --build .                      (!!)
sudo cmake --build . --target install
```
(!)  ```cmake .. -DCMAKE_INSTALL_PREFIX=/ruta/instalacion  -DCMAKE_BUILD_TYPE=Release```
En la versión 5.18.0, la compilación en Ubuntu 20.04, arroja algunos errores que se solucionan con estos pasos:
* ```src/server/api/parameters_parser.cpp```: línea 50, eliminar ```std::move()```.
* ```src/storage/io_config.cpp```: línea 18, eliminar ```{``` y ```}```.
* ```include/updates/csv_file_parser.hpp```: Línea 150, eliminar ```std::move()```.
Estas modificaciones ya están realizadas en la versión patched disponible en el directorio dependencias.

(!!) Este proceso necesita al menos 3GB en RAM.


