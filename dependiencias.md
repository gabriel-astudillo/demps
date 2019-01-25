# Dependencias


## JQ

Utilitario de línea de comando para procesar archivos JSON. Ver https://stedolan.github.io/jq/.

## JSON

JSON for Modern C++: https://github.com/nlohmann/json

1) Bajar la biblioteca desde https://github.com/nlohmann/json/blob/develop/single_include/nlohmann/json.hpp
2) Incluirla como:
```
#include "json.hpp"
using json = nlohmann::json;
```


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

3) Incluir la biblioteca ```<GeographicLib/LocalCartesian.hpp>```. En el linker, agregar ```-lGeographic```

## CGAL

The Computational Geometry Algorithms Library, https://www.cgal.org

Versión: ≥ 4.10

* Linux: ```apt-get install libcgal-dev```
* Mac: ```port install cgal```

En el linker, agregar ```-lCGAL```.

## OSRM backend

Open Source Routing Machine: The OpenStreetMap Data Routing Engine, http://project-osrm.org. Dependencias: ```libtbb-dev```, ```libbz2-dev```, ```liblua5.3-dev```, ```libexpat1-dev```. Sitio original https://github.com/Project-OSRM/osrm-backend/wiki/Building-OSRM

1) Bajar de https://github.com/Project-OSRM/osrm-backend/releases, descomprimir y entrar al directorio respectivo. Los mapas están procesados en base a la versión 5.18.0.
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

En el linker, agregar ```-losrm```.



