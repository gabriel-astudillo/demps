# DEMPS
**D**isaster **E**vacuation and **M**obility **P**atterns **S**imulator

Este simulador implementa movimientos de personas dentro de una ciudad costera, en el caso de una evacuación de los habitantes hacia las zonas seguras que pueda tener dicho lugar. 

**Modelo de movilidad implementado**

El modelo de movilidad implementado se basa en el Modelo de Fuerza Social propuesto por Helbing & Molnar (1998) "*Social Force Model for Pedestrian Dynamics*". 

**Estructura de directorios**

* ```sim/```: directorio donde se realiza la simulación.
* ```sim/ciudadX.config``` : Archivo JSON de configuración para la simulación de la *ciudadX*.
* ```sim/ejemplos/``` : ejemplo de archivos de visualización de una simulación.
* ```sim/input/``` : Directorio que contiene los archivos de mapas de las ciudades a simular. Cada ciudad es un subdirectorio, que contiene los archivos:
* ```sim/input/animacion.html```: plantilla HTML para crear la animación de la simmulación. Ver sección *Visualización* más abajo.
* ```sim/input/ciudadX/``` : Directorio de los archivos de mapas (GeoJSON) de la *ciudadX*. 
* ```sim/input/ciudadX/zones.geojson``` : zona geográfica a simular. Incluye zonas iniciales, seguras y de inundación.



# Compilación e instalación
1) Verificar que su sistema tiene las [dependencias](https://github.com/gabriel-astudillo/demps/blob/master/dependiencias.md) necesarias para la compilación.

2) Clonar este repositorio 

```
git clone https://github.com/gabriel-astudillo/demps.git
cd demps
make
make install
```

El ejecutable se deja en el directorio ```sim```. Para realizar una simulación, utilizar el script ```run.sh```.

```
cd sim
chmod +x run.sh
./run.sh
```

3) Si instaló una versión de OSRM > 5.18, entonces debe procesar nuevamente los mapas de la ciudades. Por ejemplo, para procesar el mapa de la ciudadX, se debe hacer:

```
cd sim/input/ciudadX
demps-map-download.py zones.geojson
```


# Visualización de la simulación

Los resultados de la simulación se almacenan en el directorio ```sim/output/CiudadX```, el que tiene la siguiente estructura:

* ```agents/```: directorio que contiene el movimiento de las personas.
* ```agents/<Tn.txt>```: archivo que contiene la posición geográfica de todas las personas simuladas, en el instante de simulación 'Tn'. Cada línea tiene la siguiente estructura:

```
<IDpersona> <latitud> <longitud> <TipoPersona> 
  IDpersona  : {0,1,2,3,...}
  TipoPersona: {0,1,2}
```

* ```input/```: directorio con la configuración geográfica de la simulación.
* ```input/zones.geojson``` : zona geográfica a simular.
* ```animacion.html``` : Animación del movimiento de los peatones utilizando MapboxGL JS https://docs.mapbox.com/mapbox-gl-js/api/
* ```animacion.config.json```: archivo de configuración utilizado por el archivo anterior.
  
Para visualizar la animación, el archivo ```animacion.html``` debe ser accedido a través de un servidor web.
                                   





