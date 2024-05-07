#!/usr/bin/env python3

from geojson import Feature, Point, FeatureCollection, Polygon
import subprocess as sub
import geojson
import json
import sys
import os

#filename=os.path.basename(sys.argv[1]).split(".")[0]


try:
    file_geojson = sys.argv[1]
except:
    sys.exit("Use: "+ sys.argv[0] + " <file_geojson> [<file_openstreemap>]")

try:
    file_osm_pbf = sys.argv[2]
except:
    file_osm_pbf = ""

filename="map"

# 0.01 grados significa aprox 800 metros
offset = 0.01 * 1; 

with open(file_geojson, 'r') as data_file:
    data=json.load(data_file)
fcollection=FeatureCollection(data['features'])

"""
min_lon=min([x[0] for x in fcollection["features"][0]["geometry"]["coordinates"][0]]);
min_lat=min([x[1] for x in fcollection["features"][0]["geometry"]["coordinates"][0]]);
max_lon=max([x[0] for x in fcollection["features"][0]["geometry"]["coordinates"][0]]);
max_lat=max([x[1] for x in fcollection["features"][0]["geometry"]["coordinates"][0]]);

print("min_lon:", min_lon, "; min_lat:", min_lat, "; max_lon:", max_lon, "; max_lat: ", max_lat)
"""

min_lon = 180
min_lat = 90
max_lon = -180
max_lat = -90
for feature in fcollection["features"]:
    if(feature["properties"]["zoneType"] == "initial" or feature["properties"]["zoneType"] == "safe" or feature["properties"]["zoneType"] == "flood"):
        #print(feature["properties"]["zoneType"]);
        min_lon_f=min([x[0] for x in feature["geometry"]["coordinates"][0]])
        min_lat_f=min([x[1] for x in feature["geometry"]["coordinates"][0]])
        max_lon_f=max([x[0] for x in feature["geometry"]["coordinates"][0]])
        max_lat_f=max([x[1] for x in feature["geometry"]["coordinates"][0]])
        #print("min_lon_f:", min_lon_f, "; min_lat_f:", min_lat_f, "; max_lon_f:", max_lon_f, "; max_lat_f: ", max_lat_f)
        min_lon = min(min_lon_f, min_lon);
        min_lat = min(min_lat_f, min_lat);
        max_lon = max(max_lon_f, max_lon);
        max_lat = max(max_lat_f, max_lat);
        
# Se agrega el offset
min_lon -= offset
min_lat -= offset
max_lon += offset
max_lat += offset        

boxCoords = repr(min_lon) + ',' + repr(min_lat) + ',' + repr(max_lon) + ',' + repr(max_lat)

print("Geographic Box to use:")
print("min_lon:", min_lon, "; min_lat:", min_lat, "; max_lon:", max_lon, "; max_lat: ", max_lat, sep="")

if(os.path.exists(file_osm_pbf) ):
    print("1) Getting data data from file:", file_osm_pbf, sep="")
    print("--> Creating ", filename, ".pbf" ,sep="")
    sub.call(['osmconvert', file_osm_pbf, '-b=' + boxCoords,'-o=' + filename + '.pbf'],stdout=sub.PIPE,stderr=sub.PIPE)
else:    
    print("1) Getting data data from OpenStreeMap: ")
    
    osmURL  = 'https://api.openstreetmap.org/api/0.6/map?bbox=' 
    osmURL += boxCoords
    print("-->Download from openstreetmap: ", filename, ".osm", sep="")
    print("-->", osmURL, sep="")

    sub.call(['wget',osmURL,'-O',filename + '.osm'],stdout=sub.PIPE,stderr=sub.PIPE)
    
    if(os.stat(filename + '.osm').st_size == 0):
        sys.exit("Error in retrieve file: " + filename + '.osm')

    
    print("-->Converting: ", filename, ".osm", " -> ", filename, ".pbf" ,sep="")
    sub.call(['osmconvert',filename + '.osm', '-o=' + filename + '.pbf'],stdout=sub.PIPE,stderr=sub.PIPE)
    


print("2) Extract features from: ", filename, ".pbf", sep="")
sub.call(['osrm-extract',filename + '.pbf','-p','/usr/local/share/osrm/profiles/foot.lua'],stdout=sub.PIPE,stderr=sub.PIPE)

print("3) Contract : ", filename, ".osrm", sep="")
sub.call(['osrm-contract',filename + '.osrm'],stdout=sub.PIPE,stderr=sub.PIPE)
