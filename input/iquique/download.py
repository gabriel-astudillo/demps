#!/usr/bin/env python

from geojson import Feature, Point, FeatureCollection, Polygon
import subprocess as sub
import geojson
import json
import sys
import os

filename=os.path.basename(sys.argv[1]).split(".")[0]

with open(sys.argv[1],'r') as data_file:
    data=json.load(data_file)
fcollection=FeatureCollection(data['features'])
min_lon=min([x[0] for x in fcollection["features"][0]["geometry"]["coordinates"][0]]);
min_lat=min([x[1] for x in fcollection["features"][0]["geometry"]["coordinates"][0]]);
max_lon=max([x[0] for x in fcollection["features"][0]["geometry"]["coordinates"][0]]);
max_lat=max([x[1] for x in fcollection["features"][0]["geometry"]["coordinates"][0]]);

sub.call(['wget','https://api.openstreetmap.org/api/0.6/map?bbox=' + repr(min_lon) + ',' + repr(min_lat) + ',' + repr(max_lon) + ',' + repr(max_lat),'-O',filename + '.osm'],stdout=sub.PIPE,stderr=sub.PIPE)

sub.call(['osmconvert',filename + '.osm', '-o=' + filename + '.pbf'],stdout=sub.PIPE,stderr=sub.PIPE)
sub.call(['osrm-extract',filename + '.pbf','-p','/usr/local/share/osrm/profiles/foot.lua'],stdout=sub.PIPE,stderr=sub.PIPE)
sub.call(['osrm-contract',filename + '.osrm'],stdout=sub.PIPE,stderr=sub.PIPE)
