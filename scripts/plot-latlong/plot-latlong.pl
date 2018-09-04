#!/usr/bin/env perl

##############################################################################
## Plots user supplied lat/longs on a user chosen map.
##
## plot-latlong reads lat/long values from stdin and writes a PNG to
## stdout.  The input should contain one pair of lat/long values per line,
## with the values separated by whitespace.  Blank lines and lines starting
## with the pound character ('#') are ignored.
##
## Command line options are
##
##  -m <map-name> specifies the name of a map to use (default: the first
##     map listed in .mapinfo: 'World'); see .mapinfo for the valid map names;
##     see the file CONFIG for a description of the .mapinfo format
##
##  -s <point-size> specifies the size of the points to draw (default: 1);
##     points are filled squares, and the size is the width in pixels
##
##  -c causes the pixel coordinates of each lat/long to be printed to stderr;
##     the coordinates (0, 0) are at the upper left corner, and values increase
##     to the right and down
##
##  -i <map-info-file> specifies an alternate location for .mapinfo (other
##     than in the current directory or $HOME).
##
##----------------------------------------------------------------------------
##
## The code for handling the Alber/Lambert map projection is derived from
## GTrace v1.0.0beta (http://www.caida.org/tools/visualization/gtrace),
## which was written by Ram Periakaruppan.  The included set of maps are also
## derived from the GTrace distribution.  GTrace redistributed these maps
## with the permission of VisualRoute (http://www.visualroute.com),
## the original source of the maps.
##
##----------------------------------------------------------------------------
##
## Copyright (C) 2003,2004,2005 The Regents of the University of California.
## All Rights Reserved.
## 
## Permission to use, copy, modify and distribute any part of this
## plot-latlong software package for educational, research and non-profit
## purposes, without fee, and without a written agreement is hereby
## granted, provided that the above copyright notice, this paragraph
## and the following paragraphs appear in all copies.
##  
## Those desiring to incorporate this into commercial products or use
## for commercial purposes should contact the Technology Transfer
## Office, University of California, San Diego, 9500 Gilman Drive, La
## Jolla, CA 92093-0910, Ph: (858) 534-5815, FAX: (858) 534-7345.
## 
## IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY
## PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
## DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
## SOFTWARE, EVEN IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF
## THE POSSIBILITY OF SUCH DAMAGE.
## 
## THE SOFTWARE PROVIDED HEREIN IS ON AN "AS IS" BASIS, AND THE
## UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO PROVIDE MAINTENANCE,
## SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS. THE UNIVERSITY
## OF CALIFORNIA MAKES NO REPRESENTATIONS AND EXTENDS NO WARRANTIES
## OF ANY KIND, EITHER IMPLIED OR EXPRESS, INCLUDING, BUT NOT LIMITED
## TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A
## PARTICULAR PURPOSE, OR THAT THE USE OF THE SOFTWARE WILL NOT INFRINGE
## ANY PATENT, TRADEMARK OR OTHER RIGHTS.
##  
## The plot-latlong software is developed by the plot-latlong Team at the
## University of California, San Diego under the Cooperative Association
## for Internet Data Analysis (CAIDA) Program.  Support for this work is
## provided by the National Communications System (NCS) via NSF grant
## ANI-0221172, entitled "Routing and Peering Analysis for Enhancing
## Internet Performance and Security."
##
##############################################################################

use strict;
use File::Basename;
use Cwd 'abs_path';
use GD;
use Getopt::Std;
use vars qw($opt_m $opt_s $opt_c $opt_i);

sub usage
{
  die "usage: cat datapoints | plot-latlong [-m <map-name>] [-s <point-size>] [-c] [-i <map-info-file>] >output.png\n";
}

if (not getopts('m:s:ci:'))
{
  usage();
}

my $DEBUG = 0;
my $PI = 3.141592654;

my $point_size = $opt_s || 1;

my $first_map;
my $map_directory;  # directory containing map images
my %configuration;

my $base_path = dirname(abs_path($0)); # GAM 21/08/2018

my @mapinfo_locations = ("$base_path/plot-latlong-conf/mapinfo", "$ENV{HOME}/.mapinfo"); # GAM 21/08/2018
if ($opt_i) {
    unshift @mapinfo_locations, $opt_i;
}
load_configuration(@mapinfo_locations);

my $selected_map = $opt_m || $first_map;

# lat/long of the upper-left and lower-right corners of the map image
my $map_top_lat;
my $map_top_long;
my $map_bottom_lat;
my $map_bottom_long;

# parameters for linear projection: pixels per degree
my $map_lat_scale;
my $map_long_scale;

# parameters for Alber projection (supplied by user):
# (in degrees)
my $R;              # radius of sphere
#my $phi_1;          # standard parallel
#my $phi_2;          # standard parallel
#my $phi_0;          # origin latitude
#my $lambda_0;       # origin longitude
my $false_easting;  # the false easting amount
my $false_northing; # the false northing amount

# values for Alber projection calculated from user parameters:
# (in radians)
my $PHI_1;
my $PHI_2;
my $PHI_0;
my $LAMBDA_0;
my $N;
my $C;

my $conversion_fn;  # pointer to function for converting lat/long to (x, y)

my $map;

#############################################################################

my $green;
my $red;
my $blue;

$map = load_map($selected_map);
$red   = $map->colorAllocate(255, 0  , 0  ); # GAM 21/08/2018
$green = $map->colorAllocate(0  , 255, 0  ); # GAM 21/08/2018
$blue  = $map->colorAllocate(0  , 0  , 255); # GAM 21/08/2018

my @agent_color = ($red, $green, $blue);
 
while (<>)
{
  chomp;
  next if /^\s*$/;
  next if /^\s*#/;

  # REQUIRE: -90 <= $lat <= 90
  #my ($lat, $long) = split /\s+/; #Original
  my ($id, $lat, $long, $agent_type) = split /\s+/; # GAM 21/08/2018
  
  my $adjusted_long = $long;
  if ($long < $map_top_long) {
      $adjusted_long += 360;
  } elsif ($long > $map_bottom_long) {
      $adjusted_long -= 360;
  }
  my ($x, $y) = &$conversion_fn($lat, $adjusted_long);

  print STDERR "$lat $long $x $y\n" if $DEBUG || $opt_c;

  if ($point_size == 1)
  {
    #$map->setPixel($x, $y, $red);
	$map->setPixel($x, $y, $agent_color[$agent_type]); # GAM 21/08/2018
  }
  else
  {
    my $half = int($point_size / 2);
    my $left_x = $x - $half;
    my $top_y = $y - $half;
    $map->filledRectangle($left_x, $top_y,
			  $left_x + $point_size - 1, $top_y + $point_size - 1,
			  $agent_color[$agent_type]); # GAM 03/09/2018 was: $red);			  
  }
}

print $map->png if not $DEBUG;

#############################################################################
#############################################################################

sub convert_latlong_to_xy_linear
{
  my ($lat, $long) = @_;

  my $lat_rel = $map_top_lat - $lat;
  my $long_rel = $long - $map_top_long;

  return ($long_rel * $map_long_scale, $lat_rel * $map_lat_scale);
}

#############################################################################


# See the publication "Map Projections Used by the U.S. Geological
# Survey Bulletin 1532" for details about this projection.
#
# However, the present coder does not know exactly which projection this
# function corresponds to in the USGS Bulletin.

sub round {
    my ($number) = @_;
    return int($number + .5 * ($number <=> 0));
}

sub convert_latlong_to_xy_alber
{
  my ($lat, $long) = @_;

  my $phi = ($lat * $PI) / 180.0;
  my $lambda = ($long * $PI) / 180.0;

  my $p   = ($R * sqrt($C - 2.0 * $N * sin($phi))) / $N;
  my $p_0 = ($R * sqrt($C - 2.0 * $N * sin($PHI_0))) / $N;
  my $theta = $N * ($lambda - $LAMBDA_0);

  my $x = $false_easting + round($p * sin($theta));
  my $y = $false_northing - round($p_0 - $p * cos($theta));

  return ($x, $y);
}

############################################################################

sub load_map
{
  my ($name) = @_;

  if (not exists $configuration{"MAP $name"})
  {
    die "ERROR: Map '$name' not found in map configuration file.\n";
  }

  print("OPEN: $name ", $configuration{"MAP $name"}, "\n") if $DEBUG;

  my $filename;
  ($filename, $map_top_lat, $map_top_long, $map_bottom_lat, $map_bottom_long)
    = split(/\s+/, $configuration{"MAP $name"});

  my $path = "$map_directory/$filename";
  my $retval = new GD::Image($path)
    or die "ERROR: Couldn't open map image '$path': $!\n";

  my ($map_width, $map_height) = $retval->getBounds();

  print("DIM: $map_width $map_height\n") if $DEBUG;

  $map_lat_scale = $map_height / ($map_top_lat - $map_bottom_lat);
  $map_long_scale = $map_width / ($map_bottom_long - $map_top_long);

  print("SCALE: $map_lat_scale $map_long_scale\n") if $DEBUG;

  # -- projections -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

  if (not exists $configuration{"PROJECTION $name"})
  {
    $conversion_fn = \&convert_latlong_to_xy_linear;
  }
  else
  {
    my @F = split(/\s+/, $configuration{"PROJECTION $name"});
    if ($F[0] eq "ALBER")
    {
      ($R, $false_easting, $false_northing) = @F[1,6,7];
      my ($phi_1, $phi_2, $phi_0, $lambda_0) = @F[2..5];

      $PHI_1 = ($phi_1 * $PI) / 180.0;
      $PHI_2 = ($phi_2 * $PI) / 180.0;
      $PHI_0 = ($phi_0 * $PI) / 180.0;
      $LAMBDA_0 = ($lambda_0 * $PI) / 180.0;

      $N = (sin($PHI_1) + sin($PHI_2)) / 2.0;
      $C = cos($PHI_1) ** 2 + 2.0 * $N * sin($PHI_1);

      $conversion_fn = \&convert_latlong_to_xy_alber;
    }
    else
    {
      die "INTERNAL ERROR: unknown projection '$F[0]'\n";
    }
  }

  return $retval;
}

#############################################################################

sub load_configuration
{
  foreach my $filename (@_)
  {
    if (-f $filename)
    {
      $map_directory = dirname($filename) . "/mapimages";
	  load_configuration_aux($filename);
      return;
    }
  }

  die "ERROR: Couldn't find map configuration file; looked for: @_\n";
}

sub load_configuration_aux
{
  my ($filename) = @_;

  open CONFIG, "$filename"
    or die "ERROR: Couldn't open map configuration file '$filename': $!\n";
  while (<CONFIG>)
  {
    chomp;
    next if /^\s*$/;
    next if /^\s*#/;

    my @F = split /\s+/;
    if ($F[0] eq "MAP")
    {
      if ($#F == 6)
      {
	my ($top_lat, $top_long, $bot_lat, $bot_long) = @F[3..6];
	if (!check_coordinate($top_lat)
	    || !check_coordinate($top_long)
	    || !check_coordinate($bot_lat)
	    || !check_coordinate($bot_long))
	{
	  die "ERROR: Line $.: boundary coordinates are malformed in map configuration file.\n";
	}

	if ($top_lat < $bot_lat || $top_long > $bot_long)
	{
	  die "ERROR: Line $.: boundary coordinates have wrong relations in map configuration file.\n";
	}

	$first_map = $F[1] if not defined $first_map;
	$configuration{"MAP $F[1]"} = join(" ", @F[2..6]);
	next;
      }
    }
    elsif ($F[0] eq "PROJECTION")
    {
      if ($#F >= 2)
      {
	if ($F[2] eq "ALBER")
	{
	  if ($#F == 9)
	  {
	    foreach my $x (@F[3..9])
	    {
	      if (not check_number($x))
	      {
		die "ERROR: Line $.: projection parameters are malformed in map configuration file.\n";
	      }
	    }

	    $configuration{"PROJECTION $F[1]"} = join(" ", @F[2..9]);
	    next;
	  }
	}
	else
	{
	  die "ERROR: Line $.: unknown map projection '$F[2]' in map configuration file.\n";
	}
      }
    }
    else
    {
      die "ERROR: Line $.: unknown key '$F[0]' in map configuration file.\n";
    }

    die "ERROR: Line $. of map configuration file is malformed.\n";
  }
  close CONFIG;
}

#############################################################################

sub check_coordinate
{
  my ($x) = @_;

  return $x =~ /^(\-?)\d+(\.\d+)?$/;
}


sub check_number
{
  return check_coordinate(@_);
}
