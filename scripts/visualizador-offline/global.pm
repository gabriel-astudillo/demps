#!/usr/bin/env perl

package global;

$sim_img_results_directory = "mapimages_sim/";
#my $sim_img_results_directory = "../../results/";

$plot_latlong_path_rel = "../plot-latlong/plot-latlong.pl";

#$plot_latlong_options  = "-m iquique -s 2"; iquique_zonas
$plot_latlong_options  = "-m iquique_zonas -s 2"; 


#$plot_latlong_options  = "-m curaumaPUCV -s 3";
#$plot_latlong_options  = "-m vdm -s 2";

1;