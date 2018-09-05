#!/usr/bin/env perl

use strict;
use File::Basename;
use Cwd 'abs_path'; 
use Getopt::Std;
use vars qw($opt_r);

use threads qw[ yield ];
use threads::shared;

# Directorio base de este script
my $base_path = dirname(abs_path($0));

require "$base_path/global.pm";

getopts('r:');

if (! $opt_r ) {
  usage();
}

if (! -e $opt_r) {
	die "Directorio $opt_r no existe.\n";
}

# directorio con los resultados de la simulación
my $sim_data_results_directory = "$base_path/$opt_r";  
if (! -e $sim_data_results_directory) {
	die ("Directorio $sim_data_results_directory no existe.");
}


# Script que crea los PNG a partir de un archivo de lat/long
my $plot_latlong_script = "$base_path/" . $global::plot_latlong_path_rel;

if (! -e $plot_latlong_script) {
	die ("Script $plot_latlong_script no existe.");
}

$plot_latlong_script .= " " . $global::plot_latlong_options;

# Directorio donde se almacenan los archivos PNG creados
# $global::sim_img_results_directory 

if (! -e $global::sim_img_results_directory) {
	mkdir $global::sim_img_results_directory;
}


###########SPINNER#############
my $ready : shared = 0;
my $isOk : shared  = 0;
my $perc : shared  = 0;

async {
    local $| = 1;
    while ( !$ready ) {
        do {
            select undef, undef, undef, 0.2;
            printf "\r ($_) $perc%" if ($isOk);
          }
          for qw[ / - \ | ];
    }
    print "\r       OK\n";
    $ready = 0;
  }
  ->detach;
###############################

my @files;
opendir(DIR, $sim_data_results_directory);
@files = grep { $_ ne '.' && $_ ne '..' } readdir(DIR);
closedir(DIR);
@files = sort {$a <=> $b}  @files;

my $index = 0;
foreach my $file (@files) {
	$isOk = 1; # Para el spinner
	$perc = int($index / $#files * 100);
	my $path_file = "$sim_data_results_directory/$file";
	my $file_img = $file;
	
	#$file_img =~ s/(\d+)\..*/$1.png/;
	$file_img =~ /(\d+)\..*/;
	my $suff;
	$suff = sprintf("%010d", $1);
	$file_img = "$suff.png";
	
	my $cmd = "cat $path_file | $plot_latlong_script > $global::sim_img_results_directory/$file_img";
	`$cmd`;
	$isOk = 0; # Para el spinner
	$index++;
}

$isOk = 0;  # Para el spinner
$ready = 1; # Para el spinner
yield while $ready; # Para el spinner

exit;

sub usage {
  die "Uso: $0 -r <dir resultados simulación>\n\n\tDirectorio debe ser relativo al directorio de este script.\n\n";
}


