#!/usr/bin/env perl

use strict;
use File::Basename;
use Cwd 'abs_path'; 
use Getopt::Std;
use vars qw($opt_r $opt_p);

use threads qw[ yield ];
use threads::shared;

use Parallel::ForkManager;

# Directorio base de este script
my $base_path = dirname(abs_path($0));

require "$base_path/global.pm";

getopts('r:p:');

if (! $opt_r || ! $opt_p) {
  usage();
}


my $numProc = $opt_p;

$opt_r =~ /^(\/).*/;
my $opt_r_absoluta = $1;

#if (! -e $opt_r) {
#	die "Directorio $opt_r no existe.\n";
#}

# directorio con los resultados de la simulación
my $sim_data_results_directory;

if($opt_r_absoluta) {
	$sim_data_results_directory = $opt_r; 
}
else{
	$sim_data_results_directory = "$base_path/$opt_r"; 
}

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

my $img_results_directory = "$base_path/$global::sim_img_results_directory"; 
if (! -e $img_results_directory) {
	mkdir $img_results_directory;
}

my @files;
opendir(DIR, $sim_data_results_directory);
#@files = grep { $_ ne '.' && $_ ne '..' } readdir(DIR);
@files = grep { /^\d+\.txt$/ }  readdir(DIR);
closedir(DIR);
@files = sort {$a <=> $b}  @files;

my $totalFiles = $#files;

if ($totalFiles <= 0) {
	print "No hay archivos validos para convertir\n";
	exit 1;
}


my $delta      = int($totalFiles/$numProc);

my $pm = Parallel::ForkManager->new($numProc);
 
CREATE_IMG:
for (my $i = 0; $i < $numProc; $i++) {
	
	$pm->start and next CREATE_IMG;
	
	############SPINNER#############
	my $ready : shared = 0;
	my $isOk : shared  = 0;
	my $perc : shared  = 0;

	if($i == 0){
	async {
	    local $| = 1;
	    while ( !$ready && $i == 0) {
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
  	}
	############SPINNER#############
	
	my $ini = $i * $delta;   
	my $fin = ($i + 1) * $delta - 1;
	
	if ( ($i + 1) == $numProc ) {
		$fin = $totalFiles;
	}
	
	my @filesIn = @files[$ini..$fin];
	#print "$i : $ini" . '-->' .  $fin . " ". $#filesIn . "\n";
	
	my $index = 0;
	foreach	my $file (@filesIn) {
		#print "$i : $file\n";
		$isOk ^= 1; # SPINNER
		$perc = int($index++ / $#filesIn * 100);
		
		my $path_file = "$sim_data_results_directory/$file";
		my $file_img = $file;
	
		$file_img =~ /(\d+)\..*/;
		$file_img = "$1.png";
	
		my $t = $1;
		$t += 0;
	
		# El parámetro -t indica el tiempo de simulación que
		# corresponde al archivo de log.
		my $cmd = "cat $path_file | $plot_latlong_script -t $t > $img_results_directory/$file_img";
		#print "$cmd\n";
		`$cmd`;
		
		$isOk ^= 1; # SPINNER
	}
	
	$isOk = 0;  # SPINNER
	$ready = 1; # SPINNER
	
	if($i == 0){            # SPINNER
		yield while $ready; # SPINNER
	}                       # SPINNER

	$pm->finish;
}

$pm->wait_all_children;


exit;

sub usage {
  die "Uso: $0 -r <dir resultados simulación> -p <num_proc>\n\n\tDirectorio debe ser relativo (al directorio de este script) o absoluto.\n\n";
}

