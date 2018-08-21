#!/usr/bin/env perl

use strict;

use File::Basename;
use Cwd 'abs_path'; 

use GD;
use Tk;
use Tk::JPEG;
use Tk::PNG;

use Tk::Animation;
use Tk::MultiMediaControls;

# Directorio base de este script
my $base_path = dirname(abs_path($0));
require "$base_path/global.pm";



# Las imagenes a cargar están definidas en
# $global::sim_img_results_directory

if (! -e $global::sim_img_results_directory) {
	die "Directorio $global::sim_img_results_directory no existe.\n";
}

my @files;
opendir(DIR,$global::sim_img_results_directory);
my @files = grep { /\.png$/}  readdir(DIR);
closedir(DIR);
@files = sort {$a <=> $b}  @files;


my $main = new MainWindow;

my $label;

my $animate;
$animate = $main->Animation;

foreach (@files) {
        $animate->add_frame($main->Photo(-file => $global::sim_img_results_directory . $_));
    }

$animate->set_image(0);

$label = $main->Label(-image => $animate)->pack;;
	
my $mmc = $main->MultiMediaControls(

    # Define, from left to right, the window's controller buttons.

    -buttons                     => [ qw/ home rewind play stop fastforward / ],

    # Define callbacks for the buttons' various states.

    -fastforwardhighlightcommand => [ $animate => 'fast_forward',   4 ],
    -fastforwardcommand          => [ $animate => 'fast_forward',   1 ],
    -homecommand                 => [ $animate => 'set_image',      0 ],
    -pausecommand                => [ $animate => 'pause_animation'   ],
    -playcommand                 => [ $animate => 'resume_animation'  ],
    -rewindhighlightcommand      => [ $animate => 'fast_reverse',  -4 ],
    -rewindcommand               => [ $animate => 'fast_reverse',   1 ],
    -stopcommand                 => [ $animate => 'stop_animation'    ],

    # Define callbacks for the left and right arrow keys.

    -leftcommand                 => [ $animate => 'prev_image'        ],
    -rightcommand                => [ $animate => 'next_image'        ],

)->pack;

MainLoop;