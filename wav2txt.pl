#!/usr/bin/perl

use Audio::Wav ;

############
# config 
############

$inputfile = "C:\\robapps\\devel\\perldev\\avr_tnc\\packet2.wav" ;

$wav = new Audio::Wav ;
$wr = $wav -> read( $inputfile ) ;
$sp = $wr -> length_samples() ;
$lastfive = 0 ;

for ( $s = 0 ; $s < $sp ; $s++ )
{
  $voltage = $wr -> read() ;
  #print "Sample $s = $voltage\n" ;
  $lastfive += $voltage ;
  if ( ($s % 5) == 0 )
  { 
    print int(($lastfive / 5) /256 ) . "\n" ;
    $lastfive = 0 ;
  }
}

close ;
exit ;

# eof 