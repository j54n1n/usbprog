#!/usr/bin/perl

use warnings;
use strict;
use Time::HiRes qw< sleep time >;
use Term::ANSIColor  qw(:constants);


my $firmware;
my $n = 1;
my @firmwares = qw< JTAGICE2 XSVF Player at89prog avrispmk2 blinkdemo openocd
simpleport usbprogrs232 >;

$|++;

if(defined $ARGV[0]) {
  foreach my $f (@firmwares) {
    if($ARGV[0] =~ /$f/i) {
      $firmware = $f;
      last;
    }
  }
}

if(!$firmware) {
  print STDERR "$0: firmware\n\n";
  print STDERR "firmware may be:\n";
  foreach my $f (@firmwares) {
    print STDERR "  - $f\n";
  }
  exit 1;
} else {
  print "Selected firmware: $firmware\n\n";
}

my $line;
while($line = <STDIN>) {
  my $t = time();
  chomp $line;
  exit 0 if $line =~ /q/;

  my $i = system("
    avrdude -p m32 -c avrispv2 -P usb -B 10 -U lfuse:w:0xa0:m -U hfuse:w:0xd8:m &&
    avrdude -p m32 -V -c avrispv2 -P usb -B 0.8 -U flash:w:main.hex -e"
  );

  if($i != 0) {
    print RED "Could not flash firmware.\n.", RESET;
    next;
  }

  my $fail = 1;
  for(my $i = 0; $i < 40; $i++)
  {
    sleep 0.1;

    my $cmd = `usbprog devices`;

    if($cmd =~ /\[\s*(\d+)\](.*?)c620$/m) {
      print "Upload $firmware... ";
      system("usbprog device $1 upload $firmware");
      print GREEN "Success. $n (". (time()-$t) .")", RESET;
      $n++;
      $fail = 0;
    }
  }
  if($fail)
  {
    print RED "Fail: ", RESET;
  }
}
