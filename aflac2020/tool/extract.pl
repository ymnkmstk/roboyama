#!/usr/bin/perl

die "please specify log file name" if ( @ARGV == 0 );
my $logfile = $ARGV[0];
open(ifd,$logfile) || die "cannot open '$logfile'";

my @BASE64 = ();
my @BASE64STR = unpack("C*","ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/");
my $i;
for ( $i = 0; $i <= $#BASE64STR; ++$i ) {
    $BASE64[$BASE64STR[$i]] = $i < 32 ? $i : ($i - 64);
}
# for ( $i = 0; $i <= $#BASE64; ++$i ) {
#     print "$i = ".$BASE64[$i]."\n";
# }

my $distance = 0;
my $azimuth = 0;
my $locX = 0;
my $locY = 0;

print "distance,azimuth,locX,locY\n";
my $newdistance = 0;
my $newazimuth = 0;
my $newlocX = 0;
my $newlocY = 0;
my @disthist = ();
my @locXhist = ();
my @locYhist = ();
while ( <ifd> ) {
    if ( /distance = (-?[0-9]+), azimuth = (-?[0-9]+), x = (-?[0-9]+), y = (-?[0-9]+)/ ) {
	$newdistance = $1;
	$newazimuth = $2;
	$newlocX = $3;
	$newlocY = $4;
    } elsif ( /disthis2 = ([^\r\n]+)/ ) {
	@disthist = unpack("C*",$1);
    } elsif ( /locXhis2 = ([^\r\n]+)/ ) {
	@locXhist = unpack("C*",$1);
    } elsif ( /locYhis2 = ([^\r\n]+)/ ) {
	@locYhist = unpack("C*",$1);
	if ( $#disthist != $#locXhist || $#locXhist != $#locYhist ) {
	    die "unmatch numbers $#disthist, $#locXhist, $#locYhist";
	}
	my $i;
	for ( $i = 0; $i <= $#disthist; ++$i ) {
	    $distance += $BASE64[$disthist[$i]];
	    $locX += $BASE64[$locXhist[$i]];
	    $locY += $BASE64[$locYhist[$i]];
	    print "$distance, ?, $locX, $locY\n" if ( $i < $#disthist );
	}
	if ( $newdistance != $distance || $newlocX != $locX || $newlocY != $locY ) {
	    die "different $newdistance != $distance || $newlocX != $locX || $newlocY != $locY";
	}
	print "$newdistance, $newazimuth, $newlocX, $newlocY\n";
    }
}
close(ifd);
