#!/usr/bin/perl

die "please specify log file name" if ( @ARGV == 0 );
my $logfile = $ARGV[0];
open(ifd,$logfile) || die "cannot open '$logfile'";

# my @BASE64 = ();
# my @BASE64STR = unpack("C*","ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/");
# my $i;
# for ( $i = 0; $i <= $#BASE64STR; ++$i ) {
#     $BASE64[$BASE64STR[$i]] = $i < 32 ? $i : ($i - 64);
# }
# for ( $i = 0; $i <= $#BASE64; ++$i ) {
#     print "$i = ".$BASE64[$i]."\n";
# }

print "distance,azimuth,locX,locY,angL,angR\n";
my @distval = (0);
my @locXval = (0);
my @locYval = (0);
my @angLval = (0);
my @angRval = (0);
while ( <ifd> ) {
    findCalc("distance","disthist",46,\@distval);
    findCalc("x","locXhist",46,\@locXval);
    findCalc("y","locYhist",46,\@locYval);
    findCalc("angL","angLhist",10,\@angLval);
    findCalc("angR","angRhist",10,\@angRval);
    if ( /hsv =/ ) {
	if ( $#distval != $#locXval || $#locXval != $#locYval || $#locYval != $#angLval || $#angLval != $#angRval ) {
	    die "unmatch numbers $#distval, $#locXval, $#locYval, $#angLval, $#angRval";
	}
	my $i;
	for ( $i = 0; $i <= $#distval; ++$i ) {
	    print "$distval[$i], $locXval[$i], $locYval[$i], $angLval[$i], $angRval[$i]\n"
	}
    }
}
close(ifd);

sub findCalc
{
    my $pat1  = shift;
    my $pat2  = shift;
    my $offs  = shift;
    my $ref   = shift;
    if ( /$pat1 = (-?[0-9]+)/ ) {
	my $val = $1;
	$ref->[0] = pop(@$ref);
	$ref->[1] = $val;
    }
    if ( /$pat2 = ([^\r\n]+)/ ) {
	my @hist = unpack("C*",$1);
	my $i;
	my $val = $ref->[0];
	my $res = $ref->[1];
	for ( $i = 0; $i <= $#hist; ++$i ) {
	    $val += $hist[$i] - 0x22 - $offs;
	    $ref->[$i] = $val;
	}
	die "differnet $pat1/$pat2 $val != $res" if ( $val != $res );
    }
}
