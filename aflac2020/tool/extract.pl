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

print "angL,angR\n";
my @angLval = (0);
my @angRval = (0);
while ( <ifd> ) {
    findCalc("angL",10,\@angLval);
    findCalc("angR",10,\@angRval);
    if ( /DataLogger:angR =/ ) {
	my @vals = ($#angLval,$#angRval); # $#distval,$#locXval,$#locYval,$#colRval, $#colGval, $#colBval,
	my $i;
	for ( $i = 1; $i <= $#vals; ++$i ) {
	    if ( $vals[$i-1] != $vals[$i] ) {
		die "unmatch numbers ".pack("i");
	    }
	}
	my $i;
	for ( $i = 0; $i <= $vals[0]; ++$i ) {
	    print "$angLval[$i],$angRval[$i]\n"
	}
    }
}
close(ifd);

sub findCalc
{
    my $pat   = shift;
    my $offs  = shift;
    my $ref   = shift;
    my @arr   = @$ref;
    if ( /DataLogger:$pat = (-?[0-9]+) ([^\r\n]+)[\r\n]*$/ ) {
	my $res = $1;
	my @hist = unpack("C*",$2);
	my $val;
	if ( $#arr == -1 ) {
	    $val = 0;
	} else {
	    $val = pop(@arr);
	}
	my $i;
	for ( $i = 0; $i <= $#hist; ++$i ) {
	    $val += $hist[$i] - 0x22 - $offs;
	    $ref->[$i] = $val;
	}
	die "differnet $pat $val != $res" if ( $val != $res );
    }
}
