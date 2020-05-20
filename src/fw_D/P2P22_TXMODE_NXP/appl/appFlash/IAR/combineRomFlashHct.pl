use strict;

print "combineRomFlashHct.pl\n";

my $rom_hct_filename = $ARGV[0];
my $flash_hct_filename = $ARGV[1];
my $combined_hct_filename = $ARGV[2];

my $line;
my @line_parameters;
my $address = 0;

open(ROM_HCT_FILE, "$rom_hct_filename") or die "could not open $rom_hct_filename";
open(FLASH_HCT_FILE, "$flash_hct_filename") or die "could not open $flash_hct_filename";
open(COMBINED_HCT_FILE, ">$combined_hct_filename") or die "could not open $combined_hct_filename";


while($line = <ROM_HCT_FILE>) {
    @line_parameters = split(/\t/,$line);
    $line_parameters[0] =~ s/@//;
    $address = hex($line_parameters[0]);
    if($address < hex("0x4000")){
      print COMBINED_HCT_FILE $line;
    }
}

while($line = <FLASH_HCT_FILE>) {
	@line_parameters = split(/\t/,$line);
	$line_parameters[0] =~ s/@//;
	$address = hex($line_parameters[0]);
	if($address >= hex("0x4000")){
      print COMBINED_HCT_FILE $line;
	}
	
}

close(ROM_HCT_FILE);
close(FLASH_HCT_FILE);
close(COMBINED_HCT_FILE);
print "Ready";