use strict;
my $copy_flag = 1;

print "updateFlashLinkerFile.pl\n";

my $master_linker_filename = $ARGV[0];
my $symbols_filename = $ARGV[1];
my $linker_filename = $ARGV[2];
 

open(MASTER_LINKER_FILE, "$master_linker_filename") or die "could not open $master_linker_filename";
open(LINKER_FILE, ">$linker_filename") or die "could not open $linker_filename";
open(ROM_SYMBOLS_FILE, "$symbols_filename") or die "could not open $symbols_filename";


while(my $line = <MASTER_LINKER_FILE>){
	if($line =~ /LNKextStart/) {
		$copy_flag = 0;
		print LINKER_FILE $line;
		while(my $symbol_line = <ROM_SYMBOLS_FILE>){
			print LINKER_FILE $symbol_line;
		}
	}elsif($line =~ /LNKextEnd/) {
		$copy_flag = 1;
		print LINKER_FILE $line;
	}elsif($copy_flag == 1) {
        print LINKER_FILE $line;
    }
	
}

close(MASTER_LINKER_FILE);
close(LINKER_FILE);
close(ROM_SYMBOLS_FILE);
print "Ready";