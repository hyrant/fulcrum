#!/usr/bin/perl

use strict;

open(EXPORTSC, ">", "exports.c") or die;
open(EXPORTSLD, ">", "exports.ld") or die;
open(APILD, ">", "../api.ld") or die;

my @export;
while (<>) {
    chomp;
    s/#.*$//;
    s/^\s+//;
    s/\s+$//;
    next if (/^\s*$/);
    
    my ($kernel,$user) = split(/,/);
    $kernel =~ s/^\s+//;
    $kernel =~ s/\s+$//;
    $user = $kernel if (not defined($user) or $user =~ /^\s*$/);
    $user =~ s/^\s+//;
    $user =~ s/\s+$//;
    
    push @export, {
        "kernel" => $kernel,
        "user" => $user,
    };
}
my $KERNEL_SIZE = 24 * 1024;
my $ORIGIN = 0x08000000 + $KERNEL_SIZE;

printf EXPORTSLD q!
    .apiexport 0x%08X : {
        KEEP(*(.export))
    } >rom
!, $ORIGIN - 4 * scalar(@export);

printf APILD q!
SECTIONS {
    .apiimport 0x%08X (NOLOAD) : {
        KEEP(*(.import))
    } >rom
}
!, $ORIGIN - 4 * scalar(@export);

print EXPORTSC q!
void _api_exports(void) __attribute__((naked, section(".export")));
void _api_exports(void)
{
    asm volatile(
!;

my $location = 0x08000000 + $KERNEL_SIZE - 4 * scalar(@export);
foreach my $output (reverse(@export)) {
    print EXPORTSC '"B.W ' . $output->{kernel} . ' \n"'."\n";
    printf APILD "PROVIDE( %s = 0x%08X );\n", $output->{user}, $location|1;
    
    $location += 4;
}

print EXPORTSC q!
    );
}
!;

close(APILD);
close(EXPORTSC);
close(EXPORTSLD);
