###########################################################
# Makefile generated by xIDE for uEnergy                   
#                                                          
# Project: BeaconTest
# Configuration: Debug
# Generated: jue 1. nov 19:47:47 2018
#                                                          
# WARNING: Do not edit this file. Any changes will be lost 
#          when the project is rebuilt.                    
#                                                          
###########################################################

XIDE_PROJECT=BeaconTest
XIDE_CONFIG=Debug
OUTPUT=BeaconTest
OUTDIR=C:/Users/uidj2522/Desktop/Beacon/CSR1010_BEACON
DEFS=

OUTPUT_TYPE=0
LIBRARY_VERSION=Auto
SWAP_INTO_DATA=0
USE_FLASH=0
ERASE_NVM=1
CSFILE_CSR100x=
CSFILE_CSR101x_A05=
MASTER_DB=
LIBPATHS=
INCPATHS=

DBS=


INPUTS=\
      beacon.c\
      $(DBS)

KEYR=\
      beacon_CSR101x.keyr


-include BeaconTest.mak
include $(SDK)/genmakefile.uenergy