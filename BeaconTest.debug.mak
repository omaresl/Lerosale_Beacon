###########################################################
# Makefile generated by xIDE for uEnergy                   
#                                                          
# Project: BeaconTest
# Configuration: Debug
# Generated: vie. 9. nov. 04:05:46 2018
#                                                          
# WARNING: Do not edit this file. Any changes will be lost 
#          when the project is rebuilt.                    
#                                                          
###########################################################

XIDE_PROJECT=BeaconTest
XIDE_CONFIG=Debug
OUTPUT=BeaconTest
OUTDIR=C:/Users/Omar Sevilla/Google Drive/Lerosale/CSR1010/CSR1010_BEACON
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
      i2c_comms.c\
      config.c\
      $(DBS)

KEYR=\
      beacon_CSR101x.keyr


-include BeaconTest.mak
include $(SDK)/genmakefile.uenergy
