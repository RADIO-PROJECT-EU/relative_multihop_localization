###########################################################
# Makefile generated by xIDE for uEnergy                   
#                                                          
# Project: timers
# Configuration: Debug
# Generated: ��� 4. ��� 09:23:52 2016
#                                                          
# WARNING: Do not edit this file. Any changes will be lost 
#          when the project is rebuilt.                    
#                                                          
###########################################################

XIDE_PROJECT=timers
XIDE_CONFIG=Debug
OUTPUT=timers
OUTDIR=C:/CSR_uEnergy_SDK-2.6.0.10/apps/radio
DEFS=

OUTPUT_TYPE=0
USE_FLASH=0
ERASE_NVM=1
CSFILE_CSR101x_A05=C:\CSR_uEnergy_SDK-2.6.0.10\apps\radio\timers_csr101x_A05.keyr
MASTER_DB=
LIBPATHS=
INCPATHS=
STRIP_SYMBOLS=0
OTAU_BOOTLOADER=0
OTAU_CSFILE=
OTAU_NAME=
OTAU_SECRET=
OTAU_VERSION=7

DBS=\
\
      app_gatt_db.db\
      dev_info_service_db.db\
      battery_service_db.db

INPUTS=\
      main.c\
      debug_interface.c\
      gap_service.c\
      gatt_access.c\
      dev_info_service.c\
      battery_service.c\
      nvm_access.c\
      gatt_server.c\
      buzzer.c\
      hw_access.c\
      ../../../Users/chris/workspace/CSRmesh-1.3-Examples-Applications_icp/56025_CSRmesh-1.3-Examples-Applications_icp/applications/CSRMeshLight/csr_mesh_light_hw.c\
      ../../../Users/chris/workspace/CSRmesh-1.3-Examples-Applications_icp/56025_CSRmesh-1.3-Examples-Applications_icp/applications/CSRMeshLight/iot_hw.c\
      ../../../Users/chris/workspace/CSRmesh-1.3-Examples-Applications_icp/56025_CSRmesh-1.3-Examples-Applications_icp/applications/CSRMeshLight/gunilamp_hw.c\
      ../../../Users/chris/workspace/CSRmesh-1.3-Examples-Applications_icp/56025_CSRmesh-1.3-Examples-Applications_icp/applications/CSRMeshLight/fast_pwm.c\
      ../../../Users/chris/workspace/CSRmesh-1.3-Examples-Applications_icp/56025_CSRmesh-1.3-Examples-Applications_icp/applications/CSRMeshLight/pio_ctrlr_code.asm\
      $(DBS)

KEYR=\
      timers_csr101x_A05.keyr


-include timers.mak
include $(SDK)/genmakefile.uenergy
