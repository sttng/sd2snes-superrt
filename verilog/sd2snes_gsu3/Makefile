CORE = gsu3

# FX3-only Super FX core (Super FX 3 carts, e.g. DOOM FX3): the pipelined
# gsu_fx3.v instead of gsu.v.  All sources and IP come from ../sd2snes_gsu;
# main.v and address.v pick the FX3 variant with the GSU3 Verilog macro, which
# sd2snes_gsu3.xise (mk2) and main.qsf (mk3) set.  Classic Super FX carts keep
# fpga_gsu.
VSRC_DIR = ../sd2snes_gsu

VSRC = address.v cheat.v dac.v dcm.v gsu_fx3.v main.v mcu_cmd.v msu.v sd_dma.v spi.v
VHSRC =

HEADER = config.vh

COMMON_IP = dac_buf gsu_cache gsu_fmult gsu_mult gsu_umult msu_databuf snescmd_buf

XIL_IP =
XIL_IPCORE_DIR = ../sd2snes_gsu/ip/mk2

# mk3: main.qsf in this directory pulls the sources and IP from ../sd2snes_gsu
INT_IP_DIR = ../sd2snes_gsu/ip/mk3
INT_IP = pll

include ../settings.mk
include ../common.mk
