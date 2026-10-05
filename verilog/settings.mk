# HOST: Build host
# ================
# Sets up some build parameters according to the environment used.
# Possible values:
# CYGWIN (also use for MINGW64)
# LINUX (also use for a pure WSL environment)
# WSL (calls Xilinx tools in WSL since ISE is terminally broken on Windows 11)
HOST = LINUX

# XILINX_HOME, XILINX_TARGET, INTEL_BIN:
# Set paths to Xilinx/Intel tools. Adjust these for your environment.
# ===================================================================
XILINX_HOME = /opt/Xilinx/14.7/ISE_DS
XILINX_TARGET = lin64
INTEL_BIN = ~/altera_lite/25.1std/quartus/bin

# specify number of concurrent SmartXPlorer runs
XPLORER_CPUS = 8

# Allow overriding settings in an optional custom `settings.local.mk` file
-include ../settings.local.mk
