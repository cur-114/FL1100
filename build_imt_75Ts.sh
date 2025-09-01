#!/bin/bash
git pull
source /tools/Xilinx/Vivado/2024.2/settings64.sh
rm -rf pcileech_squirrel
vivado -mode batch -source vivado_generate_project_immortal_75Ts.tcl
vivado -mode batch -source vivado_build.tcl
rm -rf vivado*.log
rm -rf vivado*.jou
