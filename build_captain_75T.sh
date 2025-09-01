#!/bin/bash
git pull
source /tools/Xilinx/Vivado/2024.2/settings64.sh
rm -rf pcileech_enigma_x1
vivado -mode batch -source vivado_generate_project_captain_75T.tcl
vivado -mode batch -source vivado_build_captain_75T.tcl
rm -rf vivado*.log
rm -rf vivado*.jou
