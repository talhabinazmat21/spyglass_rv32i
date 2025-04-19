#-----------------------------------------------------------
# SpyGlass Project Setup
#-----------------------------------------------------------




set sg_proj_dir ./spyglass_project
set sg_proj_name soc_lint_analysis
set sg_log_file $sg_proj_dir/spyglass.log




file mkdir $sg_proj_dir

# Create SpyGlass project
# new_project $sg_proj_dir/$sg_proj_name


 if { [catch { new_project $sg_proj_dir/$sg_proj_name } error_string] } {
    puts "ERROR: New_Project : $error_string"
    puts "Closing current project forcefully"
    close_project -force
    puts "Creating new project..."
    new_project $sg_proj_dir/$sg_proj_name -force
 }


set_option top top_rv32i_soc

# Enable SystemVerilog and Mixed-Language Support
set_option language_mode mixed
set_option enableSV yes
set_option enableSV09 yes
#-----------------------------------------------------------
# Enable Lint + Advanced Lint
#-----------------------------------------------------------

read_file -type hdl soc/core/lib.sv
read_file -type hdl soc/debug/debug_pkg.sv
read_file -type hdl soc/core/alignment_units.sv
read_file -type hdl soc/core/alu_control.sv
read_file -type hdl soc/core/alu.sv
read_file -type hdl soc/core/branch_controller.sv
read_file -type hdl soc/core/csr_file.sv
read_file -type hdl soc/core/imm_gen.sv
read_file -type hdl soc/core/main_control.sv
read_file -type hdl soc/core/reg_file.sv
read_file -type hdl soc/core/rom.sv
read_file -type hdl soc/core/forwarding_unit.sv
read_file -type hdl soc/core/hazard_controller.sv
read_file -type hdl soc/core/pipeline_controller.sv
read_file -type hdl soc/core/decompressor.sv
read_file -type hdl soc/core/iadu.sv
read_file -type hdl soc/core/atomic_extension.sv
read_file -type hdl soc/core/data_path.sv
read_file -type hdl soc/core/control_unit.sv
read_file -type hdl soc/core/core_dbg_fsm.sv
read_file -type hdl soc/core/rv32i_top.sv
read_file -type hdl soc/WishboneInterconnect/wb_intercon_1.2.2-r1/wb_mux.v
read_file -type hdl soc/WishboneInterconnect/wb_intercon.sv
read_file -type hdl soc/WishboneInterconnect/wishbone_controller.sv
read_file -type hdl soc/uncore/gpio/gpio_defines.v
read_file -type hdl soc/uncore/gpio/bidirec.sv
read_file -type hdl soc/uncore/gpio/gpio_top.sv
read_file -type hdl soc/uncore/spi/fifo4.v
read_file -type hdl soc/uncore/spi/simple_spi_top.v
read_file -type hdl soc/uncore/uart/uart_defines.v
read_file -type hdl soc/uncore/uart/raminfr.v
read_file -type hdl soc/uncore/uart/uart_receiver.v
read_file -type hdl soc/uncore/uart/uart_regs.v
read_file -type hdl soc/uncore/uart/uart_rfifo.v
read_file -type hdl soc/uncore/uart/uart_sync_flops.v
read_file -type hdl soc/uncore/uart/uart_tfifo.v
read_file -type hdl soc/uncore/uart/uart_top.v
read_file -type hdl soc/uncore/uart/uart_transmitter.v
read_file -type hdl soc/uncore/uart/uart_wb.v
read_file -type hdl soc/uncore/clint/clint_wb.sv
read_file -type hdl soc/uncore/clint/clint_top.sv
read_file -type hdl soc/uncore/ptc/ptc_defines.v
read_file -type hdl soc/uncore/ptc/ptc_top.v
read_file -type hdl soc/debug/dtm.sv
read_file -type hdl soc/debug/dm.sv
read_file -type hdl soc/debug/debug_top.sv
read_file -type hdl soc/core/sram_wrapper.sv
read_file -type hdl soc/core/data_mem.sv
read_file -type hdl pads/tpz018nv_270a/tpz018nv.v
read_file -type hdl soc/rv32i_soc.sv
read_file -type hdl top_rv32i_soc.sv


read_file -type awl ./waiver.awl

# set_goal_option default_waiver_file ./waiver.awl

current_goal Design_Read -top top_rv32i_soc


link_design -force


current_methodology /home/icdesign/synopsys/spyglass/V-2023.12-1/SPYGLASS_HOME/GuideWare/2023.12/soc/rtl_handoff


current_goal lint/lint_abstract_validate -top top_rv32i_soc
run_goal
current_goal lint/lint_rtl -top top_rv32i_soc
run_goal
current_goal lint/lint_rtl_enhanced -top top_rv32i_soc
run_goal
current_goal lint/lint_turbo_rtl -top top_rv32i_soc
run_goal
current_goal lint/lint_functional_rtl -top top_rv32i_soc
run_goal
current_goal lint/lint_abstract -top top_rv32i_soc
run_goal
current_goal lint/lint_top_down -top top_rv32i_soc
run_goal






