# Filelist for VCS simulation

# Compilation Flags
#-timescale=1ns/10ps
#-sverilog


#+define+BOOT
#+define+VCS_SIM   
#+define+USE_SRAM
#+define+tracer
#+define+debug
+define+PD_BUILD

# lib that many module accesses should be compiled first
soc/core/lib.sv
soc/debug/debug_pkg.sv

# Core files
soc/core/alignment_units.sv
soc/core/alu_control.sv
soc/core/alu.sv
soc/core/branch_controller.sv
soc/core/csr_file.sv
soc/core/imm_gen.sv
soc/core/main_control.sv
soc/core/reg_file.sv
soc/core/rom.sv
soc/core/forwarding_unit.sv
soc/core/hazard_controller.sv
soc/core/pipeline_controller.sv
soc/core/decompressor.sv
soc/core/iadu.sv
soc/core/atomic_extension.sv
soc/core/data_path.sv
soc/core/control_unit.sv
soc/core/core_dbg_fsm.sv
soc/core/rv32i_top.sv

# Wishbone interconnect files
soc/WishboneInterconnect/wb_intercon_1.2.2-r1/wb_mux.v
soc/WishboneInterconnect/wb_intercon.sv
#soc/WishboneInterconnect/wb_intercon.svh
soc/WishboneInterconnect/wishbone_controller.sv

# Peripheral files
soc/uncore/gpio/gpio_defines.v
soc/uncore/gpio/bidirec.sv
soc/uncore/gpio/gpio_top.sv
soc/uncore/spi/fifo4.v
soc/uncore/spi/simple_spi_top.v
soc/uncore/uart/uart_defines.v
soc/uncore/uart/raminfr.v
soc/uncore/uart/uart_receiver.v
soc/uncore/uart/uart_regs.v
soc/uncore/uart/uart_rfifo.v
soc/uncore/uart/uart_sync_flops.v
soc/uncore/uart/uart_tfifo.v
soc/uncore/uart/uart_top.v
soc/uncore/uart/uart_transmitter.v
soc/uncore/uart/uart_wb.v
soc/uncore/clint/clint_wb.sv
soc/uncore/clint/clint_top.sv
soc/uncore/ptc/ptc_defines.v
soc/uncore/ptc/ptc_top.v

# Debug Unit 
soc/debug/dtm.sv
soc/debug/dm.sv
soc/debug/debug_top.sv

# sram 
# verilog model for simulation
#soc/sram/tsmc_32k_rtl.v
#soc/sram/tsmc_8k_rtl.v
soc/core/sram_wrapper.sv

# rom
#soc/rom/tsmc_rom_1k_rtl.v

# system verilog models for prototyping
soc/core/data_mem.sv


# rv32i soc top
soc/rv32i_soc.sv

# pad library and top module file 
#pads/tpz018nv_270a/tpz018nv.v
// pads/top_rv32i_soc.sv
top_rv32i_soc.sv

# Testbench files
#tb/pkg.sv
#tb/tracer_pkg.sv
#tb/tracer.sv
#tb/rv32i_soc_tb.sv
# tb/rv32_soc_with_pad_tb.sv


# Optionally, include any other files you want for the simulation.
