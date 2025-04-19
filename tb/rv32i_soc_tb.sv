module rv32i_soc_tb;
    logic clk;
    logic reset_n;
    logic o_flash_sclk;
    logic o_flash_cs_n;
    logic o_flash_mosi;
    logic i_flash_miso;
    logic o_uart_tx;
    logic i_uart_rx;
    logic pwm_pad_o;
    logic tck_i;
    logic tms_i;
    logic tdi_i;
    logic tdo_o;

    parameter DMEM_DEPTH = 65536;
    parameter IMEM_DEPTH = 65536;
    parameter NO_OF_GPIO_PINS = 24;
	
logic [31:0] initial_imem [0:IMEM_DEPTH - 1];
logic [31:0] initial_dmem [0:DMEM_DEPTH - 1];

    // GPIO - Leds and Switches
    wire [NO_OF_GPIO_PINS - 1:0] en_gpio;
    wire [NO_OF_GPIO_PINS - 1:0] i_gpio;
    wire [NO_OF_GPIO_PINS - 1:0] o_gpio;

    // Dut instantiation
    rv32i_soc #(
        .IMEM_DEPTH(IMEM_DEPTH),
        .DMEM_DEPTH(DMEM_DEPTH),
        .NO_OF_GPIO_PINS(NO_OF_GPIO_PINS)
    )DUT(
        .*
    );

    `ifdef tracer 
        tracer tracer_inst (
        .clk_i(clk),
        .rst_ni(reset_n),
        .hart_id_i(1),
        .rvfi_insn_t(DUT.rv32i_core_inst.data_path_inst.rvfi_insn),
        .rvfi_rs1_addr_t(DUT.rv32i_core_inst.data_path_inst.rvfi_rs1_addr),
        .rvfi_rs2_addr_t(DUT.rv32i_core_inst.data_path_inst.rvfi_rs2_addr),
        .rvfi_rs3_addr_t(),
        .rvfi_rs3_rdata_t(),
        .rvfi_mem_rmask(),
        .rvfi_mem_wmask(),
        .rvfi_rs1_rdata_t(DUT.rv32i_core_inst.data_path_inst.rvfi_rs1_rdata),
        .rvfi_rs2_rdata_t(DUT.rv32i_core_inst.data_path_inst.rvfi_rs2_rdata),
        .rvfi_rd_addr_t(DUT.rv32i_core_inst.data_path_inst.rvfi_rd_addr),
        .rvfi_rd_wdata_t(DUT.rv32i_core_inst.data_path_inst.rvfi_rd_wdata),
        .rvfi_pc_rdata_t(DUT.rv32i_core_inst.data_path_inst.rvfi_pc_rdata),
        .rvfi_pc_wdata_t(DUT.rv32i_core_inst.data_path_inst.rvfi_pc_wdata),
        .rvfi_mem_addr(DUT.rv32i_core_inst.data_path_inst.rvfi_mem_addr),
        .rvfi_mem_wdata(DUT.rv32i_core_inst.data_path_inst.rvfi_mem_wdata),
        .rvfi_mem_rdata(DUT.rv32i_core_inst.data_path_inst.rvfi_mem_rdata),
        .rvfi_valid(DUT.rv32i_core_inst.data_path_inst.rvfi_valid)
        );
    `endif



    // Clock generator 
    initial clk = 0;
    always #5 clk = ~clk;

    // signal geneartion here
    initial begin 
        reset_n = 0;
        repeat(2) @(negedge clk);
        reset_n = 1; // dropping reset after two clk cycles
    end


   // initializing the instruction memory after every reset
   initial begin
        `ifdef USE_SRAM
            $readmemh("tb/machine.hex", DUT.inst_mem_inst.tsmc_32k_inst.u0.mem_core_array);
//$readmemh("tb/machine.hex", DUT.inst_mem_inst.memory);
        `else
            $readmemh("/home/qamar/Documents/verif/riscv-dv/out_2025-04-06/asm_test/inst_formatted.hex", initial_imem);
            $readmemh("/home/qamar/Documents/verif/riscv-dv/out_2025-04-06/asm_test/data_formatted.hex", initial_dmem);
		force DUT.inst_mem_inst.dmem = initial_imem;
                force DUT.data_mem_inst.dmem = initial_dmem;
		#1; 
		release DUT.inst_mem_inst.dmem;
		release DUT.data_mem_inst.dmem;
        `endif
        `ifndef VCS_SIM
            for (int i = 0; i<DMEM_DEPTH; i = i+1) 
            begin 
                force DUT.data_mem_inst.dmem = 0; 
            end
            #1;
            for (int i = 0; i<DMEM_DEPTH; i = i+1) 
            begin 
                release DUT.data_mem_inst.dmem;
            end
        `endif
    // $readmemh("/home/it/Documents/RVSOC-FreeRTOS-Kernel-DEMO/instr_formatted.hex",DUT.inst_mem_inst.dmem); // VIVADO
    // $readmemh("/home/it/Documents/RVSOC-FreeRTOS-Kernel-DEMO/data_formatted.hex",DUT.data_mem_inst.dmem); // VIVADO
   end // wait 

   initial begin 
    //    repeat(100000) @(posedge clk);
    //    for(int i = 0; i<= 14'h0fff; i = i+1) begin 
    //        $display("imem[%02d] = %8h", i, DUT.inst_mem_inst.memory[i]);
    //    end
   `ifdef tracer     
	repeat(500000) @(posedge clk);
    `else 
	repeat(5000) @(posedge clk);  
    `endif
       for(int i = 0; i < 100; i = i+1) begin 
            `ifdef USE_SRAM
                $display("dmem[%02d] => %8h <=> %8h <= imem[%02d] ", i, DUT.data_mem_inst.tsmc_8k_inst.u0.mem_core_array[i], DUT.inst_mem_inst.tsmc_32k_inst.u0.mem_core_array[i], i);
            `else 
                $display("dmem[%02d] => %8h <=> %8h <= imem[%02d] ", i, DUT.data_mem_inst.dmem[i], DUT.inst_mem_inst.dmem[i], i);
            `endif
       end
        for(int i = 0; i<32; i = i+1) begin 
            $display("reg_file[%02d] = %03d", i, DUT.rv32i_core_inst.data_path_inst.reg_file_inst.reg_file[i]);
        end
       $finish;
   end
initial begin
    `ifdef VCS_SIM    
        $dumpfile("waveform.vcd");
        $dumpvars(0, DUT);
    `endif 
//   $dumpvars(0, DUT.data_mem_inst);
//   $dumpvars(0, DUT.inst_mem_inst);
end


endmodule

