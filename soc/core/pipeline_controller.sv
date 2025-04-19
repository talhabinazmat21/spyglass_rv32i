module pipeline_controller (
    input logic load_hazard,
    input logic branch_hazard,
    input logic stall_pipl,
    input logic atomic_unit_stall,
    input logic atomic_unit_hazard,
    input logic trap,
    input logic trap_ret,

    output logic if_id_reg_clr, 
    output logic id_exe_reg_clr,
    output logic exe_mem_reg_clr,
    output logic mem_wb_reg_clr,

    output logic if_id_reg_en, 
    output logic id_exe_reg_en,
    output logic exe_mem_reg_en,
    output logic mem_wb_reg_en,
    output logic pc_reg_en
);

    assign if_id_reg_clr = branch_hazard | trap | trap_ret;
    assign id_exe_reg_clr = (exe_mem_reg_en & load_hazard) | branch_hazard | trap;
    assign exe_mem_reg_clr = branch_hazard | trap | atomic_unit_hazard;
    assign mem_wb_reg_clr = trap | atomic_unit_stall; 

    assign if_id_reg_en   = ~(stall_pipl | load_hazard | atomic_unit_hazard| atomic_unit_stall);
    assign id_exe_reg_en  = ~(stall_pipl |               atomic_unit_hazard| atomic_unit_stall);
    assign exe_mem_reg_en = ~(stall_pipl |                                   atomic_unit_stall);
    assign mem_wb_reg_en  = 1'b1; 
    assign pc_reg_en      = ~(stall_pipl | load_hazard | atomic_unit_hazard | atomic_unit_stall);

endmodule 