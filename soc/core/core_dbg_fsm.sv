import debug_pkg::*;
module core_dbg_fsm (
    input logic clk_i, 
    input logic reset_i,
    input logic ebreak_inst_id,
    input logic dbg_resumereq_i, 
    input logic dbg_haltreq_i,

    output logic core_resumeack_o,
    output logic core_running_o,
    output logic core_halted_o,

    input logic [31:0] current_pc_id,
    input logic [31:0] next_pc_if1,
    input logic        fetch_busy_i,
 
    output logic [31:0] dcsr_o, 
    output logic [31:0] dpc_o,

    // abstract register access interface 
    input  logic        dbg_ar_en,
    input  logic        dbg_ar_wr,
    input  logic [15:0] dbg_ar_ad,
    input  logic [31:0] dbg_ar_do
);

    logic [31:0] dpc, dcsr;
    dcause_e debug_cause;
    logic debug_step;
    

    enum logic [1:0] {RUNNING, HALTED, RESUME} pstate, nstate;
    always_ff@(posedge clk_i or posedge reset_i)
    begin
        if(reset_i)
            pstate <= RUNNING;
        else
            pstate <= nstate;
    end
    always_comb
    begin
        case(pstate)
            RUNNING: nstate = (dbg_haltreq_i || (ebreak_inst_id && dcsr[15]) || (debug_step && !fetch_busy_i))? HALTED : RUNNING;
            HALTED:	nstate = dbg_resumereq_i? RESUME : HALTED;
            RESUME: nstate = dbg_resumereq_i? RESUME : RUNNING;
            default: nstate = RUNNING;
        endcase
    end
    assign core_resumeack_o = (pstate == RESUME);
    assign core_running_o = (pstate == RUNNING);
    assign core_halted_o = ((pstate == HALTED) || (pstate == RESUME));

    //dcsr
    always_ff@(posedge clk_i or posedge reset_i)
    begin
        if(reset_i)
            debug_cause <= NO_DBG_CAUSE;
        else if(pstate == RUNNING && ebreak_inst_id && dcsr[15])
            debug_cause <= DBG_EBREAK;
        else if(pstate == RUNNING && dbg_haltreq_i)
            debug_cause <= DBG_HALTREQ;
        else if(pstate == RUNNING && debug_step)
            debug_cause <= DBG_STEP;
    end
    assign debug_step = (dcsr[2]);
    always_ff@(posedge clk_i or posedge reset_i)
    begin
            if(reset_i)
                dcsr <= 0;
            else if(dbg_ar_en && dbg_ar_wr && (dbg_ar_ad == 16'h07b0))
                dcsr <= dbg_ar_do;//add dcsr
    end
    //dpc
    always_ff@(posedge clk_i or posedge reset_i)
    begin
        if(reset_i)
            dpc <= 0;
        else if(dbg_ar_en && dbg_ar_wr && (dbg_ar_ad == 16'h07b1)) 
            dpc <= dbg_ar_do;
        else if(pstate == RUNNING && ebreak_inst_id && dcsr[15])
            dpc <= current_pc_id;
        else if(pstate == RUNNING && (dbg_haltreq_i || debug_step))
            dpc <= next_pc_if1;
    end

    assign dcsr_o = {4'd4, 12'd0, dcsr[15], 1'b0, dcsr[13:9], debug_cause, 1'b0, dcsr[4], 1'b0, dcsr[2], 2'd3};
    assign dpc_o  = dpc;


endmodule : core_dbg_fsm
