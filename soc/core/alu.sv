import riscv_types::*;

module alu (
    input alu_t alu_ctrl,
    input logic [31:0] op1,
    input logic [31:0] op2,
    output logic [31:0] alu_result, 
    output logic zero
);

    always_comb begin 
        case(alu_ctrl)
            ADD: alu_result = op1 + op2;
            SUB: alu_result = op1 - op2;
            SLT: alu_result = $signed(op1) < $signed(op2) ? 1'b1 : 1'b0;
            SLL: alu_result = op1 << op2[4:0];
            SRL: alu_result = op1 >> op2[4:0];
            SRA: alu_result = $signed(op1) >>> op2[4:0];
            XOR: alu_result = op1 ^ op2;
            AND: alu_result = op1 & op2;
            OR: alu_result  = op1 | op2;
            SLTU: alu_result = $unsigned(op1) < $unsigned(op2);
            default: alu_result = 32'dz;
        endcase
    end
    
    
    assign zero = (alu_result == 0);
endmodule

