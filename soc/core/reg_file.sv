module reg_file #(
    parameter DEPTH = 32,
    parameter WIDTH = 32
) (
    input  logic clk, 
    input  logic reset_n, 
    input  logic reg_write, 
    input  logic [4:0] raddr1, 
    input  logic [4:0] raddr2, 
    input  logic [4:0] waddr, 
    input  logic [31:0] wdata,
    output logic [31:0] rdata1,
    output logic [31:0] rdata2
);

    logic [WIDTH - 1:0] reg_file [1:DEPTH-1]; // Exclude x0 (index 0)

    int i;
    always @(posedge clk or negedge reset_n) begin 
        if (!reset_n) begin 
            for (i = 1; i < DEPTH; i = i + 1)
                reg_file[i] <= 0;
        end else if (reg_write && (waddr != 5'd0)) begin
            reg_file[waddr] <= wdata;
        end
    end

    assign rdata1 = (raddr1 == 5'd0) ? 32'd0 : reg_file[raddr1];
    assign rdata2 = (raddr2 == 5'd0) ? 32'd0 : reg_file[raddr2];

endmodule
