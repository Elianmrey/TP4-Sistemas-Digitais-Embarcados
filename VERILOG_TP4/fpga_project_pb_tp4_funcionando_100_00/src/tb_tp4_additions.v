`timescale 1ns/1ps
module tb_tp4_additions;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg en = 1'b0;
    reg [7:0] addr = 8'd0;
    wire [15:0] bram_data;

    reg [1:0] op;
    reg [15:0] a, b;
    wire [31:0] y;
    wire ov;
    reg failed;

    tp4_bram dut_bram(
        .clk(clk), .en(en), .addr(addr), .data(bram_data)
    );
    tp4_alu dut_alu(
        .op(op), .a(a), .b(b), .y(y), .overflow(ov)
    );

    task check_alu;
        input [1:0] t_op;
        input [15:0] t_a;
        input [15:0] t_b;
        input [31:0] exp_y;
        input exp_ov;
        input [255:0] label;
        begin
            op=t_op; a=t_a; b=t_b; #1;
            if ((y !== exp_y) || (ov !== exp_ov)) begin
                $display("FAIL ALU %s: y=%h exp=%h ov=%b exp=%b",
                         label, y, exp_y, ov, exp_ov);
                failed=1'b1;
            end else
                $display("PASS ALU %s: y=%h ov=%b", label, y, ov);
        end
    endtask

    initial begin
        $dumpfile("build/tb_tp4_additions.vcd");
        $dumpvars(0, tb_tp4_additions);
        failed=1'b0;

        check_alu(2'b00,16'd0,16'd0,32'd0,1'b0,"soma 0+0");
        check_alu(2'b00,16'd1,16'd1,32'd2,1'b0,"soma 1+1");
        check_alu(2'b00,16'hFFFF,16'd1,32'h00010000,1'b1,"soma overflow");
        check_alu(2'b00,16'd1234,16'd5678,32'd6912,1'b0,"soma 1234+5678");
        check_alu(2'b01,16'd10,16'd3,32'd7,1'b0,"sub 10-3");
        check_alu(2'b01,16'd0,16'd1,32'hFFFFFFFF,1'b1,"underflow 0-1");
        check_alu(2'b01,16'hFFFF,16'd1,32'h0000FFFE,1'b0,"sub FFFF-1");
        check_alu(2'b10,16'd12,16'd11,32'd132,1'b0,"mult 12x11");

        addr=8'd42;
        en=1'b1;
        @(posedge clk);
        @(negedge clk);
        if (bram_data !== 16'd42) begin
            $display("FAIL BSRAM lookup: addr=42 data=%0d", bram_data);
            failed=1'b1;
        end else
            $display("PASS BSRAM lookup: addr=42 data=%0d", bram_data);

        en=1'b0;
        if (failed)
            $fatal(1,"tb_tp4_additions: existem falhas");
        $display("PASS tb_tp4_additions: ALU e BSRAM");
        $finish;
    end
endmodule
