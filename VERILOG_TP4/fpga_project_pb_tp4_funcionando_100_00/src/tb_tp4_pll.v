`timescale 1ns/1ps
module tb_tp4_pll;
    reg clk=1'b0, rst_n=1'b0;
    wire clk_out, locked_raw, locked_stable;
    always #5 clk=~clk;

    tp4_pll #(.LOCK_STABLE_CYCLES(4)) dut(
        .clk_in(clk),.rst_n(rst_n),
        .clk_out(clk_out),.locked_raw(locked_raw),.locked_stable(locked_stable)
    );

    initial begin
        $dumpfile("build/tb_tp4_pll.vcd");
        $dumpvars(0,tb_tp4_pll);
        #20 rst_n=1'b1;
        wait(locked_stable===1'b1);
        $display("PASS tb_tp4_pll: LOCK bruto e estabilidade detectados");
        #20 $finish;
    end
endmodule
