`timescale 1ns/1ps
module tb_tp4_compute_top;
    reg clk=1'b0, rst_n=1'b0, event_toggle=1'b0;
    reg [15:0] lm35_raw=16'd10, pot_raw=16'd20;
    wire tp4_clk, pll_locked_raw, pll_locked_stable;
    wire [15:0] tp4_signature;
    wire tp4_result_valid;
    always #5 clk=~clk;

    tp4_compute_top #(.PLL_LOCK_STABLE_CYCLES(4)) dut(
        .clk_27m(clk),.rst_n(rst_n),.event_toggle_27m(event_toggle),
        .lm35_raw_27m(lm35_raw),.pot_raw_27m(pot_raw),
        .tp4_clk(tp4_clk),.pll_locked_raw(pll_locked_raw),
        .pll_locked_stable(pll_locked_stable),
        .tp4_signature(tp4_signature),.tp4_result_valid(tp4_result_valid)
    );

    initial begin
        $dumpfile("build/tb_tp4_compute_top.vcd");
        $dumpvars(0,tb_tp4_compute_top);
        #20 rst_n=1'b1;
        wait(pll_locked_stable===1'b1);
        #40 event_toggle=~event_toggle;
        wait(tp4_result_valid===1'b1);
        #1;
        // 10+20=30; LUT[30]=30; 30 XOR 30 = 0.
        if (tp4_signature!==16'd0)
            $fatal(1,"TP4 compute: assinatura=%0d",tp4_signature);
        $display("PASS tb_tp4_compute_top: PLL + ALU + BSRAM");
        $finish;
    end
endmodule
