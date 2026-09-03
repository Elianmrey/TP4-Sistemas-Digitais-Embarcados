`timescale 1ns/1ps
module tb_sensor_dsp;
    reg clk=1'b0, rst_n=1'b0, sample_valid=1'b0;
    reg [9:0] lm35_adc=10'd0, pot_adc=10'd0;
    wire [15:0] lm35_temp_x10;
    wire [7:0] pot_percent;
    wire [15:0] lm35_raw,pot_raw;
    wire data_valid;
    reg failed;

    always #5 clk=~clk;

    sensor_dsp dut(
        .clk(clk),.rst_n(rst_n),.sample_valid(sample_valid),
        .lm35_adc(lm35_adc),.pot_adc(pot_adc),
        .lm35_temp_x10(lm35_temp_x10),.pot_percent(pot_percent),
        .lm35_raw(lm35_raw),.pot_raw(pot_raw),.data_valid(data_valid)
    );

    task automatic apply_sample;
        input [9:0] t_adc;
        input [9:0] p_adc;
        begin
            @(negedge clk);
            lm35_adc=t_adc;
            pot_adc=p_adc;
            sample_valid=1'b1;
            @(posedge clk);
            #1;
            if (!data_valid) begin
                $display("FAIL DSP: data_valid nao emitido");
                failed=1'b1;
            end
            @(negedge clk);
            sample_valid=1'b0;
        end
    endtask

    initial begin
        $dumpfile("build/tb_sensor_dsp.vcd");
        $dumpvars(0,tb_sensor_dsp);
        failed=1'b0;
        #20 rst_n=1'b1;

        // LM35 ~25,0 C; POT ~50%.
        apply_sample(10'd77,10'd512);

        if (lm35_temp_x10<16'd24 || lm35_temp_x10>16'd26) begin
            $display("FAIL DSP LM35=%0d.%0d C",lm35_temp_x10/10,lm35_temp_x10%10);
            failed=1'b1;
        end else
            $display("PASS DSP LM35=%0d.%0d C",lm35_temp_x10/10,lm35_temp_x10%10);

        if (pot_percent<8'd49 || pot_percent>8'd51) begin
            $display("FAIL DSP POT=%0d%%",pot_percent);
            failed=1'b1;
        end else
            $display("PASS DSP POT=%0d%%",pot_percent);

        if (lm35_raw!==16'd77 || pot_raw!==16'd512) begin
            $display("FAIL DSP RAW lm35=%0d pot=%0d",lm35_raw,pot_raw);
            failed=1'b1;
        end

        apply_sample(10'd0,10'd1023);
        if (lm35_temp_x10!==16'd0 || pot_percent!==8'd100) begin
            $display("FAIL DSP full-scale/zero");
            failed=1'b1;
        end

        if (failed) $fatal(1,"tb_sensor_dsp: falha");
        $display("PASS tb_sensor_dsp");
        $finish;
    end
endmodule
