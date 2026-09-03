`timescale 1ns/1ps
// Testbench do escravo SPI externo da Tang Nano, operando em Mode 0.
module tb_spi_slave_pi4;
    reg sys_clk = 1'b0;
    reg rst_n = 1'b0;
    reg arm_spi_sclk = 1'b0;
    reg arm_spi_mosi = 1'b0;
    reg arm_spi_cs_n = 1'b1;
    wire arm_spi_miso;

    reg [15:0] bmp_temp = 16'h1234;
    reg [31:0] bmp_pressure_pa = 32'd101325;
    reg [15:0] lm35_temp_x10 = 16'h00FB; // 25,1 C
    reg [7:0]  pot_percent = 8'd50;
    reg [15:0] lm35_raw = 16'h0055;
    reg [15:0] pot_raw = 16'h0200;

    wire frame_valid;
    wire [7:0] command_byte;
    reg [111:0] rx_frame;
    reg [111:0] master_tx_frame;
    reg sampled_miso;
    reg saw_frame_valid;
    integer i;

    always #5 sys_clk = ~sys_clk;
    always @(posedge sys_clk) if (frame_valid) saw_frame_valid <= 1'b1;

    spi_slave_pi4 dut (
        .sys_clk(sys_clk), .rst_n(rst_n),
        .arm_spi_sclk(arm_spi_sclk), .arm_spi_mosi(arm_spi_mosi),
        .arm_spi_miso(arm_spi_miso), .arm_spi_cs_n(arm_spi_cs_n),
        .bmp_temp(bmp_temp), .bmp_pressure_pa(bmp_pressure_pa),
        .lm35_temp_x10(lm35_temp_x10),
        .pot_percent(pot_percent), .lm35_raw(lm35_raw), .pot_raw(pot_raw),
        .frame_valid(frame_valid), .command_byte(command_byte)
    );

    task automatic transfer_bit;
        input mosi_bit;
        output miso_bit;
        begin
            arm_spi_mosi = mosi_bit;
            #80;
            arm_spi_sclk = 1'b1;
            #30;
            miso_bit = arm_spi_miso;
            #70;
            arm_spi_sclk = 1'b0;
            #100;
        end
    endtask

    initial begin
        $dumpfile("build/tb_spi_slave_pi4.vcd");
        $dumpvars(0, tb_spi_slave_pi4);
        // Frame esperado: [A5][1234][00FB][32][0055][0200][00018BCD].
        master_tx_frame = 112'd0;
        master_tx_frame[111:104] = 8'h3C;
        rx_frame = 112'd0;
        saw_frame_valid = 1'b0;

        #40 rst_n = 1'b1;
        #100;
        arm_spi_cs_n = 1'b0;
        #160;

        for (i = 0; i < 112; i = i + 1) begin
            transfer_bit(master_tx_frame[111-i], sampled_miso);
            rx_frame[111-i] = sampled_miso;
        end

        #100;
        arm_spi_cs_n = 1'b1;
        #160;

        if (rx_frame !== 112'hA5_1234_00FB_32_0055_0200_00018BCD)
            $fatal(1, "SPI slave: frame incorreto: recebido=%h", rx_frame);
        if (command_byte !== 8'h3C)
            $fatal(1, "SPI slave: comando incorreto: %h", command_byte);
        if (!saw_frame_valid)
            $fatal(1, "SPI slave: frame_valid nao foi emitido");
        if (arm_spi_miso !== 1'b0)
            $fatal(1, "SPI slave: MISO deveria estar baixo com CS inativo");

        $display("PASS tb_spi_slave_pi4: frame=%h comando=%h", rx_frame, command_byte);
        $finish;
    end
endmodule
