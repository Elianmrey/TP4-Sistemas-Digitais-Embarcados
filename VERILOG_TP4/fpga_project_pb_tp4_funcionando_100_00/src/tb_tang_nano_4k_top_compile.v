`timescale 1ns/1ps
module tb_tang_nano_4k_top_compile;
    reg sys_clk=1'b0, rst_n=1'b0;
    reg mcp_miso=1'b0;
    reg arm_spi_sclk=1'b0, arm_spi_mosi=1'b0, arm_spi_cs_n=1'b1;
    tri bmp_sda, bmp_scl;
    wire mcp_cs_n,mcp_sclk,mcp_mosi,arm_spi_miso;
    wire [2:0] led;

    always #5 sys_clk=~sys_clk;
    pullup(bmp_sda);
    pullup(bmp_scl);

    tang_nano_4k_top #(
        .CLK_HZ(100),
        .SPI_SAMPLE_PERIOD(20),
        .I2C_PERIOD(30),
        .I2C_CONVERSION(20),
        .I2C_HZ(10)
    ) dut(
        .sys_clk(sys_clk),.rst_n(rst_n),
        .mcp_cs_n(mcp_cs_n),.mcp_sclk(mcp_sclk),.mcp_mosi(mcp_mosi),
        .mcp_miso(mcp_miso),.bmp_sda(bmp_sda),.bmp_scl(bmp_scl),
        .arm_spi_sclk(arm_spi_sclk),.arm_spi_mosi(arm_spi_mosi),
        .arm_spi_miso(arm_spi_miso),.arm_spi_cs_n(arm_spi_cs_n),.led(led)
    );

    initial begin
        $dumpfile("build/tb_tang_nano_4k_top_compile.vcd");
        $dumpvars(0,tb_tang_nano_4k_top_compile);
        #30 rst_n=1'b1;
        #5000;
        if (^led===1'bx) $fatal(1,"Top-level: LED indefinido");
        if (mcp_cs_n===1'bx || mcp_sclk===1'bx || mcp_mosi===1'bx)
            $fatal(1,"Top-level: sinais MCP3008 indefinidos");
        $display("PASS tb_tang_nano_4k_top_compile: elaboracao concluida");
        $finish;
    end
endmodule
