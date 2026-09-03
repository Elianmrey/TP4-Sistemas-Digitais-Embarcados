`timescale 1ns/1ps
module tb_mcp3008;
    reg clk=1'b0, rst_n=1'b0, miso=1'b0;
    wire cs_n,sclk,mosi;
    wire [9:0] ch0_data,ch7_data;
    wire data_valid,busy;
    reg [4:0] edge_count;
    reg [9:0] response;
    integer transaction_count;
    reg saw_data_valid;
    reg failed;

    always #5 clk=~clk;

    always @(negedge cs_n) begin
        edge_count <= 5'd0;
        if (transaction_count==0) response <= 10'b1010101010;
        else response <= 10'b0101010101;
        transaction_count <= transaction_count+1;
        miso <= 1'b0;
    end

    // Prepara DOUT antes da borda de subida em que o DUT captura.
    // O D9 e capturado quando o contador do DUT vale 7; o D0 quando vale 16.
    always @(negedge sclk) begin
        if (!cs_n) begin
            if ((edge_count>=5'd7) && (edge_count<=5'd16))
                miso <= response[16-edge_count];
            else
                miso <= 1'b0;
        end
    end

    always @(posedge sclk)
        if (!cs_n) edge_count <= edge_count+1'b1;

    always @(posedge clk)
        if (data_valid) saw_data_valid <= 1'b1;

    mcp3008_controller #(
        .CLK_HZ(100), .SPI_HZ(5), .SAMPLE_PERIOD(20)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .spi_cs_n(cs_n), .spi_sclk(sclk), .spi_mosi(mosi), .spi_miso(miso),
        .ch0_data(ch0_data), .ch7_data(ch7_data),
        .data_valid(data_valid), .busy(busy)
    );

    initial begin
        $dumpfile("build/tb_mcp3008.vcd");
        $dumpvars(0,tb_mcp3008);
        transaction_count=0;
        edge_count=0;
        saw_data_valid=1'b0;
        failed=1'b0;

        #30 rst_n=1'b1;
        @(posedge data_valid);
        @(negedge clk);

        if (transaction_count<2) begin
            $display("FAIL MCP3008: menos de duas transacoes"); failed=1'b1;
        end
        if (ch0_data!==10'b1010101010) begin
            $display("FAIL MCP3008: CH0=%b",ch0_data); failed=1'b1;
        end
        if (ch7_data!==10'b0101010101) begin
            $display("FAIL MCP3008: CH7=%b",ch7_data); failed=1'b1;
        end
        if (!saw_data_valid) begin
            $display("FAIL MCP3008: data_valid nao observado"); failed=1'b1;
        end
        if (cs_n!==1'b1 || sclk!==1'b0) begin
            $display("FAIL MCP3008: barramento fora do repouso"); failed=1'b1;
        end

        if (failed) $fatal(1,"tb_mcp3008: falha");
        $display("PASS tb_mcp3008: CH0=%b CH7=%b transacoes=%0d",
                 ch0_data,ch7_data,transaction_count);
        $finish;
    end
endmodule
