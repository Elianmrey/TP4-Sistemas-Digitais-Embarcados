`timescale 1ns/1ps

// ============================================================================
// MCP3008 SPI Controller - Mode 0
// ============================================================================
// Sequencia de uma conversao, com START transmitido no primeiro clock:
//
//   Borda de subida (1-based):
//     1 = START
//     2 = SGL/DIFF
//     3 = D2
//     4 = D1
//     5 = D0
//     6 = ciclo de aquisicao/conversao (DOUT ainda nao e dado util)
//     7 = NULL
//     8 = D9
//     9 = D8
//    10 = D7
//    11 = D6
//    12 = D5
//    13 = D4
//    14 = D3
//    15 = D2
//    16 = D1
//    17 = D0
//
// Portanto sao necessarios 17 clocks. O controlador anterior capturava
// edge_count 6..15, isto e, NULL + D9..D1, descartando D0. Para entradas
// proximas de fundo de escala isso produz exatamente 1023 >> 1 = 511.
//
// A captura correta e edge_count 7..16 (zero-based), correspondente a
// D9..D0.
// ============================================================================
module mcp3008_controller #(
    parameter integer CLK_HZ = 27_000_000,
    parameter integer SPI_HZ = 500_000,
    parameter integer SAMPLE_PERIOD = 1_350_000
)(
    input  wire clk,
    input  wire rst_n,
    output reg  spi_cs_n,
    output reg  spi_sclk,
    output reg  spi_mosi,
    input  wire spi_miso,
    output reg [9:0] ch0_data,
    output reg [9:0] ch7_data,
    output reg       data_valid,
    output reg       busy
);

    localparam integer DIV_HALF = (CLK_HZ/(2*SPI_HZ) < 1) ? 1 : CLK_HZ/(2*SPI_HZ);
    localparam integer DIV_W = (DIV_HALF <= 2) ? 1 : $clog2(DIV_HALF);
    localparam integer TIMER_W = (SAMPLE_PERIOD <= 2) ? 1 : $clog2(SAMPLE_PERIOD+1);

    // edge_count zero-based:
    // 0..4  = START, SGL/DIFF, D2, D1, D0
    // 5     = ciclo de aquisicao/conversao
    // 6     = NULL
    // 7..16 = D9..D0
    localparam [4:0] ADC_DATA_FIRST_RISE = 5'd7;
    localparam [4:0] ADC_LAST_RISE       = 5'd16;
    localparam [4:0] ADC_FINISH_EDGE     = 5'd17;

    localparam [2:0] IDLE     = 3'd0,
                     START    = 3'd1,
                     TRANSFER = 3'd2,
                     FINISH   = 3'd3;

    reg [2:0] state;
    reg [DIV_W-1:0] div_cnt;
    reg [TIMER_W-1:0] sample_timer;
    reg [4:0] edge_count;
    reg [15:0] tx_shift;
    reg [9:0] rx_shift;
    reg channel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            div_cnt      <= 0;
            sample_timer <= 0;
            edge_count   <= 0;
            tx_shift     <= 0;
            rx_shift     <= 0;
            channel      <= 1'b0;
            spi_cs_n     <= 1'b1;
            spi_sclk     <= 1'b0;
            spi_mosi     <= 1'b0;
            ch0_data     <= 10'd0;
            ch7_data     <= 10'd0;
            data_valid   <= 1'b0;
            busy         <= 1'b0;
        end else begin
            data_valid <= 1'b0;

            case (state)
                IDLE: begin
                    spi_cs_n <= 1'b1;
                    spi_sclk <= 1'b0;
                    busy     <= 1'b0;

                    if (sample_timer == SAMPLE_PERIOD-1) begin
                        sample_timer <= 0;
                        state        <= START;
                        busy         <= 1'b1;
                    end else begin
                        sample_timer <= sample_timer + 1'b1;
                    end
                end

                START: begin
                    spi_cs_n   <= 1'b0;
                    spi_sclk   <= 1'b0;
                    edge_count <= 0;
                    rx_shift   <= 10'd0;
                    div_cnt    <= 0;

                    // Single-ended:
                    // CH0 = 11000
                    // CH7 = 11111
                    if (channel == 1'b0)
                        tx_shift <= {5'b11000, 11'b0};
                    else
                        tx_shift <= {5'b11111, 11'b0};

                    // START ja fica estavel antes da primeira subida.
                    spi_mosi <= 1'b1;
                    state    <= TRANSFER;
                end

                TRANSFER: begin
                    if (div_cnt == DIV_HALF-1) begin
                        div_cnt <= 0;

                        if (spi_sclk == 1'b0) begin
                            // SPI Mode 0: recebe na borda de subida.
                            spi_sclk <= 1'b1;

                            // Captura exatamente D9..D0.
                            if ((edge_count >= ADC_DATA_FIRST_RISE) &&
                                (edge_count <= ADC_LAST_RISE)) begin
                                rx_shift <= {rx_shift[8:0], spi_miso};
                            end

                            edge_count <= edge_count + 1'b1;
                        end else begin
                            // SPI Mode 0: prepara o proximo bit na descida.
                            spi_sclk <= 1'b0;
                            tx_shift <= {tx_shift[14:0], 1'b0};
                            spi_mosi <= tx_shift[14];

                            // Depois da descida que segue a 17a subida,
                            // todos os 10 bits D9..D0 ja foram capturados.
                            if (edge_count == ADC_FINISH_EDGE)
                                state <= FINISH;
                        end
                    end else begin
                        div_cnt <= div_cnt + 1'b1;
                    end
                end

                FINISH: begin
                    spi_cs_n <= 1'b1;
                    spi_sclk <= 1'b0;
                    busy     <= 1'b0;

                    if (channel == 1'b0) begin
                        ch0_data <= rx_shift;
                    end else begin
                        ch7_data   <= rx_shift;
                        data_valid <= 1'b1;
                    end

                    channel <= ~channel;
                    state   <= IDLE;
                end

                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule
