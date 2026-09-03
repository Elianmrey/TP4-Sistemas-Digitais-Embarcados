`timescale 1ns/1ps
// ============================================================================
// SPI slave dedicado a Raspberry Pi 4.
// Modo SPI 0: Raspberry Pi = master; Tang Nano 4K = slave.
//
// FRAME_SIZE = 14 bytes (112 bits):
//   [0]     = 0xA5
//   [1:2]   = BMP180 temperatura em 0,1 C (16 bits, signed)
//   [3:4]   = LM35 processado em 0,1 C (16 bits)
//   [5]     = potenciometro processado em % (8 bits)
//   [6:7]   = LM35 ADC bruto (16 bits; apenas 10 LSB validos)
//   [8:9]   = POT ADC bruto (16 bits; apenas 10 LSB validos)
//   [10:13] = BMP180 pressao compensada em Pa (32 bits, unsigned)
// A pressão é anexada ao final para não modificar nenhum campo existente.

//
// A interface SPI externa continua totalmente independente do SPI do MCP3008.
// ============================================================================
module spi_slave_pi4 (
    input  wire       sys_clk,
    input  wire       rst_n,
    input  wire       arm_spi_sclk,
    input  wire       arm_spi_mosi,
    output wire       arm_spi_miso,
    input  wire       arm_spi_cs_n,
        input wire [15:0] bmp_temp,
    input wire [31:0] bmp_pressure_pa,

    input  wire [15:0] lm35_temp_x10,
    input  wire [7:0]  pot_percent,
    input  wire [15:0] lm35_raw,
    input  wire [15:0] pot_raw,
    output reg        frame_valid,
    output reg [7:0]  command_byte
);
    reg sclk_meta, sclk_sync, sclk_prev;
    reg mosi_meta, mosi_sync;
    reg cs_meta, cs_sync, cs_prev;
        reg [111:0] tx_shift;
    reg [111:0] rx_shift;
    reg [7:0] bit_count;

    reg miso_reg;

    wire cs_fall   = cs_prev && !cs_sync;
    wire cs_rise   = !cs_prev && cs_sync;
    wire sclk_rise = !sclk_prev && sclk_sync;
    wire sclk_fall = sclk_prev && !sclk_sync;

    assign arm_spi_miso = miso_reg;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_meta   <= 1'b0;
            sclk_sync   <= 1'b0;
            sclk_prev   <= 1'b0;
            mosi_meta   <= 1'b0;
            mosi_sync   <= 1'b0;
            cs_meta     <= 1'b1;
            cs_sync     <= 1'b1;
            cs_prev     <= 1'b1;
                        tx_shift    <= 112'd0;
            rx_shift    <= 112'd0;
            bit_count   <= 8'd0;

            miso_reg    <= 1'b0;
            frame_valid <= 1'b0;
            command_byte <= 8'd0;
        end else begin
            sclk_meta <= arm_spi_sclk;
            sclk_sync <= sclk_meta;
            sclk_prev <= sclk_sync;

            mosi_meta <= arm_spi_mosi;
            mosi_sync <= mosi_meta;

            cs_meta <= arm_spi_cs_n;
            cs_sync <= cs_meta;
            cs_prev <= cs_sync;

            frame_valid <= 1'b0;

            if (cs_fall) begin
                // Frame de 112 bits transmitido MSB-first.
                tx_shift <= {
                    8'hA5,
                    bmp_temp,
                    lm35_temp_x10,
                    pot_percent,
                    lm35_raw,
                    pot_raw,
                    bmp_pressure_pa
                };
                rx_shift  <= 112'd0;
                bit_count <= 8'd0;

                // MSB do 0xA5 pronto antes da primeira subida.
                miso_reg <= 1'b1;
            end else if (cs_rise) begin
                                command_byte <= rx_shift[111:104];

                                frame_valid  <= (bit_count >= 8'd111);

                miso_reg     <= 1'b0;
            end else if (!cs_sync) begin
                if (sclk_rise) begin
                                        rx_shift <= {rx_shift[110:0], mosi_sync};
                    if (bit_count != 8'd111)

                        bit_count <= bit_count + 1'b1;
                end

                if (sclk_fall) begin
                                        tx_shift <= {tx_shift[110:0], 1'b0};
                    miso_reg <= tx_shift[110];

                end
            end
        end
    end
endmodule
