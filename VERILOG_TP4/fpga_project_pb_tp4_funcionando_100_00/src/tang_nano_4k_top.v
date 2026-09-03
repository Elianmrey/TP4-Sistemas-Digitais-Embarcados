`timescale 1ns/1ps
// ============================================================================
// TOP-LEVEL — Tang Nano 4K / GW1NSR-4C
//
// CAMINHO LEGADO PRESERVADO:
//   MCP3008 -> SENSOR_DSP -> SPI Slave Raspberry Pi
//   BMP180  -> SPI Slave Raspberry Pi
//
// TP4 ADICIONAL:
//   27 MHz -> PLLVR 54 MHz -> CDC -> ALU -> BSRAM
//
// A pinagem e as interfaces externas permanecem inalteradas.
// ============================================================================

module tang_nano_4k_top #(
    parameter integer CLK_HZ = 27_000_000,
    parameter integer SPI_SAMPLE_PERIOD = 1_350_000,
    parameter integer I2C_PERIOD = 27_000_000,
    parameter integer I2C_CONVERSION = 121_500,
    parameter integer I2C_HZ = 100_000
)(
    input wire sys_clk,
    input wire rst_n,
    output wire mcp_cs_n,
    output wire mcp_sclk,
    output wire mcp_mosi,
    input wire mcp_miso,
    inout wire bmp_sda,
    inout wire bmp_scl,
    input wire arm_spi_sclk,
    input wire arm_spi_mosi /* synthesis syn_keep=1 */,
    output wire arm_spi_miso,
    input wire arm_spi_cs_n,
    output wire [2:0] led
);

    wire [9:0] lm35_raw_adc;
    wire [9:0] pot_raw_adc;

    wire [15:0] lm35_temp_x10;
    wire [7:0]  pot_percent;
    wire [15:0] lm35_raw_dsp;
    wire [15:0] pot_raw_dsp;
    wire        dsp_valid;

    wire mcp_valid;
    wire bmp_valid;
    wire mcp_busy;
    wire bmp_busy;
    wire bmp_error;
    wire [31:0] bmp_pressure_pa;
    wire signed [15:0] bmp_temp;

    wire pi_frame_valid;
    wire [7:0] pi_command_byte;

    reg mcp_valid_d;
    reg [2:0] led_reg;

    // ------------------------------------------------------------------------
    // 1) MCP3008 — caminho legado.
    // ------------------------------------------------------------------------
    mcp3008_controller #(
        .CLK_HZ(CLK_HZ),
        .SAMPLE_PERIOD(SPI_SAMPLE_PERIOD)
    ) u_mcp (
        .clk(sys_clk),
        .rst_n(rst_n),
        .spi_cs_n(mcp_cs_n),
        .spi_sclk(mcp_sclk),
        .spi_mosi(mcp_mosi),
        .spi_miso(mcp_miso),
        .ch0_data(lm35_raw_adc),
        .ch7_data(pot_raw_adc),
        .data_valid(mcp_valid),
        .busy(mcp_busy)
    );

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            mcp_valid_d <= 1'b0;
        else
            mcp_valid_d <= mcp_valid;
    end

    // ------------------------------------------------------------------------
    // 2) SENSOR_DSP — mesma interface e mesma matematica externa.
    //    As multiplicacoes passam a usar MULT18X18 fisico na sintese Gowin.
    // ------------------------------------------------------------------------
    sensor_dsp u_sensor_dsp (
        .clk(sys_clk),
        .rst_n(rst_n),
        .sample_valid(mcp_valid_d),
        .lm35_adc(lm35_raw_adc),
        .pot_adc(pot_raw_adc),
        .lm35_temp_x10(lm35_temp_x10),
        .pot_percent(pot_percent),
        .lm35_raw(lm35_raw_dsp),
        .pot_raw(pot_raw_dsp),
        .data_valid(dsp_valid)
    );

    // ------------------------------------------------------------------------
    // 3) BMP180 — caminho legado preservado.
    // ------------------------------------------------------------------------
    bmp180_controller #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ),
        .TEMP_WAIT_MS(5),
        .PRESS_WAIT_MS(8)
    ) u_bmp (
        .clk(sys_clk),
        .rst_n(rst_n),
        .bmp_sda(bmp_sda),
        .bmp_scl(bmp_scl),
        .pressure_pa(bmp_pressure_pa),
        .temperature_x10(bmp_temp),
        .data_valid(bmp_valid),
        .busy(bmp_busy),
        .error(bmp_error)
    );

    // ------------------------------------------------------------------------
    // 4) SPI slave — frame externo preservado.
    // ------------------------------------------------------------------------
    spi_slave_pi4 u_pi_spi (
        .sys_clk(sys_clk),
        .rst_n(rst_n),
        .arm_spi_sclk(arm_spi_sclk),
        .arm_spi_mosi(arm_spi_mosi),
        .arm_spi_miso(arm_spi_miso),
        .arm_spi_cs_n(arm_spi_cs_n),
        .bmp_temp(bmp_temp),
        .bmp_pressure_pa(bmp_pressure_pa),
        .lm35_temp_x10(lm35_temp_x10),
        .pot_percent(pot_percent),
        .lm35_raw(lm35_raw_dsp),
        .pot_raw(pot_raw_dsp),
        .frame_valid(pi_frame_valid),
        .command_byte(pi_command_byte)
    );

    // ------------------------------------------------------------------------
    // Preservacao da logica de recepcao SPI sem alterar o frame externo.
    // As redes sao mantidas pela sintese para que a porta arm_spi_mosi e
    // o decodificador de comando nao sejam considerados logica morta.
    // ------------------------------------------------------------------------
    wire [7:0] pi_command_byte_kept /* synthesis syn_keep=1 */;
    wire       pi_frame_valid_kept /* synthesis syn_keep=1 */;
    assign pi_command_byte_kept = pi_command_byte;
    assign pi_frame_valid_kept  = pi_frame_valid;

    // ------------------------------------------------------------------------
    // 5) Evento TP4 no dominio de 27 MHz.
    // ------------------------------------------------------------------------
    reg tp4_event_toggle_27m;

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)
            tp4_event_toggle_27m <= 1'b0;
        else if (dsp_valid)
            tp4_event_toggle_27m <= ~tp4_event_toggle_27m;
    end

    // ------------------------------------------------------------------------
    // Nets TP4 preservadas para analise.
    // ------------------------------------------------------------------------
    wire tp4_clk /* synthesis syn_keep=1 */;
    wire tp4_pll_locked_raw /* synthesis syn_keep=1 */;
    wire tp4_pll_locked_stable /* synthesis syn_keep=1 */;
    wire [15:0] tp4_signature /* synthesis syn_keep=1 */;
    wire tp4_result_valid /* synthesis syn_keep=1 */;

    // ------------------------------------------------------------------------
    // 6) Arquitetura TP4 real e isolada.
    // ------------------------------------------------------------------------
    (* syn_keep = "true", keep_hierarchy = "yes" *)
    tp4_compute_top u_tp4_compute (
        .clk_27m(sys_clk),
        .rst_n(rst_n),
        .event_toggle_27m(tp4_event_toggle_27m),
        .lm35_raw_27m(lm35_raw_dsp),
        .pot_raw_27m(pot_raw_dsp),
        .tp4_clk(tp4_clk),
        .pll_locked_raw(tp4_pll_locked_raw),
        .pll_locked_stable(tp4_pll_locked_stable),
        .tp4_signature(tp4_signature),
        .tp4_result_valid(tp4_result_valid)
    );

    // ------------------------------------------------------------------------
    // 7) LEDs existentes — logica preservada.
    // ------------------------------------------------------------------------
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            led_reg <= 3'b000;
        end else begin
            if (bmp_valid)
                led_reg[0] <= ~led_reg[0];
            if (dsp_valid) begin
                led_reg[1] <= ~led_reg[1];
                led_reg[2] <= ~led_reg[2];
            end
        end
    end

    assign led = ~led_reg;
endmodule
