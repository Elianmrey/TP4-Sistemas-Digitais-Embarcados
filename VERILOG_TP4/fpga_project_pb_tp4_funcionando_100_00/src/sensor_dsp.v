`timescale 1ns/1ps
// ============================================================================
// SENSOR_DSP
// Processamento digital dos sinais do MCP3008 na Tang Nano 4K.
//
// PRESERVACAO TP3:
// - Interface externa preservada.
// - Larguras e escalas preservadas.
// - Latencia funcional preservada: resultado registrado no clock em que
//   sample_valid esta ativo.
// - Nenhum frame SPI e alterado.
//
// TP4:
// - As multiplicacoes LM35 e POT utilizam o primitivo DSP MULT18X18 da Gowin
//   durante a sintese para GW1NSR-4C.
// - Para simulacao RTL, o mesmo wrapper usa uma multiplicacao comportamental,
//   evitando dependencia da biblioteca de primitivas Gowin.
// ============================================================================

(* USE_DSP = "yes", keep_hierarchy = "yes" *)
module sensor_dsp #(
    parameter integer ADC_MAX = 1023,
    parameter integer LM35_SCALE_X10 = 3300,
    parameter integer POT_SCALE_PERCENT = 100
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sample_valid,
    input  wire [9:0] lm35_adc,
    input  wire [9:0] pot_adc,
    output reg [15:0] lm35_temp_x10,
    output reg [7:0]  pot_percent,
    output reg [15:0] lm35_raw,
    output reg [15:0] pot_raw,
    output reg        data_valid
) /* synthesis syn_dspstyle = "DSP" */;

    localparam [17:0] LM35_SCALE_CONST = LM35_SCALE_X10;
    localparam [17:0] POT_SCALE_CONST  = POT_SCALE_PERCENT;

    wire [35:0] lm35_product /* synthesis syn_keep=1 */;
    wire [35:0] pot_product /* synthesis syn_keep=1 */;

    // Dois multiplicadores fisicos DSP.
    tp4_dsp_mult18x18 u_dsp_lm35 (
        .clk     (clk),
        .rst_n   (rst_n),
        .a       ({8'd0, lm35_adc}),
        .b       (LM35_SCALE_CONST),
        .product (lm35_product)
    );

    tp4_dsp_mult18x18 u_dsp_pot (
        .clk     (clk),
        .rst_n   (rst_n),
        .a       ({8'd0, pot_adc}),
        .b       (POT_SCALE_CONST),
        .product (pot_product)
    );

    // Arredondamento inteiro: (N + 511) / 1023.
    // O bit adicional acomoda a soma de arredondamento.
    wire [22:0] lm35_round_sum = {1'b0, lm35_product[21:0]} + 23'd511;
    wire [17:0] pot_round_sum  = pot_product[17:0] + 18'd511;

    wire [22:0] lm35_div_result = lm35_round_sum / 23'd1023;
    wire [17:0] pot_div_result  = pot_round_sum  / 18'd1023;

    wire [15:0] lm35_scaled = lm35_div_result[15:0];
    wire [16:0] pot_scaled  = pot_div_result[16:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lm35_temp_x10 <= 16'd0;
            pot_percent   <= 8'd0;
            lm35_raw      <= 16'd0;
            pot_raw       <= 16'd0;
            data_valid    <= 1'b0;
        end else begin
            data_valid <= 1'b0;

            if (sample_valid) begin
                lm35_temp_x10 <= lm35_scaled;
                pot_percent   <= (pot_scaled > 17'd100) ? 8'd100 : pot_scaled[7:0];
                lm35_raw      <= {6'd0, lm35_adc};
                pot_raw       <= {6'd0, pot_adc};
                data_valid    <= 1'b1;
            end
        end
    end
endmodule

// ============================================================================
// Wrapper do multiplicador DSP Gowin.
// Sintese Gowin/GW1NSR-4C: instancia diretamente MULT18X18.
// Simulacao RTL: usa operador * para modelar o mesmo resultado.
// ============================================================================
module tp4_dsp_mult18x18 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [17:0] a,
    input  wire [17:0] b,
    output wire [35:0] product
);
`ifdef SYNTHESIS
    wire [35:0] dsp_product /* synthesis syn_keep=1 */;

    MULT18X18 u_mult18x18 (
        .DOUT (dsp_product),
        .SOA  (),
        .SOB  (),
        .A    (a),
        .B    (b),
        .SIA  (18'd0),
        .SIB  (18'd0),
        .ASIGN(1'b0),
        .BSIGN(1'b0),
        .ASEL (1'b0),
        .BSEL (1'b0),
        .CE   (1'b1),
        .CLK  (clk),
        .RESET(~rst_n)
    );

    defparam u_mult18x18.AREG = 1'b0;
    defparam u_mult18x18.BREG = 1'b0;
    defparam u_mult18x18.OUT_REG = 1'b0;
    defparam u_mult18x18.PIPE_REG = 1'b0;
    defparam u_mult18x18.ASIGN_REG = 1'b0;
    defparam u_mult18x18.BSIGN_REG = 1'b0;
    defparam u_mult18x18.SOA_REG = 1'b0;
    defparam u_mult18x18.MULT_RESET_MODE = "ASYNC";

    assign product = dsp_product[21:0];
`else
    assign product = a * b;
`endif
endmodule
