`timescale 1ns/1ps
// ============================================================================
// TP4 COMPUTE TOP
//
// 27 MHz -> PLLVR 54 MHz -> CDC seguro -> ALU -> BSRAM.
//
// O caminho TP4 e adicional e isolado do frame SPI. O evento dos sensores e
// transportado por toggle; o barramento de dados permanece estavel porque o
// periodo entre amostras do sensor e muito maior que a latencia do CDC.
// ============================================================================

module tp4_compute_top #(
    parameter integer PLL_LOCK_STABLE_CYCLES = 108000
)(
    input  wire        clk_27m,
    input  wire        rst_n,
    input  wire        event_toggle_27m,
    input  wire [15:0] lm35_raw_27m,
    input  wire [15:0] pot_raw_27m,
    output wire        tp4_clk,
    output wire        pll_locked_raw,
    output wire        pll_locked_stable,
    output reg  [15:0] tp4_signature,
    output reg         tp4_result_valid
);

    tp4_pll #(
        .LOCK_STABLE_CYCLES(PLL_LOCK_STABLE_CYCLES)
    ) u_tp4_pll (
        .clk_in        (clk_27m),
        .rst_n         (rst_n),
        .clk_out       (tp4_clk),
        .locked_raw    (pll_locked_raw),
        .locked_stable (pll_locked_stable)
    );

    // ------------------------------------------------------------------------
    // CDC.
    // ------------------------------------------------------------------------
    reg event_meta, event_sync, event_sync_d;
    reg [15:0] lm35_meta, lm35_sync;
    reg [15:0] pot_meta, pot_sync;

    always @(posedge tp4_clk or negedge rst_n) begin
        if (!rst_n) begin
            event_meta   <= 1'b0;
            event_sync   <= 1'b0;
            event_sync_d <= 1'b0;
            lm35_meta    <= 16'd0;
            lm35_sync    <= 16'd0;
            pot_meta     <= 16'd0;
            pot_sync     <= 16'd0;
        end else begin
            event_meta   <= event_toggle_27m;
            event_sync   <= event_meta;
            event_sync_d <= event_sync;

            lm35_meta <= lm35_raw_27m;
            lm35_sync <= lm35_meta;
            pot_meta  <= pot_raw_27m;
            pot_sync  <= pot_meta;
        end
    end

    wire new_sample /* synthesis syn_keep=1 */ = event_sync ^ event_sync_d;

    // ------------------------------------------------------------------------
    // ALU TP4: soma de 16 bits.
    // ------------------------------------------------------------------------
    wire [31:0] alu_sum /* synthesis syn_keep=1 */;
    wire        alu_overflow /* synthesis syn_keep=1 */;

    tp4_alu u_tp4_alu (
        .op       (2'b00),
        .a        (lm35_sync),
        .b        (pot_sync),
        .y        (alu_sum),
        .overflow (alu_overflow)
    );

    // ------------------------------------------------------------------------
    // BSRAM TP4: endereco derivado do resultado da ALU.
    // ------------------------------------------------------------------------
    wire [7:0]  bram_addr /* synthesis syn_keep=1 */ = alu_sum[7:0];
    wire [15:0] bram_data /* synthesis syn_keep=1 */;

    (* keep_hierarchy = "yes" *)
    tp4_bram u_tp4_bram (
        .clk  (tp4_clk),
        .en   (pll_locked_stable),
        .addr (bram_addr),
        .data (bram_data)
    );

    // Observacao do resultado da BSRAM. O estado pertence ao bloco TP4 e nao
    // e inserido no protocolo externo.
    reg observe_pending;
    reg [15:0] signature_reg /* synthesis syn_preserve=1 */;

    always @(posedge tp4_clk or negedge rst_n) begin
        if (!rst_n) begin
            observe_pending <= 1'b0;
            signature_reg   <= 16'd0;
            tp4_signature   <= 16'd0;
            tp4_result_valid <= 1'b0;
        end else begin
            tp4_result_valid <= 1'b0;

            if (!pll_locked_stable) begin
                observe_pending <= 1'b0;
            end else begin
                observe_pending <= new_sample;

                if (observe_pending) begin
                    signature_reg    <= bram_data ^ alu_sum[15:0];
                    tp4_signature    <= bram_data ^ alu_sum[15:0];
                    tp4_result_valid <= 1'b1;
                end
            end
        end
    end

    // Rede preservada para analise/observacao do resultado TP4.
    wire [15:0] tp4_signature_kept /* synthesis syn_keep=1 */ = signature_reg;
endmodule
