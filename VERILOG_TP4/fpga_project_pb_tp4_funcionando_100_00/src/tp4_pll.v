`timescale 1ns/1ps
// ============================================================================
// TP4 CLOCK — PLL REAL DA GOWIN PARA GW1NSR-4C
//
// Entrada: 27 MHz.
// Saida TP4: 54 MHz.
// IDIV=1, FBDIV=2 e ODIV=16 resultam em:
//   Fout = 27 * 2 / 1 = 54 MHz
//   VCO  = 54 * 16 = 864 MHz
//
// Sintese: primitivo PLLVR real da Gowin.
// Simulacao: modelo deterministico que preserva a hierarquia e permite
// testbench sem depender da biblioteca proprietaria.
//
// LOCK e monitorado por LOCK_STABLE_CYCLES consecutivos. O guia Gowin
// recomenda monitorar LOCK continuamente por pelo menos 2 ms.
// ============================================================================

module tp4_pll #(
    parameter integer LOCK_STABLE_CYCLES = 108000
)(
    input  wire clk_in,
    input  wire rst_n,
    output wire clk_out /* synthesis syn_keep=1 */,
    output wire locked_raw /* synthesis syn_keep=1 */,
    output wire locked_stable
);

`ifdef SYNTHESIS
    wire clkoutp_unused /* synthesis syn_keep=1 */;
    wire clkoutd_unused /* synthesis syn_keep=1 */;
    wire clkoutd3_unused /* synthesis syn_keep=1 */;
    wire [5:0] idsel;
    wire [5:0] fbdsel;
    wire [5:0] odsel;
    wire [3:0] psda;
    wire [3:0] dutyda;
    wire [3:0] fdly;

    assign idsel  = 6'b000000;
    assign fbdsel = 6'b000000;
    assign odsel  = 6'b000000;
    assign psda   = 4'b0000;
    assign dutyda = 4'b0000;
    assign fdly   = 4'b0000;

    PLLVR u_pllvr (
        .CLKOUT  (clk_out),
        .LOCK    (locked_raw),
        .CLKOUTP (clkoutp_unused),
        .CLKOUTD (clkoutd_unused),
        .CLKOUTD3(clkoutd3_unused),
        .VREN    (1'b1),
        .RESET   (~rst_n),
        .RESET_P (1'b0),
        .CLKIN   (clk_in),
        .CLKFB   (1'b0),
        .FBDSEL  (fbdsel),
        .IDSEL   (idsel),
        .ODSEL   (odsel),
        .PSDA    (psda),
        .DUTYDA  (dutyda),
        .FDLY    (fdly)
    );

    defparam u_pllvr.FCLKIN = "27";
    defparam u_pllvr.DYN_IDIV_SEL = "false";
    defparam u_pllvr.IDIV_SEL = 0;
    defparam u_pllvr.DYN_FBDIV_SEL = "false";
    defparam u_pllvr.FBDIV_SEL = 1;
    defparam u_pllvr.ODIV_SEL = 16;
    defparam u_pllvr.PSDA_SEL = "0000";
    defparam u_pllvr.DYN_DA_EN = "false";
    defparam u_pllvr.DUTYDA_SEL = "1000";
    defparam u_pllvr.CLKOUT_FT_DIR = 1'b1;
    defparam u_pllvr.CLKOUTP_FT_DIR = 1'b1;
    defparam u_pllvr.CLKOUT_DLY_STEP = 0;
    defparam u_pllvr.CLKOUTP_DLY_STEP = 0;
    defparam u_pllvr.CLKFB_SEL = "internal";
    defparam u_pllvr.CLKOUT_BYPASS = "false";
    defparam u_pllvr.CLKOUTP_BYPASS = "false";
    defparam u_pllvr.CLKOUTD_BYPASS = "false";
    defparam u_pllvr.DYN_SDIV_SEL = 2;
    defparam u_pllvr.CLKOUTD_SRC = "CLKOUT";
    defparam u_pllvr.CLKOUTD3_SRC = "CLKOUT";
`else
    assign clk_out = clk_in;
    assign locked_raw = rst_n;
`endif

    localparam integer LOCK_W =
        (LOCK_STABLE_CYCLES <= 2) ? 1 : $clog2(LOCK_STABLE_CYCLES);

    reg [LOCK_W-1:0] lock_count /* synthesis syn_preserve=1 */;
    reg lock_stable_reg /* synthesis syn_preserve=1 */;

    always @(posedge clk_out or negedge rst_n) begin
        if (!rst_n) begin
            lock_count      <= {LOCK_W{1'b0}};
            lock_stable_reg <= 1'b0;
        end else if (!locked_raw) begin
            lock_count      <= {LOCK_W{1'b0}};
            lock_stable_reg <= 1'b0;
        end else if (!lock_stable_reg) begin
            if (LOCK_STABLE_CYCLES <= 1) begin
                lock_stable_reg <= 1'b1;
            end else if (lock_count == LOCK_STABLE_CYCLES-1) begin
                lock_stable_reg <= 1'b1;
            end else begin
                lock_count <= lock_count + 1'b1;
            end
        end
    end

    assign locked_stable = lock_stable_reg;
endmodule
