`timescale 1ns/1ps
// ============================================================================
// TP4-1.1 — Unidade aritmetica adicional.
// Operacoes unsigned de 16 bits:
//   00 = soma
//   01 = subtracao
//   10 = multiplicacao
//
// A interface permanece a mesma do projeto anterior.
// Overflow:
//   soma       -> carry em bit 16
//   subtracao  -> underflow/borrow (a < b)
//   multiplicacao -> resultado de 32 bits, sem overflow por definicao
// ============================================================================

module tp4_alu(
    input  wire [1:0]  op,
    input  wire [15:0] a,
    input  wire [15:0] b,
    output reg  [31:0] y,
    output wire        overflow
);
    always @* begin
        case (op)
            2'b00: y = {16'd0, a} + {16'd0, b};
            2'b01: y = {16'd0, a} - {16'd0, b};
            2'b10: y = a * b;
            default: y = 32'd0;
        endcase
    end

    assign overflow =
        (op == 2'b00) ? (y[31:16] != 16'd0) :
        (op == 2'b01) ? (a < b) :
                        1'b0;
endmodule
