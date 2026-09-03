`timescale 1ns/1ps
// BMP180 independente para Tang Nano 4K.
// I2C open-drain, OSS=0, pressão em Pa, temperatura em 0,1 graus Celsius.
module bmp180_controller #(
    parameter integer CLK_HZ = 27_000_000,
    parameter integer I2C_HZ = 100_000,
    parameter integer TEMP_WAIT_MS = 5,
    parameter integer PRESS_WAIT_MS = 8
) (
    input  wire clk,
    input  wire rst_n,
    inout  wire bmp_sda,
    inout  wire bmp_scl,
    output reg [31:0] pressure_pa,
    output reg signed [15:0] temperature_x10,
    output reg data_valid,
    output reg busy,
    output reg error
);
    localparam integer I2C_QUARTER = (CLK_HZ/(I2C_HZ<<2) < 1) ? 1 : CLK_HZ/(I2C_HZ<<2);
    localparam integer I2C_DIV_W = (I2C_QUARTER <= 2) ? 1 : $clog2(I2C_QUARTER);
    localparam integer TEMP_TICKS = (CLK_HZ/1000*TEMP_WAIT_MS <= 1) ? 1 : CLK_HZ/1000*TEMP_WAIT_MS;
    localparam integer PRESS_TICKS = (CLK_HZ/1000*PRESS_WAIT_MS <= 1) ? 1 : CLK_HZ/1000*PRESS_WAIT_MS;
    localparam integer WAIT_W = (PRESS_TICKS <= 2) ? 1 : $clog2(PRESS_TICKS);

    localparam [4:0] ST_IDLE=5'd0, ST_START=5'd1, ST_SEND=5'd2,
                     ST_ACK=5'd3, ST_RECV=5'd4, ST_MASTER_ACK=5'd5,
                     ST_STOP=5'd6, ST_WAIT=5'd7, ST_CALC=5'd8;

    localparam [2:0] OP_CAL_READ=3'd0, OP_TEMP_WRITE=3'd1,
                     OP_TEMP_READ=3'd2, OP_PRESS_WRITE=3'd3,
                     OP_PRESS_READ=3'd4;

    localparam [5:0] C_IDLE=6'd0, C_TEMP_X1_START=6'd1,
                     C_TEMP_X1_WAIT=6'd2, C_TEMP_X2_DIV_START=6'd3,
                     C_TEMP_X2_DIV_WAIT=6'd4, C_TEMP_X2_APPLY=6'd5,
                     C_B3_SQ_START=6'd6, C_B3_SQ_WAIT=6'd7,
                     C_B3_X1_START=6'd8, C_B3_X1_WAIT=6'd9,
                     C_B3_X2_START=6'd10, C_B3_X2_WAIT=6'd11,
                     C_B4_X1_START=6'd12, C_B4_X1_WAIT=6'd13,
                     C_B4_X2_START=6'd14, C_B4_X2_WAIT=6'd15,
                     C_B4_X3_START=6'd16, C_B4_X3_WAIT=6'd17,
                     C_B7_START=6'd18, C_P_DIV_START=6'd19,
                     C_P_DIV_WAIT=6'd20, C_P_APPLY=6'd21,
                     C_P_SQ_START=6'd22, C_P_SQ_WAIT=6'd23,
                     C_P_X1_START=6'd24, C_P_X1_WAIT=6'd25,
                     C_P_X2_START=6'd26, C_P_X2_WAIT=6'd27,
                     C_P_FINISH=6'd28, C_DONE=6'd29;

    reg [4:0] state;
    reg [1:0] phase;
    reg [3:0] bit_count;
    reg [I2C_DIV_W-1:0] i2c_div_count;
    reg [WAIT_W-1:0] wait_count;
    reg sda_low, scl_low, ack_ok;
    reg [7:0] tx_byte, rx_byte, rx_msb, rx_lsb;
    reg [2:0] op;
    reg [2:0] seq;
    reg [5:0] read_remaining;
    reg [1:0] read_index;
    reg [4:0] cal_byte_index;

    reg [7:0] cal_bytes [0:21];
    reg signed [15:0] ac1, ac2, ac3, b1, b2, mb, mc, md;
    reg [15:0] ac4, ac5, ac6;
    reg [15:0] ut_raw, up_raw;

    reg [5:0] calc_state;
    reg signed [31:0] x1, x2, b5, b6, b3, b4, temp_calc, p_calc;
    reg signed [39:0] b7;
    reg signed [31:0] b6_square;

    // Multiplicador serial: nenhum operador * é usado no datapath.
    reg [47:0] mul_mcand, mul_accum, mul_product;
    reg [23:0] mul_mplier;
    reg [4:0] mul_count;
    reg mul_neg, mul_busy;

    // Divisor restaurador compartilhado de 40 bits.
    reg [39:0] div_dividend, div_divisor, div_quotient, div_result;
    reg [40:0] div_remainder;
    reg [5:0] div_count;
    reg div_neg, div_floor, div_post_shift, div_busy;

    wire i2c_tick = (i2c_div_count == I2C_QUARTER-1);
    assign bmp_sda = sda_low ? 1'b0 : 1'bz;
    assign bmp_scl = scl_low ? 1'b0 : 1'bz;

    function signed [31:0] sx16;
        input signed [15:0] v;
        begin sx16 = v; end
    endfunction
    function signed [31:0] ux16;
        input [15:0] v;
        begin ux16 = {16'd0,v}; end
    endfunction
    function [23:0] abs24;
        input signed [31:0] v;
        begin
            if (v < 0) abs24 = (~v[23:0]) + 24'd1;
            else abs24 = v[23:0];
        end
    endfunction
    function [39:0] abs40;
        input signed [39:0] v;
        begin
            if (v < 0) abs40 = (~v[39:0]) + 40'd1;
            else abs40 = v[39:0];
        end
    endfunction
    function signed [39:0] scale50000;
        input signed [39:0] v;
        begin
            scale50000 = (v <<< 15) + (v <<< 14) + (v <<< 9) +
                         (v <<< 8) + (v <<< 6) + (v <<< 4);
        end
    endfunction
    function signed [39:0] signed_div_result;
        input [39:0] q;
        input neg;
        input floor_mode;
        input rem_nonzero;
        input post_shift;
        reg signed [39:0] t;
        begin
            t = q;
            if (neg) t = -t;
            if (floor_mode && neg && rem_nonzero) t = t - 40'sd1;
            if (post_shift) t = t <<< 1;
            signed_div_result = t;
        end
    endfunction

    wire [40:0] div_rem_shift = {div_remainder[39:0],div_dividend[39]};
    wire div_take = (div_rem_shift >= {1'b0,div_divisor});
    wire [40:0] div_rem_step = div_take ?
                                div_rem_shift - {1'b0,div_divisor} : div_rem_shift;
    wire [39:0] div_q_step = {div_quotient[38:0],div_take};
    wire [47:0] mul_accum_step = mul_accum +
                                  (mul_mplier[0] ? mul_mcand : 48'd0);
    wire signed [47:0] mul_signed_step = mul_neg ?
                                         -$signed(mul_accum_step) :
                                         $signed(mul_accum_step);
    // Recortes explicitos: equivalem ao truncamento implicito anterior,
    // mas deixam a intencao numerica clara para o sintetizador.
    wire signed [31:0] mul_shift11 = $signed(mul_signed_step[42:11]);
    wire signed [31:0] mul_shift13 = $signed(mul_signed_step[44:13]);
    wire signed [31:0] mul_shift15 = $signed(mul_signed_step[46:15]);
    wire signed [31:0] mul_shift16 = $signed(mul_signed_step[47:16]);
    wire signed [31:0] b3_sum32 = (sx16(ac1) <<< 2) + x1 + mul_shift11 + 32'sd2;
    wire signed [31:0] b3_value32 = b3_sum32 >>> 2;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            phase <= 0;
            bit_count <= 0;
            i2c_div_count <= 0;
            wait_count <= 0;
            sda_low <= 0;
            scl_low <= 0;
            ack_ok <= 0;
            tx_byte <= 0;
            rx_byte <= 0;
            rx_msb <= 0;
            rx_lsb <= 0;
            op <= OP_CAL_READ;
            seq <= 0;
            read_remaining <= 0;
            read_index <= 0;
            cal_byte_index <= 0;
            for (i=0; i<22; i=i+1) cal_bytes[i] <= 0;
            ac1<=0; ac2<=0; ac3<=0; ac4<=0; ac5<=0; ac6<=0;
            b1<=0; b2<=0; mb<=0; mc<=0; md<=0;
            ut_raw<=0; up_raw<=0;
            calc_state<=C_IDLE;
            x1<=0; x2<=0; b5<=0; b6<=0; b3<=0; b4<=0;
            b7<=0; temp_calc<=0; p_calc<=0; b6_square<=0;
            mul_mcand<=0; mul_accum<=0; mul_product<=0; mul_mplier<=0;
            mul_count<=0; mul_neg<=0; mul_busy<=0;
            div_dividend<=0; div_divisor<=0; div_quotient<=0;
            div_remainder<=0; div_result<=0; div_count<=0;
            div_neg<=0; div_floor<=0; div_post_shift<=0; div_busy<=0;
            pressure_pa<=0; temperature_x10<=0; data_valid<=0;
            busy<=1; error<=0;
        end else begin
            data_valid <= 0;
            if (state == ST_CALC) begin
                busy <= 1;
                if (mul_busy) begin
                    if (mul_count == 5'd23) begin
                        mul_product <= mul_signed_step;
                        mul_busy <= 0;
                        case (calc_state)
                            C_TEMP_X1_WAIT: begin x1<=mul_shift15; calc_state<=C_TEMP_X2_DIV_START; end
                            C_B3_SQ_WAIT: begin b6_square<=mul_signed_step[31:0]; calc_state<=C_B3_X1_START; end
                            C_B3_X1_WAIT: begin x1<=mul_shift11; calc_state<=C_B3_X2_START; end
                            C_B3_X2_WAIT: begin
                                x2<=mul_shift11;
                                b3<=b3_value32;
                                calc_state<=C_B4_X1_START;
                            end
                            C_B4_X1_WAIT: begin x1<=mul_shift13; calc_state<=C_B4_X2_START; end
                            C_B4_X2_WAIT: begin x2<=mul_shift16; calc_state<=C_B4_X3_START; end
                            C_B4_X3_WAIT: begin b4<=mul_shift15; calc_state<=C_B7_START; end
                            C_P_SQ_WAIT: begin x1<=$signed(mul_signed_step[31:0]); calc_state<=C_P_X1_START; end
                            C_P_X1_WAIT: begin x1<=mul_shift16; calc_state<=C_P_X2_START; end
                            C_P_X2_WAIT: begin x2<=mul_shift16; calc_state<=C_P_FINISH; end
                            default: begin calc_state<=C_IDLE; state<=ST_IDLE; end
                        endcase
                    end else begin
                        mul_accum<=mul_accum_step;
                        mul_mcand<=mul_mcand<<1;
                        mul_mplier<=mul_mplier>>1;
                        mul_count<=mul_count+1'b1;
                    end
                end else if (div_busy) begin
                    if (div_count == 6'd39) begin
                        div_result<=signed_div_result(div_q_step,div_neg,div_floor,
                                                      div_rem_step!=0,div_post_shift);
                        div_busy<=0;
                        case (calc_state)
                            C_TEMP_X2_DIV_WAIT: calc_state<=C_TEMP_X2_APPLY;
                            C_P_DIV_WAIT: calc_state<=C_P_APPLY;
                            default: begin calc_state<=C_IDLE; state<=ST_IDLE; end
                        endcase
                    end else begin
                        div_remainder<=div_rem_step;
                        div_quotient<=div_q_step;
                        div_dividend<={div_dividend[38:0],1'b0};
                        div_count<=div_count+1'b1;
                    end
                end else begin
                    case (calc_state)
                        C_TEMP_X1_START: begin
                            mul_mcand<={24'd0,abs24($signed({16'd0,ut_raw})-sx16(ac6))};
                            mul_mplier<=abs24(ux16(ac5)); mul_accum<=0; mul_count<=0;
                            mul_neg<=(($signed({16'd0,ut_raw})-sx16(ac6))<0);
                            mul_busy<=1; calc_state<=C_TEMP_X1_WAIT;
                        end
                        C_TEMP_X2_DIV_START: begin
                            div_dividend<=abs40(sx16(mc)<<<11);
                            div_divisor<=abs40(x1+sx16(md)); div_quotient<=0; div_remainder<=0; div_count<=0;
                            div_neg<=(sx16(mc)<0)^((x1+sx16(md))<0);
                            div_floor<=1; div_post_shift<=0; div_busy<=1; calc_state<=C_TEMP_X2_DIV_WAIT;
                        end
                        C_TEMP_X2_APPLY: begin
                            x2<=$signed(div_result[31:0]);
                            b5<=x1+$signed(div_result[31:0]);
                            temp_calc<=(x1+$signed(div_result[31:0])+32'sd8)>>>4;
                            b6<=x1+$signed(div_result[31:0])-32'sd4000;
                            calc_state<=C_B3_SQ_START;
                        end
                        C_B3_SQ_START: begin
                            mul_mcand<={24'd0,abs24(b6)}; mul_mplier<=abs24(b6);
                            mul_accum<=0; mul_count<=0; mul_neg<=0; mul_busy<=1; calc_state<=C_B3_SQ_WAIT;
                        end
                        C_B3_X1_START: begin
                            mul_mcand<={24'd0,abs24(b2)}; mul_mplier<=abs24(b6_square>>>12);
                            mul_accum<=0; mul_count<=0; mul_neg<=(b2<0); mul_busy<=1; calc_state<=C_B3_X1_WAIT;
                        end
                        C_B3_X2_START: begin
                            mul_mcand<={24'd0,abs24(ac2)}; mul_mplier<=abs24(b6);
                            mul_accum<=0; mul_count<=0; mul_neg<=(ac2<0)^(b6<0); mul_busy<=1; calc_state<=C_B3_X2_WAIT;
                        end
                        C_B4_X1_START: begin
                            mul_mcand<={24'd0,abs24(ac3)}; mul_mplier<=abs24(b6);
                            mul_accum<=0; mul_count<=0; mul_neg<=(ac3<0)^(b6<0); mul_busy<=1; calc_state<=C_B4_X1_WAIT;
                        end
                        C_B4_X2_START: begin
                            mul_mcand<={24'd0,abs24(b1)}; mul_mplier<=abs24(b6_square>>>12);
                            mul_accum<=0; mul_count<=0; mul_neg<=(b1<0); mul_busy<=1; calc_state<=C_B4_X2_WAIT;
                        end
                        C_B4_X3_START: begin
                            mul_mcand<={24'd0,abs24(ux16(ac4))};
                            mul_mplier<=abs24((((x1+x2+32'sd2)>>>2)+32'sd32768));
                            mul_accum<=0; mul_count<=0; mul_neg<=0; mul_busy<=1; calc_state<=C_B4_X3_WAIT;
                        end
                        C_B7_START: begin
                            b7<=scale50000($signed({24'd0,up_raw})-b3); calc_state<=C_P_DIV_START;
                        end
                        C_P_DIV_START: begin
                            if (b4==0) begin p_calc<=0; calc_state<=C_P_SQ_START; end
                            else begin
                                if (b7<40'sh0080000000) begin div_dividend<=abs40(b7<<<1); div_post_shift<=0; end
                                else begin div_dividend<=abs40(b7); div_post_shift<=1; end
                                div_divisor<=abs40(b4); div_quotient<=0; div_remainder<=0; div_count<=0;
                                div_neg<=(b7<0)^(b4<0); div_floor<=0; div_busy<=1; calc_state<=C_P_DIV_WAIT;
                            end
                        end
                        C_P_APPLY: begin p_calc<=$signed(div_result[31:0]); calc_state<=C_P_SQ_START; end
                        C_P_SQ_START: begin
                            mul_mcand<={24'd0,abs24(p_calc>>>8)}; mul_mplier<=abs24(p_calc>>>8);
                            mul_accum<=0; mul_count<=0; mul_neg<=0; mul_busy<=1; calc_state<=C_P_SQ_WAIT;
                        end
                        C_P_X1_START: begin
                            mul_mcand<={24'd0,abs24(x1)}; mul_mplier<=abs24(32'sd3038);
                            mul_accum<=0; mul_count<=0; mul_neg<=(x1<0); mul_busy<=1; calc_state<=C_P_X1_WAIT;
                        end
                        C_P_X2_START: begin
                            mul_mcand<={24'd0,abs24(p_calc)}; mul_mplier<=abs24(-32'sd7357);
                            mul_accum<=0; mul_count<=0; mul_neg<=(p_calc<0)^1'b1; mul_busy<=1; calc_state<=C_P_X2_WAIT;
                        end
                        C_P_FINISH: begin
                            p_calc<=p_calc+((x1+x2+32'sd3791)>>>4); calc_state<=C_DONE;
                        end
                        C_DONE: begin
                            pressure_pa<=p_calc[31:0]; temperature_x10<=temp_calc[15:0];
                            data_valid<=1; busy<=1; op<=OP_TEMP_WRITE; seq<=0; tx_byte<=8'hEE; bit_count<=7; phase<=0; state<=ST_START; calc_state<=C_IDLE;
                        end
                        default: begin calc_state<=C_IDLE; state<=ST_IDLE; end
                    endcase
                end
            end else if (state == ST_WAIT) begin
                sda_low<=0; scl_low<=0; i2c_div_count<=0;
                if (wait_count == ((op==OP_TEMP_WRITE)?TEMP_TICKS:PRESS_TICKS)-1) begin
                    wait_count<=0;
                    if (op==OP_TEMP_WRITE) begin
                        op<=OP_TEMP_READ; seq<=0; read_remaining<=2; read_index<=0;
                        tx_byte<=8'hEE; bit_count<=7; phase<=0; state<=ST_START;
                    end else begin
                        op<=OP_PRESS_READ; seq<=0; read_remaining<=2; read_index<=0;
                        tx_byte<=8'hEE; bit_count<=7; phase<=0; state<=ST_START;
                    end
                end else wait_count<=wait_count+1'b1;
            end else if (i2c_tick) begin
                i2c_div_count<=0;
                case (state)
                    ST_IDLE: begin
                        op<=OP_CAL_READ; seq<=0; tx_byte<=8'hEE; bit_count<=7; phase<=0; read_remaining<=22; cal_byte_index<=0; state<=ST_START;
                    end
                    ST_START: begin
                        case (phase)
                            0: begin sda_low<=0; scl_low<=0; phase<=1; end
                            1: begin sda_low<=1; scl_low<=0; phase<=2; end
                            2: begin sda_low<=1; scl_low<=1; phase<=3; end
                            default: begin phase<=0; state<=ST_SEND; end
                        endcase
                    end
                    ST_SEND: begin
                        case (phase)
                            0: begin sda_low<=~tx_byte[bit_count]; scl_low<=1; phase<=1; end
                            1: begin scl_low<=0; phase<=2; end
                            2: begin scl_low<=0; phase<=3; end
                            default: begin scl_low<=1; phase<=0; if (bit_count==0) state<=ST_ACK; else bit_count<=bit_count-1'b1; end
                        endcase
                    end
                    ST_ACK: begin
                        case (phase)
                            0: begin sda_low<=0; scl_low<=1; phase<=1; end
                            1: begin scl_low<=0; phase<=2; end
                            2: begin ack_ok<=(bmp_sda===1'b0); phase<=3; end
                            default: begin
                                scl_low<=1; phase<=0;
                                if (!ack_ok && !(bmp_sda===1'b0)) begin error<=1; state<=ST_STOP; end
                                else begin
                                    case (op)
                                        OP_CAL_READ: begin
                                            if (seq==0) begin tx_byte<=8'hAA; bit_count<=7; seq<=1; state<=ST_SEND; end
                                            else if (seq==1) begin tx_byte<=8'hEF; bit_count<=7; seq<=2; state<=ST_START; end
                                            else begin bit_count<=7; rx_byte<=0; state<=ST_RECV; end
                                        end
                                        OP_TEMP_WRITE: begin
                                            if (seq==0) begin tx_byte<=8'hF4; bit_count<=7; seq<=1; state<=ST_SEND; end
                                            else if (seq==1) begin tx_byte<=8'h2E; bit_count<=7; seq<=2; state<=ST_SEND; end
                                            else state<=ST_STOP;
                                        end
                                        OP_TEMP_READ: begin
                                            if (seq==0) begin tx_byte<=8'hF6; bit_count<=7; seq<=1; state<=ST_SEND; end
                                            else if (seq==1) begin tx_byte<=8'hEF; bit_count<=7; seq<=2; state<=ST_START; end
                                            else begin bit_count<=7; rx_byte<=0; state<=ST_RECV; end
                                        end
                                        OP_PRESS_WRITE: begin
                                            if (seq==0) begin tx_byte<=8'hF4; bit_count<=7; seq<=1; state<=ST_SEND; end
                                            else if (seq==1) begin tx_byte<=8'h34; bit_count<=7; seq<=2; state<=ST_SEND; end
                                            else state<=ST_STOP;
                                        end
                                        default: begin
                                            if (seq==0) begin tx_byte<=8'hF6; bit_count<=7; seq<=1; state<=ST_SEND; end
                                            else if (seq==1) begin tx_byte<=8'hEF; bit_count<=7; seq<=2; state<=ST_START; end
                                            else begin bit_count<=7; rx_byte<=0; state<=ST_RECV; end
                                        end
                                    endcase
                                end
                            end
                        endcase
                    end
                    ST_RECV: begin
                        case (phase)
                            0: begin sda_low<=0; scl_low<=1; phase<=1; end
                            1: begin scl_low<=0; phase<=2; end
                            2: begin rx_byte[bit_count]<=bmp_sda; phase<=3; end
                            default: begin scl_low<=1; phase<=0; if (bit_count==0) state<=ST_MASTER_ACK; else bit_count<=bit_count-1'b1; end
                        endcase
                    end
                    ST_MASTER_ACK: begin
                        case (phase)
                            0: begin sda_low<=(read_remaining!=1); scl_low<=1; phase<=1; end
                            1: begin scl_low<=0; phase<=2; end
                            2: begin phase<=3; end
                            default: begin
                                scl_low<=1; phase<=0;
                                if (op==OP_CAL_READ) begin cal_bytes[cal_byte_index]<=rx_byte; cal_byte_index<=cal_byte_index+1'b1; end
                                else begin if (read_index==0) rx_msb<=rx_byte; else rx_lsb<=rx_byte; read_index<=read_index+1'b1; end
                                if (read_remaining>1) begin read_remaining<=read_remaining-1'b1; rx_byte<=0; bit_count<=7; state<=ST_RECV; end
                                else state<=ST_STOP;
                            end
                        endcase
                    end
                    ST_STOP: begin
                        case (phase)
                            0: begin sda_low<=1; scl_low<=1; phase<=1; end
                            1: begin scl_low<=0; phase<=2; end
                            2: begin sda_low<=0; scl_low<=0; phase<=3; end
                            default: begin
                                phase<=0;
                                if (error) begin busy<=0; state<=ST_IDLE; end
                                else if (op==OP_CAL_READ) begin
                                    ac1<=$signed({cal_bytes[0],cal_bytes[1]}); ac2<=$signed({cal_bytes[2],cal_bytes[3]}); ac3<=$signed({cal_bytes[4],cal_bytes[5]});
                                    ac4<={cal_bytes[6],cal_bytes[7]}; ac5<={cal_bytes[8],cal_bytes[9]}; ac6<={cal_bytes[10],cal_bytes[11]};
                                    b1<=$signed({cal_bytes[12],cal_bytes[13]}); b2<=$signed({cal_bytes[14],cal_bytes[15]}); mb<=$signed({cal_bytes[16],cal_bytes[17]});
                                    mc<=$signed({cal_bytes[18],cal_bytes[19]}); md<=$signed({cal_bytes[20],cal_bytes[21]});
                                    op<=OP_TEMP_WRITE; seq<=0; tx_byte<=8'hEE; bit_count<=7; state<=ST_START;
                                end else if (op==OP_TEMP_WRITE || op==OP_PRESS_WRITE) begin wait_count<=0; state<=ST_WAIT; end
                                else if (op==OP_TEMP_READ) begin ut_raw<={rx_msb,rx_lsb}; op<=OP_PRESS_WRITE; seq<=0; tx_byte<=8'hEE; bit_count<=7; state<=ST_START; end
                                else begin up_raw<={rx_msb,rx_lsb}; calc_state<=C_TEMP_X1_START; state<=ST_CALC; end
                            end
                        endcase
                    end
                    default: state<=ST_IDLE;
                endcase
            end else i2c_div_count<=i2c_div_count+1'b1;
        end
    end
endmodule
