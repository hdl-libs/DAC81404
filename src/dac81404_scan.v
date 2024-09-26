// +FHEADER-------------------------------------------------------------------------------
// Copyright (c) 2024 john_tito All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ---------------------------------------------------------------------------------------
// Author        : john_tito
// Module Name   : dac81404_scan
// ---------------------------------------------------------------------------------------
// Revision      : 1.0
// Description   : File Created
// ---------------------------------------------------------------------------------------
// Synthesizable : Yes
// Clock Domains : clk
// Reset Strategy: sync reset
// -FHEADER-------------------------------------------------------------------------------

// verilog_format: off
`resetall
`timescale 1ns / 1ps
`default_nettype none
// verilog_format: on

module dac81404_scan #(
    parameter integer CHANNEL_NUM = 4,
    parameter integer LCDAC_DLY   = 13,  // 1000 / 150 * N ns > 80 ns
    parameter integer LCDAC_HOLD  = 4    // 1000 / 150 * N ns > 20 ns
) (
    input wire clk,
    input wire rst,

    input  wire [CHANNEL_NUM-1:0] cfg_ch_enable,
    input  wire                   cfg_auto_mode,
    input  wire                   cfg_brdcast_mode,
    input  wire                   cfg_sync_mode,
    output reg                    sts_busy,
    input  wire                   scan_req,

    input  wire        tx_busy,
    input  wire        tx_ready,
    output reg         tx_valid,
    output reg  [23:0] tx_data,
    input  wire        rx_valid,

    output reg dac_ldacn,

    input  wire [63:0] s_tdata,
    input  wire        s_tvalid,
    output reg         s_tready
);

    localparam FSM_IDLE = 4'd0;
    localparam FSM_INIT = 4'd1;
    localparam FSM_BRDCAST = 4'd2;
    localparam FSM_DOUT = 4'd3;
    localparam FSM_WAIT = 4'd4;
    localparam FSM_END = 4'd5;


    reg [7:0] cstate = FSM_IDLE;
    reg [7:0] nstate = FSM_IDLE;

    genvar ii;

    reg  [15:0] tx_data_reg          [0:CHANNEL_NUM-1];

    reg         cfg_auto_scan = 1'b0;
    reg  [ 1:0] current_index;
    wire [ 1:0] next_index;
    wire        roll_over;

    reg         done;

    reg  [ 7:0] ldacn_dly_cnt;
    reg  [ 7:0] ldacn_hold_cnt;

    always @(posedge clk) begin
        if (rst) begin
            cfg_auto_scan <= 1'b0;
        end else begin
            if (done) begin
                cfg_auto_scan <= 1'b0;
            end else if (scan_req & cfg_auto_mode & ~cfg_auto_scan & s_tvalid) begin
                cfg_auto_scan <= 1'b1;
            end
        end
    end

    // *******************************************************************************
    // fsm body
    // *******************************************************************************
    always @(posedge clk) begin
        if (rst) begin
            cstate <= FSM_IDLE;
        end else begin
            cstate <= nstate;
        end
    end

    always @(*) begin
        if (rst) begin
            nstate = FSM_IDLE;
        end else begin
            case (cstate)
                FSM_IDLE: begin
                    if (cfg_auto_scan & (|cfg_ch_enable) & ~tx_busy) begin
                        nstate = FSM_INIT;
                    end else begin
                        nstate = FSM_IDLE;
                    end
                end
                FSM_INIT: begin
                    if (cfg_brdcast_mode) begin
                        nstate = FSM_BRDCAST;
                    end else begin
                        nstate = FSM_DOUT;
                    end
                end
                FSM_BRDCAST: begin
                    if (tx_ready & tx_valid) begin
                        nstate = FSM_WAIT;
                    end else begin
                        nstate = FSM_BRDCAST;
                    end
                end
                FSM_DOUT: begin
                    if (tx_ready & tx_valid) begin
                        nstate = FSM_WAIT;
                    end else begin
                        nstate = FSM_DOUT;
                    end
                end
                FSM_WAIT: begin
                    if (!tx_busy) begin
                        if (cfg_brdcast_mode | (~cfg_brdcast_mode & roll_over)) begin
                            nstate = FSM_END;
                        end else begin
                            nstate = FSM_DOUT;
                        end
                    end else begin
                        nstate = FSM_WAIT;
                    end
                end
                FSM_END: nstate = FSM_IDLE;
                default: nstate = FSM_IDLE;
            endcase
        end
    end

    // *******************************************************************************
    // latch data from axis slave port
    // *******************************************************************************
    always @(posedge clk) begin
        if (rst) begin
            tx_data_reg[3] <= 16'd0;
            tx_data_reg[2] <= 16'd0;
            tx_data_reg[1] <= 16'd0;
            tx_data_reg[0] <= 16'd0;
            s_tready       <= 1'b0;
        end else begin
            case (nstate)
                FSM_INIT: begin
                    tx_data_reg[3] <= s_tdata[16*3+:16];
                    tx_data_reg[2] <= s_tdata[16*2+:16];
                    tx_data_reg[1] <= s_tdata[16*1+:16];
                    tx_data_reg[0] <= s_tdata[16*0+:16];
                    s_tready       <= 1'b1;
                end
                default: begin
                    tx_data_reg[3] <= s_tdata[16*3+:16];
                    tx_data_reg[2] <= s_tdata[16*2+:16];
                    tx_data_reg[1] <= s_tdata[16*1+:16];
                    tx_data_reg[0] <= s_tdata[16*0+:16];
                    s_tready       <= 1'b0;
                end
            endcase
        end
    end

    // *******************************************************************************
    // provide data for transmitters
    // *******************************************************************************
    always @(posedge clk) begin
        if (rst) begin
            tx_data  <= 8'h00;
            tx_valid <= 1'b0;
        end else begin
            case (nstate)
                FSM_BRDCAST: begin
                    tx_data  <= {8'h0F, tx_data_reg[0]};
                    tx_valid <= 1'b1;
                end
                FSM_DOUT: begin
                    tx_data  <= {6'b000100, next_index, tx_data_reg[next_index]};
                    tx_valid <= 1'b1;
                end
                default: begin
                    tx_data  <= 8'h00;
                    tx_valid <= 1'b0;
                end
            endcase
        end
    end

    // *******************************************************************************
    // generate busy state flag for user
    // *******************************************************************************
    always @(posedge clk) begin
        if (rst) begin
            sts_busy <= 1'b1;
        end else begin
            case (nstate)
                FSM_IDLE: sts_busy <= 1'b0;
                default:  sts_busy <= 1'b1;
            endcase
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            done <= 1'b0;
        end else begin
            case (nstate)
                FSM_END: done <= 1'b1;
                default: done <= 1'b0;
            endcase
        end
    end

    // *******************************************************************************
    // generate load signal for synchronous update
    // assert ld after spi_cs rising edge, delay 8 clocks then hold 8 clocks
    // *******************************************************************************
    always @(posedge clk) begin
        if (rst) begin
            ldacn_dly_cnt <= 0;
        end else begin
            if (cfg_sync_mode) begin
                if (done) begin
                    ldacn_dly_cnt <= LCDAC_DLY;
                end else if (ldacn_dly_cnt) begin
                    ldacn_dly_cnt <= ldacn_dly_cnt - 4'h1;
                end else begin
                    ldacn_dly_cnt <= 0;
                end
            end else begin
                ldacn_dly_cnt <= 0;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            ldacn_hold_cnt <= 0;
            dac_ldacn      <= 1'b1;
        end else begin
            if (cfg_sync_mode) begin
                if (ldacn_dly_cnt == 1) begin
                    ldacn_hold_cnt <= LCDAC_HOLD;
                    dac_ldacn      <= 1'b0;
                end else if (ldacn_hold_cnt) begin
                    ldacn_hold_cnt <= ldacn_hold_cnt - 4'h1;
                    dac_ldacn      <= 1'b0;
                end else begin
                    ldacn_hold_cnt <= 0;
                    dac_ldacn      <= 1'b1;
                end
            end else begin
                ldacn_hold_cnt <= 0;
                dac_ldacn      <= 1'b1;
            end
        end
    end

    // *******************************************************************************
    // round check on all enabled channel
    // *******************************************************************************
    round_arb #(
        .SCAN_DIR   (1'b0),
        .CHANNEL_NUM(CHANNEL_NUM),
        .INDEX_WIDTH(2)
    ) round_arb_inst (
        .clk          (clk),
        .rst          (rst),
        .ch_enable    (cfg_ch_enable),
        .current_index(current_index),
        .next_index   (next_index),
        .roll_over    (roll_over),
        .next_bin     ()
    );

    always @(posedge clk) begin
        if (rst) begin
            current_index <= 3;
        end else begin
            case (cstate)
                FSM_IDLE: begin
                    current_index <= 3;
                end
                FSM_INIT: begin
                    current_index <= next_index;
                end
                FSM_DOUT: begin
                    if (tx_ready & tx_valid) begin
                        current_index <= next_index;
                    end
                end
                default: ;
            endcase
        end
    end

endmodule

// verilog_format: off
`resetall
// verilog_format: on