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
// Module Name   : dac81404_conf
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

module dac81404_conf (
    input wire clk,
    input wire rst,

    input  wire [ 7:0] cfg_addr,       // SPI操作地址
    input  wire [15:0] cfg_wr_data,    // SPI写数据
    output reg  [15:0] cfg_rd_data,    // SPI读数据
    input  wire        cfg_start,      // SPI传输开始
    input  wire        cfg_auto_mode,  // DAC 自动扫描, auto scan
    output reg  [ 3:0] cfg_ch_enable,  // DAC 通道使能, auto scan
    output reg         sts_busy,       // SPI 传输繁忙
    output reg         sts_done,       // SPI 传输完成

    input  wire        tx_busy,
    input  wire        tx_ready,
    output reg         tx_valid,
    output reg  [23:0] tx_data,
    input  wire        rx_valid,
    input  wire [23:0] rx_data
);

    localparam [7:0] ADDR_DACPWDWN = 8'h09;

    localparam FSM_IDLE = 4'd0;
    localparam FSM_INIT = 4'd1;
    localparam FSM_DIN = 4'd2;
    localparam FSM_WAIT_DIN = 4'd3;
    localparam FSM_DOUT = 4'd4;
    localparam FSM_WAIT = 4'd5;

    reg [ 7:0] cstate = FSM_IDLE;
    reg [ 7:0] nstate = FSM_IDLE;

    reg [23:0] cfg_wr_data_reg = 'd0;
    reg [ 3:0] ch_pd_reg = 'd0;

    reg        is_read;
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
                    if (~cfg_auto_mode & cfg_start & ~tx_busy & tx_ready) begin
                        nstate = FSM_INIT;
                    end else begin
                        nstate = FSM_IDLE;
                    end
                end
                FSM_INIT: begin
                    nstate = FSM_DIN;
                end
                FSM_DIN: begin
                    if (tx_ready & tx_valid) begin
                        if (is_read) begin
                            nstate = FSM_WAIT_DIN;
                        end else begin
                            nstate = FSM_WAIT;
                        end
                    end else begin
                        nstate = FSM_DIN;
                    end
                end
                FSM_WAIT_DIN: begin
                    if (!tx_busy) begin
                        nstate = FSM_DOUT;
                    end else begin
                        nstate = FSM_WAIT_DIN;
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
                        nstate = FSM_IDLE;
                    end else begin
                        nstate = FSM_WAIT;
                    end
                end
                default: begin
                    nstate = FSM_IDLE;
                end
            endcase
        end
    end

    // *******************************************************************************
    // provide data for transmitters
    // *******************************************************************************
    always @(posedge clk) begin
        if (rst) begin
            tx_data  <= 24'h000000;
            tx_valid <= 1'b0;
        end else begin
            case (nstate)
                FSM_DIN: begin
                    tx_valid <= 1'b1;
                    tx_data  <= cfg_wr_data_reg;
                end
                FSM_DOUT: begin
                    tx_valid <= 1'b1;
                    tx_data  <= 24'h000000;
                end
                default: begin
                    tx_valid <= 1'b0;
                    tx_data  <= 24'h000000;
                end
            endcase
        end
    end

    // *******************************************************************************
    // latch the data read from external device
    // clean the data when FSM start
    // *******************************************************************************
    always @(posedge clk) begin
        if (rst) begin
            cfg_rd_data <= 16'h0000;
        end else begin
            case (nstate)
                FSM_INIT: begin
                    cfg_rd_data <= 16'h0000;
                end
                FSM_WAIT: begin
                    if (rx_valid) begin
                        cfg_rd_data <= rx_data[15:0];
                    end else begin
                        cfg_rd_data <= cfg_rd_data;
                    end
                end
                default: cfg_rd_data <= cfg_rd_data;
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
            sts_done <= 1'b0;
        end else begin
            case (cstate)
                FSM_WAIT: sts_done <= ~tx_busy;
                default:  sts_done <= 1'b0;
            endcase
        end
    end

    // *******************************************************************************
    // latch config data in case user changes these things when in thansfer
    // *******************************************************************************
    always @(posedge clk) begin
        if (rst) begin
            cfg_wr_data_reg <= 24'h000000;
        end else begin
            case (nstate)
                FSM_INIT: begin
                    cfg_wr_data_reg <= {cfg_addr, cfg_wr_data};
                end
                default: ;
            endcase
        end
    end

    // *******************************************************************************
    // add user addr here, operating on these address doesnt initate a transmission
    // *******************************************************************************
    always @(posedge clk) begin
        if (rst) begin
            is_read <= 1'b0;
        end else begin
            case (nstate)
                FSM_INIT: begin
                    is_read <= cfg_addr[7];
                end
                default: begin
                    is_read <= is_read;
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            cfg_ch_enable <= 4'hF;
            ch_pd_reg     <= 4'hF;
        end else begin
            if (cfg_start) begin
                if (cfg_addr == ADDR_DACPWDWN) begin
                    ch_pd_reg <= cfg_wr_data[3:0];
                end
            end
            cfg_ch_enable <= (~ch_pd_reg);
        end
    end

endmodule

// verilog_format: off
`resetall
// verilog_format: on
