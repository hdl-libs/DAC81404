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
// Module Name   : dac81404_scan_wrapper
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

module dac81404_scan_wrapper (
    input wire clk,
    input wire rst,

    input wire        baud_load,
    input wire [31:0] baud_div,

    input wire [3:0] cfg_ch_enable,
    input wire       cfg_auto_mode,
    input wire       cfg_brdcast_mode,
    input wire       cfg_sync_mode,

    output wire sts_spi_busy,
    input  wire sync,

    output wire spi_scsn,
    output wire spi_sclk,
    output wire spi_mosi,
    input  wire spi_miso,
    output wire dac_ldacn,

    input  wire [63:0] s_tdata,
    input  wire        s_tvalid,
    output wire        s_tready
);

    wire        tx_busy;
    wire        tx_valid;
    wire        tx_ready;
    wire [23:0] tx_data;
    wire        rx_valid;

    dac81404_scan dac81404_scan_inst (
        .clk             (clk),
        .rst             (rst),
        .cfg_ch_enable   (cfg_ch_enable),
        .cfg_auto_mode   (cfg_auto_mode),
        .cfg_brdcast_mode(cfg_brdcast_mode),
        .cfg_sync_mode   (cfg_sync_mode),
        .sts_busy        (sts_spi_busy),
        .scan_req        (sync),
        .tx_busy         (tx_busy),
        .tx_ready        (tx_ready),
        .tx_valid        (tx_valid),
        .tx_data         (tx_data),
        .rx_valid        (rx_valid),
        .dac_ldacn       (dac_ldacn),
        .s_tdata         (s_tdata),
        .s_tvalid        (s_tvalid),
        .s_tready        (s_tready)
    );

    spi_master #(
        .DATA_WIDTH(24),
        .CPHA      (1'b1),
        .MSB       (1'b1)
    ) spi_master_inst (
        .clk     (clk),
        .rst     (rst),
        .load    (baud_load),
        .baud_div(baud_div),
        .spi_scsn(spi_scsn),
        .spi_sclk(spi_sclk),
        .spi_miso(spi_miso),
        .spi_mosi(spi_mosi),
        .tx_busy (tx_busy),
        .tx_valid(tx_valid),
        .tx_data (tx_data),
        .tx_ready(tx_ready),
        .rx_data (),
        .rx_valid(rx_valid),
        .tx_done ()
    );
endmodule

// verilog_format: off
`resetall
// verilog_format: on
