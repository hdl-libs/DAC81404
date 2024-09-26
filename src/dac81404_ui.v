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
// Module Name   : dac81404_ui
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

module dac81404_ui #(
    parameter integer C_APB_ADDR_WIDTH = 16,
    parameter integer C_APB_DATA_WIDTH = 32,
    parameter integer C_S_BASEADDR     = 0
) (
    //
    input  wire                          clk,
    input  wire                          rstn,
    //
    input  wire [(C_APB_ADDR_WIDTH-1):0] s_paddr,
    input  wire                          s_psel,
    input  wire                          s_penable,
    input  wire                          s_pwrite,
    input  wire [(C_APB_DATA_WIDTH-1):0] s_pwdata,
    output wire                          s_pready,
    output wire [(C_APB_DATA_WIDTH-1):0] s_prdata,
    output wire                          s_pslverr,
    //
    output reg                           baud_load,
    output reg  [                  31:0] baud_div,
    //
    output reg  [                   7:0] cfg_addr,          // SPI操作地址
    output reg  [                  15:0] cfg_wr_data,       // SPI写数据
    input  wire [                  15:0] cfg_rd_data,       // SPI读数据
    output reg                           cfg_spi_start,     // SPI传输开始
    input  wire                          sts_spi_busy,      // SPI 传输繁忙
    input  wire                          sts_spi_done,      // SPI 传输完成
    input  wire                          sts_scan_busy,     // 扫描繁忙
    //
    output wire                          cfg_auto_mode,     // DAC 自动扫描, auto scan
    output wire                          cfg_sync_mode,     // DAC 同步模式, auto scan
    output wire                          cfg_brdcast_mode,  // DAC 广播模式, auto scan
    input  wire [                   3:0] cfg_ch_enable,     // DAC 通道使能, auto scan
    //
    output reg                           sync,              // 同步脉冲
    output reg                           soft_rst,          // 软件复位
    output reg                           dac_rstn,          // dac芯片复位
    output reg                           dac_clrn,          // dac寄存器清空
    output reg                           dac_ldacn,         // dac寄存器载入
    input  wire                          dac_faultn         // dac错误输出
);

    // verilog_format: off
    localparam integer RST_HOLD     = 255;
    localparam integer LD_HOLD      = 255;
    localparam integer CLR_HOLD     = 255;
    localparam [7:0] ADDR_CTRL          = C_S_BASEADDR;
    localparam [7:0] ADDR_STATE         = ADDR_CTRL         + 8'h4;
    localparam [7:0] ADDR_ADDR          = ADDR_STATE        + 8'h4;
    localparam [7:0] ADDR_WR_DATA       = ADDR_ADDR         + 8'h4;
    localparam [7:0] ADDR_RD_DATA       = ADDR_WR_DATA      + 8'h4;
    localparam [7:0] ADDR_SCAN_PRRIOD   = ADDR_RD_DATA      + 8'h4;
    localparam [7:0] ADDR_ENABLE_CH     = ADDR_SCAN_PRRIOD  + 8'h4;
    localparam [7:0] ADDR_BAUD_DIV      = ADDR_ENABLE_CH    + 8'h4;
    // verilog_format: on

    reg        rstn_i = 0;
    reg [ 7:0] rst_cnt;
    reg [ 7:0] clr_cnt;
    reg [ 7:0] ld_cnt;

    reg [31:0] ctrl_reg;
    reg [31:0] status_reg;
    reg [31:0] scan_period;
    reg [31:0] scan_cnt;

    //------------------------------------------------------------------------------------

    localparam [31:0] IPIDENTIFICATION = 32'hF7DEC7A5;
    localparam [31:0] REVISION = "V1.0";
    localparam [31:0] BUILDTIME = 32'h20231013;

    reg  [                31:0] test_reg;
    wire                        wr_active;
    wire                        rd_active;

    wire                        user_reg_rreq;
    wire                        user_reg_wreq;
    reg                         user_reg_rack;
    reg                         user_reg_wack;
    wire [C_APB_ADDR_WIDTH-1:0] user_reg_raddr;
    reg  [C_APB_DATA_WIDTH-1:0] user_reg_rdata;
    wire [C_APB_ADDR_WIDTH-1:0] user_reg_waddr;
    wire [C_APB_DATA_WIDTH-1:0] user_reg_wdata;

    assign user_reg_rreq  = ~s_pwrite & s_psel & s_penable;
    assign user_reg_wreq  = s_pwrite & s_psel & s_penable;
    assign s_pready       = user_reg_rack | user_reg_wack;
    assign user_reg_raddr = s_paddr;
    assign user_reg_waddr = s_paddr;
    assign s_prdata       = user_reg_rdata;
    assign user_reg_wdata = s_pwdata;
    assign s_pslverr      = 1'b0;

    assign rd_active      = user_reg_rreq;
    assign wr_active      = user_reg_wreq & user_reg_wack;

    always @(posedge clk) begin
        user_reg_rack <= user_reg_rreq & ~user_reg_rack;
        user_reg_wack <= user_reg_wreq & ~user_reg_wack;
    end

    //------------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rstn) begin
            soft_rst <= 1'b1;
        end else begin
            if (wr_active && (user_reg_waddr == ADDR_CTRL) && user_reg_wdata[31]) begin
                soft_rst <= 1'b1;
            end else begin
                soft_rst <= 1'b0;
            end
        end
    end

    //-------------------------------------------------------------------------------------------------------------------------------------------
    //Read Register
    //-------------------------------------------------------------------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (soft_rst) begin
            user_reg_rdata <= 32'd0;
        end else begin
            user_reg_rdata <= 32'd0;
            if (user_reg_rreq) begin
                case (user_reg_raddr)
                    ADDR_CTRL:        user_reg_rdata <= ctrl_reg;
                    ADDR_STATE:       user_reg_rdata <= status_reg;
                    ADDR_ADDR:        user_reg_rdata <= cfg_addr;
                    ADDR_WR_DATA:     user_reg_rdata <= cfg_wr_data;
                    ADDR_RD_DATA:     user_reg_rdata <= cfg_rd_data;
                    ADDR_ENABLE_CH:   user_reg_rdata <= cfg_ch_enable;
                    ADDR_SCAN_PRRIOD: user_reg_rdata <= scan_period;
                    ADDR_BAUD_DIV:    user_reg_rdata <= baud_div;
                    default:          user_reg_rdata <= 32'hdeadbeef;
                endcase
            end
        end
    end

    always @(posedge clk) begin
        if (soft_rst) begin
            cfg_addr    <= 0;
            cfg_wr_data <= 0;
            scan_period <= 0;
            baud_div    <= 8;
        end else begin
            cfg_addr    <= cfg_addr;
            cfg_wr_data <= cfg_wr_data;
            scan_period <= scan_period;
            baud_div    <= baud_div;
            if (wr_active) begin
                case (user_reg_waddr)
                    ADDR_ADDR:        cfg_addr <= user_reg_wdata;
                    ADDR_WR_DATA:     cfg_wr_data <= user_reg_wdata;
                    ADDR_SCAN_PRRIOD: scan_period <= user_reg_wdata;
                    ADDR_BAUD_DIV:    baud_div <= user_reg_wdata;
                    default:          ;
                endcase
            end
        end
    end

    always @(posedge clk) begin
        if (soft_rst) begin
            status_reg <= 0;
        end else begin
            if (wr_active && (user_reg_waddr == ADDR_STATE)) begin
                status_reg <= status_reg & ~user_reg_wdata;
            end else begin
                status_reg[0] <= sts_spi_busy;
                status_reg[1] <= (status_reg[1] | sts_spi_done) & (~cfg_spi_start);
                status_reg[4] <= sts_scan_busy;
                status_reg[8] <= ~dac_faultn;
            end
        end
    end

    always @(posedge clk) begin
        if (soft_rst) begin
            ctrl_reg <= 0;
        end else begin
            if (wr_active && (user_reg_waddr == ADDR_CTRL)) begin
                ctrl_reg <= user_reg_wdata;
            end else begin
                ctrl_reg <= {cfg_brdcast_mode, cfg_sync_mode, cfg_auto_mode, ~dac_ldacn, ~dac_clrn, ~dac_rstn, 4'b0000, 3'b000, cfg_spi_start};
            end
        end
    end

    // ctrl[0]
    always @(posedge clk) begin
        if (soft_rst) begin
            cfg_spi_start <= 1'b0;
        end else begin
            if (wr_active && (user_reg_waddr == ADDR_CTRL) && user_reg_wdata[0]) begin
                cfg_spi_start <= ~sts_spi_busy;
            end else begin
                cfg_spi_start <= 1'b0;
            end
        end
    end

    // ctrl[8]
    always @(posedge clk) begin
        if (soft_rst) begin
            rst_cnt  <= RST_HOLD;
            dac_rstn <= 1'b0;
        end else begin
            if (wr_active && (user_reg_waddr == ADDR_CTRL) && user_reg_wdata[8]) begin
                rst_cnt  <= RST_HOLD;
                dac_rstn <= 1'b0;
            end else begin
                if (rst_cnt > 0) begin
                    rst_cnt  <= rst_cnt - 1;
                    dac_rstn <= 1'b0;
                end else begin
                    rst_cnt  <= 0;
                    dac_rstn <= 1;
                end
            end
        end
    end

    // ctrl[9]
    always @(posedge clk) begin
        if (soft_rst) begin
            clr_cnt  <= 0;
            dac_clrn <= 0;
        end else begin
            if (wr_active && (user_reg_waddr == ADDR_CTRL) && user_reg_wdata[9]) begin
                clr_cnt  <= CLR_HOLD;
                dac_clrn <= 1'b0;
            end else begin
                if (clr_cnt > 0) begin
                    clr_cnt  <= clr_cnt - 1;
                    dac_clrn <= 1'b0;
                end else begin
                    clr_cnt  <= 0;
                    dac_clrn <= 1;
                end
            end
        end
    end

    // ctrl[10]
    always @(posedge clk) begin
        if (soft_rst) begin
            ld_cnt    <= 0;
            dac_ldacn <= 1;
        end else begin
            if (wr_active && (user_reg_waddr == ADDR_CTRL) && user_reg_wdata[10]) begin
                ld_cnt    <= LD_HOLD;
                dac_ldacn <= 1'b0;
            end else begin
                if (ld_cnt > 0) begin
                    ld_cnt    <= ld_cnt - 1;
                    dac_ldacn <= 1'b0;
                end else begin
                    ld_cnt    <= 0;
                    dac_ldacn <= 1;
                end
            end
        end
    end

    assign cfg_auto_mode    = ctrl_reg[11];
    assign cfg_sync_mode    = ctrl_reg[12];
    assign cfg_brdcast_mode = ctrl_reg[13];

    // ctrl[14]
    always @(posedge clk) begin
        if (soft_rst) begin
            baud_load <= 1'b0;
        end else begin
            if (wr_active && (user_reg_waddr == ADDR_CTRL) && user_reg_wdata[14]) begin
                baud_load <= 1'b1;
            end else begin
                baud_load <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (soft_rst) begin
            scan_cnt <= 0;
            sync     <= 1'b0;
        end else begin
            if (cfg_auto_mode && (scan_period > 0)) begin
                if (scan_cnt < scan_period - 1) begin
                    scan_cnt <= scan_cnt + 1;
                    sync     <= 1'b0;
                end else begin
                    scan_cnt <= 0;
                    sync     <= 1'b1;
                end
            end else begin
                scan_cnt <= 0;
                sync     <= 1'b0;
            end
        end
    end

endmodule

// verilog_format: off
`resetall
// verilog_format: on