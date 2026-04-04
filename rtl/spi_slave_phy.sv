// ************************************************************************************************************************
//
// Copyright (c) 2025 Sachith Rathnayake
// All Rights Reserved.
//
// This software, HDL source code, hardware designs, documentation, and all
// associated files (collectively, the "Work") are provided for viewing purposes
// only.
//
// No permission is granted to copy, reproduce, modify, merge, publish,
// distribute, sublicense, create derivative works from, or use the Work, in
// whole or in part, for any purpose without explicit written permission from
// the copyright holder.
//
// Commercial use requires a separate license agreement. Unauthorized use,
// reproduction, modification, or distribution of the Work may result in civil
// and criminal penalties.
//
// THE WORK IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
// PARTICULAR PURPOSE, AND NON-INFRINGEMENT. IN NO EVENT SHALL THE COPYRIGHT HOLDER
// BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION WITH THE WORK
// OR THE USE OR OTHER DEALINGS IN THE WORK.
//
// For commercial licensing inquiries or support, please contact:
// Phone:      +94 778 911 111
// Email:      sachith.rathnayake.92@gmail.com
// LinkedIn:   https://www.linkedin.com/in/sachith-rathnayake-profile-link/
// GitHub:     https://github.com/LordSach
//
// ************************************************************************************************************************
//
// PROJECT      :   Avionics Box
// PRODUCT      :   SPI Slave
// FILE         :   spi_slave_phy.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   SPI Slave for reusable and expandable use.
//
// ************************************************************************************************************************
//
// REVISIONS:
//
//  Date           Developer               Description
//  -----------    --------------------    -----------
//  10-FEB-2023    Sachith Rathnayake      Creation
//
// ************************************************************************************************************************

`timescale 1ns/1ps

module spi_slave_phy (
    clk,
    rst,

    // spi slave interface
    sclk,
    cs_n,
    mosi,
    miso,

    // user interface
    din,
    din_vld,
    din_rdy,
    dout,
    dout_vld
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter WORD_SIZE = 8;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam BIT_CNT_WIDTH = $clog2(WORD_SIZE);
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic                   clk;
    input   logic                   rst;

    // spi slave interface
    input   logic                   sclk;
    input   logic                   cs_n;
    input   logic                   mosi;
    output  logic                   miso;

    // user interface
    input   logic [WORD_SIZE-1:0]   din;
    input   logic                   din_vld;
    output  logic                   din_rdy;
    output  logic [WORD_SIZE-1:0]   dout;
    output  logic                   dout_vld;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic                           sclk_meta;
    logic                           cs_n_meta;
    logic                           mosi_meta;

    logic                           sclk_reg;
    logic                           cs_n_reg;
    logic                           mosi_reg;

    logic                           spi_clk_reg;
    logic                           spi_clk_redge_en;
    logic                           spi_clk_fedge_en;
    
    logic [BIT_CNT_WIDTH-1:0]       bit_cnt;
    logic                           bit_cnt_max;
    
    logic                           last_bit_en;
    logic                           load_data_en;
    logic [WORD_SIZE-1:0]           data_shreg;
    
    logic                           slave_ready;
    
    logic                           shreg_busy;
    
    logic                           rx_data_vld;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    // INPUT SYNCHRONIZATION REGISTERS
    always_ff @(posedge clk) begin : sync_ffs_p
        sclk_meta <= sclk;
        cs_n_meta <= cs_n;
        mosi_meta <= mosi;
        sclk_reg  <= sclk_meta;
        cs_n_reg  <= cs_n_meta;
        mosi_reg  <= mosi_meta;
    end

    // The SPI clock register for clock edge detection.
    always_ff @(posedge clk) begin
        if (rst) begin
            spi_clk_reg <= 1'b0;
        end
        else begin
            spi_clk_reg <= sclk_reg;
        end
    end

    // Falling edge is detect when sclk_reg=0 and spi_clk_reg=1.
    assign spi_clk_fedge_en = ~sclk_reg & spi_clk_reg;
    // Rising edge is detect when sclk_reg=1 and spi_clk_reg=0.
    assign spi_clk_redge_en = sclk_reg & ~spi_clk_reg;

    // RECEIVED BITS COUNTER

    // The counter counts received bits from the master. Counter is enabled when
    // falling edge of SPI clock is detected and not asserted cs_n_reg.
    always_ff @(posedge clk) begin
        if (rst) begin
            bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
        end
        else begin
            if (spi_clk_fedge_en && !cs_n_reg) begin
                bit_cnt <= (bit_cnt_max)? {BIT_CNT_WIDTH{1'b0}} : bit_cnt + 1;
            end
        end
    end

    // The flag of maximal value of the bit counter.
    assign bit_cnt_max = (bit_cnt == WORD_SIZE-1);

    // LAST BIT FLAG REGISTER

    always_ff @(posedge clk) last_bit_en <= (rst)? 1'b0 : bit_cnt_max;

    // Received data from master are valid when falling edge of SPI clock is
    // detected and the last bit of received byte is detected.
    assign rx_data_vld = spi_clk_fedge_en & last_bit_en;

    // SHIFT REGISTER BUSY FLAG REGISTER

    // Data shift register is busy until it sends all input data to SPI master.
    always_ff @(posedge clk) begin
        if (rst) begin
            shreg_busy      <= 1'b0;
        end
        else begin
            if (din_vld && (cs_n_reg || rx_data_vld)) begin
                shreg_busy  <= 1'b1;
            end
            else if (rx_data_vld) begin
                shreg_busy  <= 1'b0;
            end
            else begin
                shreg_busy  <= shreg_busy;
            end
        end
    end

    // The SPI slave is ready for accept new input data when cs_n_reg is assert and
    // shift register not busy or when received data are valid.
    assign slave_ready = (cs_n_reg & ~shreg_busy) || rx_data_vld;

    // The new input data is loaded into the shift register when the SPI slave
    // is ready and input data are valid.
    assign load_data_en = slave_ready & din_vld;

    // DATA SHIFT REGISTER

    // The shift register holds data for sending to master, capture and store
    // incoming data from master.
    always_ff @(posedge clk) begin
        if (load_data_en) begin
            data_shreg      <= din;
        end
        else begin
            if (spi_clk_redge_en & !cs_n_reg) begin
                data_shreg  <= {data_shreg[WORD_SIZE-2:0], mosi_reg};
            end
        end
    end

    // MISO REGISTER

    // The output MISO register ensures that the bits are transmit to the master
    // when is not assert cs_n_reg and falling edge of SPI clock is detected.
    always_ff @(posedge clk) begin
        if (load_data_en) begin
            mosi        <= din[WORD_SIZE-1];
        end
        else begin
            if (spi_clk_fedge_en & !cs_n_reg) begin
                miso    <= data_shreg[WORD_SIZE-1];
            end
        end
    end

    // ASSIGNING OUTPUT SIGNALS
    assign din_rdy  = slave_ready;
    assign dout     = data_shreg;
    assign dout_vld = rx_data_vld;

endmodule