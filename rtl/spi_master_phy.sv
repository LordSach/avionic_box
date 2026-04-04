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
// PRODUCT      :   SPI Master PHY
// FILE         :   spi_master_phy.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   Scalable SPI master physical port interfacing architecture for re-usability and expandability
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

module spi_master_phy (
     // System interface
    CLK,    // system clock
    RST,    // high active synchronous reset

    // SPI master interface
    SCLK,   // SPI clock
    CS_N,   // SPI chip select, active low
    MOSI,   // SPI serial data from master to slave
    MISO,   // SPI serial data from slave to master

    // Input user interface (write from master to slave)
    DIN,            // data for transmission to SPI slave
    DIN_ADDR,// SPI slave address
    DIN_LAST,// when 1, last data word; after transmit CS_N will be deasserted
    DIN_VLD, // when 1, data for transmission are valid
    DIN_RDY, // when 1, SPI master is ready to accept valid data

    // Output user interface (read from slave to master)
    DOUT,    // received data from SPI slave
    DOUT_VLD // when 1, received data are valid
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // NA
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter CLK_FREQ    = 50_000_000; // system clock frequency in Hz
    parameter SCLK_FREQ   = 5_000_000;  // SPI clock frequency in Hz (condition: SCLK_FREQ <= CLK_FREQ/10)
    parameter WORD_SIZE   = 8;          // size of transfer word in bits, must be power of two
    parameter SLAVE_COUNT = 1;          // count of SPI slaves
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam DIVIDER_VALUE    = (CLK_FREQ / SCLK_FREQ) / 2;
    localparam WIDTH_CLK_CNT    = $clog2(DIVIDER_VALUE);
    localparam WIDTH_ADDR       = $clog2(SLAVE_COUNT);
    localparam BIT_CNT_WIDTH    = $clog2(WORD_SIZE);
    localparam ADDR_WIDTH       = $clog2(SLAVE_COUNT);
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    typedef enum logic [2:0] { 
        idle,
        first_edge,
        second_edge,
        transmit_end,
        transmit_gap
    } stat_t;
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // System interface
    input  logic                    CLK;        // system clock
    input  logic                    RST;        // high active synchronous reset

    // SPI master interface
    output logic                    SCLK;       // SPI clock
    output logic [SLAVE_COUNT-1:0]  CS_N;       // SPI chip select, active low
    output logic                    MOSI;       // SPI serial data from master to slave
    input  logic                    MISO;       // SPI serial data from slave to master

    // Input user interface (write from master to slave)
    input  logic [WORD_SIZE-1:0]    DIN;        // data for transmission to SPI slave
    input  logic [ADDR_WIDTH-1:0]   DIN_ADDR; // SPI slave address
    input  logic                    DIN_LAST;   // when 1, last data word; after transmit CS_N will be deasserted
    input  logic                    DIN_VLD;    // when 1, data for transmission are valid
    output logic                    DIN_RDY;    // when 1, SPI master is ready to accept valid data

    // Output user interface (read from slave to master)
    output logic [WORD_SIZE-1:0]    DOUT;       // received data from SPI slave
    output logic                    DOUT_VLD;   // when 1, received data are valid
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic [WIDTH_ADDR-1:0]          addr_reg;
    logic [WIDTH_CLK_CNT-1:0]       sys_clk_cnt;
    logic                           sys_clk_cnt_max;
    logic                           spi_clk;
    logic                           spi_clk_rst;
    logic                           din_last_reg_n;
    logic                           first_edge_en;
    logic                           second_edge_en;
    logic                           chip_select_n;
    logic                           load_data;
    logic                           miso_reg;
    logic [WORD_SIZE-1:0]           shreg;
    logic [BIT_CNT_WIDTH-1:0]       bit_cnt;
    logic                           bit_cnt_max;
    logic                           rx_data_vld;
    logic                           master_ready;
    stat_t                          present_state;
    stat_t                          next_state;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    initial begin
        assert (DIVIDER_VALUE >= 5) else $error("condition: SCLK_FREQ <= CLK_FREQ/10");
    end

    assign load_data = master_ready & DIN_VLD;
    assign DIN_RDY   = master_ready;

    //---------------------------------------------------------------------------------------------------------------------
    // SYSTEM CLOCK COUNTER
    //---------------------------------------------------------------------------------------------------------------------

    assign sys_clk_cnt_max = (sys_clk_cnt == DIVIDER_VALUE-1);

    always_ff @(posedge CLK) begin : sys_clk_cnt_reg
        if (RST || sys_clk_cnt_max) begin
            sys_clk_cnt <= {WIDTH_CLK_CNT{1'b0}};
        end
        else begin
            sys_clk_cnt <= sys_clk_cnt + 1'b1;
        end
    end

    //---------------------------------------------------------------------------------------------------------------------
    // SPI CLOCK GENERATOR AND REGISTER
    //---------------------------------------------------------------------------------------------------------------------

    always_ff @(posedge CLK) begin : spi_clk_gen
        if (RST || spi_clk_rst) begin
            spi_clk <= 1'b0;
        end
        else begin
           spi_clk  <= ~spi_clk; 
        end
    end

    assign SCLK = spi_clk;

    //---------------------------------------------------------------------------------------------------------------------
    // BIT COUNTER
    //---------------------------------------------------------------------------------------------------------------------

    assign bit_cnt_max = (bit_cnt == WORD_SIZE-1);

    always_ff @(posedge CLK) begin : bit_cntr
        if (RST || spi_clk_rst) begin
            bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
        end
        else begin
            bit_cnt  <= bit_cnt + 1'b1; 
        end
    end

    //---------------------------------------------------------------------------------------------------------------------
    // SPI MASTER ADDRESSING
    //---------------------------------------------------------------------------------------------------------------------

    always_ff @(posedge CLK) begin : addr_reg_p
        if (RST) begin
            addr_reg <= {WIDTH_ADDR{1'b0}};
        end
        else if (load_data) begin
            addr_reg  <= DIN_ADDR; 
        end
    end

    generate
        // Single slave case
        if (SLAVE_COUNT == 1) begin : one_slave_g
            assign CS_N[0] = chip_select_n;
        end
        // Multiple slaves case
        else begin : more_slaves_g
            for (genvar i = 0; i < SLAVE_COUNT; i++) begin : cs_n_g
                always_comb begin
                    if (addr_reg == i) begin
                        CS_N[i] = chip_select_n;
                    end
                    else begin
                        CS_N[i] = 1'b1;
                    end
                end
            end
        end
    endgenerate

    //---------------------------------------------------------------------------------------------------------------------
    // DIN LAST RESISTER
    //---------------------------------------------------------------------------------------------------------------------

    always_ff @(posedge CLK) begin : din_last_reg_n_p
        if (RST) begin
            din_last_reg_n <= 1'b0;
        end
        else if (load_data) begin
            din_last_reg_n  <= ~DIN_LAST; 
        end
    end

    //---------------------------------------------------------------------------------------------------------------------
    // MISO SAMPLE REGISTER
    //---------------------------------------------------------------------------------------------------------------------

    always_ff @(posedge CLK) begin : miso_reg_p
        miso_reg  <= (first_edge_en)? MISO : miso_reg;
    end

    //---------------------------------------------------------------------------------------------------------------------
    // DATA SHIFT REGISTER
    //---------------------------------------------------------------------------------------------------------------------

    always_ff @(posedge CLK) begin : shreg_p
        if (load_data) begin
            shreg <= DIN; 
        end
        else if (second_edge_en) begin
            shreg <= {shreg[WORD_SIZE-2:0],miso_reg};
        end
    end

    assign DOUT = shreg;
    assign MOSI = shreg[WORD_SIZE-1];

    //---------------------------------------------------------------------------------------------------------------------
    // DATA OUT VALID RESISTER
    //---------------------------------------------------------------------------------------------------------------------

    always_ff @(posedge CLK) begin : dout_vld_reg_p
        if (RST) begin
            DOUT_VLD <= 1'b0;
        end
        else if (load_data) begin
            DOUT_VLD  <= rx_data_vld; 
        end
    end

    //---------------------------------------------------------------------------------------------------------------------
    // SPI MASTER FSM
    //---------------------------------------------------------------------------------------------------------------------

    // PRESENT STATE REGISTER
    always_ff @(posedge CLK) begin : fsm_present_state_p
        if (RST) begin
            present_state <= idle;
        end
        else if (load_data) begin
            present_state  <= next_state;
        end
    end

    // NEXT STATE LOGIC
    always_comb begin : fsm_next_state_p
        unique case (present_state)
            idle: begin
                next_state = (DIN_VLD)? first_edge : idle;
            end
            first_edge: begin
                next_state = (sys_clk_cnt_max)? second_edge : first_edge;
            end
            second_edge: begin
                next_state = (sys_clk_cnt_max)? ((bit_cnt_max)? transmit_end : first_edge) : second_edge;
            end
            transmit_end: begin
                next_state = (sys_clk_cnt_max)? transmit_gap : transmit_end;
            end
            transmit_gap: begin
                next_state = (sys_clk_cnt_max)? idle : transmit_gap;
            end
            default: begin
                next_state = idle;
            end
        endcase 
    end

    // OUTPUTS LOGIC
    always_comb begin : fsm_outputs_p
        unique case (present_state)
            idle: begin
                master_ready   = 1'b1;
                chip_select_n  = ~din_last_reg_n;
                spi_clk_rst    = 1'b1;
                first_edge_en  = 1'b0;
                second_edge_en = 1'b0;
                rx_data_vld    = 1'b0;
            end
            first_edge: begin
                master_ready   = 1'b0;
                chip_select_n  = 1'b0;
                spi_clk_rst    = 1'b0;
                first_edge_en  = sys_clk_cnt_max;
                second_edge_en = 1'b0;
                rx_data_vld    = 1'b0;
            end
            second_edge: begin
                master_ready   <= 1'b0;
                chip_select_n  <= 1'b0;
                spi_clk_rst    <= 1'b0;
                first_edge_en  <= 1'b0;
                second_edge_en <= sys_clk_cnt_max;
                rx_data_vld    <= 1'b0;
            end
            transmit_end: begin
                master_ready   <= 1'b0;
                chip_select_n  <= 1'b0;
                spi_clk_rst    <= 1'b1;
                first_edge_en  <= 1'b0;
                second_edge_en <= 1'b0;
                rx_data_vld    <= sys_clk_cnt_max;
            end
            transmit_gap: begin
                master_ready   <= 1'b0;
                chip_select_n  <= ~din_last_reg_n;
                spi_clk_rst    <= 1'b1;
                first_edge_en  <= 1'b0;
                second_edge_en <= 1'b0;
                rx_data_vld    <= 1'b0;
            end
            default: begin
                master_ready   <= 1'b0;
                chip_select_n  <= ~din_last_reg_n;
                spi_clk_rst    <= 1'b1;
                first_edge_en  <= 1'b0;
                second_edge_en <= 1'b0;
                rx_data_vld    <= 1'b0;
            end
        endcase 
    end

endmodule