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
// PROJECT      :   Avionix Box
// PRODUCT      :   ADC Manager Engine for AD4111
// FILE         :   adc_manager_engine_ad4111.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   ADC Manager Engine for AD4111 devices
//
// ************************************************************************************************************************
//
// REVISIONS:
//
//  Date           Developer               Description
//  -----------    --------------------    -----------
//  7-APR-2026    Sachith Rathnayake      Creation
//
// ************************************************************************************************************************

`timescale 1ns/1ps

module adc_manager_engine_ad4111 (
    rst_n,
    clk, // 100 MHz to provide 10 MHz SPI clock speed for AD4111
    // writer interface
    wr_en_o,
    wr_data_o,
    wr_addr_o,
    wr_done_o,
    wr_busy,
    wr_timeout_error,

    // reader interface
    rd_en_o,
    rd_data_i,
    rd_addr_o,
    rd_busy,
    rd_timeout_error,

    // SPI Master Interface
    o_spi_clk,
    i_spi_miso,
    o_spi_mosi,
    o_spi_cs_n
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter DATA_WIDTH    = 32;
    parameter BUFFER_DEPTH  = 16;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam ADDR_WIDTH       = $clog2(BUFFER_DEPTH);

    /*
        f_global            - adc manager angine's clock frequency
        f_spi               - desired SPI clock frequency
        CLKS_PER_HALF_BIT   - Clock cycles per half cycle of SPI generated clock

        f_spi               = f_global / (2*CLKS_PER_HALF_BIT)
        CLKS_PER_HALF_BIT   = f_global / (2*f_spi)

                            = 100 / (2 * 10) = 5
    */

    parameter SPI_MODE          = 3;
    parameter CLKS_PER_HALF_BIT = 5;
    parameter MAX_BYTES_PER_CS  = 2;
    parameter CS_INACTIVE_CLKS  = 1;
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic                   rst_n;
    input   logic                   clk;

    // writer interface
    output  logic                   wr_en_o;
    output  logic [DATA_WIDTH-1:0]  wr_data_o;
    output  logic [ADDR_WIDTH-1:0]  wr_addr_o;
    output  logic                   wr_done_o;
    input   logic                   wr_busy;
    input   logic                   wr_timeout_error;

    // reader interface
    output  logic                   rd_en_o;
    input   logic [DATA_WIDTH-1:0]  rd_data_i;
    output  logic [ADDR_WIDTH-1:0]  rd_addr_o;
    input   logic                   rd_busy;
    input   logic                   rd_timeout_error;

    // SPI Master Interface
    output  logic                   o_spi_clk;
    input   logic                   i_spi_miso;
    output  logic                   o_spi_mosi;
    output  logic                   o_spi_cs_n;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // internal signals

    //---------------------------------------------------------------------------------------------------------------------
    // Module Instantiations
    //---------------------------------------------------------------------------------------------------------------------

    spi_master_phy #(
        .SPI_MODE(SPI_MODE),
        .CLKS_PER_HALF_BIT(CLKS_PER_HALF_BIT),
        .MAX_BYTES_PER_CS(MAX_BYTES_PER_CS),
        .CS_INACTIVE_CLKS(CS_INACTIVE_CLKS)
    ) spi_master_phy_int (
        // Control/Data Signals,
        .rst_n(rst_n),     // FPGA Reset
        .clk(clk),       // FPGA Clock
        
        // TX (MOSI) Signals
        .i_tx_count(),  // # bytes per CS low
        .i_tx_byte(),       // Byte to transmit on MOSI
        .i_tx_valid(),         // Data Valid Pulse with i_tx_byte
        .i_tx_ready(),      // Transmit Ready for next byte
        
        // RX (MISO) Signals
        .o_rx_count(),  // Index RX byte
        .o_rx_valid(),     // Data Valid pulse (1 clock cycle)
        .o_rx_byte(),   // Byte received on MISO
    
        // SPI Interface
        .o_spi_clk(o_spi_clk),
        .i_spi_miso(i_spi_miso),
        .o_spi_mosi(o_spi_mosi),
        .o_spi_cs_n(o_spi_cs_n)
    );
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    // implementation

endmodule