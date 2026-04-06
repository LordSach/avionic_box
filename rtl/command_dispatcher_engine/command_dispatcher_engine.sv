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
// PRODUCT      :   Command Dispatcher Engine
// FILE         :   command_dispatcher_engine.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   The module which reads and write to OBC and dispatches the commands
//                  to other submodules.
//
// ************************************************************************************************************************
//
// REVISIONS:
//
//  Date           Developer               Description
//  -----------    --------------------    -----------
//  6-APR-2026    Sachith Rathnayake      Creation
//
// ************************************************************************************************************************

`timescale 1ns/1ps

module command_dispatcher_engine (
    rst_n,
    clk,

    // OBC SPI Interface 0
    i_spi_0_clk,
    o_spi_0_miso,
    i_spi_0_mosi,
    i_spi_0_cs_n,

    // OBC SPI Interface 1
    i_spi_1_clk,
    o_spi_1_miso,
    i_spi_1_mosi,
    i_spi_1_cs_n

    // Sensor Double Buffer Access Contoller Interface

    // Actuator Double Buffer Access Controller Interface

    // Control paths for each ADC Manager 

    // Control paths for each PWM Driver Engine

    // interface for Built In Test Module

);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // OBC SPI Interface 0 parameters
    parameter SPI_MODE_0 = 3;

    // OBC SPI Interface 1 parameters
    parameter SPI_MODE_1 = 3;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // comman local parameters
    localparam BYTE_WIDTH  = 8;
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input    logic                   rst_n;          // FPGA Reset; active low
    input    logic                   clk;            // FPGA Clock

    // OBC SPI Interface 0
    input    logic                   i_spi_0_clk;
    output   logic                   o_spi_0_miso;
    input    logic                   i_spi_0_mosi;
    input    logic                   i_spi_0_cs_n;

    // OBC SPI Interface 1
    input    logic                   i_spi_1_clk;
    output   logic                   o_spi_1_miso;
    input    logic                   i_spi_1_mosi;
    input    logic                   i_spi_1_cs_n;

    // Sensor Double Buffer Access Contoller Interface

    // Actuator Double Buffer Access Controller Interface

    // Control paths for each ADC Manager 

    // Control paths for each PWM Driver Engine

    // interface for Built In Test Module
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // for spi_slave_phy_0
    logic                   ex_valid_0;
    logic [BYTE_WIDTH-1:0]  rx_byte_0;
    logic                   tx_valid_0;
    logic [BYTE_WIDTH-1:0]  tx_byte_0;

    // for spi_slave_phy_1
    logic                   ex_valid_1;
    logic [BYTE_WIDTH-1:0]  rx_byte_1;
    logic                   tx_valid_1;
    logic [BYTE_WIDTH-1:0]  tx_byte_1;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Sub Module Instantiation 
    //---------------------------------------------------------------------------------------------------------------------
    
    spi_slave_phy #(
        .SPI_MODE(SPI_MODE_0)
    ) spi_slave_phy_0 (
        // Control/Data Signals,
        .rst_n(rst_n),    // FPGA Reset, active low
        .clk(clk),      // FPGA Clock

        .o_rx_valid(ex_valid_0),    // Data Valid pulse (1 clock cycle)
        .o_rx_byte(rx_byte_0),  // Byte received on MOSI
        .i_tx_valid(tx_valid_0),    // Data Valid pulse to register i_tx_byte
        .i_tx_byte(tx_byte_0),  // Byte to serialize to MISO.

        // SPI Interface
        .i_spi_clk(i_spi_0_clk),
        .o_spi_miso(o_spi_0_miso),
        .i_spi_mosi(i_spi_0_mosi),
        .i_spi_cs_n(i_spi_0_cs_n)        // active low
    );

    spi_slave_phy #(
        .SPI_MODE(SPI_MODE_1)
    ) spi_slave_phy_1 (
        // Control/Data Signals,
        .rst_n(rst_n),    // FPGA Reset, active low
        .clk(clk),      // FPGA Clock

        .o_rx_valid(ex_valid_1),    // Data Valid pulse (1 clock cycle)
        .o_rx_byte(rx_byte_1),  // Byte received on MOSI
        .i_tx_valid(tx_valid_1),    // Data Valid pulse to register i_tx_byte
        .i_tx_byte(tx_byte_1),  // Byte to serialize to MISO.

        // SPI Interface
        .i_spi_clk(i_spi_1_clk),
        .o_spi_miso(o_spi_1_miso),
        .i_spi_mosi(i_spi_1_mosi),
        .i_spi_cs_n(i_spi_1_cs_n)        // active low
    );
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    

endmodule