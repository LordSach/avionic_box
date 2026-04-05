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
// Description: SPI (Serial Peripheral Interface) Master
//              With single chip-select (AKA Slave Select) capability
//
//              Supports arbitrary length byte transfers.
// 
//              Instantiates a SPI Master and adds single CS.
//              If multiple CS signals are needed, will need to use different
//              module, OR multiplex the CS from this at a higher level.
//
// Note:        i_Clk must be at least 2x faster than i_SPI_Clk
//
// Parameters:  SPI_MODE, can be 0, 1, 2, or 3.  See above.
//              Can be configured in one of 4 modes:
//              Mode | Clock Polarity (CPOL/CKP) | Clock Phase (CPHA)
//               0   |             0             |        0
//               1   |             0             |        1
//               2   |             1             |        0
//               3   |             1             |        1
//
//              CLKS_PER_HALF_BIT - Sets frequency of o_SPI_Clk.  o_SPI_Clk is
//              derived from i_Clk.  Set to integer number of clocks for each
//              half-bit of SPI data.  E.g. 100 MHz i_Clk, CLKS_PER_HALF_BIT = 2
//              would create o_SPI_CLK of 25 MHz.  Must be >= 2
//
//              MAX_BYTES_PER_CS - Set to the maximum number of bytes that
//              will be sent during a single CS-low pulse.
// 
//              CS_INACTIVE_CLKS - Sets the amount of time in clock cycles to
//              hold the state of Chip-Selct high (inactive) before next 
//              command is allowed on the line.  Useful if chip requires some
//              time when CS is high between trasnfers.
// ************************************************************************************************************************
//
// REVISIONS:
//
//  Date           Developer               Description
//  -----------    --------------------    -----------
//  5-APR-2026    Sachith Rathnayake      Creation
//
// ************************************************************************************************************************

`timescale 1ns/1ps

module spi_master_phy (
    // Control/Data Signals,
    i_Rst_L,     // FPGA Reset
    i_Clk,       // FPGA Clock
     
    // TX (MOSI) Signals
    i_TX_Count,  // # bytes per CS low
    i_TX_Byte,       // Byte to transmit on MOSI
    i_TX_DV,         // Data Valid Pulse with i_TX_Byte
    o_TX_Ready,      // Transmit Ready for next byte
     
    // RX (MISO) Signals
    o_RX_Count,  // Index RX byte
    o_RX_DV,     // Data Valid pulse (1 clock cycle)
    o_RX_Byte,   // Byte received on MISO
  
    // SPI Interface
    o_SPI_Clk,
    i_SPI_MISO,
    o_SPI_MOSI,
    o_SPI_CS_n
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // NA
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter SPI_MODE          = 0;
    parameter CLKS_PER_HALF_BIT = 2;
    parameter MAX_BYTES_PER_CS  = 2;
    parameter CS_INACTIVE_CLKS  = 1;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam IDLE         = 2'b00;
    localparam TRANSFER     = 2'b01;
    localparam CS_INACTIVE  = 2'b10;
    localparam CNTR_WIDTH   = $clog2(MAX_BYTES_PER_CS+1);
    localparam CS_CLKS_W    = $clog2(CS_INACTIVE_CLKS);
    localparam TX_CNT_WIDTH = $clog2(MAX_BYTES_PER_CS+1);
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // NA
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // Control/Data Signals,
    input   logic                   i_Rst_L;     // FPGA Reset
    input   logic                   i_Clk;       // FPGA Clock
    
    // TX (MOSI) Signals
    input   logic [CNTR_WIDTH-1:0]  i_TX_Count;  // # bytes per CS low
    input   logic [7:0]             i_TX_Byte;       // Byte to transmit on MOSI
    input   logic                   i_TX_DV;         // Data Valid Pulse with i_TX_Byte
    output  logic                   o_TX_Ready;      // Transmit Ready for next byte
    
    // RX (MISO) Signals
    output  logic [CNTR_WIDTH-1:0]  o_RX_Count;  // Index RX byte
    output  logic                   o_RX_DV;     // Data Valid pulse (1 clock cycle)
    output  logic [7:0]             o_RX_Byte;   // Byte received on MISO
 
    // SPI Interface
    output  logic                   o_SPI_Clk;
    input   logic                   i_SPI_MISO;
    output  logic                   o_SPI_MOSI;
    output  logic                   o_SPI_CS_n;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic [1:0]                     r_SM_CS;
    logic                           r_CS_n;
    logic [CS_CLKS_W-1:0]           r_CS_Inactive_Count;
    logic [TX_CNT_WIDTH-1:0]        r_TX_Count;
    logic                           w_Master_Ready;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    // Instantiate Master
    spi_master_phy_core 
    #(.SPI_MODE(SPI_MODE),
        .CLKS_PER_HALF_BIT(CLKS_PER_HALF_BIT)
        ) SPI_Master_Inst
    (
    // Control/Data Signals,
    .i_Rst_L(i_Rst_L),     // FPGA Reset
    .i_Clk(i_Clk),         // FPGA Clock
    
    // TX (MOSI) Signals
    .i_TX_Byte(i_TX_Byte),         // Byte to transmit
    .i_TX_DV(i_TX_DV),             // Data Valid Pulse 
    .o_TX_Ready(w_Master_Ready),   // Transmit Ready for Byte
    
    // RX (MISO) Signals
    .o_RX_DV(o_RX_DV),       // Data Valid pulse (1 clock cycle)
    .o_RX_Byte(o_RX_Byte),   // Byte received on MISO

    // SPI Interface
    .o_SPI_Clk(o_SPI_Clk),
    .i_SPI_MISO(i_SPI_MISO),
    .o_SPI_MOSI(o_SPI_MOSI)
    );

    // Purpose: Control CS line using State Machine
  always @(posedge i_Clk or negedge i_Rst_L)
  begin
    if (~i_Rst_L)
    begin
      r_SM_CS <= IDLE;
      r_CS_n  <= 1'b1;   // Resets to high
      r_TX_Count <= 0;
      r_CS_Inactive_Count <= CS_INACTIVE_CLKS;
    end
    else
    begin

      case (r_SM_CS)      
      IDLE:
        begin
          if (r_CS_n & i_TX_DV) // Start of transmission
          begin
            r_TX_Count <= i_TX_Count - 1'b1; // Register TX Count
            r_CS_n     <= 1'b0;       // Drive CS low
            r_SM_CS    <= TRANSFER;   // Transfer bytes
          end
        end

      TRANSFER:
        begin
          // Wait until SPI is done transferring do next thing
          if (w_Master_Ready)
          begin
            if (r_TX_Count > 0)
            begin
              if (i_TX_DV)
              begin
                r_TX_Count <= r_TX_Count - 1'b1;
              end
            end
            else
            begin
              r_CS_n  <= 1'b1; // we done, so set CS high
              r_CS_Inactive_Count <= CS_INACTIVE_CLKS;
              r_SM_CS             <= CS_INACTIVE;
            end // else: !if(r_TX_Count > 0)
          end // if (w_Master_Ready)
        end // case: TRANSFER

      CS_INACTIVE:
        begin
          if (r_CS_Inactive_Count > 0)
          begin
            r_CS_Inactive_Count <= r_CS_Inactive_Count - 1'b1;
          end
          else
          begin
            r_SM_CS <= IDLE;
          end
        end

      default:
        begin
          r_CS_n  <= 1'b1; // we done, so set CS high
          r_SM_CS <= IDLE;
        end
      endcase // case (r_SM_CS)
    end
  end // always @ (posedge i_Clk or negedge i_Rst_L)


  // Purpose: Keep track of RX_Count
  always @(posedge i_Clk)
  begin
    begin
      if (r_CS_n)
      begin
        o_RX_Count <= 0;
      end
      else if (o_RX_DV)
      begin
        o_RX_Count <= o_RX_Count + 1'b1;
      end
    end
  end

  assign o_SPI_CS_n = r_CS_n;

  assign o_TX_Ready  = ((r_SM_CS == IDLE) | (r_SM_CS == TRANSFER && w_Master_Ready == 1'b1 && r_TX_Count > 0)) & ~i_TX_DV;

endmodule