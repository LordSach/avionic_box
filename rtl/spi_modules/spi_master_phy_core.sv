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
// FILE         :   spi_master_phy_core.sv
// AUTHOR       :   Sachith Rathnayake
// Description: SPI (Serial Peripheral Interface) Master
//              Creates master based on input configuration.
//              Sends a byte one bit at a time on MOSI
//              Will also receive byte data one bit at a time on MISO.
//              Any data on input byte will be shipped out on MOSI.
//
//              To kick-off transaction, user must pulse i_tx_valid.
//              This module supports multi-byte transmissions by pulsing
//              i_tx_valid and loading up i_tx_byte when o_tx_ready is high.
//
//              This module is only responsible for controlling Clk, MOSI, 
//              and MISO.  If the SPI peripheral requires a chip-select, 
//              this must be done at a higher level.
//
// Note:        clk must be at least 2x faster than i_SPI_Clk
//
// Parameters:  SPI_MODE, can be 0, 1, 2, or 3.  See above.
//              Can be configured in one of 4 modes:
//              Mode | Clock Polarity (CPOL/CKP) | Clock Phase (CPHA)
//               0   |             0             |        0
//               1   |             0             |        1
//               2   |             1             |        0
//               3   |             1             |        1
//              More: https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus#Mode_numbers
//              CLKS_PER_HALF_BIT - Sets frequency of o_spi_clk.  o_spi_clk is
//              derived from clk.  Set to integer number of clocks for each
//              half-bit of SPI data.  E.g. 100 MHz clk, CLKS_PER_HALF_BIT = 2
//              would create o_SPI_CLK of 25 MHz.  Must be >= 2
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

module spi_master_phy_core (
  // Control/Data Signals,
  rst_n,     // FPGA Reset
  clk,       // FPGA Clock

  // TX (MOSI) Signals
  i_tx_byte,        // Byte to transmit on MOSI
  i_tx_valid,          // Data Valid Pulse with i_tx_byte
  o_tx_ready,       // Transmit Ready for next byte

  // RX (MISO) Signals
  o_rx_valid,     // Data Valid pulse (1 clock cycle)
  o_rx_byte,   // Byte received on MISO

  // SPI Interface
  o_spi_clk,
  i_spi_miso,
  o_spi_mosi
);

  //---------------------------------------------------------------------------------------------------------------------
  // Global constant headers
  //---------------------------------------------------------------------------------------------------------------------
  
  // NA
  
  //---------------------------------------------------------------------------------------------------------------------
  // parameter definitions
  //---------------------------------------------------------------------------------------------------------------------
  
  parameter SPI_MODE = 0;
  parameter CLKS_PER_HALF_BIT = 2;
  
  //---------------------------------------------------------------------------------------------------------------------
  // localparam definitions
  //---------------------------------------------------------------------------------------------------------------------
  
  // NA
  
  //---------------------------------------------------------------------------------------------------------------------
  // type definitions
  //---------------------------------------------------------------------------------------------------------------------
  
  // NA
  
  //---------------------------------------------------------------------------------------------------------------------
  // I/O signals
  //---------------------------------------------------------------------------------------------------------------------
  
  // Control/Data Signals,
  input   logic         rst_n;     // FPGA Reset
  input   logic         clk;       // FPGA Clock

  // TX (MOSI) Signals
  input   logic [7:0]   i_tx_byte;        // Byte to transmit on MOSI
  input   logic         i_tx_valid;          // Data Valid Pulse with i_tx_byte
  output  logic         o_tx_ready;       // Transmit Ready for next byte

  // RX (MISO) Signals
  output  logic         o_rx_valid;     // Data Valid pulse (1 clock cycle)
  output  logic [7:0]   o_rx_byte;   // Byte received on MISO

  // SPI Interface
  output  logic         o_spi_clk;
  input   logic         i_spi_miso;
  output  logic         o_spi_mosi;
  
  //---------------------------------------------------------------------------------------------------------------------
  // Internal signals
  //---------------------------------------------------------------------------------------------------------------------
  
  // SPI Interface (All Runs at SPI Clock Domain)
  logic w_CPOL;     // Clock polarity
  logic w_CPHA;     // Clock phase

  logic [$clog2(CLKS_PER_HALF_BIT*2)-1:0] r_SPI_Clk_Count;
  logic r_SPI_Clk;
  logic [4:0] r_SPI_Clk_Edges;
  logic r_Leading_Edge;
  logic r_Trailing_Edge;
  logic       r_TX_DV;
  logic [7:0] r_TX_Byte;

  logic [2:0] r_RX_Bit_Count;
  logic [2:0] r_TX_Bit_Count;
    
  //---------------------------------------------------------------------------------------------------------------------
  // Implementation
  //---------------------------------------------------------------------------------------------------------------------
    
  // CPOL: Clock Polarity
  // CPOL=0 means clock idles at 0, leading edge is rising edge.
  // CPOL=1 means clock idles at 1, leading edge is falling edge.
  assign w_CPOL  = (SPI_MODE == 2) | (SPI_MODE == 3);

  // CPHA: Clock Phase
  // CPHA=0 means the "out" side changes the data on trailing edge of clock
  //              the "in" side captures data on leading edge of clock
  // CPHA=1 means the "out" side changes the data on leading edge of clock
  //              the "in" side captures data on the trailing edge of clock
  assign w_CPHA  = (SPI_MODE == 1) | (SPI_MODE == 3);



  // Purpose: Generate SPI Clock correct number of times when DV pulse comes
  always @(posedge clk or negedge rst_n)
  begin
    if (~rst_n)
    begin
      o_tx_ready      <= 1'b0;
      r_SPI_Clk_Edges <= 0;
      r_Leading_Edge  <= 1'b0;
      r_Trailing_Edge <= 1'b0;
      r_SPI_Clk       <= w_CPOL; // assign default state to idle state
      r_SPI_Clk_Count <= 0;
    end
    else
    begin

      // Default assignments
      r_Leading_Edge  <= 1'b0;
      r_Trailing_Edge <= 1'b0;
      
      if (i_tx_valid)
      begin
        o_tx_ready      <= 1'b0;
        r_SPI_Clk_Edges <= 16;  // Total # edges in one byte ALWAYS 16
      end
      else if (r_SPI_Clk_Edges > 0)
      begin
        o_tx_ready <= 1'b0;
        
        if (r_SPI_Clk_Count == CLKS_PER_HALF_BIT*2-1)
        begin
          r_SPI_Clk_Edges <= r_SPI_Clk_Edges - 1'b1;
          r_Trailing_Edge <= 1'b1;
          r_SPI_Clk_Count <= 0;
          r_SPI_Clk       <= ~r_SPI_Clk;
        end
        else if (r_SPI_Clk_Count == CLKS_PER_HALF_BIT-1)
        begin
          r_SPI_Clk_Edges <= r_SPI_Clk_Edges - 1'b1;
          r_Leading_Edge  <= 1'b1;
          r_SPI_Clk_Count <= r_SPI_Clk_Count + 1'b1;
          r_SPI_Clk       <= ~r_SPI_Clk;
        end
        else
        begin
          r_SPI_Clk_Count <= r_SPI_Clk_Count + 1'b1;
        end
      end  
      else
      begin
        o_tx_ready <= 1'b1;
      end
      
      
    end // else: !if(~rst_n)
  end // always @ (posedge clk or negedge rst_n)


  // Purpose: Register i_tx_byte when Data Valid is pulsed.
  // Keeps local storage of byte in case higher level module changes the data
  always @(posedge clk or negedge rst_n)
  begin
    if (~rst_n)
    begin
      r_TX_Byte <= 8'h00;
      r_TX_DV   <= 1'b0;
    end
    else
      begin
        r_TX_DV <= i_tx_valid; // 1 clock cycle delay
        if (i_tx_valid)
        begin
          r_TX_Byte <= i_tx_byte;
        end
      end // else: !if(~rst_n)
  end // always @ (posedge clk or negedge rst_n)


  // Purpose: Generate MOSI data
  // Works with both CPHA=0 and CPHA=1
  always @(posedge clk or negedge rst_n)
  begin
    if (~rst_n)
    begin
      o_spi_mosi     <= 1'b0;
      r_TX_Bit_Count <= 3'b111; // send MSb first
    end
    else
    begin
      // If ready is high, reset bit counts to default
      if (o_tx_ready)
      begin
        r_TX_Bit_Count <= 3'b111;
      end
      // Catch the case where we start transaction and CPHA = 0
      else if (r_TX_DV & ~w_CPHA)
      begin
        o_spi_mosi     <= r_TX_Byte[3'b111];
        r_TX_Bit_Count <= 3'b110;
      end
      else if ((r_Leading_Edge & w_CPHA) | (r_Trailing_Edge & ~w_CPHA))
      begin
        r_TX_Bit_Count <= r_TX_Bit_Count - 1'b1;
        o_spi_mosi     <= r_TX_Byte[r_TX_Bit_Count];
      end
    end
  end


  // Purpose: Read in MISO data.
  always @(posedge clk or negedge rst_n)
  begin
    if (~rst_n)
    begin
      o_rx_byte       <= 8'h00;
      o_rx_valid      <= 1'b0;
      r_RX_Bit_Count  <= 3'b111;
    end
    else
    begin

      // Default Assignments
      o_rx_valid   <= 1'b0;

      if (o_tx_ready) // Check if ready is high, if so reset bit count to default
      begin
        r_RX_Bit_Count <= 3'b111;
      end
      else if ((r_Leading_Edge & ~w_CPHA) | (r_Trailing_Edge & w_CPHA))
      begin
        o_rx_byte[r_RX_Bit_Count] <= i_spi_miso;  // Sample data
        r_RX_Bit_Count            <= r_RX_Bit_Count - 1'b1;
        if (r_RX_Bit_Count == 3'b000)
        begin
          o_rx_valid   <= 1'b1;   // Byte done, pulse Data Valid
        end
      end
    end
  end
  
  
  // Purpose: Add clock delay to signals for alignment.
  always @(posedge clk or negedge rst_n)
  begin
    if (~rst_n)
    begin
      o_spi_clk  <= w_CPOL;
    end
    else
      begin
        o_spi_clk <= r_SPI_Clk;
      end
  end

endmodule