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
// Description: SPI (Serial Peripheral Interface) Slave
//              Creates slave based on input configuration.
//              Receives a byte one bit at a time on MOSI
//              Will also push out byte data one bit at a time on MISO.  
//              Any data on input byte will be shipped out on MISO.
//              Supports multiple bytes per transaction when CS_n is kept 
//              low during the transaction.
//
// Note:        i_Clk must be at least 4x faster than i_SPI_Clk
//              MISO is tri-stated when not communicating.  Allows for multiple
//              SPI Slaves on the same interface.
//
// Parameters:  SPI_MODE, can be 0, 1, 2, or 3.  See above.
//              Can be configured in one of 4 modes:
//              Mode | Clock Polarity (CPOL/CKP) | Clock Phase (CPHA)
//               0   |             0             |        0
//               1   |             0             |        1
//               2   |             1             |        0
//               3   |             1             |        1
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
   // Control/Data Signals,
   rst_n,    // FPGA Reset, active low
   clk,      // FPGA Clock
   o_rx_valid,    // Data Valid pulse (1 clock cycle)
   o_rx_byte,  // Byte received on MOSI
   i_tx_valid,    // Data Valid pulse to register i_tx_byte
   i_tx_byte,  // Byte to serialize to MISO.

   // SPI Interface
   i_spi_clk,
   o_spi_miso,
   i_spi_mosi,
   i_spi_cs_n        // active low
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    //NA
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter SPI_MODE = 3;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam BYTE_WIDTH       = 8;
    localparam BIT_CNT_WIDTH    = $clog2(BYTE_WIDTH);

    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // NA

    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // Control/Data Signals,
   input    logic                   rst_n;          // FPGA Reset; active low
   input    logic                   clk;            // FPGA Clock
   output   logic                   o_rx_valid;     // Data Valid pulse (1 clock cycle)
   output   logic [BYTE_WIDTH-1:0]  o_rx_byte;      // Byte received on MOSI
   input    logic                   i_tx_valid;     // Data Valid pulse to register i_tx_byte
   input    logic [BYTE_WIDTH-1:0]  i_tx_byte;      // Byte to serialize to MISO.

   // SPI Interface
   input    logic                   i_spi_clk;
   output   logic                   o_spi_miso;
   input    logic                   i_spi_mosi;
   input    logic                   i_spi_cs_n;     // active low

    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    


    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    // SPI Interface (All Runs at SPI Clock Domain)
    logic                           w_CPOL;         // Clock polarity
    logic                           w_CPHA;         // Clock phase
    logic                           w_SPI_Clk;      // Inverted/non-inverted depending on settings
    logic                           w_SPI_MISO_Mux;
  
    logic [BIT_CNT_WIDTH-1:0]       r_RX_Bit_Count;
    logic [BIT_CNT_WIDTH-1:0]       r_TX_Bit_Count;
    logic [BYTE_WIDTH-1:0]          r_Temp_RX_Byte;
    logic [BYTE_WIDTH-1:0]          r_RX_Byte;
    logic                           r_RX_Done;
    logic                           r2_RX_Done;
    logic                           r3_RX_Done;
    logic [BYTE_WIDTH-1:0]          r_TX_Byte;
    logic                           r_SPI_MISO_Bit;
    logic                           r_Preload_MISO;

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

  assign w_SPI_Clk =(w_CPHA) ? ~i_spi_clk : i_spi_clk;



  // Purpose: Recover SPI Byte in SPI Clock Domain
  // Samples line on correct edge of SPI Clock
  always @(posedge w_SPI_Clk or posedge i_spi_cs_n)
  begin
    if (i_spi_cs_n)
    begin
      r_RX_Bit_Count <= 0;
      r_RX_Done      <= 1'b0;
    end
    else
    begin
      r_RX_Bit_Count <= r_RX_Bit_Count + 1;

      // Receive in LSB, shift up to MSB
      r_Temp_RX_Byte <= {r_Temp_RX_Byte[6:0], i_spi_mosi};
    
      if (r_RX_Bit_Count == 3'b111)
      begin
        r_RX_Done <= 1'b1;
        r_RX_Byte <= {r_Temp_RX_Byte[6:0], i_spi_mosi};
      end
      else if (r_RX_Bit_Count == 3'b010)
      begin
        r_RX_Done <= 1'b0;        
      end

    end // else: !if(i_spi_cs_n)
  end // always @ (posedge w_SPI_Clk or posedge i_spi_cs_n)



  // Purpose: Cross from SPI Clock Domain to main FPGA clock domain
  // Assert o_rx_valid for 1 clock cycle when o_rx_byte has valid data.
  always @(posedge clk or negedge rst_n)
  begin
    if (~rst_n)
    begin
      r2_RX_Done <= 1'b0;
      r3_RX_Done <= 1'b0;
      o_rx_valid <= 1'b0;
      o_rx_byte  <= 8'h00;
    end
    else
    begin
      // Here is where clock domains are crossed.
      // This will require timing constraint created, can set up long path.
      r2_RX_Done <= r_RX_Done;

      r3_RX_Done <= r2_RX_Done;

      if (r3_RX_Done == 1'b0 && r2_RX_Done == 1'b1) // rising edge
      begin
        o_rx_valid   <= 1'b1;  // Pulse Data Valid 1 clock cycle
        o_rx_byte <= r_RX_Byte;
      end
      else
      begin
        o_rx_valid <= 1'b0;
      end
    end // else: !if(~rst_n)
  end // always @ (posedge i_Bus_Clk)


  // Control preload signal.  Should be 1 when CS is high, but as soon as
  // first clock edge is seen it goes low.
  always @(posedge w_SPI_Clk or posedge i_spi_cs_n)
  begin
    if (i_spi_cs_n)
    begin
      r_Preload_MISO <= 1'b1;
    end
    else
    begin
      r_Preload_MISO <= 1'b0;
    end
  end


  // Purpose: Transmits 1 SPI Byte whenever SPI clock is toggling
  // Will transmit read data back to SW over MISO line.
  // Want to put data on the line immediately when CS goes low.
  always @(posedge w_SPI_Clk or posedge i_spi_cs_n)
  begin
    if (i_spi_cs_n)
    begin
      r_TX_Bit_Count <= 3'b111;  // Send MSb first
      r_SPI_MISO_Bit <= r_TX_Byte[3'b111];  // Reset to MSb
    end
    else
    begin
      r_TX_Bit_Count <= r_TX_Bit_Count - 1;

      // Here is where data crosses clock domains from clk to w_SPI_Clk
      // Can set up a timing constraint with wide margin for data path.
      r_SPI_MISO_Bit <= r_TX_Byte[r_TX_Bit_Count];

    end // else: !if(i_spi_cs_n)
  end // always @ (negedge w_SPI_Clk or posedge i_SPI_CS_n_SW)


  // Purpose: Register TX Byte when DV pulse comes.  Keeps registed byte in 
  // this module to get serialized and sent back to master.
  always @(posedge clk or negedge rst_n)
  begin
    if (~rst_n)
    begin
      r_TX_Byte <= 8'h00;
    end
    else
    begin
      if (i_tx_valid)
      begin
        r_TX_Byte <= i_tx_byte; 
      end
    end // else: !if(~rst_n)
  end // always @ (posedge clk or negedge rst_n)

  // Preload MISO with top bit of send data when preload selector is high.
  // Otherwise just send the normal MISO data
  assign w_SPI_MISO_Mux = r_Preload_MISO ? r_TX_Byte[3'b111] : r_SPI_MISO_Bit;

  // Tri-state MISO when CS is high.  Allows for multiple slaves to talk.
  assign o_spi_miso = i_spi_cs_n ? 1'bZ : w_SPI_MISO_Mux;

endmodule // spi_slave_phy