///////////////////////////////////////////////////////////////////////////////
// Description:       Simple test bench for SPI Master PHY core module
///////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
module spi_master_phy_core_tb ();
  
  parameter SPI_MODE = 3; // CPOL = 1, CPHA = 1
  parameter CLKS_PER_HALF_BIT = 4;  // 6.25 MHz
  parameter MAIN_CLK_DELAY = 2;  // 25 MHz

  logic r_Rst_L     = 1'b0;  
  logic w_SPI_Clk;
  logic r_Clk       = 1'b0;
  logic w_SPI_MOSI;

  // Master Specific
  logic [7:0] r_Master_TX_Byte = 0;
  logic r_Master_TX_DV = 1'b0;
  logic w_Master_TX_Ready;
  logic r_Master_RX_DV;
  logic [7:0] r_Master_RX_Byte;

  // Clock Generators:
  always #(MAIN_CLK_DELAY) r_Clk = ~r_Clk;

  // Instantiate UUT
  spi_master_phy_core #(
    .SPI_MODE(SPI_MODE),
    .CLKS_PER_HALF_BIT(CLKS_PER_HALF_BIT)
  ) spi_master_phy_core_uut (
   // Control/Data Signals,
   .rst_n(r_Rst_L),     // FPGA Reset
   .clk(r_Clk),         // FPGA Clock
   
   // TX (MOSI) Signals
   .i_tx_byte(r_Master_TX_Byte),     // Byte to transmit on MOSI
   .i_tx_valid(r_Master_TX_DV),         // Data Valid Pulse with i_TX_Byte
   .o_tx_ready(w_Master_TX_Ready),   // Transmit Ready for Byte
   
   // RX (MISO) Signals
   .o_rx_valid(r_Master_RX_DV),       // Data Valid pulse (1 clock cycle)
   .o_rx_byte(r_Master_RX_Byte),   // Byte received on MISO

   // SPI Interface
   .o_spi_clk(w_SPI_Clk),
   .i_spi_miso(w_SPI_MOSI),
   .o_spi_mosi(w_SPI_MOSI)
   );


  // Sends a single byte from master.
  task SendSingleByte(input [7:0] data);
    @(posedge r_Clk);
    r_Master_TX_Byte <= data;
    r_Master_TX_DV   <= 1'b1;
    @(posedge r_Clk);
    r_Master_TX_DV <= 1'b0;
    @(posedge w_Master_TX_Ready);
  endtask // SendSingleByte

  int mismatch_count;
  
  initial
    begin
      // Required for EDA Playground
      $dumpfile("dump.vcd"); 
      $dumpvars;
      
      repeat(10) @(posedge r_Clk);
      r_Rst_L  = 1'b0;
      repeat(10) @(posedge r_Clk);
      r_Rst_L          = 1'b1;


      mismatch_count = 0;
      
      // Test single byte
      SendSingleByte(8'hC1);
      $display("Sent out 0xC1, Received 0x%X", r_Master_RX_Byte); 

      if(r_Master_RX_Byte != 8'hC1) mismatch_count++;
      
      // Test double byte
      SendSingleByte(8'hBE);
      $display("Sent out 0xBE, Received 0x%X", r_Master_RX_Byte); 
      if(r_Master_RX_Byte != 8'hBE) mismatch_count++;

      SendSingleByte(8'hEF);
      $display("Sent out 0xEF, Received 0x%X", r_Master_RX_Byte); 
      if(r_Master_RX_Byte != 8'hEF) mismatch_count++;

      if(mismatch_count>0) begin
        $display("\n\n%d number of Mismatches found. \n\n\t\tTest: FAILED\n\n", mismatch_count); 
      end
      else begin
        $display("\n\n\t\tTest: PASSED\n\n"); 
      end

      repeat(10) @(posedge r_Clk);
      $finish();      
    end // initial begin

endmodule // SPI_Slave
