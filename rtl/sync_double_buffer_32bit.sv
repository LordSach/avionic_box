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
// PRODUCT      :   sync_double_buffer_32bit
// FILE         :   sync_double_buffer_32bit.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   Simple reusable and expandable design concept for double buffer
//
// ************************************************************************************************************************
//
// REVISIONS:
//
//  Date           Developer               Description
//  -----------    --------------------    -----------
//  3-Apr-2026    Sachith Rathnayake      Design
//
// ************************************************************************************************************************

`timescale 1ns/1ps

module sync_double_buffer_32bit (
    clk,
    rst_n,

    // writer interface
    wr_en_i,
    wr_data_i,
    wr_addr_i,
    wr_done_i,

    // reader interface
    rd_en_i,
    rd_data_o,
    rd_addr_i,
    rd_done_i
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    //NA
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter DATA_WIDTH    = 32;
    parameter BUFFER_DEPTH  = 16;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam ADDR_WIDTH = $clog2(BUFFER_DEPTH);
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic                   clk;
    input   logic                   rst_n;

    // writer interface
    input   logic                   wr_en_i;
    input   logic [DATA_WIDTH-1:0]  wr_data_i;
    input   logic [ADDR_WIDTH-1:0]  wr_addr_i;
    input   logic                   wr_done_i;

    // reader interface
    input   logic                   rd_en_i;
    output  logic [DATA_WIDTH-1:0]  rd_data_o;
    input   logic [ADDR_WIDTH-1:0]  rd_addr_i;
    input   logic                   rd_done_i;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // Two buffers
    logic [DATA_WIDTH-1:0]          buffer0 [0:BUFFER_DEPTH-1];
    logic [DATA_WIDTH-1:0]          buffer1 [0:BUFFER_DEPTH-1];

    // toggle to track which buffer is active for reading
    logic                           active_buf; // if 0 => read from buffer0, or if 1 => read from buffer1
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    always_ff @(posedge clk) begin : ACTIVE_BUF_TOGGLE
        if (!rst_n) begin
            active_buf  <= 1'b0;
        end
        else begin
            // toggle active_buf when wr_done_i pulse is asserted and rd_en_i is not asserted 
            active_buf  <= (wr_done_i & !rd_en_i)? ~active_buf : active_buf;        
        end
    end

    always_ff @(posedge clk) begin : WRITE_LOGIC_BLOCK
        if (wr_en_i) begin
            // selection of which buffer to write
            if (active_buf) begin
                buffer0[wr_addr_i] <= wr_data_i;
            end
            else begin
                buffer1[wr_addr_i] <= wr_data_i;
            end
        end 
    end

    always_comb begin : READ_COMB_LOGIC
        if (active_buf) begin
            rd_data_o = buffer1[rd_addr_i];
        end
        else begin
            rd_data_o = buffer0[rd_addr_i];
        end
    end

endmodule