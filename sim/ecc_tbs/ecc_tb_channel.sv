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
// PRODUCT      :   ECC Channel for ecc_tb_top
// FILE         :   ecc_tb_channel.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   ECC channel module for ECC tb top, Receive codeword and corrupt either 1 or 2 random bits
//
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

module ecc_tb_channel (
    clk_i,
    nflips_i,
    nflips_o,
    flip1_i,
    flip2_i,
    flip1_o,
    flip2_o,
    d_i,
    q_o
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter n = 8;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam  INT   = 32;
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic           clk_i;

    input   logic [INT-1:0] nflips_i;
    output  logic [INT-1:0] nflips_o;

    input   logic [INT-1:0] flip1_i;
    input   logic [INT-1:0] flip2_i;
    output  logic [INT-1:0] flip1_o;
    output  logic [INT-1:0] flip2_o;

    input   logic [n:0]     d_i;
    output  logic [n:0]     q_o;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic [n:0] cw;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    always @(posedge clk_i)
    begin
        nflips_o <= nflips_i;
        flip1_o  <= flip1_i;
        flip2_o  <= flip2_i;
    end


    always @(posedge clk_i)
    begin
        q_o <= d_i;
        if (nflips_i > 0) q_o[flip1_i] <= ~d_i[flip1_i];
        if (nflips_i > 1) q_o[flip2_i] <= ~d_i[flip2_i];
    end

endmodule

