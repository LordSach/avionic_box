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
// PRODUCT      :   ECC Encoder
// FILE         :   ecc_enc.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   ECC Encoder module
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

module ecc_enc (
    d_i,              //information bit vector input
    q_o               //encoded data word output
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter K       = 8; //Information bit vector size
    parameter LATENCY = 0; //0: no latency (combinatorial design)
                        //1: registered outputs
                        //2: registered inputs+outputs
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam m = calculate_m(K);
    localparam n = m + K;
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------

    /*verilator lint_off VARHIDDEN*/
    function integer calculate_m;
    input integer k;

    integer m;
    begin
    m=1;
    while (2**m < m+k+1) m=m+1;

    calculate_m = m;
    end
    endfunction //calculate_m
    /*verilator lint_on VARHIDDEN*/

    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic   [K-1:0] d_i;
    output  logic   [n  :0] q_o;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic [m:1] p;
    logic       p0;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    /*verilator lint_off PINCONNECTEMPTY*/
    ecc_enc_core #(K) ecc_enc_inst (
        .d_i  (d_i ),
        .q_o  ( ),
        .p_o  ( p  ),
        .p0_o ( p0 )
    );
    /*verilator lint_on PINCONNECTEMPTY*/
    
    assign q_o = {p0,p,d_i};

endmodule