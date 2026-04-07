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
// PRODUCT      :   ECC Encoder core
// FILE         :   ecc_enc_core.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   Ecc Encoder Core module
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

module ecc_enc_core (
    d_i,      //information bit vector input
    q_o,      //encoded data word output

    p_o,      //parity vector output
    p0_o      //extended parity bit
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter K       = 8; //Information bit vector size
    parameter P0_LSB  = 1; //0: p0 is located at MSB
                           //1: p0 is located at LSB
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter m = calculate_m(K);
    parameter n = m + K;
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic [K-1:0] d_i;      //information bit vector input
    output  logic [n  :0] q_o;      //encoded data word output

    output  logic [m  :1] p_o;      //parity vector output
    output  logic         p0_o;      //extended parity bit
    
    //---------------------------------------------------------------------------------------------------------------------
    // Functions
    //---------------------------------------------------------------------------------------------------------------------

    /*verilator lint_off VARHIDDEN*/
    function integer calculate_m;
    input integer k;

    integer m;
    begin
    m=1;
    while (2**m < m+k+1) m++;

    calculate_m = m;
    end
    endfunction //calculate_m
    /*verilator lint_on VARHIDDEN*/


    function [n:1] store_dbits_in_codeword;
    input [K-1:0] d;

    integer bit_idx, cw_idx;
    begin
    //This function puts the information bits vector in the correct location
    //Information bits are stored in non-power-of-2 locations

    //clear all bits
    store_dbits_in_codeword = 0;

    bit_idx=0; //information vector bit index
    for (cw_idx=1; cw_idx<=n; cw_idx++)
        if (2**$clog2(cw_idx) != cw_idx)
        store_dbits_in_codeword[cw_idx] = d[bit_idx++];
    end
    endfunction //store_dbits_in_codeword


    function [m:1] calculate_p;
    input [n:1] cw;

    integer p_idx, cw_idx;
    begin
    //clear p
    calculate_p = 0;

    for (p_idx =1; p_idx <=m; p_idx++)  //parity-index
    for (cw_idx=1; cw_idx<=n; cw_idx++) //codeword-index
        if (|(2**(p_idx-1) & cw_idx)) calculate_p[p_idx] = calculate_p[p_idx] ^ cw[cw_idx];
    end
    endfunction //calculate_p


    function [n:1] store_p_in_codeword;
    input [n:1] cw;
    input [m:1] p;

    integer i;
    begin
    //databits don't change ... copy into codeword
    store_p_in_codeword = cw;

    //put parity vector at power-of-2 locations
    for (i=1; i<=m; i=i+1)
        store_p_in_codeword[2**(i-1)] = p[i];
    end
    endfunction //store_p_in_codeword

    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic [n:1] cw_w_dbits; //codeword with loaded data bits
    logic [n:1] cw;         //codeword with information + parity bits
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    /*
    Below diagram indicates the locations of the parity and data bits
    in the final 'p' vector.
    It also shows what databits each parity bit operates on

        1  2  3  4  5  6  7  8  9 10 11 12 13  14  15
    p1 p2 d1 p4 d2 d3 d4 p8 d5 d6 d7 d8 d9 d10 d11
    p1  x     x     x     x     x     x     x       x
    p2     x  x        x  x        x  x         x   x
    p4           x  x  x  x              x  x   x   x
    p8                       x  x  x  x  x  x   x   x
    */

    //Step 1: Load all databits in codeword
    assign cw_w_dbits = store_dbits_in_codeword(d_i);

    //Step 2: Calculate p-vector
    assign p_o = calculate_p(cw_w_dbits);

    //Step 3: Store p-vector in codeword
    assign cw = store_p_in_codeword(cw_w_dbits, p_o);

    //Step 4: Calculate p0 (extended parity bit)
    //        and store it in the codeword
    assign p0_o = ^cw;
    assign q_o  = P0_LSB ? {cw,p0_o} : {p0_o,cw};

endmodule