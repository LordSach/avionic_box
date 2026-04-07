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
// PRODUCT      :   ECC Decoder
// FILE         :   ecc_dec.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   ECC decoder with SECDED
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

module ecc_dec (
    d_i,              //ecoded data word input
    q_o,              //corrected information vector output
    syndrome_o,       //syndrome output
    sb_err_o,         //single bit error detected
    db_err_o,         //double bit error detected
    sb_fix_o          //single information bit error corrected
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
    
    input   logic [n  :0]   d_i;
    output  logic [K-1:0]   q_o;
    output  logic [m  :0]   syndrome_o;
    output  logic           sb_err_o;
    output  logic           db_err_o;
    output  logic           sb_fix_o;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic [K-1:0] ibv;
    logic [m-1:0] pbv;
    logic         p0;
    logic [n  :0] cw;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Functions
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

    function is_power_of_2(
        input int arg
    );
        is_power_of_2 = (arg & (arg-1)) == 0;
    endfunction

    function [n:1] gen_codeword;
    input [K-1:0] ibv;
    input [m  :1] pbv;

    integer i,j;
    begin
        //This function puts the information and parity bits vector at the correct location

        //clear all bits
        gen_codeword = 0;

        //store information bits
        j=0; //information vector bit index
        for (i=1; i<= n; i=i+1)
        begin
            if (!is_power_of_2(i))
            begin
                gen_codeword[i] = ibv[j];
                j = j+1;
            end
        end //next i


        //store parity bits
        //put parity vector at power-of-2 locations
        for (i=1; i<=m; i=i+1)
            gen_codeword[2**(i-1)] = pbv[i];
    end
    endfunction //gen_codeword

    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    assign ibv = d_i[0 +: K];
    assign pbv = d_i[K +: m];
    assign p0  = d_i[n];
    assign cw  = {p0, gen_codeword(ibv, pbv)};


    ecc_dec_core #(K,LATENCY,0) ecc_dec_inst (
    .rst_ni     ( 1'b0       ),
    .clk_i      ( 1'b0       ),
    .clkena_i   ( 1'b0       ),
    .d_i        ( cw         ),
    .q_o        ( q_o        ),
    .syndrome_o ( syndrome_o ),
    .sb_err_o   ( sb_err_o   ),
    .db_err_o   ( db_err_o   ),
    .sb_fix_o   ( sb_fix_o   )
    );

endmodule