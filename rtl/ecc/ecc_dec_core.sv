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
// PRODUCT      :   ECC Decoder Core
// FILE         :   ecc_dec_core.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   Core of the ECC Decoder
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

module ecc_dec_core (
    //clock/reset ports (if LATENCY > 0)
    rst_ni,     //asynchronous reset
    clk_i,      //clock input
    clkena_i,   //clock enable input

    //data ports
    d_i,        //encoded code word input
    q_o,        //information bit vector output
    syndrome_o, //syndrome vector output

    //flags
    sb_err_o,   //single bit error detected
    db_err_o,   //double bit error detected
    sb_fix_o    //repaired error in the information bits
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter K       = 8;  //Information bit vector size
    parameter LATENCY = 0;  //0: no latency (combinatorial design)
                            //1: registered outputs
                            //2: registered inputs+outputs
    parameter P0_LSB  = 1;  //0: p0 is located at MSB
                            //1: p0 is located at LSB

    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter m = calculate_m(K);
    parameter n = m + K;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Functions
    //---------------------------------------------------------------------------------------------------------------------

    /*verilator lint_off VARHIDDEN*/
    function integer calculate_m(input integer k);
    integer m;
    begin
    m=1;
    while (2**m < m+k+1) m++;

    calculate_m = m;
    end
    endfunction //calculate_m
    /*verilator lint_on VARHIDDEN*/


    function [m:1] calculate_syndrome(input [n:0] cw);
    integer p_idx, cw_idx;
    begin
    //clear syndrome
    calculate_syndrome = 0;

    for (p_idx =1; p_idx <=m; p_idx++)  //parity vector index
    for (cw_idx=1; cw_idx<=n; cw_idx++) //code-word index
        if (|(2**(p_idx-1) & cw_idx)) calculate_syndrome[p_idx] = calculate_syndrome[p_idx] ^ cw[cw_idx];
    end
    endfunction //calculate_syndrome


    function [n:0] correct_codeword(input [n:0] cw, input [m:1] syndrome);
    /*
        Correct all bits, including parity bits and extended parity bit.
        This simplifies this section and keeps the logic simple.

        The parity-bits are not used when extracting the information bits vector.
        Dead-logic-removal gets rid of the generated logic for the parity bits.
    */

    //assign code word
    correct_codeword = cw;

    //then invert bit indicated by syndrome
    correct_codeword[syndrome] = ~correct_codeword[syndrome];
    endfunction //correct_codeword


    function [K-1:0] extract_q(input [n:0] cw);
    integer bit_idx, cw_idx;
    begin
    //This function extracts the information bits vector from the codeword
    //information bits are stored in non-power-of-2 locations

    bit_idx=0; //information bit vector index
    for (cw_idx=1; cw_idx<=n; cw_idx++) //codeword index
        if (2**$clog2(cw_idx) != cw_idx)
        extract_q[bit_idx++] = cw[cw_idx];
    end
    endfunction //extract_q


    function is_power_of_2(input int arg);
    is_power_of_2 = (arg & (arg-1)) == 0;
    endfunction


    function information_error(input [m:1] syndrome);
    begin
    //This function checks if an error was detected/corrected in the information bits
    information_error = |syndrome & !is_power_of_2(syndrome);
    end
    endfunction //information_error
    
    //---------------------------------------------------------------------------------------------------------------------
    // Type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic           rst_ni;     //asynchronous reset
    input   logic           clk_i;      //clock input
    input   logic           clkena_i;   //clock enable input

    //data ports
    input   logic [n  :0]   d_i;        //encoded code word input
    output  logic [K-1:0]   q_o;        //information bit vector output
    output  logic [m  :0]   syndrome_o; //syndrome vector output

    //flags
    output  logic           sb_err_o;   //single bit error detected
    output  logic           db_err_o;   //double bit error detected
    output  logic           sb_fix_o;   //repaired error in the information bits
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic           parity;      //full codeword parity check
    logic           parity_reg;
    logic   [m  :1] syndrome;    //bit error indication/location
    logic   [m  :1] syndrome_reg;
    logic   [n  :0] cw_fixed;    //corrected code word
    
    logic   [n  :0] d;
    logic   [n  :0] d_reg;
    logic   [K-1:0] q;
    logic           sb_err;
    logic           db_err;
    logic           sb_fix;
    
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

    //Step 1: Locate Parity bit
    assign d = P0_LSB ? d_i : {d_i[n-1:0],d_i[n]};

    //Step 2: Calculate code word parity
    assign parity = ^d;

    //Step 3: Calculate syndrome
    assign syndrome = calculate_syndrome(d);

    //Step 4: Generate intermediate registers (if any)
    generate
    if (LATENCY > 1)
    begin : gen_regi
        always @(posedge clk_i or negedge rst_ni)
            if (!rst_ni)
            begin
                d_reg        <= {n+1{1'b0}};
                parity_reg   <= 1'b0;
                syndrome_reg <= {m{1'b0}};
            end
            else if (clkena_i)
            begin
                d_reg        <= d;
                parity_reg   <= parity;
                syndrome_reg <= syndrome;
            end
    end
    else
    begin : gen_noregi
        assign d_reg        = d;
        assign parity_reg   = parity;
        assign syndrome_reg = syndrome;
    end
    endgenerate

    //Step 5: Correct erroneous bit (if any)
    assign cw_fixed = correct_codeword(d_reg, syndrome_reg); 

    //Step 6: Extract information bits vector
    assign q = extract_q(cw_fixed);

    //Step 7: Generate status flags
    assign sb_err =  parity_reg & |syndrome_reg;
    assign db_err = ~parity_reg & |syndrome_reg;
    assign sb_fix =  parity_reg & |information_error(syndrome_reg);

    //Step 8: Generate output registers (if required)
    generate
    if (LATENCY > 0) //
    begin : gen_rego //Generate output registers
        always @(posedge clk_i or negedge rst_ni)
            if (!rst_ni)
            begin
                q_o        <= {K{1'b0}};
                syndrome_o <= {m+1{1'b0}};
                sb_err_o   <= 1'b0;
                db_err_o   <= 1'b0;
                sb_fix_o   <= 1'b0;
            end
            else if (clkena_i)
            begin
                q_o        <= q;
                syndrome_o <= P0_LSB ? {syndrome_reg, parity_reg} : {parity_reg, syndrome_reg};
                sb_err_o   <= sb_err;
                db_err_o   <= db_err;
                sb_fix_o   <= sb_fix;
            end
    end
    else
    begin : gen_norego //No output registers
        always_comb
        begin
            q_o        = q;
            syndrome_o = P0_LSB ? {syndrome_reg, parity_reg} : {parity_reg, syndrome_reg};
            sb_err_o   = sb_err;
            db_err_o   = db_err;
            sb_fix_o   = sb_fix;
        end
    end
    endgenerate

endmodule