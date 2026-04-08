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
// PRODUCT      :   Chechker for ECC Testbench
// FILE         :   ecc_tb_checker.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   Hamming Encoder / Decoder Testbench - Checker
//                  Receive codeword and corrupt either 1 or 2 random bits
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

module ecc_tb_checker (
    clk_i,

    nflips_i,
    flip1_i,
    flip2_i,

    enc_d_i,
    valid_i,
    dec_q_i,

    sb_err_i,
    db_err_i,
    sb_fix_i
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter K         = 8;
    parameter P0_LSB    = 0;
    parameter LATENCY   = 0;

    //---------------------------------------------------------------------------------------------------------------------
    // Tasks
    //---------------------------------------------------------------------------------------------------------------------
    int good, bad, ugly;
    int internal_idx;

    task reset_counters();
        good = 0;
        bad  = 0;
    endtask
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam  int m = calculate_m(K);
    localparam  int n = m + K;
    localparam  INT   = 32;  

    localparam P0_LOCATION = (P0_LSB == 0) ? n : 0;

    //---------------------------------------------------------------------------------------------------------------------
    // Functions
    //---------------------------------------------------------------------------------------------------------------------
    function integer calculate_m;
        input integer k;

        integer m;
        begin
            m=1;
            while (2**m < m+k+1) m++;

            calculate_m = m;
        end
    endfunction //calculate_m


    function is_power_of_2(input int n);
        is_power_of_2 = (n & (n-1)) == 0;
    endfunction

    function int internal_index(input int original, input int n);
        if (original == n)
            internal_index = 0;
        else
            internal_index = original + 1;
    endfunction
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic           clk_i;

    input   logic [INT-1:0] nflips_i;
    input   logic [INT-1:0] flip1_i;
    input   logic [INT-1:0] flip2_i;

    input   logic [K-1:0]   enc_d_i;
    input   logic           valid_i;
    input   logic [K-1:0]   dec_q_i;

    input   logic           sb_err_i;
    input   logic           db_err_i;
    input   logic           sb_fix_i;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // internal signals
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    initial
    begin
        good = 0; //correct
        bad  = 0; //test errors
        ugly = 0; //total errors
    end

    always @(posedge clk_i) begin
        if (!valid_i) begin
            //Check if enc_d == dec_q
            if (enc_d_i !== dec_q_i && !db_err_i)
            begin
                bad++; ugly++;
                $display ("Data mismatch, expected %0h, received %0h", enc_d_i, dec_q_i);
            end
            else good++;

            //Check flags
            case (nflips_i)
                0: begin
                    //no flags should be asserted
                    if (sb_err_i)
                    begin
                        $display ("sb_err asserted: WRONG");
                        bad++; ugly++;
                    end
                    else good++;

                    if (db_err_i)
                    begin
                        $display ("db_err asserted: WRONG");
                        bad++; ugly++;
                    end
                    else good++;

                    if (sb_fix_i)
                    begin
                        $display ("sb_fix asserted: WRONG");
                        bad++; ugly++;
                    end
                    else good++;
                end

                1: begin
                    internal_idx = internal_index(flip1_i, n);
                    // sb_err should be asserted except for p0 (internal index 0)
                    if (internal_idx == 0) begin
                        if (sb_err_i) begin
                            $display("sb_err asserted on p0 bit flip (original bit %0d): WRONG", flip1_i);
                            bad++; ugly++;
                        end else good++;
                    end else if (!sb_err_i) begin
                        $display("sb_err not asserted: WRONG (original bit %0d, internal index %0d)", flip1_i, internal_idx);
                        bad++; ugly++;
                    end else good++;
                
                    // db_err should never be asserted for single-bit errors
                    if (db_err_i) begin
                        $display("db_err asserted for single-bit error (original bit %0d): WRONG", flip1_i);
                        bad++; ugly++;
                    end else good++;
                
                    // sb_fix: asserted only if internal index is a data bit (non-zero and not power of two)
                    if (internal_idx != 0 && !is_power_of_2(internal_idx)) begin
                        if (!sb_fix_i) begin
                            $display("sb_fix not asserted on data bit (original bit %0d, internal index %0d): WRONG", flip1_i, internal_idx);
                            bad++; ugly++;
                        end else good++;
                    end else begin
                        if (sb_fix_i) begin
                            $display("sb_fix asserted on non-data bit (original bit %0d, internal index %0d): WRONG", flip1_i, internal_idx);
                            bad++; ugly++;
                        end else good++;
                    end
                end

                2: begin
                        //db_err should be asserted
                        if (!db_err_i)
                        begin
                            $display ("db_err not asserted: WRONG (flipped bits%0d and %0d", flip1_i, flip2_i);
                            bad++; ugly++;
                        end
                        else good++;
                
                        //sb_err should not be asserted
                if (sb_err_i)
                        begin
                            $display ("sb_err asserted AND db_err not asserted: WRONG (flipped bits%0d and %0d)", flip1_i, flip2_i);
                            bad++; ugly++;
                        end
                        else good++;

                        //sb_fix should not be asserted
                        if (sb_fix_i)
                        begin
                            $display ("sb_fix asserted AND db_err not asserted: WRONG(flipped bits%0d and %0d)", flip1_i, flip2_i);
                            bad++; ugly++;
                        end
                        else good++;
                end
            endcase
        end
    end

endmodule

