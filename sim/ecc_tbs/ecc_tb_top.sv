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
// PRODUCT      :   ECC Testbech Top
// FILE         :   ecc_tb_top.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   Testbench Top module of the ECC | Hamming Encoder / Decoder Testbench
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

module ecc_tb_top ;

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter K           = 72;  //Limitation $urandom is a 32bit number
    parameter P0_LSB      = 0;
    parameter DEC_LATENCY = 0;
    parameter RUNS        = 100_000;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam m   = calculate_m(K);
    localparam n   = m + K;
    localparam INT = 32;

    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic           clk;
    logic           rst_n;

    logic [K-1:0]   enc_d;
    logic [K-1:0]   ch_enc_d;
    logic [K-1:0]   dec_q;
    logic [n  :0]   enc_q;
    logic [n  :0]   ch_q;
    logic           dec_sb_err;
    logic           dec_db_err;
    logic           dec_sb_fix;

    logic [INT-1:0] nflips;
    logic [INT-1:0] ch_nflips;
    logic [INT-1:0] flip1;
    logic [INT-1:0] ch_flip1;
    logic [INT-1:0] flip2;
    logic [INT-1:0] ch_flip2;

    //---------------------------------------------------------------------------------------------------------------------
    // Functions
    //--------------------------------------------------------------------------------------------------------------------

    function integer calculate_m;
        input integer k;

        integer m;
    begin
        m=1;
        while (2**m < m+k+1) m++;

        calculate_m = m;
    end
    endfunction //calculate_m

    //---------------------------------------------------------------------------------------------------------------------
    // Tasks
    //--------------------------------------------------------------------------------------------------------------------

    task welcome_msg();
        $display("\n\n");
        $display ("------------------------------------------------------------");
        $display ("- Hamming ECC Encoder/Decoder Testbench---------------------");
        $display ("------------------------------------------------------------");
        $display ("  K       = %0d", K);
        $display ("  m       = %0d", m);
        $display ("  n       = %0d", n);
        $display ("  cw_bits = %0d", n+1);
        $display ("  P0      = %0d", P0_LSB ? 0 : n);
        $display ("------------------------------------------------------------");
        $display ("\n");
    endtask

    task goodbye_msg();
        $display("\n\n");
        $display ("------------------------------------------------------------");
        $display ("- Hamming ECC Encoder/Decoder Testbench---------------------");
        $display ("------------------------------------------------------------");
        $display ("  Regression test complete");
        $display ("  Status = %s", checker_inst.ugly ? "FAILED" : "PASSED");
        $display ("------------------------------------------------------------");
    endtask


    task tst_done();
        //wait for data to propagate pipeline
        nflips = 0;
        repeat (5) @(posedge clk);

        //Display test results
        $display ("Test done. Checks good=%0d. Checks bad=%0d. Checks ugly=%0d", checker_inst.good, checker_inst.bad, checker_inst.ugly);
    endtask

    task rst_errors();
        checker_inst.reset_counters();
    endtask

    task tst_clean_seq (input int runs);
        //basic test, no bit flips
        $display("--------------------");
        $display("- Running clean_seq (%0d runs)", runs);
        $display("--------------------");

        rst_errors();
        nflips = 0;

        for (enc_d = 0; enc_d < runs; enc_d++)
        begin
        @(posedge clk);
        end

        tst_done();
    endtask


    task tst_clean_rnd (input int runs);
        //basic test, no bit flips
        $display("--------------------");
        $display("- Running clean_rnd (%0d runs)", runs);
        $display("--------------------");

        rst_errors();
        nflips = 0;

        repeat (runs)
        begin
            enc_d = $random();
        @(posedge clk);
        end

        tst_done();
    endtask


    task tst_1bflip_seq (input int runs);
        //single bit flip
        $display("--------------------");
        $display("- Running 1bflip_seq (%0d runs)", runs);
        $display("--------------------");

        rst_errors();
        nflips = 1;

        for (flip1 = 0; flip1 <= runs; flip1++)
        begin
            enc_d = $random();
        @(posedge clk);
        end

        tst_done();
    endtask


    task tst_1bflip_rnd (input int runs, input int maxrange);
        //single bit flip
        $display("--------------------");
        $display("- Running 1bflip_rnd (%0d runs)", runs);
        $display("--------------------");

        rst_errors();
        nflips = 1;

        repeat (runs)
        begin
            flip1 = $urandom_range(maxrange);
            enc_d = $random();
        @(posedge clk);
        end

        tst_done();
    endtask


    task tst_2bflip_seq (input int runs);
        //single bit flip
        $display("--------------------");
        $display("- Running 2bflip_seq (%0d runs)", runs);
        $display("--------------------");

        rst_errors();
        nflips = 2;

        for (flip1 = 0; flip1 < runs  ; flip1++)
        for (flip2 = 0; flip2 < runs-1; flip2++)
        begin
            if (flip2==flip1) flip2++;
        
            enc_d = $random();
        @(posedge clk);
        end

        tst_done();
    endtask


    task tst_2bflip_rnd (input int runs, input int maxrange);
        //single bit flip
        $display("--------------------");
        $display("- Running 2bflip_rnd (%0d runs)", runs);
        $display("--------------------");

        rst_errors();
        nflips = 2;

        repeat (runs)
        begin
            flip1 = $urandom_range(maxrange);
        flip2 = $urandom_range(maxrange-1);

        if (flip2==flip1) flip2++;

            enc_d = $random();
        @(posedge clk);
        end

        tst_done();
    endtask
    

    task tst_rnd (input int runs, input int maxrange);
        //single bit flip
        $display("--------------------");
        $display("- Running rnd (%0d runs)", runs);
        $display("--------------------");

        rst_errors();
        nflips = 2;

        repeat (runs)
        begin
            nflips = $urandom_range(2);
            flip1  = $urandom_range(maxrange);
        flip2  = $urandom_range(maxrange-1);

        if (flip2==flip1) flip2++;

            enc_d = $random();
        @(posedge clk);
        end

        tst_done();
    endtask
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    //generate clock
    always #10 clk = ~clk;


    //testvector generator


    //instantiate encoder
    ecc_enc #(
        .K      ( K      ),
        .P0_LSB ( P0_LSB ) )
    dut_enc (
        .d_i  ( enc_d ),      //information bit vector input
        .q_o  ( enc_q ),      //encoded data word output

        .p_o  (       ),      //parity vector output
        .p0_o (       ));     //extended parity bit


    //instantiate channel
    ecc_tb_channel #(n)
    channel_inst (
        .clk_i    ( clk       ),

        .nflips_i ( nflips    ),
        .nflips_o ( ch_nflips ),

        .flip1_i  ( flip1     ),
        .flip2_i  ( flip2     ),
        .flip1_o  ( ch_flip1  ),
        .flip2_o  ( ch_flip2  ),

        .d_i      ( enc_q     ),
        .q_o      ( ch_q      ));


    //delay data; same delay as channel
    always @(posedge clk) ch_enc_d <= enc_d;


    //instantiate decoder
    ecc_dec #(
        .K          ( K           ),
        .P0_LSB     ( P0_LSB      ),
        .LATENCY    ( DEC_LATENCY ))
    dut_dec (
        .rst_ni     ( rst_n      ),   //asynchronous reset
        .clk_i      ( clk        ),   //clock input
        .clkena_i   ( 1'b1       ),   //clock enable input

        //data ports
        .d_i        ( ch_q       ),   //encoded code word input
        .q_o        ( dec_q      ),   //information bit vector output
        .syndrome_o (            ),   //syndrome vector output

        //flags
        .sb_err_o   ( dec_sb_err ),   //single bit error detected
        .db_err_o   ( dec_db_err ),   //double bit error detected
        .sb_fix_o   ( dec_sb_fix ));  //repaired error in the information bits


    //instantiate checker
    ecc_tb_checker #(
        .K        ( K      ),
        .P0_LSB   ( P0_LSB ))
    checker_inst (
        .clk_i    ( clk        ),

        .nflips_i ( ch_nflips  ),
        .flip1_i  ( ch_flip1   ),
        .flip2_i  ( ch_flip2   ),

        .enc_d_i  ( ch_enc_d   ),
        .dec_q_i  ( dec_q      ),

        .sb_err_i ( dec_sb_err ),
        .db_err_i ( dec_db_err ),
        .sb_fix_i ( dec_sb_fix ));


    //Tests
    initial
    begin
        clk   = 0;

        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;

        welcome_msg();
        tst_clean_seq(K);
        tst_clean_rnd(RUNS *K);
        tst_1bflip_seq(n);
        tst_1bflip_rnd(RUNS *K, n);
        tst_2bflip_seq(n+1);
        tst_2bflip_rnd(RUNS *K, n);
        tst_rnd(RUNS *K,n);
        goodbye_msg();
        
        $finish();
    end

endmodule
