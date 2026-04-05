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
// PRODUCT      :   Async Reset Synchronizer
// FILE         :   async_reset_synchronizer.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   Synchronizer for asynchronous resets which are crossing one clock domain to the other
//
// ************************************************************************************************************************
//
// REVISIONS:
//
//  Date           Developer               Description
//  -----------    --------------------    -----------
//  5-APR-2026    Sachith Rathnayake      Design
//
// ************************************************************************************************************************

`timescale 1ns/1ps

module async_reset_synchronizer (
    rst_n,
    clk,
    async_rst_n,// source domain active-low reset
    sync_rst_n  // destination domain synchronized active-low reset
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter SYNC_STAGES = 2;  // Typically 2 or 3
    parameter CYCLE_COUNT = 6;  // number of cycles to count for avoiding glitch periods
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // NA
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // NA
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic           rst_n;         // Destination active-low reset 
    input   logic           clk;           // Destination clock domain
    input   logic           async_rst_n;   // Active-low asynchronous reset (from source)
    output  logic           sync_rst_n;    // Synchronized reset output (active-low)
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    (* syn_preserve *)
    logic [SYNC_STAGES-1:0]     sync_chain;
    (* syn_preserve *)
    logic [CYCLE_COUNT-1:0]     reset_duration;
    (* syn_preserve *)
    logic                       glitch_filtered_async_rst_n;
    (* syn_preserve *)
    logic [SYNC_STAGES-1:0]     r_async_rst_n;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------

    // parameter constraint check
    initial begin
        if (SYNC_STAGES < 2) $error("SYNC_STAGES must be >= 2");
        if (CYCLE_COUNT < 2) $error("CYCLE_COUNT must be >= 2");
    end

    // input stabalizer 
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            r_async_rst_n   <= {SYNC_STAGES{1'b0}};
        end
        else begin
            r_async_rst_n   <= {r_async_rst_n[SYNC_STAGES-2:0],async_rst_n};
        end 
    end

    // glitch fliltering logic
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            reset_duration  <= {CYCLE_COUNT{1'b1}};
        end
        else if (!r_async_rst_n[SYNC_STAGES-1]) begin
            reset_duration  <= {reset_duration[CYCLE_COUNT-2:0], 1'b0};
        end
        else begin
            reset_duration  <= {reset_duration[CYCLE_COUNT-2:0], 1'b1};
        end
    end

    always_ff @(posedge clk) glitch_filtered_async_rst_n <= (!rst_n)? 1'b1 : (|reset_duration);
    
    // reset synchronizer logic
    always_ff @(posedge clk or negedge glitch_filtered_async_rst_n or negedge rst_n) begin
        if (!rst_n || !glitch_filtered_async_rst_n) begin
            sync_chain      <= {SYNC_STAGES{1'b0}}; // All stages reset asynchronously
        end
        else begin
            sync_chain      <= {sync_chain[SYNC_STAGES-2:0], 1'b1};
        end
    end

    assign sync_rst_n       = sync_chain[SYNC_STAGES-1];

endmodule