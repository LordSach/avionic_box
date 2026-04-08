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
// PRODUCT      :   PWM Generator
// FILE         :   pwm_generator.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   This is the PWM Generator module for PWM Driver Engine
//
// ************************************************************************************************************************
//
// REVISIONS:
//
//  Date           Developer               Description
//  -----------    --------------------    -----------
//  9-APR-2026    Sachith Rathnayake      Creation
//
// ************************************************************************************************************************

`timescale 1ns/1ps

module pwm_generator (
    clk,
    rst_n,          // active-low synchronous reset

    // Runtime configuration (written by AXI-Lite wrapper)
    enable,
    cycles_per_period,  // total frame length in clocks
    duty_cycles,    // HIGH time in clocks

    // Outputs
    pwm_out,        // servo signal pin
    irq_period      // 1-cycle pulse at frame start
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter CPP_WIDTH         = 32;
    parameter DC_WIDTH          = 32;
    
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam COUNTER_WIDTH    = CPP_WIDTH;
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // typedefs
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input  wire                 clk;
    input  wire                 rst_n;               // active-low synchronous reset

    // Runtime configuration
    input  wire                 enable;              // clock enable for the system
    input  wire [CPP_WIDTH  :0] cycles_per_period;   // total cycles (clk) for the period of PWM signal (resolution)
    input  wire [DC_WIDTH   :0] duty_cycles;         // number of cycles (clk) the PWM signal stays HIGH

    // Outputs
    output reg                  pwm_out;             // servo signal pin
    output reg                  irq_period;          // 1-cycle pulse at frame start
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    logic [COUNTER_WIDTH:0]     counter;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    always @(posedge clk) begin
        if (!rst_n) begin
            counter         <= {COUNTER_WIDTH{1'b0}};
            pwm_out         <= 1'b0;
            irq_period      <= 1'b0;
        end else if (!enable) begin
            counter         <= {COUNTER_WIDTH{1'b0}};
            pwm_out         <= 1'b0;
            irq_period      <= 1'b0;
        end else begin
            irq_period      <= 1'b0; // default: deasserted

            if (counter >= (cycles_per_period - 1'b1)) begin
                counter     <= {COUNTER_WIDTH{1'b0}};
                irq_period  <= 1'b1; //interrupt
            end else begin
                counter     <= counter + 1'b1;
            end

            pwm_out         <= (counter < duty_cycles) ? 1'b1 : 1'b0;
        end
    end

endmodule