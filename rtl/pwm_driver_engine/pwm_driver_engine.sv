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
// PRODUCT      :   PWM Driver Engine
// FILE         :   pwm_driver_engine.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   PWM Driver Engine design for controlling pwm base actuators 
//
// ************************************************************************************************************************
//
// REVISIONS:
//
//  Date           Developer               Description
//  -----------    --------------------    -----------
//  7-APR-2026    Sachith Rathnayake      Creation
//
// ************************************************************************************************************************

`timescale 1ns/1ps

module pwm_driver_engine (
    rst_n,
    clk,
    // writer interface
    wr_en_o,
    wr_data_o,
    wr_addr_o,
    wr_done_o,
    wr_busy,
    wr_timeout_error,

    // reader interface
    rd_en_o,
    rd_data_i,
    rd_addr_o,
    rd_busy,
    rd_timeout_error,

    // PWM output interface
    pwm_out,
    irq_period
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter DATA_WIDTH    = 32;
    parameter BUFFER_DEPTH  = 16;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam ADDR_WIDTH       = $clog2(BUFFER_DEPTH);
    localparam CPP_WIDTH        = DATA_WIDTH;
    localparam DC_WIDTH         = DATA_WIDTH;
    localparam CR_WIDTH         = DATA_WIDTH;
    localparam NUM_OF_STATES    = 4;
    localparam AMF_STATE_WIDTH  = $clog2(NUM_OF_STATES);
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    typedef enum logic [AMF_STATE_WIDTH-1:0] { 
        AMF_INITIATE,
        AMF_IDLE,
        AMF_READ_FROM_DB,
        AMF_WRITE_TO_DB
     } actuator_manager_fsm_state;
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input   logic                   rst_n;
    input   logic                   clk;

    // writer interface
    output  logic                   wr_en_o;
    output  logic [DATA_WIDTH-1:0]  wr_data_o;
    output  logic [ADDR_WIDTH-1:0]  wr_addr_o;
    output  logic                   wr_done_o;
    input   logic                   wr_busy;
    input   logic                   wr_timeout_error;

    // reader interface
    output  logic                   rd_en_o;
    input   logic [DATA_WIDTH-1:0]  rd_data_i;
    output  logic [ADDR_WIDTH-1:0]  rd_addr_o;
    input   logic                   rd_busy;
    input   logic                   rd_timeout_error;

    output  logic                   pwm_out; // servo signal pin
    output  logic                   irq_period;// 1-cycle pulse at frame start
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // pwm control register map (These registers are to be updated by the acuator manager FSM continuously from the Double Buffer)
    logic [CR_WIDTH-1   :0]         config_req_reg;

    logic                           enable;              // clock enable for the system
    logic [CPP_WIDTH-1  :0]         cycles_per_period;   // total cycles (clk) for the period of PWM signal (resolution)
    logic [DC_WIDTH-1   :0]         duty_cycles;         // number of cycles (clk) the PWM signal stays HIGH

    // Actuator Manager FSM register
    actuator_manager_fsm_state      afm_state;

    //---------------------------------------------------------------------------------------------------------------------
    // Module Instantiations
    //---------------------------------------------------------------------------------------------------------------------

    pwm_generator #(
        .CPP_WIDTH(CPP_WIDTH),
        .DC_WIDTH(DC_WIDTH)
    ) pwm_gen_int (
        .clk(clk),
        .rst_n(rst_n),          // active-low synchronous reset

        // Runtime configuration (written by AXI-Lite wrapper)
        .enable(enable),
        .period_cycles(cycles_per_period),  // total frame length in clocks
        .duty_cycles(duty_cycles),    // HIGH time in clocks

        // Outputs
        .pwm_out(pwm_out),        // servo signal pin
        .irq_period(irq_period)      // 1-cycle pulse at frame start
    );
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    // Acuator Manager FSM
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            afm_state           <= AMF_INITIATE;

            enable              <= 1'b0;
            cycles_per_period   <= {CPP_WIDTH{1'b0}};
            duty_cycles         <= {DC_WIDTH{1'b0}};
        end
        else begin
            unique case (afm_state)
                AMF_INITIATE: begin
                    // define initiate sequence (BIT boot setup)

                    afm_state   <= AMF_IDLE;
                end
                AMF_IDLE: begin
                    // wait for the rd_busy & rd_timeout_error to be low to jump to AMF_READ_FROM_DB
                end
                AMF_READ_FROM_DB: begin
                    // read from Double buffer dedicated to this PWM Driver Engine for con figuration updates
                end
                AMF_WRITE_TO_DB: begin
                    // write the status of the PWM Driver Engine to DB (for BIT). 
                end 
                default: begin
                    afm_state           <= AMF_INITIATE;

                    enable              <= 1'b0;
                    cycles_per_period   <= {CPP_WIDTH{1'b0}};
                    duty_cycles         <= {DC_WIDTH{1'b0}};
                end
            endcase
        end
    end

endmodule