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
// PROJECT      : Avionics Box
// PRODUCT      : sync_double_buffer_32bit
// FILE         : sync_double_buffer_32bit.sv
// AUTHOR       : Sachith Rathnayake
// DESCRIPTION  : Asynchronous double buffer (ping‑pong) with full handshake, write stall,
//                CDC synchronisers, buffer initialisation, watchdog timers and system reset.
//                Suitable for RT4G150 (Microchip radiation‑hardened FPGA) and DO‑254 compliance.
//
// FEATURES:
//   - Separate write clock (wr_clk) and read clock (rd_clk) – fully asynchronous.
//   - Two dual‑port buffers (buffer0, buffer1) with independent write/read ports.
//   - Writer always writes to the background buffer; reader always reads from the foreground.
//   - Handshake (buffer swap) uses toggle‑based request/acknowledge with two‑flop synchronisers.
//   - Writer stalls all writes during the handshake – no corruption of the foreground buffer.
//   - After reset, both buffers are initialised to zero (deterministic startup).
//   - Watchdog timers in each clock domain detect handshake deadlock (> 2*BUFFER_DEPTH+100 cycles).
//   - Any watchdog timeout forces a full system reset (both domains, buffers cleared, pointers reset).
//   - Error outputs (wr_timeout_error, rd_timeout_error) are active‑high and latched until reset.
//   - Busy outputs (wr_busy, rd_busy) indicate that a handshake or initialisation is in progress.
//
// PROTOCOL REQUIREMENTS (mandatory for correct operation):
//   1. Writer (sensor):
//        - Assert wr_en_i and wr_addr_i for each 32‑bit word to be written.
//        - After writing the last word of a complete frame, assert wr_done_i for exactly one wr_clk cycle.
//        - Do not assert wr_en_i or wr_done_i while wr_busy = 1 (handshake or initialisation active).
//   2. Reader (OBC):
//        - Read operation:Do not assert rd_en_i while rd_busy = 1.
//          Assert rd_en_i for one rd_clk cycle while presenting rd_addr_i.
//          The read data appears on rd_data_o on the next rd_clk cycle (pipelined read).
//        - **CRITICAL:** To allow buffer swapping, the reader must deassert rd_en_i for at least one
//          rd_clk cycle after completing a read burst. The handshake (swap) is performed only when
//          rd_en_i = 0. If rd_en_i is held high continuously, the swap will never occur and the
//          writer may overwrite the background buffer without swapping, causing data loss.
//   3. Watchdog timeout recovery:
//        - If the handshake deadlocks (e.g., due to metastability or SEU), the watchdog times out
//          and asserts wr_timeout_error / rd_timeout_error. The system reset is automatically
//          triggered, clearing both buffers and resetting all pointers. The current frame is lost,
//          but the system recovers deterministically. The external controller must monitor these
//          error flags and take appropriate action (e.g., retransmit the lost frame).
//
// RESET BEHAVIOUR:
//   - Asynchronous assertion of wr_rst_n or rd_rst_n (or a watchdog timeout) forces sys_rst_n low.
//   - sys_rst_n is stretched for 16 wr_clk cycles and then de‑asserted synchronously to both clocks.
//   - After reset, the writer automatically initialises both buffers to zero (WR_INIT_BUFS state).
//   - wr_busy and rd_busy are high during initialisation and handshake.
//
// INTERFACE SIGNALS (see port comments for details):
//   wr_*  – write clock domain (sensor / ADC manager)
//   rd_*  – read clock domain (OBC SPI)
//
// LIMITATIONS:
//   - The memory arrays (buffer0, buffer1) do not have ECC protection. For space applications that
//     require single‑event upset (SEU) tolerance, implement external ECC or periodic scrubbing.
//   - The design assumes that BUFFER_DEPTH is a power of two or any value; the initialisation uses
//     BUFFER_DEPTH-1 comparison, so any depth is supported.
//
// SYNTHESIS & TIMING CONSTRAINTS (to be added externally):
//   - Declare wr_clk and rd_clk as asynchronous clock groups in the SDC file:
//        set_clock_groups -asynchronous -group {wr_clk} -group {rd_clk}
//        set_false_path -from [get_clocks wr_clk] -to [get_clocks rd_clk]
//        set_false_path -from [get_clocks rd_clk] -to [get_clocks wr_clk]
//
// REVISIONS:
//  Date         Developer             Description
//  -----------  --------------------  ------------------------------------------------
//  3-Apr-2026   Sachith Rathnayake    Initial design – asynchronous double buffer
//  4-Apr-2026   Sachith Rathnayake    Added watchdog timers, full system reset, DO‑254 comments
// ************************************************************************************************************************

`timescale 1ns/1ps

module sync_double_buffer_32bit (
    // writer interface
    wr_rst_n,
    wr_clk,
    wr_en_i,
    wr_data_i,
    wr_addr_i,
    wr_done_i,
    wr_busy,
    wr_timeout_error,

    // reader interface
    rd_rst_n,
    rd_clk,
    rd_en_i,
    rd_data_o,
    rd_addr_i,
    rd_busy,
    rd_timeout_error
);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    //NA
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    parameter DATA_WIDTH    = 32;
    parameter BUFFER_DEPTH  = 16;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    localparam ADDR_WIDTH = $clog2(BUFFER_DEPTH);

    localparam WRITE_WATCHDOG_CYCLE_COUNT   = 2*BUFFER_DEPTH + 100;
    localparam READ_WATCHDOG_CYCLE_COUNT    = 2*BUFFER_DEPTH + 100;

    localparam RESET_SCRETCH_WIDTH          = 4;
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // wr toggle handshake sequence states
    typedef enum logic {RD_TG_IDLE, RD_TG_ACK} rd_toggle_st;

    // rd toggle handshake sequence states
    typedef enum logic [1:0] {WR_TG_IDLE, WR_INIT_BUFS, WR_TG_REQ} wr_toggle_st;
    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    
    // writer interface
    input   logic                   wr_rst_n;
    input   logic                   wr_clk;
    input   logic                   wr_en_i;
    input   logic [DATA_WIDTH-1:0]  wr_data_i;
    input   logic [ADDR_WIDTH-1:0]  wr_addr_i;
    input   logic                   wr_done_i;
    output  logic                   wr_busy;
    output  logic                   wr_timeout_error;

    // reader interface
    input   logic                   rd_rst_n;
    input   logic                   rd_clk;
    input   logic                   rd_en_i;
    output  logic [DATA_WIDTH-1:0]  rd_data_o;
    input   logic [ADDR_WIDTH-1:0]  rd_addr_i;
    output  logic                   rd_busy;
    output  logic                   rd_timeout_error;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // Two buffers
    logic [DATA_WIDTH-1:0]          buffer0 [0:BUFFER_DEPTH-1];
    logic [DATA_WIDTH-1:0]          buffer1 [0:BUFFER_DEPTH-1];

    // toggle to track which buffer is active for reading
    logic                           wr_active_buf; // if 0 => read from buffer0, or if 1 => read from buffer1
    logic                           rd_active_buf; // if 0 => read from buffer0, or if 1 => read from buffer1

    rd_toggle_st                    rd_tg_state;
    wr_toggle_st                    wr_tg_state;

    logic                           toggle_req;
    logic                           toggle_req_meta;
    logic                           toggle_req_sync;

    logic                           toggle_ack;
    logic                           toggle_ack_meta;
    logic                           toggle_ack_sync;

    logic [ADDR_WIDTH-1:0]          buf_addr_counter;

    // write watchdog timer signals
    logic [$clog2(WRITE_WATCHDOG_CYCLE_COUNT)-1:0]  wr_wtchdg_tmr_count;
    logic                                           wr_timeout_error_int;

    logic                                           wr_rst_meta_n;
    logic                                           wr_rst_sync_n;

    // read watchdog timer signals
    logic [$clog2(READ_WATCHDOG_CYCLE_COUNT)-1:0]   rd_wtchdg_tmr_count;
    logic                                           rd_timeout_error_int;

    logic                                           rd_rst_meta_n;
    logic                                           rd_rst_sync_n;


    logic                                           sys_rst_n;

    logic                                           reset_req_n;
    logic                                           reset_req_n_reg;
    logic                                           sys_rst_n_reg;
    logic [RESET_SCRETCH_WIDTH-1:0]                 reset_stretch_cnt;
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------

    assign rd_busy  = (rd_tg_state != RD_TG_IDLE);
    assign wr_busy  = (wr_tg_state != WR_TG_IDLE);
    assign wr_timeout_error = !wr_timeout_error_int;
    assign rd_timeout_error = !rd_timeout_error_int;

    //---------------------------------------------------------------------------------------------------------------------
    // System reset generation (glitch‑free, synchronous stretcher)
    //   - reset_req_n combines external resets and watchdog timeouts (active low)
    //   - reset_req_n_reg samples the request on wr_clk to avoid metastability
    //   - The reset stretcher holds sys_rst_n low for 16 wr_clk cycles after the request is removed,
    //     ensuring a clean, deterministic reset pulse for both clock domains.
    //---------------------------------------------------------------------------------------------------------------------
    assign reset_req_n    = wr_rst_n && rd_rst_n && wr_timeout_error_int && rd_timeout_error_int;

    always @(posedge wr_clk) reset_req_n_reg <= reset_req_n;

    always_ff @(posedge wr_clk) begin
        if (!reset_req_n_reg) begin
            sys_rst_n_reg       <= 1'b0;
            reset_stretch_cnt   <= {RESET_SCRETCH_WIDTH{1'b0}};
        end 
        else if (reset_stretch_cnt < {RESET_SCRETCH_WIDTH{1'b1}}) begin
            reset_stretch_cnt   <= reset_stretch_cnt + 1'b1;
            sys_rst_n_reg       <= 1'b0;
        end
        else begin
            sys_rst_n_reg       <= 1'b1;
        end
    end

    assign sys_rst_n = sys_rst_n_reg;

    //---------------------------------------------------------------------------------------------------------------------
    // Reset synchronizers (asynchronous assertion, synchronous deassertion)
    //   - Converts the glitch‑free sys_rst_n into domain‑specific resets wr_rst_sync_n and rd_rst_sync_n.
    //   - Asynchronous assertion ensures immediate reset; synchronous de‑assertion prevents metastability.
    //---------------------------------------------------------------------------------------------------------------------
    always_ff @(posedge wr_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            wr_rst_meta_n   <= 1'b0;
            wr_rst_sync_n   <= 1'b0;
        end else begin
            wr_rst_meta_n   <= 1'b1;
            wr_rst_sync_n   <= wr_rst_meta_n;
        end
    end

    always_ff @(posedge rd_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            rd_rst_meta_n   <= 1'b0;
            rd_rst_sync_n   <= 1'b0;
        end else begin
            rd_rst_meta_n   <= 1'b1;
            rd_rst_sync_n   <= rd_rst_meta_n;
        end
    end

    //---------------------------------------------------------------------------------------------------------------------
    // Writer state machine (wr_clk domain)
    //   States: WR_INIT_BUFS – initialises both buffers to zero after reset
    //           WR_TG_IDLE   – idle, accepts writes and waits for wr_done_i
    //           WR_TG_REQ    – handshake active: toggle_req sent to reader, writes stalled
    //   Transitions:
    //     - WR_INIT_BUFS → WR_TG_IDLE after BUFFER_DEPTH cycles (all addresses zeroed)
    //     - WR_TG_IDLE → WR_TG_REQ when wr_done_i is asserted
    //     - WR_TG_REQ → WR_TG_IDLE when toggle_ack_sync is received (handshake complete)
    //   On timeout (wr_timeout_error_int = 0), the system reset forces the state machine back to WR_INIT_BUFS.
    //---------------------------------------------------------------------------------------------------------------------
    always_ff @(posedge wr_clk) begin
        if (!wr_rst_sync_n) begin
            wr_tg_state         <= WR_INIT_BUFS;
            toggle_req          <= 1'b0;
            wr_active_buf       <= 1'b0;
            buf_addr_counter    <= {ADDR_WIDTH{1'b0}};
        end
        else begin
            unique case (wr_tg_state)
                WR_INIT_BUFS: begin
                    if (buf_addr_counter == BUFFER_DEPTH-1) begin
                        wr_tg_state         <= WR_TG_IDLE;
                        buf_addr_counter    <= {ADDR_WIDTH{1'b0}};
                        buffer0[BUFFER_DEPTH-1]   <= {DATA_WIDTH{1'b0}};
                        buffer1[BUFFER_DEPTH-1]   <= {DATA_WIDTH{1'b0}};
                    end
                    else begin
                        buffer0[buf_addr_counter]   <= {DATA_WIDTH{1'b0}};
                        buffer1[buf_addr_counter]   <= {DATA_WIDTH{1'b0}};
                        buf_addr_counter            <= buf_addr_counter + 1'b1;
                    end
                end
                WR_TG_IDLE: begin
                    if (wr_done_i) begin
                        toggle_req  <= 1'b1;
                        wr_tg_state <= WR_TG_REQ;
                    end
                    if (wr_en_i) begin
                        if (wr_active_buf) begin
                            buffer0[wr_addr_i] <= wr_data_i;
                        end
                        else begin
                            buffer1[wr_addr_i] <= wr_data_i;
                        end
                    end 
                end
                WR_TG_REQ: begin
                    if (toggle_ack_sync) begin
                        wr_tg_state     <= WR_TG_IDLE;
                        toggle_req      <= 1'b0;
                        wr_active_buf   <= ~wr_active_buf;
                    end
                end
                default: begin
                    wr_tg_state     <= WR_TG_IDLE;
                    toggle_req      <= 1'b0;
                    wr_active_buf   <= 1'b0;
                end
            endcase
        end
    end

    //---------------------------------------------------------------------------------------------------------------------
    // CDC synchroniser: writer toggle_req → reader clock domain (rd_clk)
    //   Two‑flop synchroniser with reset (rd_rst_sync_n). The reset clears the request
    //   during system reset or after a watchdog timeout.
    //---------------------------------------------------------------------------------------------------------------------
    always_ff @(posedge rd_clk or negedge rd_rst_sync_n) begin : WR_2_RD_SYNC
        if (!rd_rst_sync_n) begin
            toggle_req_meta <= 1'b0;
            toggle_req_sync <= 1'b0;
        end
        else begin
            toggle_req_meta <= toggle_req;
            toggle_req_sync <= toggle_req_meta;
        end
    end
    
    //---------------------------------------------------------------------------------------------------------------------
    // CDC synchroniser: reader toggle_ack → writer clock domain (wr_clk)
    //   Two‑flop synchroniser with reset (wr_rst_sync_n).
    //---------------------------------------------------------------------------------------------------------------------
    always_ff @(posedge wr_clk or negedge wr_rst_sync_n) begin : RD_2_WR_SYNC
        if (!wr_rst_sync_n) begin
            toggle_ack_meta <= 1'b0;
            toggle_ack_sync <= 1'b0;
        end
        else begin
            toggle_ack_meta <= toggle_ack;
            toggle_ack_sync <= toggle_ack_meta;
        end
    end

    //---------------------------------------------------------------------------------------------------------------------
    // Reader state machine (rd_clk domain)
    //   States: RD_TG_IDLE – idle, monitors toggle_req_sync
    //           RD_TG_ACK  – handshake active: toggles rd_active_buf and sends toggle_ack
    //   Transitions:
    //     - RD_TG_IDLE → RD_TG_ACK when toggle_req_sync is high and rd_en_i = 0
    //     - RD_TG_ACK → RD_TG_IDLE when toggle_req_sync goes low (handshake complete)
    //   IMPORTANT: The handshake is only allowed when rd_en_i = 0 to avoid tearing reads.
    //---------------------------------------------------------------------------------------------------------------------
    always_ff @(posedge rd_clk) begin
        if (!rd_rst_sync_n) begin
            rd_tg_state     <= RD_TG_IDLE;
            toggle_ack      <= 1'b0;
            rd_active_buf   <= 1'b0;
        end
        else begin
            unique case (rd_tg_state)
                RD_TG_IDLE: begin
                    if (toggle_req_sync) begin
                        if (!rd_en_i) begin
                            rd_tg_state     <= RD_TG_ACK;
                            toggle_ack      <= 1'b1;
                            rd_active_buf   <= ~rd_active_buf;
                        end
                    end
                    if (rd_en_i) begin 
                        if (rd_active_buf) begin
                            rd_data_o <= buffer1[rd_addr_i];
                        end
                        else begin
                            rd_data_o <= buffer0[rd_addr_i];
                        end
                    end
                end
                RD_TG_ACK: begin
                    if (!toggle_req_sync) begin
                        rd_tg_state     <= RD_TG_IDLE;
                        toggle_ack      <= 1'b0;
                    end
                end
                default: begin
                    rd_tg_state     <= RD_TG_IDLE;
                    toggle_ack      <= 1'b0;
                    rd_active_buf   <= 1'b0;
                end
            endcase
        end
    end

    //---------------------------------------------------------------------------------------------------------------------
    // Writer watchdog timer (wr_clk domain)
    //   Counts cycles when wr_tg_state is not WR_TG_IDLE. If the counter reaches
    //   WRITE_WATCHDOG_CYCLE_COUNT-1, a timeout is signalled (wr_timeout_error_int = 0),
    //   which triggers a full system reset. The counter is reset when the state becomes idle.
    //---------------------------------------------------------------------------------------------------------------------
    always @(posedge wr_clk) begin
        if (!wr_rst_sync_n) begin
            wr_timeout_error_int <= 1'b1;
            wr_wtchdg_tmr_count <= {$clog2(WRITE_WATCHDOG_CYCLE_COUNT){1'b0}};
        end
        else begin
            if (wr_wtchdg_tmr_count == WRITE_WATCHDOG_CYCLE_COUNT-1) begin
                wr_timeout_error_int <= 1'b0;
                wr_wtchdg_tmr_count <= {$clog2(WRITE_WATCHDOG_CYCLE_COUNT){1'b0}};
            end
            else if ((wr_tg_state != WR_TG_IDLE)) begin
                wr_wtchdg_tmr_count <= wr_wtchdg_tmr_count + 1'b1;
            end
            else begin
                wr_wtchdg_tmr_count <= {$clog2(WRITE_WATCHDOG_CYCLE_COUNT){1'b0}};
            end
        end
    end

    //---------------------------------------------------------------------------------------------------------------------
    // Reader watchdog timer (rd_clk domain)
    //   Counts cycles when rd_tg_state is not RD_TG_IDLE. On timeout, rd_timeout_error_int = 0,
    //   causing a system reset. The counter is cleared when the reader returns to idle.
    //---------------------------------------------------------------------------------------------------------------------
    always @(posedge rd_clk) begin
        if (!rd_rst_sync_n) begin
            rd_timeout_error_int <= 1'b1;
            rd_wtchdg_tmr_count <= {$clog2(READ_WATCHDOG_CYCLE_COUNT){1'b0}};
        end
        else begin
            if (rd_wtchdg_tmr_count == READ_WATCHDOG_CYCLE_COUNT-1) begin
                rd_timeout_error_int <= 1'b0;
                rd_wtchdg_tmr_count <= {$clog2(READ_WATCHDOG_CYCLE_COUNT){1'b0}};
            end
            else if ((rd_tg_state != RD_TG_IDLE)) begin
                rd_wtchdg_tmr_count <= rd_wtchdg_tmr_count + 1'b1;
            end
            else begin
                rd_wtchdg_tmr_count <= {$clog2(READ_WATCHDOG_CYCLE_COUNT){1'b0}};
            end
        end
    end

endmodule