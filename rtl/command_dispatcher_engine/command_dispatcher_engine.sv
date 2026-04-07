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
// PRODUCT      :   Command Dispatcher Engine
// FILE         :   command_dispatcher_engine.sv
// AUTHOR       :   Sachith Rathnayake
// DESCRIPTION  :   The module which reads and write to OBC and dispatches the commands
//                  to other submodules.
//
// ************************************************************************************************************************
//
// REVISIONS:
//
//  Date           Developer               Description
//  -----------    --------------------    -----------
//  6-APR-2026    Sachith Rathnayake      Creation
//
// ************************************************************************************************************************

`timescale 1ns/1ps

module command_dispatcher_engine (
    rst_n,
    clk,

    // OBC SPI Interface 0
    i_spi_0_clk,
    o_spi_0_miso,
    i_spi_0_mosi,
    i_spi_0_cs_n,

    // OBC SPI Interface 1
    i_spi_1_clk,
    o_spi_1_miso,
    i_spi_1_mosi,
    i_spi_1_cs_n

    // Sensor Double Buffer Access Contoller Interface

    // Actuator Double Buffer Access Controller Interface

    // Control paths for each ADC Manager 

    // Control paths for each PWM Driver Engine

    // interface for Built In Test Module

);

    //---------------------------------------------------------------------------------------------------------------------
    // Global constant headers
    //---------------------------------------------------------------------------------------------------------------------
    
    // constants
    
    //---------------------------------------------------------------------------------------------------------------------
    // parameter definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // OBC SPI Interface 0 parameters
    parameter SPI_MODE_0 = 3;

    // OBC SPI Interface 1 parameters
    parameter SPI_MODE_1 = 3;
    
    //---------------------------------------------------------------------------------------------------------------------
    // localparam definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    // comman local parameters
    localparam BYTE_WIDTH                       = 8;
    localparam I_SPI_NUM_OF_FIELDS_OF_FRAME_FSM = 11;
    localparam O_SPI_NUM_OF_FIELDS_OF_FRAME_FSM = 13;
    localparam I_SPI_STATE_WIDTH                = $clog2(I_SPI_NUM_OF_FIELDS_OF_FRAME_FSM);
    localparam O_SPI_STATE_WIDTH                = $clog2(O_SPI_NUM_OF_FIELDS_OF_FRAME_FSM);
    localparam BYTE_CNT_WIDTH_DEC               = $clog2(BYTE_WIDTH*16);
    localparam BYTE_CNT_WIDTH_ENC               = $clog2(BYTE_WIDTH*24);
    
    //---------------------------------------------------------------------------------------------------------------------
    // type definitions
    //---------------------------------------------------------------------------------------------------------------------
    
    typedef enum logic [I_SPI_STATE_WIDTH-1:0] { 
        I_SPI_IDLE,
        I_SPI_START_BYTE_0,
        I_SPI_START_BYTE_1,
        I_SPI_FRAME_ID,
        I_SPI_COMMAND_TYPE_0,
        I_SPI_COMMAND_TYPE_1,
        I_SPI_DEVICE_ID,
        I_SPI_REG_ADDR_0,
        I_SPI_REG_ADDR_1,
        I_SPI_PAYLOAD_BYTES,
        I_SPI_CRC_16
    } spi_input_decoder_state_t;


    typedef enum logic [O_SPI_STATE_WIDTH-1:0] { 
        O_SPI_IDLE,
        O_SPI_START_BYTE_0,
        O_SPI_START_BYTE_1,
        O_SPI_FRAME_ID,
        O_SPI_STATUS_0,
        O_SPI_STATUS_1,
        O_SPI_DEVICE_ID,
        O_SPI_PAYLOAD_BYTES,
        O_SPI_TIMESTAMP_0,
        O_SPI_TIMESTAMP_1,
        O_SPI_TIMESTAMP_2,
        O_SPI_TIMESTAMP_3,
        O_SPI_CRC_16
    } spi_output_encoder_state_t;

    // Define the SPI Command Frame from the OBC
    typedef struct packed {
        logic [BYTE_WIDTH-1     :0]                 frame_id;       // Byte 0: Frame ID
        logic [BYTE_WIDTH*2-1   :0]                 command_type;   // Byte 1-2: Command Type (e.g., READ_ADC)
        logic [BYTE_WIDTH-1     :0]                 device_id;      // Byte 3: Device ID (e.g., AD7175 #1)
        logic [BYTE_WIDTH*2-1   :0]                 register_addr;  // Byte 4-5: Register Address (e.g., Channel 0)
        logic [BYTE_WIDTH*16-1  :0]                 data_payload;   // Byte 6-21: Data Payload
        logic [BYTE_WIDTH*2-1   :0]                 crc;            // Byte 22-23: CRC-16
    } spi_command_frame_t;

    // Packed union for flexible access
    typedef union packed {
        spi_command_frame_t                         as_frame;       // Structured field access
        logic [BYTE_WIDTH*24-1 :0]                  as_bits;        // Raw 192-bit vector
        logic [BYTE_WIDTH-1:0][BYTE_WIDTH*3-1:0]    as_bytes;       // Packed 2D array: 24 bytes
        // Explanation: [7:0][23:0] means 24 bytes, each 8 bits. as_bytes[23] is MSB (byte 0)
    } spi_command_union_t;

    // Define the SPI Telemetry Frame to the OBC
    typedef struct packed {
        logic [BYTE_WIDTH-1     :0]                 frame_id;       // Byte 0: Mirrors command Frame ID
        logic [BYTE_WIDTH*2-1   :0]                 status;         // Byte 1-2: Status flags (success, data valid, alarms)
        logic [BYTE_WIDTH-1     :0]                 device_id;      // Byte 3: Device ID (echoes command)
        logic [BYTE_WIDTH*24-1  :0]                 data_payload;   // Byte 4-27: Data payload (24 bytes)
        logic [BYTE_WIDTH*4-1   :0]                 timestamp;      // Byte 28-31: Free-running counter snapshot
        logic [BYTE_WIDTH*2-1   :0]                 crc;            // Byte 32-33: CRC-16
    } spi_telemetry_frame_t;

    // Complete union for SPI Telemetry Frame
    typedef union packed {
        spi_telemetry_frame_t                       as_frame;       // Structured field access
        logic [BYTE_WIDTH*34-1  :0]                 as_bits;        // Raw 272-bit vector
        logic [BYTE_WIDTH-1:0][BYTE_WIDTH*4+1:0]    as_bytes;       // Byte array (0..33)
    } spi_telemetry_union_t;

    
    //---------------------------------------------------------------------------------------------------------------------
    // I/O signals
    //---------------------------------------------------------------------------------------------------------------------
    
    input    logic                   rst_n;          // FPGA Reset; active low
    input    logic                   clk;            // FPGA Clock

    // OBC SPI Interface 0
    input    logic                   i_spi_0_clk;
    output   logic                   o_spi_0_miso;
    input    logic                   i_spi_0_mosi;
    input    logic                   i_spi_0_cs_n;

    // OBC SPI Interface 1
    input    logic                   i_spi_1_clk;
    output   logic                   o_spi_1_miso;
    input    logic                   i_spi_1_mosi;
    input    logic                   i_spi_1_cs_n;

    // Sensor Double Buffer Access Contoller Interface

    // Actuator Double Buffer Access Controller Interface

    // Control paths for each ADC Manager 

    // Control paths for each PWM Driver Engine

    // interface for Built In Test Module
    
    //---------------------------------------------------------------------------------------------------------------------
    // Internal signals
    //---------------------------------------------------------------------------------------------------------------------
    
    // for spi_slave_phy_0
    logic                       rx_valid_0;
    logic [BYTE_WIDTH-1:0]      rx_byte_0;
    logic                       tx_valid_0;
    logic [BYTE_WIDTH-1:0]      tx_byte_0;

    // for spi_slave_phy_1
    logic                       rx_valid_1;
    logic [BYTE_WIDTH-1:0]      rx_byte_1;
    logic                       tx_valid_1;
    logic [BYTE_WIDTH-1:0]      tx_byte_1;

    // spy slave phy 0 decoder state register
    spi_input_decoder_state_t       spi_i_dec_state_0;
    
    spi_command_union_t             spi_command_union_0;
    logic [BYTE_CNT_WIDTH_DEC-1]    payload_b_cnt;
    logic                           input_frame_stored;

    // spy slave phy 0 encoder state register
    spi_output_encoder_state_t      spi_o_enc_state_0;

    spi_telemetry_union_t           spi_telemetry_union_0;

    // spy slave phy 1 decoder state register
    spi_input_decoder_state_t       spi_i_dec_state_1;

    spi_command_union_t             spi_command_union_1;

    // spy slave phy 1 encoder state register
    spi_output_encoder_state_t      spi_o_enc_state_1;

    spi_telemetry_union_t           spi_telemetry_union_1;

    //---------------------------------------------------------------------------------------------------------------------
    // Sub Module Instantiation 
    //---------------------------------------------------------------------------------------------------------------------
    
    spi_slave_phy #(
        .SPI_MODE(SPI_MODE_0)
    ) spi_slave_phy_0 (
        // Control/Data Signals,
        .rst_n(rst_n),    // FPGA Reset, active low
        .clk(clk),      // FPGA Clock

        .o_rx_valid(rx_valid_0),    // Data Valid pulse (1 clock cycle)
        .o_rx_byte(rx_byte_0),  // Byte received on MOSI
        .i_tx_valid(tx_valid_0),    // Data Valid pulse to register i_tx_byte
        .i_tx_byte(tx_byte_0),  // Byte to serialize to MISO.

        // SPI Interface
        .i_spi_clk(i_spi_0_clk),
        .o_spi_miso(o_spi_0_miso),
        .i_spi_mosi(i_spi_0_mosi),
        .i_spi_cs_n(i_spi_0_cs_n)        // active low
    );

    spi_slave_phy #(
        .SPI_MODE(SPI_MODE_1)
    ) spi_slave_phy_1 (
        // Control/Data Signals,
        .rst_n(rst_n),    // FPGA Reset, active low
        .clk(clk),      // FPGA Clock

        .o_rx_valid(rx_valid_1),    // Data Valid pulse (1 clock cycle)
        .o_rx_byte(rx_byte_1),  // Byte received on MOSI
        .i_tx_valid(tx_valid_1),    // Data Valid pulse to register i_tx_byte
        .i_tx_byte(tx_byte_1),  // Byte to serialize to MISO.

        // SPI Interface
        .i_spi_clk(i_spi_1_clk),
        .o_spi_miso(o_spi_1_miso),
        .i_spi_mosi(i_spi_1_mosi),
        .i_spi_cs_n(i_spi_1_cs_n)        // active low
    );
    
    //---------------------------------------------------------------------------------------------------------------------
    // Implementation
    //---------------------------------------------------------------------------------------------------------------------
    
    // spi_slave_phy_0 frame decoder/encoder logic
    // decoder 0
    always_ff @(posedge clk) begin : SPI_SLAVE_PHY_0_DEC
        if (!rst_n) begin
            spi_i_dec_state_0           <= I_SPI_IDLE;
            spi_command_union_0.as_bits <= {BYTE_WIDTH*24{1'b0}};
            payload_b_cnt               <= {BYTE_CNT_WIDTH_DEC{1'b0}};
        end
        else begin
            unique case (spi_i_dec_state_0)
                I_SPI_IDLE: begin
                    if ((rx_valid_0 && (rx_byte_0 == 8'h5A))) begin
                        spi_i_dec_state_0   <= I_SPI_START_BYTE_0;
                    end
                end
                I_SPI_START_BYTE_0: begin
                    if ((rx_valid_0 && (rx_byte_0 == 8'h5A))) begin
                        spi_i_dec_state_0   <= I_SPI_START_BYTE_1;
                    end
                end
                I_SPI_START_BYTE_1: begin
                    if (rx_valid_0) begin
                        spi_i_dec_state_0                       <= I_SPI_FRAME_ID;
                        spi_command_union_0.as_frame.frame_id   <= rx_byte_0;
                    end
                end
                I_SPI_FRAME_ID: begin
                    if (rx_valid_0) begin
                        spi_i_dec_state_0                                           <= I_SPI_COMMAND_TYPE_0;
                        spi_command_union_0.as_frame.command_type[15-:BYTE_WIDTH]   <= rx_byte_0;
                    end
                end
                I_SPI_COMMAND_TYPE_0: begin
                    if (rx_valid_0) begin
                        spi_i_dec_state_0                                           <= I_SPI_COMMAND_TYPE_1;
                        spi_command_union_0.as_frame.command_type[7 -:BYTE_WIDTH]   <= rx_byte_0;
                    end
                end
                I_SPI_COMMAND_TYPE_1: begin
                    if (rx_valid_0) begin
                        spi_i_dec_state_0                       <= I_SPI_DEVICE_ID;
                        spi_command_union_0.as_frame.device_id  <= rx_byte_0;
                    end
                end
                I_SPI_DEVICE_ID: begin
                    if (rx_valid_0) begin
                        spi_i_dec_state_0                                           <= I_SPI_REG_ADDR_0;
                        spi_command_union_0.as_frame.register_addr[15-:BYTE_WIDTH]  <= rx_byte_0;
                    end
                end
                I_SPI_REG_ADDR_0: begin
                    if (rx_valid_0) begin
                        spi_i_dec_state_0                                           <= I_SPI_REG_ADDR_1;
                        spi_command_union_0.as_frame.register_addr[7 -:BYTE_WIDTH]  <= rx_byte_0;
                    end
                end
                I_SPI_REG_ADDR_1: begin
                    if (rx_valid_0) begin
                        spi_i_dec_state_0                                                       <= I_SPI_REG_ADDR_1;
                        spi_command_union_0.as_frame.data_payload[BYTE_WIDTH*16-1 -:BYTE_WIDTH] <= rx_byte_0;
                        payload_b_cnt                                                           <= 'd15;
                    end
                end
                I_SPI_PAYLOAD_BYTES: begin
                    if (payload_b_cnt == 'd0) begin
                        spi_i_dec_state_0                                                                   <= I_SPI_CRC_16;
                    end
                    else if (rx_valid_0) begin
                        spi_command_union_0.as_frame.data_payload[BYTE_WIDTH*payload_b_cnt-1 -:BYTE_WIDTH]  <= rx_byte_0;
                        payload_b_cnt                                                                       <= payload_b_cnt-1;
                    end
                end
                I_SPI_CRC_16: begin
                    if (rx_valid_0) begin
                        spi_i_dec_state_0                   <= I_SPI_IDLE;
                        spi_command_union_0.as_frame.crc    <= rx_byte_0;
                        input_frame_stored                  <= 1'b1;    // flag when a frame is fully stored in spi_command_union_0
                    end
                end
                default: begin
                    spi_i_dec_state_0   <= I_SPI_IDLE;
                end
            endcase
        end
    end

    // encoder 0
    always_ff @(posedge clk) begin : SPI_SLAVE_PHY_0_ENC
        if (!rst_n) begin
            spi_o_enc_state_0               <= O_SPI_IDLE;
            spi_telemetry_union_0.as_bits   <= {BYTE_WIDTH*34{1'b0}};
        end
        else begin
            unique case (spi_o_enc_state_0)
                O_SPI_IDLE: begin
                    
                end
                O_SPI_START_BYTE_0: begin
                    
                end
                O_SPI_START_BYTE_1: begin
                    
                end
                O_SPI_FRAME_ID: begin
                    
                end
                O_SPI_STATUS_0: begin
                    
                end
                O_SPI_STATUS_1: begin
                    
                end
                O_SPI_DEVICE_ID: begin
                    
                end
                O_SPI_PAYLOAD_BYTES: begin
                    
                end
                O_SPI_TIMESTAMP_0: begin
                    
                end
                O_SPI_TIMESTAMP_1: begin
                    
                end
                O_SPI_TIMESTAMP_2: begin
                    
                end
                O_SPI_TIMESTAMP_3: begin
                    
                end
                O_SPI_CRC_16: begin
                    
                end
                default: begin
                    spi_o_enc_state_0   <= O_SPI_IDLE;
                end
            endcase
        end
    end

    // spi_slave_phy_1 frame decoder/encoder logic
    // decoder 1
    always_ff @(posedge clk) begin : SPI_SLAVE_PHY_1_DEC
        if (!rst_n) begin
            spi_i_dec_state_1   <= I_SPI_IDLE;
        end
        else begin
            unique case (spi_i_dec_state_1)
                I_SPI_IDLE: begin
                        
                end
                I_SPI_START_BYTE_0: begin
                    
                end
                I_SPI_START_BYTE_1: begin
                    
                end
                I_SPI_FRAME_ID: begin
                    
                end
                I_SPI_COMMAND_TYPE_0: begin
                    
                end
                I_SPI_COMMAND_TYPE_1: begin
                    
                end
                I_SPI_DEVICE_ID: begin
                    
                end
                I_SPI_REG_ADDR_0: begin
                    
                end
                I_SPI_REG_ADDR_1: begin
                    
                end
                I_SPI_PAYLOAD_BYTES: begin
                    
                end
                I_SPI_CRC_16: begin
                    
                end
                default: begin
                    spi_i_dec_state_1   <= I_SPI_IDLE;
                end
            endcase
        end
    end

    // encoder 1
    always_ff @(posedge clk) begin : SPI_SLAVE_PHY_1_ENC
        if (!rst_n) begin
            spi_o_enc_state_1   <= O_SPI_IDLE;
        end
        else begin
            unique case (spi_o_enc_state_1)
                O_SPI_IDLE: begin
                        
                end
                O_SPI_START_BYTE_0: begin
                    
                end
                O_SPI_START_BYTE_1: begin
                    
                end
                O_SPI_FRAME_ID: begin
                    
                end
                O_SPI_STATUS_0: begin
                    
                end
                O_SPI_STATUS_1: begin
                    
                end
                O_SPI_DEVICE_ID: begin
                    
                end
                O_SPI_PAYLOAD_BYTES: begin
                    
                end
                O_SPI_TIMESTAMP_0: begin
                    
                end
                O_SPI_TIMESTAMP_1: begin
                    
                end
                O_SPI_TIMESTAMP_2: begin
                    
                end
                O_SPI_TIMESTAMP_3: begin
                    
                end
                O_SPI_CRC_16: begin
                    
                end
                default: begin
                    spi_o_enc_state_1   <= O_SPI_IDLE;
                end
            endcase
        end
    end


endmodule