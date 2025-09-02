`timescale 1ns / 1ps
`include "pcileech_header.svh"

module transfer_ring(
    input                     rst,
    input                     clk,
    input                     trigger,
    IfMemoryRead.source       read_out,
    IfMemoryWrite.source      write_out,
    IfEventRingRequest.source event_ring_out,
    input  [7:0]              slot_id_in,
    input  [7:0]              ep_id_in,
    input  [15:0]             stream_id_in,
    IfSlotContext.sink        i_slot_ctx,
    output reg [73:0]         set_ep_tr_ptr_out,
    output reg                dbg_pending_request,
    output reg [63:0]         dbg_tr_ptr,
    input                     dbg_status_tr,
    output [3:0]              dbg_state,
    output [3:0]              dbg_sub_task_type,
    output [4:0]              dbg_sub_task_state,
    output [7:0]              dbg_ep_id
);

    reg [7:0]   slot_id;
    reg         db_queue_rd_en;

    wire        db_queue_full;
    wire [23:0] db_queue_din = { stream_id_in, ep_id_in };
    wire        db_queue_empty;
    wire [23:0] db_queue_dout;
    wire [7:0]  ep_id     = db_queue_dout[7:0];
    wire [15:0] stream_id = db_queue_dout[23:8];
    
    assign dbg_ep_id = ep_id;

    fifo_db_slot_1 i_fifo_db_slot_1(
        .srst  ( rst            ),
        .clk   ( clk            ),
        .full  ( db_queue_full  ),
        .din   ( db_queue_din   ),
        .wr_en ( trigger        ),
        .empty ( db_queue_empty ),
        .dout  ( db_queue_dout  ),
        .rd_en ( db_queue_rd_en )
    );

    enum reg [3:0] {
        IDLE,
        START_READ_TRB,
        DELAY_READ_TRB,
        READ_TRB,
        PROCESS_TRB,
        NEXT_TRB,
        UPDATE_TR,
        PROCESS_SUB_TASK,
        COMPLETE,
        DEBUG_ASSERT,
        DEBUG_DELAY,
        DEBUG_WAIT,
        FIFO_DELAY,
        FIFO_READ
    } state;

    typedef enum reg [3:0] {
        NONE,
        HANDLE_NORMAL,
        HANDLE_SETUP,
        HANDLE_DATA,
        HANDLE_STATUS,
        HANDLE_EVENT_DATA,
        SEND_TRANSFER_CMD
    } sub_task_type_t;

    sub_task_type_t sub_task_type;

    localparam STATE_LOOKUP_MAP_LOOKUP   = 5'd0;
    localparam STATE_LOOKUP_MAP_CHECK    = 5'd1;
    localparam STATE_DATA_WRITE_START    = 5'd2;
    localparam STATE_DATA_READ           = 5'd3;
    localparam STATE_DATA_WRITE_ASSERT   = 5'd4;
    localparam STATE_DATA_WRITE_DEASSERT = 5'd5;
    localparam STATE_DATA_WRITE_WAIT     = 5'd6;
    localparam STATE_DATA_INTERRUPT      = 5'd7;
    localparam STATE_DATA_COMPLETION     = 5'd8;
    localparam STATE_DELAY               = 5'd9;
    localparam STATE_DATA_READ_DELAY     = 5'd10;
    localparam STATE_DATA_WRITE_DELAY    = 5'd11;
    localparam STATE_MAP_CHECK           = 5'd12;
    localparam STATE_RESPONSE_NORMAL     = 5'd13;

    reg [4:0] sub_task_state;

    //DEBUG STATE
    assign dbg_state = state;
    assign dbg_sub_task_type = sub_task_type;
    assign dbg_sub_task_state = sub_task_state;

    sub_task_type_t resume_sub_task_type;
    reg [4:0] resume_sub_task_state;

    reg [63:0]  current_pointer;
    reg         current_cycle_state;
    reg [127:0] trb;

    reg [31:0] event_delay_counter;
    localparam DELAY_AFTER_SHORT_PACKET_INTERRUPT = 32'd1000000;

    wire       trb_cycle_bit = trb[96];
    wire [5:0] trb_type      = trb[110:106];

    wire [7:0] ep_ctx_index  = (slot_id - 8'd1) * 8'd31 + ep_id - 8'd1;

    //Transfer Ring Instance is assigned to each slot
    //Handling 31 Endpoints per slot
    (* ram_style = "block" *)
    tr_ctx_t tr_ctx[31];

    wire [4:0] tr_ctx_index = ep_id - 5'd1;

    //---------------------------------------------------------
    // Status Stage Data lookup
    //---------------------------------------------------------
    reg [6:0] lookup_map_index;
    reg [2:0] lookup_delay_counter;

    wire [95:0] lookup_map_out;
    wire [7:0]  lookup_map_slot_id         = lookup_map_out[7:0];
    wire [7:0]  lookup_map_ep_id           = lookup_map_out[15:8];
    wire [7:0]  lookup_map_bm_request_type = lookup_map_out[23:16];
    wire [7:0]  lookup_map_b_request       = lookup_map_out[31:24];
    wire [15:0] lookup_map_w_value         = lookup_map_out[47:32];
    wire [15:0] lookup_map_w_index         = lookup_map_out[63:48];
    wire [15:0] lookup_map_data_length     = lookup_map_out[79:64];
    wire [7:0]  lookup_map_bram_addr       = lookup_map_out[87:80];

    wire        lookup_match_request = ( lookup_map_slot_id         == slot_id                   ) &&
                                       ( lookup_map_ep_id           == ep_id                     ) &&
                                       ( lookup_map_bm_request_type == trb_setup_bm_request_type ) &&
                                       ( lookup_map_b_request       == trb_setup_b_request       ) &&
                                       ( lookup_map_w_value         == trb_setup_w_value         ) &&
                                       ( lookup_map_w_index         == trb_setup_w_index         );

    bram_usb_resp_map i_bram_usb_resp_map(
        .addra ( lookup_map_index ),
        .clka  ( clk              ),
        .douta ( lookup_map_out   )
    );

    reg          lookup_data_en;
    reg  [7:0]   lookup_data_index;
    wire [127:0] lookup_data_out;

    bram_usb_resp_data i_bram_usb_resp_data(
        .addra ( lookup_data_index ),
        .clka  ( clk               ),
        .douta ( lookup_data_out   ),
        .ena   ( lookup_data_en    )
    );

    reg [23:0] transfer_length_counter;

    //---------------------------------------------------------
    // Status Stage Data transfer (prefix: sd_)
    //---------------------------------------------------------

    reg  [7:0]  sd_current_4dw_index;

    //wire [15:0] sd_transfer_length = (lookup_map_data_length > trb_setup_w_length) ? trb_setup_w_length : lookup_map_data_length;
    //wire        sd_short_packet    = trb_data_interrupt_on_short && (trb_setup_w_length > lookup_map_data_length);
    wire [15:0] sd_transfer_length = (lookup_map_data_length > trb_data_transfer_length) ? trb_data_transfer_length : lookup_map_data_length;
    wire        sd_short_packet    = trb_data_transfer_length > lookup_map_data_length;
    wire [7:0]  sd_4dw_count       = sd_transfer_length[11:4] + ((sd_transfer_length[3:0] == 4'd0) ? 8'd0 : 8'd1);

    //---------------------------------------------------------
    // Normal
    //---------------------------------------------------------
    reg        trb_normal_received;
    reg [63:0] trb_normal_buffer_pointer;
    reg [16:0] trb_normal_transfer_length;
    reg [4:0]  trb_normal_td_size;
    reg [9:0]  trb_normal_interrupter_target;
    reg        trb_normal_interrupt_on_short;
    reg        trb_normal_interrupt_on_cplt;
    reg        trb_normal_immediate_data;

    //---------------------------------------------------------
    // Setup Stage
    //---------------------------------------------------------
    wire [9:0]  trb_setup_interrupter_target = trb[31:22];
    wire [7:0]  trb_setup_bm_request_type    = trb[7:0];
    wire [7:0]  trb_setup_b_request          = trb[15:8];
    wire [15:0] trb_setup_w_value            = trb[31:16];
    wire [15:0] trb_setup_w_index            = trb[47:32];
    wire [15:0] trb_setup_w_length           = trb[63:48];

    // td_stalled is used to indicate we should not handle later trbs
    reg        td_stalled;

    //---------------------------------------------------------
    // Data Stage
    //---------------------------------------------------------
    reg [63:0] trb_data_ptr;
    reg [63:0] trb_data_buffer_pointer;
    reg [16:0] trb_data_transfer_length;
    reg [9:0]  trb_data_interrupter_target;
    reg        trb_data_interrupt_on_short;
    reg        trb_data_interrupt_on_cplt;
    reg        trb_data_chain_bit;

    //---------------------------------------------------------
    // Status Stage
    //---------------------------------------------------------
    reg [63:0] trb_status_ptr;
    reg [9:0]  trb_status_interrupter_target;
    reg        trb_status_interrupt_on_cplt;
    reg        trb_status_chain_bit;

    //---------------------------------------------------------
    // Memory Read
    //---------------------------------------------------------
    reg [63:0] mrd_addr;
    reg [31:0] mrd_length;
    reg        mrd_has_request;
    reg        mrd_fifo_en;

    assign read_out.address     = mrd_addr;
    assign read_out.data_length = mrd_length;
    assign read_out.has_request = mrd_has_request;
    assign read_out.rd_en       = mrd_fifo_en;

    //===========================================
    // Memory Write
    //===========================================
    reg [63:0]  mwr_addr;
    reg [31:0]  mwr_length;
    reg         mwr_has_data;

    reg [127:0] mwr_fifo_in;
    reg         mwr_fifo_en;
    reg         mwr_fifo_done;

    assign write_out.address = mwr_addr;
    assign write_out.data_length = mwr_length;
    assign write_out.has_data = mwr_has_data;
    assign write_out.din = mwr_fifo_in;
    assign write_out.wr_en = mwr_fifo_en;
    assign write_out.wr_done = mwr_fifo_done;

    //---------------------------------------------------------
    // Event Ring
    //---------------------------------------------------------
    reg         event_ring_send;
    reg         event_ring_has_request;
    reg [2:0]   event_ring_interrupter_index;
    reg [127:0] event_ring_trb_data;

    assign event_ring_out.send              = event_ring_send;
    assign event_ring_out.has_request       = event_ring_has_request;
    assign event_ring_out.interrupter_index = event_ring_interrupter_index;
    assign event_ring_out.trb_data          = event_ring_trb_data;

    //---------------------------------------------------------
    // Transfer Event Sender
    //---------------------------------------------------------
    reg [63:0] trans_event_trb_pointer;
    reg        trans_event_event_data;
    reg [23:0] trans_event_transfer_length;
    reg [7:0]  trans_event_completion_code;
    reg [2:0]  trans_event_interrupter_index;

    always @ ( posedge clk ) begin
        if (rst) begin
            slot_id <= 8'h0;

            db_queue_rd_en <= 1'b0;

            state               <= IDLE;
            sub_task_type       <= NONE;
            sub_task_state      <= 5'h0;

            resume_sub_task_type  <= NONE;
            resume_sub_task_state <= 5'h0;

            for (int i = 0; i < 31; i++) begin
                tr_ctx[i].lookup.valid       <= 1'b0;
                tr_ctx[i].lookup.bram_addr   <= 8'h0;
                tr_ctx[i].lookup.data_length <= 16'h0;
            end

            set_ep_tr_ptr_out   <= 74'h0;
            current_pointer     <= 64'h0;
            current_cycle_state <= 1'b0;
            trb <= 128'h0;

            event_delay_counter <= 32'h0;
            
            //Status Stage Data lookup
            lookup_map_index     <= 7'h0;
            lookup_data_index    <= 8'h0;
            lookup_data_en       <= 1'b0;
            lookup_delay_counter <= 3'd0;

            sd_current_4dw_index <= 8'h0;

            transfer_length_counter <= 24'h0;

            //Normal
            trb_normal_received <= 1'b0;
            trb_normal_buffer_pointer     <= 64'h0;
            trb_normal_transfer_length    <= 17'h0;
            trb_normal_td_size            <= 5'h0;
            trb_normal_interrupter_target <= 10'h0;
            trb_normal_interrupt_on_short <= 1'b0;
            trb_normal_interrupt_on_cplt  <= 1'b0;
            trb_normal_immediate_data     <= 1'b0;

            td_stalled   <= 1'b0;

            //Data Stage
            trb_data_ptr <= 64'h0;
            trb_data_buffer_pointer     <= 64'h0;
            trb_data_transfer_length    <= 17'h0;
            trb_data_interrupter_target <= 10'h0;
            trb_data_interrupt_on_short <= 1'b0;
            trb_data_interrupt_on_cplt  <= 1'b0;
            trb_data_chain_bit <= 1'b0;

            //Status Stage
            trb_status_ptr <= 64'h0;
            trb_status_interrupter_target <= 10'h0;
            trb_status_interrupt_on_cplt  <= 1'b0;
            trb_status_chain_bit <= 1'b0;

            //Memory Read
            mrd_addr        <= 64'h0;
            mrd_length      <= 32'h0;
            mrd_has_request <= 1'b0;
            mrd_fifo_en     <= 1'b0;

            //Memory Write
            mwr_addr      <= 64'h0;
            mwr_length    <= 32'h0;
            mwr_has_data  <= 1'b0;
            mwr_fifo_en   <= 1'b0;
            mwr_fifo_in   <= 128'h0;
            mwr_fifo_done <= 1'b0;

            //Event Ring
            event_ring_send              <= 1'b0;
            event_ring_has_request       <= 1'b0;
            event_ring_interrupter_index <= 3'h0;
            event_ring_trb_data          <= 128'h0;

            //Transfer Event Sender
            trans_event_trb_pointer       <= 64'h0;
            trans_event_event_data        <= 1'b0;
            trans_event_transfer_length   <= 24'h0;
            trans_event_completion_code   <= 8'h0;
            trans_event_interrupter_index <= 3'h0;
        end else begin
            if (set_ep_tr_ptr_out[0]) begin
                set_ep_tr_ptr_out <= 74'h0;
            end
            if (trigger) begin
                slot_id <= slot_id_in;
            end

            case(state)
                IDLE: begin
                    if (!db_queue_empty) begin
                        db_queue_rd_en <= 1'b1;
                        state <= FIFO_DELAY;
                    end
                end
                FIFO_DELAY: begin
                    db_queue_rd_en <= 1'b0;
                    state <= FIFO_READ;
                end
                FIFO_READ: begin
                    current_pointer     <= i_slot_ctx.ep_ctx[ep_ctx_index].p_tr_dequeue;
                    current_cycle_state <= i_slot_ctx.ep_ctx[ep_ctx_index].cycle_state;

                    state <= START_READ_TRB;
                end
                START_READ_TRB: begin
                    mrd_addr <= current_pointer;
                    mrd_length <= 32'h10;
                    mrd_has_request <= 1'b1;

                    if (read_out.state == RD_COMPLETE) begin
                        mrd_fifo_en <= 1'b1;
                        state <= DELAY_READ_TRB;
                    end
                end
                DELAY_READ_TRB: begin
                    state <= READ_TRB;
                end
                READ_TRB: begin
                    trb <= read_out.dout;
                    mrd_has_request <= 1'b0;
                    mrd_fifo_en     <= 1'b0;

                    //state <= PROCESS_TRB;
                    state <= DEBUG_ASSERT;
                end
                DEBUG_ASSERT: begin
                    //if (trb_cycle_bit != current_cycle_state) begin
                    //    state <= COMPLETE;
                    //end else begin
                        dbg_pending_request <= 1'b1;
                        dbg_tr_ptr <= current_pointer;

                        state <= DEBUG_DELAY;
                    //end
                end
                DEBUG_DELAY: begin
                    dbg_pending_request <= 1'b0;
                    state <= DEBUG_WAIT;
                end
                DEBUG_WAIT: begin
                    if (!dbg_status_tr) begin
                        state <= PROCESS_TRB;
                    end
                end
                PROCESS_TRB: begin
                    if (trb_cycle_bit != current_cycle_state) begin
                        state <= COMPLETE;
                    end else begin
                        case (trb_type)
                            6'd1: begin
                                trb_normal_received <= 1'b1;
                                
                                trb_normal_buffer_pointer     <= trb[63:0];
                                trb_normal_transfer_length    <= trb[80:64];
                                trb_normal_td_size            <= trb[85:81];
                                trb_normal_interrupter_target <= trb[95:86];
                                trb_normal_interrupt_on_short <= trb[98];
                                trb_normal_interrupt_on_cplt  <= trb[101];
                                trb_normal_immediate_data     <= trb[102];

                                transfer_length_counter <= trb[80:64];

                                state <= PROCESS_SUB_TASK;
                                sub_task_type <= HANDLE_NORMAL;
                                sub_task_state <= STATE_RESPONSE_NORMAL;
                            end
                            6'd2: begin //Setup Stage
                                state <= PROCESS_SUB_TASK;
                                sub_task_type <= HANDLE_SETUP;
                                sub_task_state <= STATE_LOOKUP_MAP_LOOKUP;

                                lookup_map_index <= 7'h0;
                            end
                            6'd3: begin //Data Stage
                                trb_data_ptr <= current_pointer;

                                trb_data_buffer_pointer     <= trb[63:0];
                                trb_data_transfer_length    <= trb[80:64];
                                trb_data_interrupter_target <= trb[31:22];
                                trb_data_interrupt_on_short <= trb[98];
                                trb_data_chain_bit          <= trb[100];
                                trb_data_interrupt_on_cplt  <= trb[101];

                                if (!trb[100]) begin
                                    trb_normal_received <= 1'b0;
                                end

                                state <= PROCESS_SUB_TASK;
                                sub_task_type <= HANDLE_DATA;
                                sub_task_state <= STATE_MAP_CHECK;
                            end
                            6'd4: begin //Status Stage
                                trb_status_ptr <= current_pointer;

                                trb_status_interrupter_target <= trb[31:22];
                                trb_status_chain_bit          <= trb[100];
                                trb_status_interrupt_on_cplt  <= trb[101];

                                if (!trb[100]) begin
                                    trb_normal_received <= 1'b0;
                                end

                                state <= PROCESS_SUB_TASK;
                                sub_task_type <= HANDLE_STATUS;

                                sub_task_state <= STATE_DELAY;
                            end
                            6'd6: begin //Link
                                current_pointer <= trb[63:0];
                                if (trb[97]) begin // Toggle Cycle
                                    current_cycle_state <= ~current_cycle_state;
                                end
                                state <= UPDATE_TR;
                            end
                            6'd7: begin //Event Data
                                // Event Data TRB must be chain bit = 0
                                // And Interrupt on Completion bit must be 1
                                trans_event_trb_pointer <= trb[63:0]; //Event Data
                                trans_event_event_data  <= 1'b1;
                                if (trb_normal_received) begin
                                    //trans_event_transfer_length <= { 7'h0, trb_normal_transfer_length };
                                    trans_event_transfer_length <= 24'h0;
                                    trans_event_completion_code <= 8'hD;
                                end else begin
                                    //trans_event_transfer_length <= { 8'h0, sd_transfer_length };
                                    trans_event_transfer_length <= transfer_length_counter;
                                    trans_event_completion_code <= sd_short_packet ? 8'hD : 8'h1;
                                end
                                trans_event_interrupter_index <= trb[88:86];

                                transfer_length_counter <= 24'h0;

                                if (!trb[100]) begin
                                    trb_normal_received <= 1'b0;
                                end

                                state <= PROCESS_SUB_TASK;
                                sub_task_type  <= SEND_TRANSFER_CMD;
                                sub_task_state <= 5'd0;
                            end
                            default: begin
                                state <= NEXT_TRB;
                            end
                        endcase
                    end
                end
                NEXT_TRB: begin
                    current_pointer <= current_pointer + 64'h10;
                    state <= UPDATE_TR;
                end
                UPDATE_TR: begin
                    set_ep_tr_ptr_out[0]    <= 1'b1;
                    set_ep_tr_ptr_out[3:1]  <= slot_id[2:0];
                    set_ep_tr_ptr_out[8:4]  <= ep_id[4:0];
                    set_ep_tr_ptr_out[72:9] <= current_pointer;
                    set_ep_tr_ptr_out[73]   <= current_cycle_state;

                    if (td_stalled) begin
                        td_stalled <= 1'b0;
                        state <= COMPLETE;
                    end else begin
                        state <= START_READ_TRB;
                    end
                end
                PROCESS_SUB_TASK: begin
                    case (sub_task_type)
                        HANDLE_NORMAL: begin
                            case (sub_task_state)
                                STATE_RESPONSE_NORMAL: begin
                                    if (trb_normal_interrupt_on_cplt || trb_normal_interrupt_on_short) begin
                                        trans_event_trb_pointer <= current_pointer;
                                        trans_event_event_data  <= 1'b0;
                                        trans_event_transfer_length <= { 7'h0, trb_normal_transfer_length };
                                        trans_event_completion_code <= 8'hD;
                                        trans_event_interrupter_index <= trb_setup_interrupter_target[2:0];

                                        state <= PROCESS_SUB_TASK;
                                        sub_task_type  <= SEND_TRANSFER_CMD;
                                        sub_task_state <= 5'd0;
                                    end else begin
                                        sub_task_type <= NONE;
                                        sub_task_state <= 5'd0;
                                        state <= NEXT_TRB;
                                    end
                                end
                            endcase
                        end
                        HANDLE_SETUP: begin
                            case (sub_task_state)
                                STATE_LOOKUP_MAP_LOOKUP: begin
                                    if (lookup_delay_counter == 3'd1) begin
                                        lookup_delay_counter <= 3'd0;
                                        sub_task_state <= STATE_LOOKUP_MAP_CHECK;
                                    end else begin
                                        lookup_delay_counter <= lookup_delay_counter + 3'd1;
                                    end
                                end
                                STATE_LOOKUP_MAP_CHECK: begin
                                    if (lookup_match_request) begin
                                        tr_ctx[tr_ctx_index].lookup.valid <= 1'b1;

                                        sub_task_type <= NONE;
                                        sub_task_state <= 5'd0;
                                        state <= NEXT_TRB;
                                    end else begin
                                        tr_ctx[tr_ctx_index].lookup.valid <= 1'b0;
                                        if (lookup_map_index == 7'd127) begin //there is no matched usb packet
                                            // wLength == 0 meaning no data stage
                                            //  so we have to send transfer event with stall
                                            //  and skip later trbs
                                            if (trb_setup_w_length == 16'h0) begin
                                                td_stalled <= 1'b1;

                                                trans_event_trb_pointer <= current_pointer;
                                                trans_event_event_data  <= 1'b0;
                                                trans_event_transfer_length <= 24'h0;
                                                trans_event_completion_code <= 8'h6;
                                                trans_event_interrupter_index <= trb_setup_interrupter_target[2:0];

                                                state <= PROCESS_SUB_TASK;
                                                sub_task_type  <= SEND_TRANSFER_CMD;
                                                sub_task_state <= 5'd0;
                                            end else begin
                                                sub_task_type <= NONE;
                                                sub_task_state <= 5'd0;
                                                state <= NEXT_TRB;
                                            end
                                        end else begin //walk next map entry
                                            lookup_map_index <= lookup_map_index + 7'd1;
                                            sub_task_state   <= STATE_LOOKUP_MAP_LOOKUP;
                                        end
                                    end
                                end
                            endcase
                        end
                        HANDLE_DATA: begin
                            case (sub_task_state)
                                STATE_MAP_CHECK: begin
                                    if (tr_ctx[tr_ctx_index].lookup.valid) begin
                                        if (sd_transfer_length != 0) begin
                                            sd_current_4dw_index <= 8'h0;
                                            sub_task_state <= STATE_DATA_WRITE_START;
                                        end else begin//0 length response
                                            sub_task_state <= STATE_DATA_INTERRUPT;
                                        end
                                    end else begin // there is no matched usb packet
                                        td_stalled <= 1'b1;
                                        
                                        trans_event_trb_pointer <= trb_data_ptr;
                                        trans_event_event_data  <= 1'b0;
                                        trans_event_transfer_length <= { 7'h0, trb_data_transfer_length };
                                        trans_event_completion_code <= 8'h6;
                                        trans_event_interrupter_index <= trb_data_interrupter_target[2:0];

                                        state <= PROCESS_SUB_TASK;
                                        sub_task_type  <= SEND_TRANSFER_CMD;
                                        sub_task_state <= 5'd0;
                                    end
                                end
                                STATE_DATA_WRITE_START: begin
                                    mwr_addr     <= trb_data_buffer_pointer;
                                    mwr_length   <= { 16'h0, sd_transfer_length };
                                    mwr_has_data <= 1'b1;
                                    if (write_out.state == WR_DATA_INIT) begin
                                        lookup_data_index <= lookup_map_bram_addr + sd_current_4dw_index;
                                        sub_task_state <= STATE_DATA_READ;
                                    end
                                end
                                STATE_DATA_READ: begin
                                    lookup_data_en    <= 1'b1;
                                    
                                    sub_task_state <= STATE_DATA_READ_DELAY;
                                end
                                STATE_DATA_READ_DELAY: begin
                                    if (lookup_delay_counter == 3'd3) begin
                                        lookup_delay_counter <= 3'd0;
                                        sub_task_state <= STATE_DATA_WRITE_ASSERT;
                                    end else begin
                                        lookup_delay_counter <= lookup_delay_counter + 3'd1;
                                    end
                                end
                                STATE_DATA_WRITE_ASSERT: begin
                                    mwr_fifo_en    <= 1'b1;
                                    lookup_data_en <= 1'b0;

                                    mwr_fifo_in <= lookup_data_out;

                                    sub_task_state <= STATE_DATA_WRITE_DELAY;
                                end
                                STATE_DATA_WRITE_DELAY: begin
                                    sub_task_state <= STATE_DATA_WRITE_DEASSERT;
                                end
                                STATE_DATA_WRITE_DEASSERT: begin
                                    mwr_fifo_en <= 1'b0;
                                    if ((sd_current_4dw_index + 8'h1) == sd_4dw_count) begin
                                        sd_current_4dw_index <= 8'h0;
                                        mwr_fifo_done <= 1'b1;

                                        sub_task_state <= STATE_DATA_WRITE_WAIT;
                                    end else begin
                                        sd_current_4dw_index <= sd_current_4dw_index + 8'h1;
                                        lookup_data_index <= lookup_data_index + 8'h1;
                                        sub_task_state <= STATE_DATA_READ;
                                    end
                                end
                                STATE_DATA_WRITE_WAIT: begin
                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_has_data  <= 1'b0;
                                        mwr_fifo_done <= 1'b0;

                                        transfer_length_counter <= { 8'h0, sd_transfer_length };

                                        sub_task_state <= STATE_DATA_INTERRUPT;
                                    end
                                end
                                STATE_DATA_INTERRUPT: begin
                                    if (trb_data_interrupt_on_cplt || (sd_short_packet && trb_data_interrupt_on_short)) begin
                                        trans_event_trb_pointer       <= trb_data_ptr;
                                        trans_event_event_data        <= 1'b0;
                                        trans_event_transfer_length   <= { 7'h0, trb_data_transfer_length } - sd_transfer_length;
                                        trans_event_completion_code   <= sd_short_packet ? 8'hD : 8'h1;
                                        trans_event_interrupter_index <= trb_status_interrupter_target[2:0];

                                        state <= PROCESS_SUB_TASK;
                                        sub_task_type  <= SEND_TRANSFER_CMD;
                                        sub_task_state <= 5'd0;
                                    end else begin
                                        sub_task_type <= NONE;
                                        sub_task_state <= 5'd0;
                                        state <= NEXT_TRB;
                                    end
                                end
                            endcase
                        end
                        HANDLE_STATUS: begin
                            case (sub_task_state)
                                STATE_DELAY: begin
                                    if (event_delay_counter >= DELAY_AFTER_SHORT_PACKET_INTERRUPT) begin
                                        event_delay_counter <= 32'h0;

                                        if (trb_status_interrupt_on_cplt) begin
                                            sub_task_state <= STATE_DATA_COMPLETION;
                                        end else begin
                                            state <= NEXT_TRB;
                                            sub_task_type <= NONE;
                                            sub_task_state <= 5'h0;
                                        end
                                    end else begin
                                        event_delay_counter <= event_delay_counter + 32'd1;
                                    end
                                end
                                STATE_DATA_COMPLETION: begin
                                    trans_event_trb_pointer       <= trb_status_ptr;
                                    trans_event_event_data        <= 1'b0;
                                    trans_event_transfer_length   <= { 7'h0, trb_data_transfer_length } - sd_transfer_length;
                                    trans_event_completion_code   <= sd_short_packet ? 8'hD : 8'h1;
                                    trans_event_interrupter_index <= trb_status_interrupter_target[2:0];

                                    state <= PROCESS_SUB_TASK;
                                    sub_task_type  <= SEND_TRANSFER_CMD;
                                    sub_task_state <= 5'd0;
                                end
                            endcase
                        end
                        SEND_TRANSFER_CMD: begin
                            case (sub_task_state)
                                5'd0: begin
                                    event_ring_has_request <= 1'b1;

                                    if (event_ring_out.ready) begin
                                        event_ring_interrupter_index <= trans_event_interrupter_index;
                                        event_ring_trb_data[63:0] <= trans_event_trb_pointer;
                                        event_ring_trb_data[87:64] <= trans_event_transfer_length;
                                        event_ring_trb_data[95:88] <= trans_event_completion_code;
                                        event_ring_trb_data[98] <= trans_event_event_data;
                                        event_ring_trb_data[111:106] <= 6'd32;
                                        event_ring_trb_data[116:112] <= ep_id[4:0];
                                        event_ring_trb_data[127:120] <= slot_id;
                                        event_ring_send <= 1'b1;
                                        sub_task_state <= 5'd1;
                                    end
                                end
                                5'd1: begin
                                    if (event_ring_out.complete) begin
                                        event_ring_send <= 1'b0;
                                        event_ring_has_request <= 1'b0;

                                        if (resume_sub_task_type == NONE) begin
                                            sub_task_type <= NONE;
                                            sub_task_state <= 5'd0;
                                            state <= NEXT_TRB;
                                        end else begin
                                            sub_task_type <= resume_sub_task_type;
                                            sub_task_state <= resume_sub_task_state;
                                            resume_sub_task_type <= NONE;
                                            resume_sub_task_state <= 5'd0;
                                            state <= PROCESS_SUB_TASK;
                                        end
                                    end
                                end
                            endcase
                        end
                    endcase
                end
                COMPLETE: begin
                    if (!trigger) begin
                        state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule