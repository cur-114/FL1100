`timescale 1ns / 1ps
`include "pcileech_header.svh"

module command_ring(
    input                 rst,
    input                 clk,
    input                 trigger,
    IfMemoryRead.source   read_out,
    IfMemoryRead.source   read_out_2,
    IfMemoryWrite.source  write_out,
    IfMemoryWrite.source  write_out_2,
    output reg [4:0]      state,
    input                 update_dequeue_pointer,
    input [57:0]          new_command_ring_pointer,
    input [63:0]          dcbaap,

    IfEnableSlot.source   enable_slot_out,
    IfEventRingRequest.source event_ring_req,
    IfSlotContext.sink    slot_ctx_in,
    output reg [4:0]      control_enabled_slot,
    output reg [67:0]     ctrl_update_slot_ctx_ptr,
    output reg [73:0]     set_ep_tr_ptr_out,
    output reg [8:0]      set_slot_state
);

    //---------------------------------------------------------
    // STATE
    //---------------------------------------------------------
    localparam TRB_TYPE_LINK               = 6'd6;
    localparam TRB_TYPE_ENABLE_SLOT        = 6'd9;
    localparam TRB_TYPE_DISABLE_SLOT       = 6'd10;
    localparam TRB_TYPE_ADDRESS_DEV        = 6'd11;
    localparam TRB_TYPE_CONFIGURE_ENDPOINT = 6'd12;
    localparam TRB_TYPE_EVALUATE_CONTEXT   = 6'd13;
    localparam TRB_TYPE_RESET_ENDPOINT     = 6'd14;
    localparam TRB_TYPE_STOP_ENDPOINT      = 6'd15;
    localparam TRB_TYPE_SET_TR_DEQUEUE     = 6'd16;
    localparam TRB_TYPE_RESET_DEVICE       = 6'd17;

    localparam IDLE            = 5'h0;
    localparam START_READ_RING = 5'h1;
    localparam DELAY_READ_RING = 5'h2;
    localparam READ_RING       = 5'h3;
    localparam PROCESS_RING    = 5'h4;
    localparam NEXT_RING       = 5'h5;
    localparam RUN_SUB_TASK    = 5'h6;

    localparam SUB_TASK_NONE           = 5'h0;
    localparam SUB_TASK_ENABLE_SLOT    = 5'h1;
    localparam SUB_TASK_SEND_CPLT      = 5'h2;
    localparam SUB_TASK_ADDR_DEV       = 5'h3;
    localparam SUB_TASK_RESET_DEVICE   = 5'd4;
    localparam SUB_TASK_STOP_EP        = 5'd5;
    localparam SUB_TASK_RESET_EP       = 5'd6;
    localparam SUB_TASK_SET_TR_DEQUEUE = 5'd7;
    localparam SUB_TASK_CONFIGURE_EP   = 5'd8;
    localparam SUB_TASK_DECONFIGURE_EP = 5'd9;
    localparam SUB_TASK_DISABLE_SLOT   = 5'd10;
    localparam SUB_TASK_CONFIGURE_EP_DISABLE   = 5'd11;
    localparam SUB_TASK_CONFIGURE_EP_ENABLE    = 5'd12;
    localparam SUB_TASK_CONFIGURE_EP_CLONE     = 5'd13;
    localparam SUB_TASK_CONFIGURE_EP_COMPLETE  = 5'd14;
    localparam SUB_TASK_EVALUATE_CONTEXT       = 5'd15;
    localparam SUB_TASK_EVALUATE_CONTEXT_SLOT  = 5'd16;
    localparam SUB_TASK_EVALUATE_CONTEXT_EP    = 5'd17;

    reg [4:0] sub_task_type;
    reg [4:0] sub_task_state;

    reg [31:0] event_delay_counter;

    //---------------------------------------------------------
    // Memory Write
    //---------------------------------------------------------
    reg [63:0] mwr_addr;
    reg [31:0] mwr_length;
    reg        mwr_has_data;

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
    // Memory Read
    //---------------------------------------------------------
    reg [63:0]  command_ring_dequeue_pointer;

    reg [63:0] mrd_addr;
    reg [31:0] mrd_length;
    reg        mrd_has_request;
    reg        mrd_fifo_en;

    assign read_out.address     = mrd_addr;
    assign read_out.data_length = mrd_length;
    assign read_out.has_request = mrd_has_request;
    assign read_out.rd_en       = mrd_fifo_en;

    reg [127:0] trb;      //Saved TRB Read Result
    reg ring_cycle_state;
    
    wire       trb_cycle_bit = trb[96];
    wire [5:0] trb_type      = trb[110:106];
    wire [7:0] trb_slot_id   = trb[127:120];
    wire [4:0] trb_ep_id     = trb[116:112];

    //---------------------------------------------------------
    // Address Device Command
    //---------------------------------------------------------

    wire [63:0] p_input_context = { trb[63:4], 4'h0 }; //Input Context Pointer
    wire block_set_address_request = trb[105];
    wire [63:0] p_input_slot_context = p_input_context + 64'h20;
    wire [63:0] p_input_ep_0_context_lo = p_input_context + 64'h40;

    reg  [63:0] p_output_context;
    wire [63:0] p_output_ep_0_context_lo = p_output_context + 64'h20;

    //---------------------------------------------------------
    // Configure Endpoint Command
    //---------------------------------------------------------

    reg  [63:0]  buffer_input_control;
    reg  [4:0]   cfg_ep_walk_bit;
    reg  [4:0]   cfg_ep_input_context_entries;

    wire [63:0]  trb_p_input_ctrl   = trb[63:0];
    wire         cfg_ep_deconfigure = trb[105];

    //---------------------------------------------------------
    // Configure Endpoint State
    //---------------------------------------------------------

    localparam STATE_CE_START_READ_INPUT_CTRL = 5'd1;
    localparam STATE_CE_READ_INPUT_CTRL_DELAY = 5'd2;
    localparam STATE_CE_READ_INPUT_CTRL       = 5'd3;
    localparam STATE_CE_CHECK_DISABLE_EP      = 5'd4;
    localparam STATE_CE_CHECK_ENABLE_EP       = 5'd5;
    localparam STATE_CE_CHECK_NEXT_EP         = 5'd6;
    localparam STATE_CE_DISABLE_EP            = 5'd7;
    localparam STATE_CE_DISABLE_EP_CLEANUP    = 5'd8;
    localparam STATE_CE_ENABLE_EP_WRITE_INIT  = 5'd9;
    localparam STATE_CE_ENABLE_EP_READ_START  = 5'd10;
    localparam STATE_CE_ENABLE_EP_READ_DELAY  = 5'd11;
    localparam STATE_CE_ENABLE_EP_READ_LO     = 5'd12;
    localparam STATE_CE_ENABLE_EP_READ_HI     = 5'd13;
    localparam STATE_CE_ENABLE_EP_WRITE_WAIT  = 5'd14;
    localparam STATE_CE_SLOT_READ_1_START     = 5'd15;
    localparam STATE_CE_SLOT_READ_1_DELAY     = 5'd16;
    localparam STATE_CE_SLOT_READ_1           = 5'd17;
    localparam STATE_CE_SLOT_READ_2_START     = 5'd18;
    localparam STATE_CE_SLOT_READ_2_DELAY     = 5'd19;
    localparam STATE_CE_SLOT_READ_2           = 5'd20;
    localparam STATE_CE_SLOT_WRITE_INIT       = 5'd21;
    localparam STATE_CE_SLOT_WRITE_WAIT       = 5'd22;
    localparam STATE_CE_COMPLETE              = 5'd23;

    //---------------------------------------------------------
    // Evaluate Context State
    //---------------------------------------------------------

    typedef enum reg [4:0] {
        STATE_EC_READ_INPUT_CTRL_START,
        STATE_EC_READ_INPUT_CTRL_DELAY,
        STATE_EC_READ_INPUT_CTRL,
        STATE_EC_CHECK_INPUT_CTRL,
        STATE_EC_COMPLETE,

        //Slot Context
        STATE_EC_READ_OSLOT_CTX_START,
        STATE_EC_READ_OSLOT_CTX_DELAY,
        STATE_EC_READ_OSLOT_CTX,
        STATE_EC_READ_ISLOT_CTX_START,
        STATE_EC_READ_ISLOT_CTX_DELAY,
        STATE_EC_READ_ISLOT_CTX,
        STATE_EC_WRITE_OSLOT_CTX_INIT,
        STATE_EC_WRITE_OSLOT_CTX_WAIT,

        //Endpoint Context
        STATE_EC_READ_IEP_CTX_START,
        STATE_EC_READ_IEP_CTX_DELAY,
        STATE_EC_READ_IEP_CTX,
        STATE_EC_WRITE_OEP_CTX_INIT,
        STATE_EC_READ_OEP_CTX_START,
        STATE_EC_READ_OEP_CTX_DELAY,
        STATE_EC_READ_OEP_CTX_LO,
        STATE_EC_READ_OEP_CTX_HI,
        STATE_EC_WRITE_OEP_CTX_WAIT
    } evaluate_context_state_t;

    //---------------------------------------------------------
    // Event Ring Request
    //---------------------------------------------------------

    reg         event_ring_send;
    reg         event_ring_has_request;
    reg [2:0]   event_ring_interrupter_index;
    reg [127:0] event_ring_trb_data;
    reg [7:0]   command_completion_code;
    reg [7:0]   command_completion_slot_id;

    assign event_ring_req.send              = event_ring_send;
    assign event_ring_req.has_request       = event_ring_has_request;
    assign event_ring_req.interrupter_index = event_ring_interrupter_index;
    assign event_ring_req.trb_data          = event_ring_trb_data;

    //---------------------------------------------------------
    // Set Endpoint State ( prefix: ses_ )
    //---------------------------------------------------------

    IfSetEPState if_set_ep_state();

    reg       ses_has_request;
    reg [2:0] ses_slot_id;
    reg [4:0] ses_ep_id;
    reg [2:0] ses_state;
    
    assign if_set_ep_state.has_request = ses_has_request;
    assign if_set_ep_state.slot_id     = ses_slot_id;
    assign if_set_ep_state.ep_id       = ses_ep_id;
    assign if_set_ep_state.state       = ses_state;

    set_ep_state i_set_ep_state(
        .rst             ( rst                  ),
        .clk             ( clk                  ),
        .set_ep_state_in ( if_set_ep_state.sink ),
        .if_slot_ctx     ( slot_ctx_in          ),
        .read_out        ( read_out_2           ),
        .write_out       ( write_out_2          )
    );

    always @ ( posedge clk ) begin
        if (rst) begin
            state <= IDLE;
            control_enabled_slot     <= 5'h0;
            ctrl_update_slot_ctx_ptr <= 68'h0;
            set_ep_tr_ptr_out        <= 74'h0;
            set_slot_state           <= 9'h0;
            
            sub_task_type  <= SUB_TASK_NONE;
            sub_task_state <= 5'h0;

            event_delay_counter <= 32'h0;
            
            //---------------------------------------------------------
            // Memory Write
            //---------------------------------------------------------

            mwr_addr      <= 64'h0;
            mwr_length    <= 32'h0;
            mwr_has_data  <= 1'b0;
            mwr_fifo_en   <= 1'b0;
            mwr_fifo_in   <= 128'h0;
            mwr_fifo_done <= 1'b0;
            
            //---------------------------------------------------------
            // Memory Read
            //---------------------------------------------------------

            command_ring_dequeue_pointer <= 64'h0;

            mrd_addr        <= 64'h0;
            mrd_length      <= 32'h0;
            mrd_has_request <= 1'b0;
            mrd_fifo_en     <= 1'b0;

            trb              <= 28'h0;
            ring_cycle_state <= 1'b1;

            //---------------------------------------------------------
            // Address Device Command
            //---------------------------------------------------------

            p_output_context <= 64'h0;

            //---------------------------------------------------------
            // Event Ring
            //---------------------------------------------------------

            event_ring_send <= 1'b0;
            event_ring_has_request <= 1'b0;
            event_ring_interrupter_index <= 3'h0;
            event_ring_trb_data <= 128'h0;
            command_completion_code <= 8'h0;
            command_completion_slot_id <= 8'h0;

            //Configure Endpoint Command
            buffer_input_control <= 64'h0;
            cfg_ep_walk_bit <= 5'h0;
            cfg_ep_input_context_entries <= 5'h0;

            //Set Endpoint State
            ses_has_request <= 1'b0;
            ses_slot_id     <= 3'b0;
            ses_ep_id       <= 5'b0;
            ses_state       <= 3'b0;
        end else begin
            if (update_dequeue_pointer)
                command_ring_dequeue_pointer <= { new_command_ring_pointer, 6'h0 };

            if (ctrl_update_slot_ctx_ptr[0])
                ctrl_update_slot_ctx_ptr <= 68'h0;
            if (set_ep_tr_ptr_out[0])
                set_ep_tr_ptr_out <= 74'h0;
            
            case (state)
                IDLE: begin
                    if (trigger)
                        state <= START_READ_RING;
                end
                START_READ_RING: begin
                    mrd_addr        <= command_ring_dequeue_pointer;
                    mrd_length      <= 32'h10;
                    mrd_has_request <= 1'b1;

                    if (read_out.state == RD_COMPLETE) begin
                        mrd_fifo_en <= 1'b1;
                        state <= DELAY_READ_RING;
                    end
                end
                DELAY_READ_RING: begin
                    state <= READ_RING;
                end
                READ_RING: begin
                    trb <= read_out.dout;
                    //cleanup
                    mrd_addr        <= 64'h0;
                    mrd_length      <= 32'h0;
                    mrd_has_request <= 1'b0;
                    mrd_fifo_en     <= 1'b0;

                    state <= PROCESS_RING;
                end
                PROCESS_RING: begin
                    if (trb_cycle_bit != ring_cycle_state) begin
                        //現在のCycle Bitと違うTRB = 最終TRBまで読み取った -> IDLE移行
                        state <= IDLE;
                    end else begin
                        //TRB Typeに合わせて処理
                        if (trb_type == TRB_TYPE_LINK) begin //Link TRB
                            command_ring_dequeue_pointer <= { trb[63:4], 4'b0000 };
                            if (trb[97]) begin //Toggle Cycle Bit
                                ring_cycle_state <= ~ring_cycle_state;
                            end

                            state <= NEXT_RING;
                        end else if (trb_type == TRB_TYPE_ENABLE_SLOT) begin
                            sub_task_type <= SUB_TASK_ENABLE_SLOT;
                            sub_task_state <= 5'h0;

                            state <= RUN_SUB_TASK;
                        end else if (trb_type == TRB_TYPE_DISABLE_SLOT) begin
                            sub_task_type <= SUB_TASK_DISABLE_SLOT;
                            sub_task_state <= 5'h0;
                            
                            state <= RUN_SUB_TASK;
                        end else if (trb_type == TRB_TYPE_ADDRESS_DEV) begin
                            sub_task_type <= SUB_TASK_ADDR_DEV;
                            sub_task_state <= 5'h0;

                            state <= RUN_SUB_TASK;
                        end else if (trb_type == TRB_TYPE_CONFIGURE_ENDPOINT) begin
                            sub_task_type <= SUB_TASK_CONFIGURE_EP;
                            sub_task_state <= 5'h0;

                            state <= RUN_SUB_TASK;
                        end else if (trb_type == TRB_TYPE_EVALUATE_CONTEXT) begin
                            sub_task_type  <= SUB_TASK_EVALUATE_CONTEXT;
                            sub_task_state <= STATE_EC_READ_INPUT_CTRL_START;

                            state <= RUN_SUB_TASK;
                        end else if (trb_type == TRB_TYPE_RESET_ENDPOINT) begin
                            sub_task_type <= SUB_TASK_RESET_EP;
                            sub_task_state <= 5'h0;

                            state <= RUN_SUB_TASK;
                        end else if (trb_type == TRB_TYPE_STOP_ENDPOINT) begin
                            sub_task_type <= SUB_TASK_STOP_EP;
                            sub_task_state <= 5'h0;

                            state <= RUN_SUB_TASK;
                        end else if (trb_type == TRB_TYPE_SET_TR_DEQUEUE) begin
                            sub_task_type <= SUB_TASK_SET_TR_DEQUEUE;
                            sub_task_state <= 5'h0;

                            state <= RUN_SUB_TASK;
                        end else if (trb_type == TRB_TYPE_RESET_DEVICE) begin
                            sub_task_type <= SUB_TASK_RESET_DEVICE;
                            sub_task_state <= 5'h0;

                            state <= RUN_SUB_TASK;
                        end else begin
                            command_completion_slot_id <= trb_slot_id;
                            command_completion_code <= 8'h0; //Unknown Command

                            sub_task_type <= SUB_TASK_SEND_CPLT;
                            sub_task_state <= 5'h0;
                        end
                    end
                end
                NEXT_RING: begin
                    //次のRING読み取り
                    if (trb_type != TRB_TYPE_LINK)
                        command_ring_dequeue_pointer <= command_ring_dequeue_pointer + 64'h10;
                    state <= START_READ_RING;
                end
                RUN_SUB_TASK: begin
                    case (sub_task_type)
                        SUB_TASK_NONE: state <= NEXT_RING;
                        SUB_TASK_ENABLE_SLOT: begin
                            case(sub_task_state)
                                // find free port/slot
                                5'h0: begin
                                    if (!slot_ctx_in.slot_context[0].slot_enabled) begin
                                        command_completion_slot_id <= 8'h1;
                                    end else if (!slot_ctx_in.slot_context[1].slot_enabled) begin
                                        command_completion_slot_id <= 8'h2;
                                    end else if (!slot_ctx_in.slot_context[2].slot_enabled) begin
                                        command_completion_slot_id <= 8'h3;
                                    end else if (!slot_ctx_in.slot_context[3].slot_enabled) begin
                                        command_completion_slot_id <= 8'h4;
                                    end else if (!slot_ctx_in.slot_context[4].slot_enabled) begin
                                        command_completion_slot_id <= 8'h5;
                                    end else if (!slot_ctx_in.slot_context[5].slot_enabled) begin
                                        command_completion_slot_id <= 8'h6;
                                    end else if (!slot_ctx_in.slot_context[6].slot_enabled) begin
                                        command_completion_slot_id <= 8'h7;
                                    end else if (!slot_ctx_in.slot_context[7].slot_enabled) begin
                                        command_completion_slot_id <= 8'h8;
                                    end else begin
                                        command_completion_slot_id <= 8'h0;
                                    end
                                    sub_task_state <= 5'h1;
                                end
                                // update port registers
                                5'h1: begin
                                    if (command_completion_slot_id != 8'h0) begin
                                        enable_slot_out.enable_slot  <= 1'b1;
                                        enable_slot_out.target_index <= command_completion_slot_id[2:0] - 3'h1;
                                        command_completion_code <= 8'h1;

                                        control_enabled_slot[1:0] <= 2'b11;
                                        control_enabled_slot[4:2] <= command_completion_slot_id[2:0] - 3'h1;
                                    end else begin
                                        command_completion_code <= 8'h9;
                                    end
                                    sub_task_state <= 5'h2;
                                end
                                5'h2: begin
                                    control_enabled_slot[4:0] <= 5'h0;
                                    enable_slot_out.enable_slot <= 1'b0;

                                    sub_task_type <= SUB_TASK_SEND_CPLT;
                                    sub_task_state <= 5'h0;
                                end
                            endcase
                        end
                        SUB_TASK_SEND_CPLT: begin
                            case (sub_task_state)
                                // Command Completion Event
                                5'h0: begin
                                    event_ring_has_request <= 1'b1;
                                    
                                    if (event_ring_req.ready) begin
                                        event_ring_interrupter_index <= 3'h0;
                                        event_ring_trb_data[63:0]    <= command_ring_dequeue_pointer;
                                        event_ring_trb_data[95:64]   <= { command_completion_code, 24'h0 };
                                        //event_ring_trb_data[95:64]   <= 32'h01000000;
                                        event_ring_trb_data[127:96]  <= { command_completion_slot_id, 8'h0, 6'h21, 10'h0 };
                                        event_ring_send <= 1'b1;

                                        sub_task_state <= 5'h1;
                                    end
                                end
                                // Wait for send TRB
                                5'h1: begin
                                    if (event_ring_req.complete) begin
                                        event_ring_send <= 1'b0;
                                        event_ring_has_request <= 1'b0;
                                        
                                        sub_task_state <= SUB_TASK_NONE;
                                        sub_task_state <= 5'h0;
                                        state <= NEXT_RING;
                                    end
                                end
                            endcase
                        end
                        SUB_TASK_ADDR_DEV: begin
                            case (sub_task_state)
                                // Read Output Context Pointer
                                5'd0: begin
                                    mrd_addr        <= dcbaap + 64'h8 * trb_slot_id;
                                    mrd_length      <= 32'h8;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en    <= 1'b1;
                                        sub_task_state <= 5'd1;
                                    end
                                end
                                5'd1: begin
                                    sub_task_state <= 5'd2;
                                end
                                5'd2: begin
                                    p_output_context <= read_out.dout[63:0];

                                    ctrl_update_slot_ctx_ptr[0]    <= 1'b1;
                                    ctrl_update_slot_ctx_ptr[3:1]  <= trb_slot_id[2:0] - 3'd1;
                                    ctrl_update_slot_ctx_ptr[67:4] <= read_out.dout[63:0];

                                    mrd_has_request <= 1'b0;
                                    mrd_fifo_en     <= 1'b0;

                                    sub_task_state <= 5'd3;
                                end
                                // Read Slot Context Data
                                5'd3: begin
                                    mrd_addr        <= p_input_slot_context;
                                    mrd_length      <= 32'h10;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en    <= 1'b1;
                                        sub_task_state <= 5'd4;
                                    end
                                end
                                5'd4: begin
                                    sub_task_state <= 5'd5;
                                end
                                5'd5: begin
                                    mwr_fifo_in <= read_out.dout;

                                    mrd_has_request <= 1'b0;
                                    mrd_fifo_en     <= 1'b0;

                                    sub_task_state <= 5'd6;
                                end
                                //Modify Slot Context Data
                                5'd6: begin
                                    if (block_set_address_request) begin
                                        mwr_fifo_in[127:96] <= 32'h8000000; // 1 << 27 (SLOT_DEAFUT << SLOT_STATE_SHIFT)

                                        set_slot_state[0] <= 1'b1;
                                        set_slot_state[3:1] <= trb_slot_id[2:0];
                                        set_slot_state[8:4] <= DEFAULT;
                                    end else begin
                                        mwr_fifo_in[127:96] <= (32'h2 << 32'h7) | trb_slot_id[2:0];
                                    end
                                    sub_task_state <= 5'd7;
                                end
                                //Write Slot Context Data to Output
                                5'd7: begin
                                    set_slot_state[8:0] <= 9'h0;

                                    mwr_addr     <= p_output_context;
                                    mwr_length   <= 32'h10;
                                    mwr_has_data <= 1'b1;
                                    if (write_out.state == WR_DATA_INIT) begin
                                        mwr_fifo_en <= 1'b1;
                                        sub_task_state <= 5'd8;
                                    end
                                end
                                5'd8: begin
                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_addr      <= 64'h0;
                                        mwr_length    <= 32'h0;
                                        mwr_has_data  <= 1'b0;
                                        mwr_fifo_done <= 1'b0;

                                        sub_task_state <= 5'd9;
                                        //sub_task_state <= 5'd0;
                                        //sub_task_type  <= SUB_TASK_SEND_CPLT;
                                    end else begin
                                        mwr_fifo_en   <= 1'b0;
                                        mwr_fifo_in   <= 128'h0;
                                        mwr_fifo_done <= 1'b1;
                                    end
                                end
                                //Start DATA INIT for EP0 Context 0x20 bytes
                                5'd9: begin
                                    mwr_addr     <= p_output_ep_0_context_lo;
                                    mwr_length   <= 32'h20;
                                    mwr_has_data <= 1'b1;
                                    
                                    if (write_out.state == WR_DATA_INIT) begin
                                        sub_task_state <= 5'd10;
                                    end
                                end
                                //Read EP0 Context data 0x20 bytes
                                5'd10: begin
                                    mrd_addr <= p_input_ep_0_context_lo;
                                    mrd_length <= 32'h20;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en <= 1'b1;
                                        sub_task_state <= 5'd11;
                                    end
                                end
                                5'd11: begin
                                    sub_task_state <= 5'd12;
                                end
                                5'd12: begin //0x00-0x10
                                    mwr_fifo_in[127:3] <= read_out.dout[127:3];
                                    mwr_fifo_in[2:0]   <= 3'd1;
                                    mwr_fifo_en        <= 1'b1;

                                    //WTF this transfer ring dequeue pointer should not 0. but 0..?
                                    set_ep_tr_ptr_out[0]    <= 1'b1;
                                    set_ep_tr_ptr_out[3:1]  <= trb_slot_id[2:0];
                                    set_ep_tr_ptr_out[8:4]  <= 5'd1;
                                    set_ep_tr_ptr_out[72:9] <= { read_out.dout[127:68], 4'h0 };
                                    set_ep_tr_ptr_out[73]   <= read_out.dout[64];

                                    sub_task_state <= 5'd13;
                                end
                                5'd13: begin //0x10-0x20
                                    mwr_fifo_in <= read_out.dout;
                                    mrd_fifo_en <= 1'b0;
                                    mrd_has_request <= 1'b0;

                                    sub_task_state <= 5'd14;
                                end
                                //DEASSERT fifo_enable + start write
                                5'd14: begin
                                    mwr_fifo_en   <= 1'b0;
                                    mwr_fifo_in   <= 128'h0;
                                    mwr_fifo_done <= 1'b1;
                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_addr      <= 64'h0;
                                        mwr_length    <= 32'h0;
                                        mwr_has_data  <= 1'b0;
                                        mwr_fifo_done <= 1'b0;

                                        //SEND COMMAND COMPLETION EVENT
                                        command_completion_code    <= 8'h1;
                                        command_completion_slot_id <= 8'h1;
                                        sub_task_state <= 5'd0;
                                        sub_task_type  <= SUB_TASK_SEND_CPLT;
                                    end
                                end
                            endcase
                        end
                        SUB_TASK_RESET_DEVICE: begin
                            case (sub_task_state)
                                5'd0: begin
                                    if (slot_ctx_in.slot_context[trb_slot_id - 1].slot_state == DEFAULT) begin
                                        command_completion_slot_id <= trb_slot_id;
                                        command_completion_code <= 8'd19;

                                        sub_task_type <= SUB_TASK_SEND_CPLT;
                                        sub_task_state <= 5'd0;
                                    end else begin
                                        set_slot_state[0] <= 1'b1;
                                        set_slot_state[3:1] <= trb_slot_id[2:0];
                                        set_slot_state[8:4] <= DEFAULT;

                                        sub_task_state <= 5'd1;
                                    end
                                end
                                5'd1: begin
                                    set_slot_state[8:0] <= 9'd0;

                                    mrd_addr        <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx;
                                    mrd_length      <= 32'h10;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en <= 1'b1;
                                        sub_task_state <= 5'd2;
                                    end
                                end
                                5'd2: begin
                                    sub_task_state <= 5'd3;
                                end
                                5'd3: begin
                                    mrd_has_request <= 1'b0;
                                    mrd_fifo_en <= 1'b0;

                                    mwr_fifo_in <= read_out.dout;
                                    sub_task_state <= 5'd4;
                                end
                                5'd4: begin
                                    mwr_fifo_in[127:123] <= 5'd1;

                                    mwr_addr <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx;
                                    mwr_length <= 32'h10;
                                    mwr_has_data <= 1'b1;
                                    if (write_out.state == WR_DATA_INIT) begin
                                        mwr_fifo_en <= 1'b1;
                                        sub_task_state <= 5'd5;
                                    end
                                end
                                5'd5: begin
                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_has_data  <= 1'b0;
                                        mwr_fifo_done <= 1'b0;

                                        sub_task_state <= 5'd6;
                                    end else begin
                                        mwr_fifo_en   <= 1'b0;
                                        mwr_fifo_in   <= 128'h0;
                                        mwr_fifo_done <= 1'b1;
                                    end
                                end
                                5'd6: begin
                                    command_completion_slot_id <= trb_slot_id;
                                    command_completion_code <= 8'd1;

                                    sub_task_type <= SUB_TASK_SEND_CPLT;
                                    sub_task_state <= 5'd0;
                                end
                            endcase
                        end
                        SUB_TASK_STOP_EP: begin
                            case (sub_task_state)
                                5'd0: begin
                                    mwr_addr     <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx + 64'h20 * trb_ep_id;
                                    mwr_length   <= 32'h20;
                                    mwr_has_data <= 1'b1;

                                    if (write_out.state == WR_DATA_INIT) begin
                                        sub_task_state <= 5'd1;
                                    end
                                end
                                5'd1: begin //Read Endpoint Context
                                    mrd_addr <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx + 64'h20 * trb_ep_id;
                                    mrd_length <= 32'h20;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en <= 1'b1;
                                        sub_task_state <= 5'd2;
                                    end
                                end
                                5'd2: begin
                                    sub_task_state <= 5'd3;
                                end
                                5'd3: begin
                                    mwr_fifo_in[127:3] <= read_out.dout[127:3];
                                    mwr_fifo_in[2:0]   <= 3'd3;
                                    mwr_fifo_en        <= 1'b1;
                                    
                                    sub_task_state <= 5'd4;
                                end
                                5'd4: begin
                                    mwr_fifo_in <= read_out.dout;
                                    mrd_fifo_en <= 1'b0;
                                    mrd_has_request <= 1'b0;

                                    sub_task_state <= 5'd5;
                                end
                                5'd5: begin
                                    mwr_fifo_en   <= 1'b0;
                                    mwr_fifo_in   <= 128'h0;
                                    mwr_fifo_done <= 1'b1;
                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_addr      <= 64'h0;
                                        mwr_length    <= 32'h0;
                                        mwr_has_data  <= 1'b0;
                                        mwr_fifo_done <= 1'b0;

                                        //SEND COMMAND COMPLETION EVENT
                                        command_completion_code    <= 8'h1;
                                        command_completion_slot_id <= 8'h1;
                                        sub_task_state <= 5'd0;
                                        sub_task_type  <= SUB_TASK_SEND_CPLT;
                                    end
                                end
                            endcase
                        end
                        SUB_TASK_RESET_EP: begin
                            case (sub_task_state)
                                5'd0: begin
                                    command_completion_code    <= 8'h1;
                                    command_completion_slot_id <= 8'h1;
                                    sub_task_state <= 5'd0;
                                    sub_task_type  <= SUB_TASK_SEND_CPLT;
                                end
                            endcase
                        end
                        SUB_TASK_SET_TR_DEQUEUE: begin
                            case (sub_task_state)
                                5'd0: begin
                                    set_ep_tr_ptr_out[0]    <= 1'b1;
                                    set_ep_tr_ptr_out[3:1]  <= trb_slot_id[2:0];
                                    set_ep_tr_ptr_out[8:4]  <= trb_ep_id;
                                    set_ep_tr_ptr_out[72:9] <= { trb[63:4], 4'h0 };
                                    set_ep_tr_ptr_out[73]   <= trb[0];

                                    sub_task_state <= 5'd1;
                                end
                                5'd1: begin
                                    if (event_delay_counter >= 32'd10000000) begin
                                        event_delay_counter <= 32'h0;

                                        sub_task_state <= 5'd2;
                                    end else begin
                                        event_delay_counter <= event_delay_counter + 32'd1;
                                    end
                                end
                                5'd2: begin
                                    mwr_addr     <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx + 64'h20 * trb_ep_id;
                                    mwr_length   <= 32'h20;
                                    mwr_has_data <= 1'b1;

                                    if (write_out.state == WR_DATA_INIT) begin
                                        sub_task_state <= 5'd3;
                                    end
                                end
                                5'd3: begin
                                    mrd_addr        <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx + 64'h20 * trb_ep_id;
                                    mrd_length      <= 32'h20;
                                    mrd_has_request <= 1'b1;
                                    
                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en <= 1'b1;    
                                        sub_task_state <= 5'd4;      
                                    end
                                end
                                5'd4: begin
                                    sub_task_state <= 5'd5;
                                end
                                5'd5: begin
                                    mwr_fifo_in[63:0]   <= read_out.dout[63:0];
                                    mwr_fifo_in[64]     <= trb[0];
                                    mwr_fifo_in[67:65]  <= 3'h0;
                                    mwr_fifo_in[127:68] <= trb[63:4];

                                    mwr_fifo_en <= 1'b1;

                                    sub_task_state <= 5'd6;
                                end
                                5'd6: begin
                                    mwr_fifo_in <= read_out.dout;
                                    mrd_fifo_en <= 1'b0;
                                    mrd_has_request <= 1'b0;

                                    sub_task_state <= 5'd7;
                                end
                                5'd7: begin
                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_addr <= 64'h0;
                                        mwr_length <= 32'h0;
                                        mwr_has_data <= 1'b0;
                                        mwr_fifo_done <= 1'b0;

                                        sub_task_state <= 5'd8;
                                    end else begin
                                        mwr_fifo_en   <= 1'b0;
                                        mwr_fifo_in   <= 128'h0;
                                        mwr_fifo_done <= 1'b1;
                                    end
                                end
                                5'd8: begin
                                    command_completion_code    <= 8'h1;
                                    command_completion_slot_id <= trb_slot_id;
                                    sub_task_state <= 5'd0;
                                    sub_task_type  <= SUB_TASK_SEND_CPLT;
                                end
                            endcase
                        end
                        SUB_TASK_DECONFIGURE_EP: begin
                            case (sub_task_state)
                                5'd0: begin
                                    cfg_ep_walk_bit <= 5'd2; //EP1からスタート
                                    sub_task_state <= 5'd1;
                                end
                                5'd1: begin
                                    ses_has_request <= 1'b1;
                                    ses_slot_id     <= trb_slot_id[2:0];
                                    ses_ep_id       <= cfg_ep_walk_bit;
                                    ses_state       <= 3'd0;

                                    if (if_set_ep_state.done) begin
                                        sub_task_state <= 5'd2;
                                    end
                                end
                                5'd2: begin
                                    ses_has_request <= 1'b0;
                                    ses_slot_id     <= 3'd0;
                                    ses_ep_id       <= 5'd0;
                                    ses_state       <= 3'd0;

                                    if (cfg_ep_walk_bit == 5'd31) begin
                                        sub_task_state <= 5'd3;
                                    end else begin
                                        cfg_ep_walk_bit <= cfg_ep_walk_bit + 5'd1;
                                        sub_task_state <= 5'd1;
                                    end
                                end
                                5'd3: begin
                                    command_completion_code    <= 8'h1;
                                    command_completion_slot_id <= trb_slot_id;

                                    sub_task_state <= 5'd0;
                                    sub_task_type  <= SUB_TASK_SEND_CPLT;
                                end
                            endcase
                        end
                        SUB_TASK_CONFIGURE_EP: begin
                            // サブタスク分割開始
                            sub_task_type <= SUB_TASK_CONFIGURE_EP_DISABLE;
                            sub_task_state <= 5'd0;
                        end
                        SUB_TASK_CONFIGURE_EP_DISABLE: begin
                            case (sub_task_state)
                                5'd0: begin
                                    if (cfg_ep_deconfigure) begin
                                        sub_task_type  <= SUB_TASK_DECONFIGURE_EP;
                                        sub_task_state <= 5'd0;
                                    end else begin
                                        sub_task_state <= STATE_CE_START_READ_INPUT_CTRL;
                                    end
                                end
                                STATE_CE_START_READ_INPUT_CTRL: begin
                                    mrd_addr        <= trb_p_input_ctrl;
                                    mrd_length      <= 32'h8;
                                    mrd_has_request <= 1'b1;
                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en <= 1'b1;
                                        sub_task_state <= STATE_CE_READ_INPUT_CTRL_DELAY;
                                    end
                                end
                                STATE_CE_READ_INPUT_CTRL_DELAY: begin
                                    sub_task_state <= STATE_CE_READ_INPUT_CTRL;
                                end
                                STATE_CE_READ_INPUT_CTRL: begin
                                    buffer_input_control <= read_out.dout[63:0];
                                    mrd_has_request <= 1'b0;
                                    mrd_fifo_en     <= 1'b0;
                                    cfg_ep_walk_bit <= 5'd2;
                                    sub_task_state <= STATE_CE_CHECK_DISABLE_EP;
                                end
                                STATE_CE_CHECK_DISABLE_EP: begin
                                    if (buffer_input_control[cfg_ep_walk_bit]) begin
                                        sub_task_state <= STATE_CE_DISABLE_EP;
                                    end else begin
                                        sub_task_state <= STATE_CE_CHECK_NEXT_EP;
                                    end
                                end
                                STATE_CE_DISABLE_EP: begin
                                    ses_has_request <= 1'b1;
                                    ses_slot_id     <= trb_slot_id[2:0];
                                    ses_ep_id       <= cfg_ep_walk_bit;
                                    ses_state       <= 3'd0;
                                    set_ep_tr_ptr_out[0]    <= 1'b1;
                                    set_ep_tr_ptr_out[3:1]  <= trb_slot_id[2:0];
                                    set_ep_tr_ptr_out[8:4]  <= cfg_ep_walk_bit;
                                    set_ep_tr_ptr_out[72:9] <= 64'h0;
                                    set_ep_tr_ptr_out[73]   <= 1'b0;
                                    if (if_set_ep_state.done) begin
                                        sub_task_state <= STATE_CE_DISABLE_EP_CLEANUP;
                                    end
                                end
                                STATE_CE_DISABLE_EP_CLEANUP: begin
                                    ses_has_request <= 1'b0;
                                    ses_slot_id     <= 3'd0;
                                    ses_ep_id       <= 5'd0;
                                    ses_state       <= 3'd0;
                                    sub_task_state <= STATE_CE_CHECK_NEXT_EP;
                                end
                                STATE_CE_CHECK_NEXT_EP: begin
                                    if (cfg_ep_walk_bit == 5'd31) begin
                                        // 無効化サブタスク終了→有効化サブタスクへ
                                        sub_task_type <= SUB_TASK_CONFIGURE_EP_ENABLE;
                                        sub_task_state <= 5'd0;
                                    end else begin
                                        cfg_ep_walk_bit <= cfg_ep_walk_bit + 5'd1;
                                        sub_task_state <= STATE_CE_CHECK_DISABLE_EP;
                                    end
                                end
                            endcase
                        end
                        SUB_TASK_CONFIGURE_EP_ENABLE: begin
                            case (sub_task_state)
                                5'd0: begin
                                    cfg_ep_walk_bit <= 5'd2;
                                    sub_task_state <= STATE_CE_CHECK_ENABLE_EP;
                                end
                                STATE_CE_CHECK_ENABLE_EP: begin
                                    if (buffer_input_control[6'd32 + cfg_ep_walk_bit]) begin
                                        sub_task_state <= STATE_CE_ENABLE_EP_WRITE_INIT;
                                    end else begin
                                        sub_task_state <= STATE_CE_CHECK_NEXT_EP;
                                    end
                                end
                                STATE_CE_ENABLE_EP_WRITE_INIT: begin
                                    mwr_addr <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx + 64'h20 * cfg_ep_walk_bit;
                                    mwr_length <= 64'h20;
                                    mwr_has_data <= 1'b1;
                                    if (write_out.state == WR_DATA_INIT) begin
                                        sub_task_state <= STATE_CE_ENABLE_EP_READ_START;
                                    end
                                end
                                STATE_CE_ENABLE_EP_READ_START: begin
                                    mrd_addr <= trb_p_input_ctrl + 64'h20 + 64'h20 * cfg_ep_walk_bit;
                                    mrd_length <= 64'h20;
                                    mrd_has_request <= 1'b1;
                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en <= 1'b1;
                                        sub_task_state <= STATE_CE_ENABLE_EP_READ_DELAY;
                                    end
                                end
                                STATE_CE_ENABLE_EP_READ_DELAY: begin
                                    sub_task_state <= STATE_CE_ENABLE_EP_READ_LO;
                                end
                                STATE_CE_ENABLE_EP_READ_LO: begin
                                    mwr_fifo_in[127:3] <= read_out.dout[127:3];
                                    mwr_fifo_in[2:0]   <= 3'd1;
                                    mwr_fifo_en        <= 1'b1;
                                    set_ep_tr_ptr_out[0]    <= 1'b1;
                                    set_ep_tr_ptr_out[3:1]  <= trb_slot_id[2:0];
                                    set_ep_tr_ptr_out[8:4]  <= cfg_ep_walk_bit;
                                    set_ep_tr_ptr_out[72:9] <= { read_out.dout[127:68], 4'h0 };
                                    set_ep_tr_ptr_out[73]   <= read_out.dout[64];
                                    sub_task_state <= STATE_CE_ENABLE_EP_READ_HI;
                                end
                                STATE_CE_ENABLE_EP_READ_HI: begin
                                    mwr_fifo_in <= read_out.dout;
                                    mrd_fifo_en <= 1'b0;
                                    mrd_has_request <= 1'b0;
                                    sub_task_state <= STATE_CE_ENABLE_EP_WRITE_WAIT;
                                end
                                STATE_CE_ENABLE_EP_WRITE_WAIT: begin
                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_has_data  <= 1'b0;
                                        mwr_fifo_done <= 1'b0;
                                        sub_task_state <= STATE_CE_CHECK_NEXT_EP;
                                    end else begin
                                        mwr_fifo_en   <= 1'b0;
                                        mwr_fifo_in   <= 128'h0;
                                        mwr_fifo_done <= 1'b1;
                                    end
                                end
                                STATE_CE_CHECK_NEXT_EP: begin
                                    if (cfg_ep_walk_bit == 5'd31) begin
                                        // 有効化サブタスク終了→クローンサブタスクへ
                                        sub_task_type <= SUB_TASK_CONFIGURE_EP_CLONE;
                                        sub_task_state <= 5'd0;
                                    end else begin
                                        cfg_ep_walk_bit <= cfg_ep_walk_bit + 5'd1;
                                        sub_task_state <= STATE_CE_CHECK_ENABLE_EP;
                                    end
                                end
                            endcase
                        end
                        SUB_TASK_CONFIGURE_EP_CLONE: begin
                            case (sub_task_state)
                                5'd0: begin
                                    sub_task_state <= STATE_CE_SLOT_READ_1_START;
                                end
                                STATE_CE_SLOT_READ_1_START: begin
                                    mrd_addr        <= trb_p_input_ctrl + 64'h20;
                                    mrd_length      <= 64'h10;
                                    mrd_has_request <= 1'b1;
                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en    <= 1'b1;
                                        sub_task_state <= STATE_CE_SLOT_READ_1_DELAY;
                                    end
                                end
                                STATE_CE_SLOT_READ_1_DELAY: begin
                                    sub_task_state <= STATE_CE_SLOT_READ_1;
                                end
                                STATE_CE_SLOT_READ_1: begin
                                    cfg_ep_input_context_entries <= read_out.dout[31:27];
                                    mrd_fifo_en <= 1'b0;
                                    mrd_has_request <= 1'b0;
                                    sub_task_state <= STATE_CE_SLOT_READ_2_START;
                                end
                                STATE_CE_SLOT_READ_2_START: begin
                                    mrd_addr <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx;
                                    mrd_length <= 64'h10;
                                    mrd_has_request <= 1'b1;
                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en <= 1'b1;
                                        sub_task_state <= STATE_CE_SLOT_READ_2_DELAY;
                                    end
                                end
                                STATE_CE_SLOT_READ_2_DELAY: begin
                                    sub_task_state <= STATE_CE_SLOT_READ_2;
                                end
                                STATE_CE_SLOT_READ_2: begin
                                    mwr_fifo_in[26:0]  <= read_out.dout[26:0];
                                    mwr_fifo_in[31:27] <= cfg_ep_input_context_entries;
                                    mwr_fifo_in[127:123] <= 5'd3;
                                    mrd_has_request <= 1'b0;
                                    mrd_fifo_en     <= 1'b0;
                                    sub_task_state <= STATE_CE_SLOT_WRITE_INIT;
                                end
                                STATE_CE_SLOT_WRITE_INIT: begin
                                    mwr_addr <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx;
                                    mwr_length <= 64'h10;
                                    mwr_has_data <= 1'b1;
                                    if (write_out.state == WR_DATA_INIT) begin
                                        mwr_fifo_en    <= 1'b1;
                                        sub_task_state <= STATE_CE_SLOT_WRITE_WAIT;
                                    end
                                end
                                STATE_CE_SLOT_WRITE_WAIT: begin
                                    mwr_fifo_en   <= 1'b0;
                                    mwr_fifo_in   <= 128'h0;
                                    mwr_fifo_done <= 1'b1;
                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_has_data  <= 1'b0;
                                        mwr_fifo_done <= 1'b0;
                                        // クローンサブタスク終了→完了サブタスクへ
                                        sub_task_type <= SUB_TASK_CONFIGURE_EP_COMPLETE;
                                        sub_task_state <= 5'd0;
                                    end
                                end
                            endcase
                        end
                        SUB_TASK_CONFIGURE_EP_COMPLETE: begin
                            case (sub_task_state)
                                5'd0: begin
                                    command_completion_code    <= 8'h1;
                                    command_completion_slot_id <= trb_slot_id;
                                    sub_task_state <= 5'd0;
                                    sub_task_type  <= SUB_TASK_SEND_CPLT;
                                end
                            endcase
                        end
                        SUB_TASK_DISABLE_SLOT: begin
                            case (sub_task_state)
                                5'd0: begin
                                    set_slot_state[0] <= 1'b1;
                                    set_slot_state[3:1] <= trb_slot_id[2:0];
                                    set_slot_state[8:4] <= DISABLE;

                                    sub_task_state <= 5'd1;
                                end
                                5'd1: begin
                                    set_slot_state[8:0] <= 9'd0;

                                    command_completion_slot_id <= trb_slot_id;
                                    command_completion_code <= 8'd1;

                                    sub_task_type <= SUB_TASK_SEND_CPLT;
                                    sub_task_state <= 5'd0;
                                end
                            endcase
                        end
                        SUB_TASK_EVALUATE_CONTEXT: begin
                            case(evaluate_context_state_t'(sub_task_state))
                                STATE_EC_READ_INPUT_CTRL_START: begin
                                    mrd_addr        <= trb_p_input_ctrl;
                                    mrd_length      <= 32'h8;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en    <= 1'b1;
                                        sub_task_state <= STATE_EC_READ_INPUT_CTRL_DELAY;
                                    end
                                end
                                STATE_EC_READ_INPUT_CTRL_DELAY: begin
                                    sub_task_state <= STATE_EC_READ_INPUT_CTRL;
                                end
                                STATE_EC_READ_INPUT_CTRL: begin
                                    buffer_input_control <= read_out.dout[63:0];
                                    mrd_has_request <= 1'b0;
                                    mrd_fifo_en     <= 1'b0;

                                    sub_task_state  <= STATE_EC_CHECK_INPUT_CTRL;
                                end
                                STATE_EC_CHECK_INPUT_CTRL: begin
                                    if ((buffer_input_control[31:0] != 32'h0) || (buffer_input_control[63:32] & ~32'h3)) begin
                                        command_completion_code    <= 8'h5;
                                        command_completion_slot_id <= trb_slot_id;

                                        sub_task_state <= 5'd0;
                                        sub_task_type  <= SUB_TASK_SEND_CPLT;
                                    end else if (buffer_input_control[32]) begin
                                        //Slot Context
                                        sub_task_type  <= SUB_TASK_EVALUATE_CONTEXT_SLOT;
                                        sub_task_state <= STATE_EC_READ_OSLOT_CTX_START;
                                    end else if (buffer_input_control[33]) begin
                                        //Endpoint Context
                                        sub_task_type  <= SUB_TASK_EVALUATE_CONTEXT_EP;
                                        sub_task_state <= STATE_EC_READ_IEP_CTX_START;
                                    end else begin
                                        sub_task_state <= STATE_EC_COMPLETE;
                                    end
                                end
                                STATE_EC_COMPLETE: begin
                                    command_completion_code    <= 8'h1;
                                    command_completion_slot_id <= trb_slot_id;

                                    sub_task_state <= 5'd0;
                                    sub_task_type  <= SUB_TASK_SEND_CPLT;
                                end
                            endcase
                        end
                        SUB_TASK_EVALUATE_CONTEXT_SLOT: begin
                            case (evaluate_context_state_t'(sub_task_state))
                                STATE_EC_READ_OSLOT_CTX_START: begin
                                    mrd_addr <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx;
                                    mrd_length <= 32'h10;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en <= 1'b1;
                                        sub_task_state <= STATE_EC_READ_OSLOT_CTX_DELAY;
                                    end
                                end
                                STATE_EC_READ_OSLOT_CTX_DELAY: begin
                                    sub_task_state <= STATE_EC_READ_OSLOT_CTX;
                                end
                                STATE_EC_READ_OSLOT_CTX: begin
                                    mwr_fifo_in[31:0]   <= read_out.dout[31:0];
                                    mwr_fifo_in[47:32]  <= 16'h0; // max exit latency
                                    mwr_fifo_in[127:48] <= read_out.dout[127:48];

                                    mrd_has_request <= 1'b0;
                                    mrd_fifo_en     <= 1'b0;

                                    sub_task_state <= STATE_EC_READ_ISLOT_CTX_START;
                                end
                                STATE_EC_READ_ISLOT_CTX_START: begin
                                    mrd_addr <= trb_p_input_ctrl + 64'h20;
                                    mrd_length <= 32'h10;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en <= 1'b1;
                                        sub_task_state <= STATE_EC_READ_ISLOT_CTX_DELAY;
                                    end
                                end
                                STATE_EC_READ_ISLOT_CTX_DELAY: begin
                                    sub_task_state <= STATE_EC_READ_ISLOT_CTX;
                                end
                                STATE_EC_READ_ISLOT_CTX: begin
                                    mwr_fifo_in[41:32] <= read_out.dout[95:86];

                                    mrd_has_request <= 1'b0;
                                    mrd_fifo_en     <= 1'b0;
                                    
                                    sub_task_state <= STATE_EC_WRITE_OSLOT_CTX_INIT;
                                end
                                STATE_EC_WRITE_OSLOT_CTX_INIT: begin
                                    mwr_addr     <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx;
                                    mwr_length   <= 32'h10;
                                    mwr_has_data <= 1'b1;

                                    if (write_out.state == WR_DATA_INIT) begin
                                        mwr_fifo_en    <= 1'b1;
                                        sub_task_state <= STATE_EC_WRITE_OSLOT_CTX_WAIT;
                                    end
                                end
                                STATE_EC_WRITE_OSLOT_CTX_WAIT: begin
                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_has_data  <= 1'b0;
                                        mwr_fifo_done <= 1'b0;

                                        if (buffer_input_control[33]) begin
                                            //Endpoint Context
                                            sub_task_type  <= SUB_TASK_EVALUATE_CONTEXT_EP;
                                            sub_task_state <= STATE_EC_READ_IEP_CTX_START;
                                        end else begin
                                            sub_task_type  <= SUB_TASK_EVALUATE_CONTEXT;
                                            sub_task_state <= STATE_EC_COMPLETE;
                                        end
                                    end else begin
                                        mwr_fifo_en   <= 1'b0;
                                        mwr_fifo_in   <= 128'h0;
                                        mwr_fifo_done <= 1'b1;
                                    end
                                end
                            endcase
                        end
                        SUB_TASK_EVALUATE_CONTEXT_EP: begin
                            case (evaluate_context_state_t'(sub_task_state))
                                STATE_EC_READ_IEP_CTX_START: begin
                                    mrd_addr        <= trb_p_input_ctrl + 64'h40;
                                    mrd_length      <= 32'h10;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en    <= 1'b1;
                                        sub_task_state <= STATE_EC_READ_IEP_CTX_DELAY;
                                    end
                                end
                                STATE_EC_READ_IEP_CTX_DELAY: begin
                                    sub_task_state <= STATE_EC_READ_IEP_CTX;
                                end
                                STATE_EC_READ_IEP_CTX: begin
                                    mwr_fifo_in[63:48] <= read_out.dout[63:48];

                                    mrd_has_request <= 1'b0;
                                    mrd_fifo_en     <= 1'b0;

                                    sub_task_state <= STATE_EC_WRITE_OEP_CTX_INIT;
                                end
                                STATE_EC_WRITE_OEP_CTX_INIT: begin
                                    mwr_addr     <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx + 64'h20;
                                    mwr_length   <= 32'h20;
                                    mwr_has_data <= 1'b1;

                                    if (write_out.state == WR_DATA_INIT) begin
                                        sub_task_state <= STATE_EC_READ_OEP_CTX_START;
                                    end
                                end
                                STATE_EC_READ_OEP_CTX_START: begin
                                    mrd_addr        <= slot_ctx_in.slot_context[trb_slot_id - 1].p_ctx + 64'h20;
                                    mrd_length      <= 32'h20;
                                    mrd_has_request <= 1'b1;

                                    if (read_out.state == RD_COMPLETE) begin
                                        mrd_fifo_en    <= 1'b1;
                                        sub_task_state <= STATE_EC_READ_OEP_CTX_DELAY;
                                    end
                                end
                                STATE_EC_READ_OEP_CTX_DELAY: begin
                                    sub_task_state <= STATE_EC_READ_OEP_CTX_LO;
                                end
                                STATE_EC_READ_OEP_CTX_LO: begin
                                    mwr_fifo_in[47:0]   <= read_out.dout[47:0];
                                    mwr_fifo_in[127:64] <= read_out.dout[127:64];

                                    mwr_fifo_en <= 1'b1;

                                    sub_task_state <= STATE_EC_READ_OEP_CTX_HI;
                                end
                                STATE_EC_READ_OEP_CTX_HI: begin
                                    mwr_fifo_in <= read_out.dout;
                                    mrd_fifo_en <= 1'b0;
                                    mrd_has_request <= 1'b0;
                                    sub_task_state <= STATE_EC_WRITE_OEP_CTX_WAIT;
                                end
                                STATE_EC_WRITE_OEP_CTX_WAIT: begin
                                    mwr_fifo_en   <= 1'b0;
                                    mwr_fifo_in   <= 128'h0;
                                    mwr_fifo_done <= 1'b1;

                                    if (write_out.state == WR_COMPLETE) begin
                                        mwr_has_data  <= 1'b0;
                                        mwr_fifo_done <= 1'b0;

                                        sub_task_type  <= SUB_TASK_EVALUATE_CONTEXT;
                                        sub_task_state <= STATE_EC_COMPLETE;
                                    end
                                end
                            endcase
                        end
                    endcase
                end
            endcase
        end
    end
endmodule