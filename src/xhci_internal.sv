`timescale 1ns / 1ps
`include "pcileech_header.svh"

module slot_context(
    input                 rst,
    input                 clk,
    IfSlotContext.source  slot_ctx_out,
    
    //[0]: run
    //[1]: 1=enable/0=disalbe
    //[4:2]: target slot index
    input [4:0]           control_enabled_slot,
    //[0]:    run
    //[3:1]:  slot id
    //[67:4]: new slot context pointer
    input [67:0]          ctrl_update_slot_ctx_ptr,
    //[0]: run
    //[3:1]: slot id
    //[8:4]: endpoint id
    //[72:9]: new endpoint transfer ring dequeue pointer
    //[73]: cycle state
    input [73:0]          set_ep_tr_ptr_in,
    //[0]  : run
    //[3:1]: slot index
    //[8:4]: slot state
    input [8:0]           set_slot_state
);
    //Slot Context
    slot_context_t     slot_ctx[8];
    (* ram_style = "block" *)
    endpoint_context_t ep_ctx[248];

    assign slot_ctx_out.slot_context = slot_ctx;
    assign slot_ctx_out.ep_ctx = ep_ctx;

    //---------------------------------
    // register handler
    //---------------------------------
    wire [2:0] control_enabled_slot_index = control_enabled_slot[4:2];
    wire [2:0] ctrl_update_slot_ctx_ptr_index = ctrl_update_slot_ctx_ptr[3:1];

    //set ep tr ptr
    wire [7:0] set_ep_tr_index = (set_ep_tr_ptr_in[3:1] - 3'd1) * 8'd8 + set_ep_tr_ptr_in[8:4] - 8'd1;

    always @ ( posedge clk ) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                slot_ctx[i].p_ctx        <= 64'h0;
                slot_ctx[i].slot_id      <= 8'h0;
                slot_ctx[i].slot_enabled <= 1'b0;
                slot_ctx[i].slot_state   <= DISABLE;//Undefined on Spec
            end
            for (int i = 0; i < 248; i++) begin
                ep_ctx[i].p_tr_dequeue <= 64'h0;
                ep_ctx[i].cycle_state  <= 1'b0;
            end
        end else begin
            if (control_enabled_slot[0]) begin
                slot_ctx[control_enabled_slot_index].slot_enabled <= control_enabled_slot[1];
                slot_ctx[control_enabled_slot_index].slot_state <= control_enabled_slot[1] ? ENABLE : DISABLE;
            end
            if (ctrl_update_slot_ctx_ptr[0]) begin
                slot_ctx[ctrl_update_slot_ctx_ptr_index].p_ctx <= ctrl_update_slot_ctx_ptr[67:4];
            end
            if (set_ep_tr_ptr_in[0]) begin
                ep_ctx[set_ep_tr_index].p_tr_dequeue <= set_ep_tr_ptr_in[72:9];
                ep_ctx[set_ep_tr_index].cycle_state  <= set_ep_tr_ptr_in[73];
            end
            if (set_slot_state[0]) begin
                slot_ctx[set_slot_state[3:1]].slot_state <= slot_state_t'(set_slot_state[8:4]);
            end
        end
    end
endmodule

module set_ep_tr_mux(
    input                 rst,
    input                 clk,
    input  [73:0]         in_1,
    input  [73:0]         in_2,
    output [73:0]         out
);

    assign out = in_1[0] ? in_1 :
                 in_2[0] ? in_2 : 74'h0;

    always @ ( posedge clk ) begin
    end
endmodule

module set_ep_state(
    input                 rst,
    input                 clk,
    IfSetEPState.sink     set_ep_state_in,
    IfSlotContext.sink    if_slot_ctx,
    IfMemoryRead.source   read_out,
    IfMemoryWrite.source  write_out
);

    enum reg [3:0] {
        IDLE,
        WRITE_START,
        READ_START,
        READ_DELAY,
        READ_DATA_LO,
        READ_DATA_HI,
        WRITE_WAIT,
        COMPLETE
    } state;

    assign set_ep_state_in.done = state == COMPLETE;

    wire [2:0] slot_index = set_ep_state_in.slot_id - 1;

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

    always @ ( posedge clk ) begin
        if (rst) begin
            state <= IDLE;

            //Memory Read
            mrd_addr        <= 64'h0;
            mrd_length      <= 32'h0;
            mrd_has_request <= 1'b0;
            mrd_fifo_en     <= 1'b0;

            //Memory Write
            mwr_addr      <= 64'h0;
            mwr_length    <= 32'h0;
            mwr_has_data  <= 1'b0;
            mwr_fifo_in   <= 128'h0;
            mwr_fifo_en   <= 1'b0;
            mwr_fifo_done <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    if (set_ep_state_in.has_request) begin
                        state <= WRITE_START;
                    end
                end
                WRITE_START: begin
                    mwr_addr     <= if_slot_ctx.slot_context[slot_index].p_ctx + 64'h20 * set_ep_state_in.ep_id;
                    mwr_length   <= 32'h20;
                    mwr_has_data <= 1'b1;

                    if (write_out.state == WR_DATA_INIT) begin
                        state <= READ_START;
                    end
                end
                READ_START: begin
                    mrd_addr        <= if_slot_ctx.slot_context[slot_index].p_ctx + 64'h20 * set_ep_state_in.ep_id;
                    mrd_length      <= 32'h20;
                    mrd_has_request <= 1'b1;
                    
                    if (read_out.state == RD_COMPLETE) begin
                        mrd_fifo_en <= 1'b1;    
                        state <= READ_DELAY;        
                    end
                end
                READ_DELAY: begin
                    state <= READ_DATA_LO;
                end
                READ_DATA_LO: begin
                    mwr_fifo_in[127:3] <= read_out.dout[127:3];
                    mwr_fifo_in[2:0]   <= set_ep_state_in.state;

                    mwr_fifo_en <= 1'b1;

                    state <= READ_DATA_HI;
                end
                READ_DATA_HI: begin
                    mwr_fifo_in <= read_out.dout;
                    mrd_fifo_en <= 1'b0;
                    mrd_has_request <= 1'b0;

                    state <= WRITE_WAIT;
                end
                WRITE_WAIT: begin
                    if (write_out.state == WR_COMPLETE) begin
                        mwr_addr <= 64'h0;
                        mwr_length <= 32'h0;
                        mwr_has_data <= 1'b0;
                        mwr_fifo_done <= 1'b0;

                        state <= COMPLETE;
                    end else begin
                        mwr_fifo_en   <= 1'b0;
                        mwr_fifo_in   <= 128'h0;
                        mwr_fifo_done <= 1'b1;
                    end
                end
                COMPLETE: begin
                    if (!set_ep_state_in.has_request) begin
                        state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule