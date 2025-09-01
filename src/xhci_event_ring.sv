`timescale 1ns / 1ps
`include "pcileech_header.svh"

module event_ring(
    input                   rst,
    input                   clk_pcie,
    
    xHCI_RtReg.sink         rt_reg_in,

    IfEventRingRequest.sink event_ring_req_in,
    IfMemoryWrite.source    write_out,
    IfMemoryRead.source     read_out,
    IfMsiXRequest.source    msix_req_out,
    IfInterrupterController.sink interrupter_controller_in
);
    //===========================================
    // STATE
    //===========================================
    localparam IDLE                          = 4'd1;
    localparam SEND_TRB                      = 4'd2;
    localparam WAIT_TRB                      = 4'd3;
    localparam SEND_MSIX                     = 4'd4;
    localparam WAIT_MSIX                     = 4'd5;
    localparam WALK_SEGMENT_TABLE_READ_SEND  = 4'd6;
    localparam WALK_SEGMENT_TABLE_READ_DELAY = 4'd7;
    localparam WALK_SEGMENT_TABLE_READ_DATA  = 4'd8;
    localparam WALK_SEGMENT_TABLE_CHECK      = 4'd9;
    localparam COMPLETE                      = 4'd10;

    reg [3:0] state;

    assign event_ring_req_in.state = state;
    assign event_ring_req_in.ready = state == IDLE;
    assign event_ring_req_in.complete = state == COMPLETE;

    //===========================================
    // Internal Registers
    //===========================================
    reg [63:0] enqueue_pointers [7:0];
    reg [63:0] current_enqueue_pointer;

    wire [63:0] next_enqueue_pointer = current_enqueue_pointer + 64'h10;
    //Internal Cycle Bit State
    reg [7:0]  cycle_bit_state;

    //===========================================
    // Walking Event Ring Segment Table
    //===========================================
    reg [15:0]  walking_segment_table_index;
    reg [63:0]  walking_segment_base_address;
    reg [15:0]  walking_segment_size;
    reg         walking_force_set_base_to_enqueue; //walk結果のSegment Baseをenqueue_pointerとして強制設定

    wire [63:0] walking_segment_last_trb_address = walking_segment_base_address + (walking_segment_size - 16'h1) * 64'h10;
    wire        walking_in_range_current         = walking_segment_base_address <= current_enqueue_pointer && current_enqueue_pointer <= walking_segment_last_trb_address;
    wire        walking_in_range_next            = walking_segment_base_address <= next_enqueue_pointer && next_enqueue_pointer <= walking_segment_last_trb_address;
    
    //===========================================
    // Memory Write
    //===========================================
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

    //===========================================
    // Memory Read
    //===========================================
    reg [63:0] mrd_addr;
    reg [31:0] mrd_length;
    reg        mrd_has_request;
    reg        mrd_fifo_en;

    assign read_out.address     = mrd_addr;
    assign read_out.data_length = mrd_length;
    assign read_out.has_request = mrd_has_request;
    assign read_out.rd_en       = mrd_fifo_en;

    //===========================================
    // MSI-X
    //===========================================
    reg       msix_fire;
    reg [2:0] msix_interrupter_index;

    assign msix_req_out.fire = msix_fire;
    assign msix_req_out.interrupter_index = msix_interrupter_index;

    //===========================================
    // Interrupter Controller (PREFIX: ic_)
    //===========================================
    reg ic_set_event_interrupt;

    assign interrupter_controller_in.set_event_interrupt = ic_set_event_interrupt;

    always @(posedge clk_pcie) begin
        if (rst) begin

            state <= IDLE;
            for (int i = 0; i < 8; i++) begin
                enqueue_pointers[i] <= 64'h0;
            end
            current_enqueue_pointer <= 64'h0;
            cycle_bit_state <= 8'h0;

            walking_segment_table_index       <= 16'h0;
            walking_segment_base_address      <= 64'h0;
            walking_segment_size              <= 16'h0;
            walking_force_set_base_to_enqueue <= 1'b0;

            mwr_addr      <= 64'h0;
            mwr_length    <= 32'h0;
            mwr_has_data  <= 1'b0;
            mwr_fifo_en   <= 1'b0;
            mwr_fifo_in   <= 128'h0;
            mwr_fifo_done <= 1'b0;

            mrd_addr        <= 64'h0;
            mrd_length      <= 32'h0;
            mrd_has_request <= 1'b0;
            mrd_fifo_en     <= 1'b0;

            ic_set_event_interrupt <= 1'b0;
        end else begin
            //IMANのInterrupter EnableがSetされた時、内部Enqueue Pointerを初期化
            if (interrupter_controller_in.interrupter_enabled) begin
                enqueue_pointers[interrupter_controller_in.interrupter_index] <= 64'h0;
            end

            case (state)
                IDLE: begin
                    if (event_ring_req_in.send) begin
                        //Enqueue Pointerが存在しない場合Interrupter RegisterからDequeue Pointerを取得
                        //Segment Table Sizeも取得
                        if (enqueue_pointers[event_ring_req_in.interrupter_index] == 64'h0) begin
                            enqueue_pointers[event_ring_req_in.interrupter_index] <= { rt_reg_in.interrupters[event_ring_req_in.interrupter_index].event_ring_dequeue_pointer, 4'h0 };
                            current_enqueue_pointer <= { rt_reg_in.interrupters[event_ring_req_in.interrupter_index].event_ring_dequeue_pointer, 4'h0 };
                        end else begin
                            current_enqueue_pointer <= enqueue_pointers[event_ring_req_in.interrupter_index];
                        end
                        
                        ic_set_event_interrupt <= 1'b1;
                        state <= SEND_TRB;
                    end
                end
                SEND_TRB: begin
                    mwr_addr     <= current_enqueue_pointer;
                    mwr_length   <= 32'h10;
                    mwr_has_data <= 1'b1;

                    if (write_out.state == WR_DATA_INIT) begin
                        mwr_fifo_en <= 1'b1;
                        mwr_fifo_in[95:0]    <= event_ring_req_in.trb_data[95:0];
                        mwr_fifo_in[96]      <= ~cycle_bit_state[event_ring_req_in.interrupter_index];
                        mwr_fifo_in[127:97]  <= event_ring_req_in.trb_data[127:97];
                        state <= WAIT_TRB;
                    end
                end
                WAIT_TRB: begin                    
                    if (write_out.state == WR_COMPLETE) begin
                        mwr_addr      <= 64'h0;
                        mwr_length    <= 32'h0;
                        mwr_has_data  <= 1'b0;
                        mwr_fifo_done <= 1'b0;

                        ic_set_event_interrupt <= 1'b0;

                        state <= SEND_MSIX;
                    end else begin
                        mwr_fifo_en <= 1'b0;
                        mwr_fifo_in <= 128'h0;
                        mwr_fifo_done <= 1'b1;
                    end
                end
                SEND_MSIX: begin
                    if (msix_req_out.ready) begin
                        msix_fire <= 1'b1;
                        msix_interrupter_index <= event_ring_req_in.interrupter_index;
                        state <= WAIT_MSIX;
                    end
                end
                WAIT_MSIX: begin
                    if (msix_req_out.complete) begin
                        msix_fire <= 1'b0;
                        msix_interrupter_index <= 3'h0;

                        walking_segment_table_index <= 16'h0;

                        state <= WALK_SEGMENT_TABLE_READ_SEND;
                    end
                end
                WALK_SEGMENT_TABLE_READ_SEND: begin
                    mrd_addr        <= { rt_reg_in.interrupters[event_ring_req_in.interrupter_index].event_ring_segment_table_base_address, 6'h0 } + walking_segment_table_index * 64'h10;
                    mrd_length      <= 32'h10;
                    mrd_has_request <= 1'b1;

                    if (read_out.state == RD_COMPLETE) begin
                        mrd_fifo_en <= 1'b1;
                        state <= WALK_SEGMENT_TABLE_READ_DELAY;                        
                    end
                end
                WALK_SEGMENT_TABLE_READ_DELAY: begin
                    state <= WALK_SEGMENT_TABLE_READ_DATA;
                end
                WALK_SEGMENT_TABLE_READ_DATA: begin
                    //Event Ring Segment Table Entry 読み取り
                    walking_segment_base_address <= { read_out.dout[63:6], 6'h0 };
                    walking_segment_size <= read_out.dout[79:64];
                    //cleanup
                    mrd_addr        <= 64'h0;
                    mrd_length      <= 32'h0;
                    mrd_has_request <= 1'b0;
                    mrd_fifo_en     <= 1'b0;
                        
                    state <= WALK_SEGMENT_TABLE_CHECK;
                end
                WALK_SEGMENT_TABLE_CHECK: begin
                    if (walking_force_set_base_to_enqueue) begin
                        enqueue_pointers[event_ring_req_in.interrupter_index] <= walking_segment_base_address;
                        state <= COMPLETE;
                    end else begin
                        if (walking_in_range_current) begin
                            //current_enqueue_pointerが取得したRing内
                            if (walking_in_range_next) begin
                                //next_enqueue_pointerが同じRing内 -> 次のenqueue pointerを設定
                                enqueue_pointers[event_ring_req_in.interrupter_index] <= next_enqueue_pointer;
                                state <= COMPLETE;
                            end else begin
                                //next_enqueue_pointerが違うRing
                                if (rt_reg_in.interrupters[event_ring_req_in.interrupter_index].event_ring_segment_table_size == walking_segment_table_index + 16'h1) begin
                                    //現在取得したSegmentが最終セグメント -> index: 0のRingに変更 + Cycle bit state 反転
                                    walking_force_set_base_to_enqueue <= 1'b1; //walk結果をenqueue_pointerとして強制設定
                                    walking_segment_table_index <= 16'h0;      //次のwalkのindexを0に設定
                                    cycle_bit_state[event_ring_req_in.interrupter_index] = ~cycle_bit_state[event_ring_req_in.interrupter_index]; //Cycle Bit state 反転
                                    state <= WALK_SEGMENT_TABLE_READ_SEND;     //再度 walk
                                end else begin
                                    walking_force_set_base_to_enqueue <= 1'b1; //walk結果をenqueue_pointerとして強制設定
                                    walking_segment_table_index <= walking_segment_table_index + 16'h1; //次のwalkのindex + 1
                                    state <= WALK_SEGMENT_TABLE_READ_SEND;     //再度 walk
                                end
                            end
                        end else begin
                            //current_enqueue_pointerが取得したRing外なので、次のringを取得
                            walking_segment_table_index <= walking_segment_table_index + 16'h1;
                            state <= WALK_SEGMENT_TABLE_READ_SEND; //再度 walk
                        end
                    end
                end
                COMPLETE: begin
                    if (!event_ring_req_in.send) begin
                        walking_force_set_base_to_enqueue <= 1'b0;
                        state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule

module event_ring_mux(
    input                   rst,
    input                   clk_pcie,

    IfEventRingRequest.sink in_1,
    IfEventRingRequest.sink in_2,
    IfEventRingRequest.sink in_3,
    
    IfEventRingRequest.source out
);
    reg [3:0] id;

    assign out.send = (id==1) ? in_1.send :
                      (id==2) ? in_2.send :
                      (id==3) ? in_3.send : 1'b0;

    assign out.interrupter_index = (id==1) ? in_1.interrupter_index :
                                   (id==2) ? in_2.interrupter_index :
                                   (id==2) ? in_3.interrupter_index : 3'h0;

    assign out.trb_data = (id==1) ? in_1.trb_data :
                          (id==2) ? in_2.trb_data :
                          (id==3) ? in_3.trb_data : 128'h0;
    
    wire [3:0] id_next_newsel = in_1.has_request ? 1 :
                                in_2.has_request ? 2 :
                                in_3.has_request ? 3 : 0;
    
    wire [3:0] id_next = out.ready ? id_next_newsel : id;

    assign in_1.state = (id==1) ? out.state : 4'd1;
    assign in_2.state = (id==2) ? out.state : 4'd1;
    assign in_3.state = (id==3) ? out.state : 4'd1;

    assign in_1.ready = (id==1) && out.ready;
    assign in_2.ready = (id==2) && out.ready;
    assign in_3.ready = (id==3) && out.ready;


    assign in_1.complete = (id==1) && out.complete;
    assign in_2.complete = (id==2) && out.complete;
    assign in_3.complete = (id==3) && out.complete;

    always @ ( posedge clk_pcie ) begin
        id <= rst ? 0 : id_next;
    end
endmodule