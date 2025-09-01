`timescale 1ns / 1ps
`include "pcileech_header.svh"

module host_mem_write_2 (
    input               rst,
    input               clk_pcie,
    input [15:0]        pcie_id,
    IfMemoryWrite.sink mem_wr_in,
    IfAXIS128.source    tlps_out
);
    //CURRENT FIFO BUFFER: 128 * 128 (4 DWORD * 128 = 2048Bytes)
    localparam MAX_PAYLOAD_SIZE = 32'd128; //32DWORDs 128Bytes

    //===========================================
    // MEMORY MODULE REGISTER
    //===========================================
    memory_write_state_t state;
    reg [3:0]  rst_timer;
    reg [31:0] data_remaining;     //全体の残りデータ
    reg [31:0] transfer_remaining; //今回の転送の残りデータ
    reg [2:0]  dwords_packet;      //今回の転送のDWORD数
    reg [63:0] write_address;

    wire [9:0] dw_length = transfer_remaining == 32'd4096 ? 10'd0 : (transfer_remaining[11:2] + (transfer_remaining[1:0] != 2'b00));
    wire [1:0] last_dw_byte = transfer_remaining[1:0];
    wire [3:0] last_dw_be = data_remaining != 32'd0     ? 4'hF    : //今回のTLPは4DWORDで埋まっている
                            transfer_remaining <= 32'd4  ? 4'h0   : //DWORD以下(3バイト以下) = 1st DWのみ有効
                            last_dw_byte == 2'd1        ? 4'b0001 : //DWORD+1バイト
                            last_dw_byte == 2'd2        ? 4'b0011 : //DWORD+2バイト
                            last_dw_byte == 2'd3        ? 4'b0111 : 4'b1111; //DWORD+3バイト
    wire [3:0] first_dw_be = transfer_remaining >= 32'd4 ? 4'hF    : //DWORD以上(4バイト以上) = 1st DWは全有効
                             last_dw_byte == 2'd1        ? 4'b0001 : //DWORD+1バイト
                             last_dw_byte == 2'd2        ? 4'b0011 : //DWORD+2バイト
                             last_dw_byte == 2'd3        ? 4'b0111 : 4'b1111; //DWORD+3バイト

    assign mem_wr_in.state = state;

    //===========================================
    // TLP OUT
    //===========================================

    reg [127:0] tlp_data;
    reg [3:0]   tkeepdw;
    reg         tfirst;
    reg         tlast;
    reg         tvalid;
    reg         has_data;

    assign tlps_out.tdata    = tlp_data;
    assign tlps_out.tkeepdw  = tkeepdw;
    assign tlps_out.tuser[0] = tfirst; //FIRST
    assign tlps_out.tuser[1] = tlast;  //LAST
    assign tlps_out.tlast    = tlast;  //LAST
    assign tlps_out.tvalid   = tvalid;
    assign tlps_out.has_data = has_data;

    reg fifo_read;
    reg fifo_rst;

    wire [127:0] fifo_data_out;
    wire         fifo_srst = fifo_rst | rst;

    fifo_128_128_mem_write i_fifo_128_128_mem_write(
        .srst       ( fifo_srst         ),
        .clk        ( clk_pcie          ),
        .full       (                   ),
        .din        ( mem_wr_in.din     ),
        .wr_en      ( mem_wr_in.wr_en   ),
        .empty      (                   ),
        .dout       ( fifo_data_out     ),
        .rd_en      ( fifo_read         )
    );

    always @(posedge clk_pcie) begin
        if (rst) begin
            state <= WR_IDLE;
            rst_timer <= 3'd0;
            
            data_remaining <= 32'd0;
            transfer_remaining <= 32'd0;
            dwords_packet <= 3'd0;
            write_address <= 64'h0;
            
            tlp_data <= 128'h0;
            tkeepdw  <= 4'h0;
            tfirst   <= 1'b0;
            tlast    <= 1'b0;
            tvalid   <= 1'b0;
            has_data <= 1'b0;

            fifo_read <= 1'b0;
            fifo_rst <= 1'b0;
        end else begin
            case (state)
                WR_IDLE: begin
                    if (mem_wr_in.has_data) begin
                        state <= WR_DATA_INIT;
                        data_remaining <= mem_wr_in.data_length;
                        write_address  <= mem_wr_in.address;
                    end
                end
                WR_DATA_INIT: begin
                    if (mem_wr_in.wr_done) begin
                        state <= WR_CALC_DATA;
                    end
                end
                WR_CALC_DATA: begin
                    if (data_remaining > MAX_PAYLOAD_SIZE) begin
                        transfer_remaining <= MAX_PAYLOAD_SIZE;
                        data_remaining <= data_remaining - MAX_PAYLOAD_SIZE;
                    end else begin
                        transfer_remaining <= data_remaining;
                        data_remaining <= 32'd0;
                    end
                    state <= WR_PREPARE_HEADER;
                end
                WR_PREPARE_HEADER: begin
                    tkeepdw  <= 4'hF;
                    tfirst   <= 1'b1;
                    tlast    <= 1'b0;
                    has_data <= 1'b1;
                    tlp_data <= {
                        write_address[31:0],
                        write_address[63:32],
                        { `_bs16(pcie_id), 8'h00, last_dw_be, first_dw_be },
                        22'b01100000_00000000_000000, dw_length
                    };
                    write_address <= write_address + transfer_remaining;
                    state <= WR_TRANSMIT_HEADER;
                end
                WR_TRANSMIT_HEADER: begin
                    if (tlps_out.tready) begin
                        tvalid   <= 1'b1;
                        has_data <= 1'b0;
                        state    <= WR_DEASSERT_HEADER;
                    end
                end
                WR_DEASSERT_HEADER: begin
                    tvalid <= 1'b0;
                    state <= WR_GET_DATA_FIFO;
                end
                WR_GET_DATA_FIFO: begin
                    fifo_read <= 1'b1;

                    if (transfer_remaining[31:4] != 28'd0) begin
                        dwords_packet <= 3'd4;
                        transfer_remaining <= transfer_remaining - 32'd16;
                    end else begin
                        if (transfer_remaining[3:0] < 5) begin
                            dwords_packet <= 3'd1;
                        end else if (transfer_remaining[3:0] < 9) begin
                            dwords_packet <= 3'd2;
                        end else if (transfer_remaining[3:0] < 13) begin
                            dwords_packet <= 3'd3;
                        end else begin
                            dwords_packet <= 3'd4;
                        end
                        transfer_remaining <= 32'd0;
                    end

                    state <= WR_WAIT_DATA_FIFO;
                end
                WR_WAIT_DATA_FIFO: begin
                    state <= WR_PREPARE_DATA;
                end
                WR_PREPARE_DATA: begin
                    fifo_read <= 1'b0;
                    tfirst   <= 1'b0;
                    tlast    <= transfer_remaining == 32'd0;
                    has_data <= transfer_remaining != 32'd0;

                    case (dwords_packet)
                        3'd1: tkeepdw <= 4'b0001;
                        3'd2: tkeepdw <= 4'b0011;
                        3'd3: tkeepdw <= 4'b0111;
                        3'd4: tkeepdw <= 4'b1111;
                    endcase
                    
                    tlp_data <= {
                            `_bs32(fifo_data_out[127:96]),
                            `_bs32(fifo_data_out[95:64]),
                            `_bs32(fifo_data_out[63:32]),
                            `_bs32(fifo_data_out[31:0])
                        };

                    state <= WR_TRANSMIT_DATA;
                end
                WR_TRANSMIT_DATA: begin
                    if (tlps_out.tready) begin
                        tvalid <= 1'b1;
                        has_data <= 1'b0;
                        state <= WR_DEASSERT_DATA;
                    end
                end
                WR_DEASSERT_DATA: begin
                    tvalid <= 1'b0;
                    if (transfer_remaining == 32'd0) begin
                        if (data_remaining == 32'd0) begin
                            state <= WR_COMPLETE;
                        end else begin
                            state <= WR_CALC_DATA;
                        end
                    end else begin
                        state <= WR_GET_DATA_FIFO;
                    end
                end
                WR_COMPLETE: begin
                    if (!mem_wr_in.has_data) begin
                        state <= WR_CLEANUP;
                    end
                end
                WR_CLEANUP: begin
                    data_remaining <= 32'd0;
                    transfer_remaining <= 32'd0;
                    dwords_packet <= 3'd0;
                    write_address <= 64'h0;

                    tlp_data <= 128'h0;
                    tkeepdw  <= 4'h0;
                    tfirst   <= 1'b0;
                    tlast    <= 1'b0;
                    tvalid   <= 1'b0;
                    has_data <= 1'b0;

                    fifo_read <= 1'b0;
                    if (rst_timer == 3'd7) begin
                        state <= WR_IDLE;
                        fifo_rst <= 1'b0;
                        rst_timer <= 3'd0;
                    end else begin
                        fifo_rst <= 1'b1;
                        rst_timer <= rst_timer + 3'd1;
                    end
                end
            endcase
        end
    end
endmodule

module host_mem_write_2_mux (
    input                 rst,
    input                 clk_pcie,

    IfMemoryWrite.source mux_source,
    IfMemoryWrite.sink   wr_in_1,
    IfMemoryWrite.sink   wr_in_2,
    IfMemoryWrite.sink   wr_in_3,
    IfMemoryWrite.sink   wr_in_4,
    IfMemoryWrite.sink   wr_in_5
);
    reg [3:0] id;

    assign mux_source.address     = (id==1) ? wr_in_1.address : 
                                    (id==2) ? wr_in_2.address :
                                    (id==3) ? wr_in_3.address :
                                    (id==4) ? wr_in_4.address :
                                    (id==5) ? wr_in_5.address : 64'h0;
    
    assign mux_source.data_length = (id==1) ? wr_in_1.data_length :
                                    (id==2) ? wr_in_2.data_length :
                                    (id==3) ? wr_in_3.data_length :
                                    (id==4) ? wr_in_4.data_length :
                                    (id==5) ? wr_in_5.data_length : 32'h0;
    
    assign mux_source.has_data    = (id==1) ? wr_in_1.has_data :
                                    (id==2) ? wr_in_2.has_data :
                                    (id==3) ? wr_in_3.has_data :
                                    (id==4) ? wr_in_4.has_data :
                                    (id==5) ? wr_in_5.has_data : 1'b0;
    
    assign mux_source.din         = (id==1) ? wr_in_1.din :
                                    (id==2) ? wr_in_2.din :
                                    (id==3) ? wr_in_3.din :
                                    (id==4) ? wr_in_4.din :
                                    (id==5) ? wr_in_5.din : 128'h0;
    
    assign mux_source.wr_en       = (id==1) ? wr_in_1.wr_en :
                                    (id==2) ? wr_in_2.wr_en :
                                    (id==3) ? wr_in_3.wr_en :
                                    (id==4) ? wr_in_4.wr_en :
                                    (id==5) ? wr_in_5.wr_en : 1'b0;
    
    assign mux_source.wr_done     = (id==1) ? wr_in_1.wr_done :
                                    (id==2) ? wr_in_2.wr_done :
                                    (id==3) ? wr_in_3.wr_done :
                                    (id==4) ? wr_in_4.wr_done :
                                    (id==5) ? wr_in_5.wr_done : 1'b0;

    assign wr_in_1.state = (id==1) ? mux_source.state : WR_IDLE;
    assign wr_in_2.state = (id==2) ? mux_source.state : WR_IDLE;
    assign wr_in_3.state = (id==3) ? mux_source.state : WR_IDLE;
    assign wr_in_4.state = (id==4) ? mux_source.state : WR_IDLE;
    assign wr_in_5.state = (id==5) ? mux_source.state : WR_IDLE;

    wire [3:0] id_next_newsel = wr_in_1.has_data ? 1 :
                                wr_in_2.has_data ? 2 :
                                wr_in_3.has_data ? 3 :
                                wr_in_4.has_data ? 4 :
                                wr_in_5.has_data ? 5 : 0;
    
    wire [3:0] id_next        = mux_source.state == WR_IDLE ? id_next_newsel : id;
    
    always @ ( posedge clk_pcie ) begin
        id <= rst ? 0 : id_next;
    end
endmodule

module host_mem_read_2 (
    input               rst,
    input               clk_pcie,
    input [15:0]        pcie_id,
    IfMemoryRead.sink  mem_rd_in,
    IfAXIS128.source    tlps_out,
    IfAXIS128.sink_lite tlps_in
);
    //CURRENT FIFO BUFFER: 128 * 128 (4 DWORD * 128 = 2048Bytes)
    localparam MAX_PAYLOAD_SIZE = 32'd128; //32DWORDs 128Bytes

    //===========================================
    // MEMORY MODULE REGISTER
    //===========================================
    memory_read_state_t state;
    reg [3:0]  rst_timer;
    reg [31:0] data_remaining;     //全体の残りデータ
    reg [31:0] transfer_remaining; //今回の転送の残りデータ
    reg [2:0]  dwords_packet;      //今回の転送のDWORD数
    reg [63:0] read_address;
    reg [9:0]  dw_received;

    wire [9:0] dw_length = transfer_remaining == 32'd4096 ? 10'd0 : (transfer_remaining[11:2] + (transfer_remaining[1:0] != 2'b00));
    wire [1:0] last_dw_byte = transfer_remaining[1:0];
    wire [3:0] last_dw_be = data_remaining != 32'd0     ? 4'hF    : //今回のTLPは4DWORDで埋まっている
                            transfer_remaining <= 32'd4  ? 4'h0   : //DWORD以下(3バイト以下) = 1st DWのみ有効
                            last_dw_byte == 2'd1        ? 4'b0001 : //DWORD+1バイト
                            last_dw_byte == 2'd2        ? 4'b0011 : //DWORD+2バイト
                            last_dw_byte == 2'd3        ? 4'b0111 : 4'b1111; //DWORD+3バイト
    wire [3:0] first_dw_be = transfer_remaining >= 32'd4 ? 4'hF    : //DWORD以上(4バイト以上) = 1st DWは全有効
                             last_dw_byte == 2'd1        ? 4'b0001 : //DWORD+1バイト
                             last_dw_byte == 2'd2        ? 4'b0011 : //DWORD+2バイト
                             last_dw_byte == 2'd3        ? 4'b0111 : 4'b1111; //DWORD+3バイト

    assign mem_rd_in.state = state;

    //===========================================
    // TLP OUT
    //===========================================
    reg [7:0] request_id;

    reg [127:0] tlp_data;
    reg [3:0]   tkeepdw;
    reg         tfirst;
    reg         tlast;
    reg         tvalid;
    reg         has_data;

    assign tlps_out.tdata    = tlp_data;
    assign tlps_out.tkeepdw  = tkeepdw;
    assign tlps_out.tuser[0] = tfirst; //FIRST
    assign tlps_out.tuser[1] = tlast;  //LAST
    assign tlps_out.tlast    = tlast;  //LAST
    assign tlps_out.tvalid   = tvalid;
    assign tlps_out.has_data = has_data;

    //===========================================
    // TLP IN
    //===========================================
    reg [31:0] first_dword;

    wire first = tlps_in.tuser[0];
    wire is_cpld = first && (tlps_in.tdata[31:25] == 7'b0100101);
    wire our_tag = (tlps_in.tdata[79:72] == request_id);

    //===========================================
    // FIFO
    //===========================================
    reg [127:0] fifo_wr_in;
    reg         fifo_wr_en;
    reg         fifo_rst;

    wire        fifo_srst = fifo_rst | rst;

    fifo_128_128_mem_read i_fifo_128_128_mem_read(
        .srst       ( fifo_srst         ),
        .clk        ( clk_pcie          ),
        .full       (                   ),
        .din        ( fifo_wr_in        ),
        .wr_en      ( fifo_wr_en        ),
        .empty      (                   ),
        .dout       ( mem_rd_in.dout    ),
        .rd_en      ( mem_rd_in.rd_en   )
    );
    
    always @(posedge clk_pcie) begin
        if (rst) begin
            state <= RD_IDLE;
            rst_timer <= 3'd0;

            data_remaining <= 32'd0;
            transfer_remaining <= 32'd0;
            dwords_packet <= 3'd0;
            read_address  <= 64'h0;
            dw_received   <= 10'h0;
            
            request_id <= 8'hff;

            tlp_data <= 128'h0;
            tkeepdw  <= 4'h0;
            tfirst   <= 1'b0;
            tlast    <= 1'b0;
            tvalid   <= 1'b0;
            has_data <= 1'b0;

            first_dword <= 32'h0;

            fifo_wr_in <= 128'h0;
            fifo_wr_en <= 1'b0;
            fifo_rst   <= 1'b0;
        end else begin
            case (state)
                RD_IDLE: begin
                    if (mem_rd_in.has_request) begin
                        state <= RD_CALC_DATA;
                        read_address   <= mem_rd_in.address;
                        data_remaining <= mem_rd_in.data_length;
                    end
                end
                RD_CALC_DATA: begin
                    if (data_remaining > MAX_PAYLOAD_SIZE) begin
                        transfer_remaining <= MAX_PAYLOAD_SIZE;
                        data_remaining <= data_remaining - MAX_PAYLOAD_SIZE;
                    end else begin
                        transfer_remaining <= data_remaining;
                        data_remaining <= 32'd0;
                    end
                    state <= RD_PREPARE_HEADER;
                end
                RD_PREPARE_HEADER: begin
                    tkeepdw  <= 4'hF;
                    tfirst   <= 1'b1;
                    tlast    <= 1'b1;
                    has_data <= 1'b1;
                    tlp_data <= {
                        read_address[31:0],
                        read_address[63:32],
                        { `_bs16(pcie_id), request_id, last_dw_be, first_dw_be },
                        22'b00100000_00000000_000000, dw_length
                    };
                    read_address <= read_address + transfer_remaining;
                    state <= RD_TRANSMIT_HEADER;
                end
                RD_TRANSMIT_HEADER: begin
                    if (tlps_out.tready) begin
                        tvalid <= 1'b1;
                        has_data <= 1'b0;
                        state <=  RD_WAIT_CPLT;
                    end
                end
                RD_WAIT_CPLT: begin
                    tvalid <= 1'b0;
                    if (tlps_in.tvalid && is_cpld && our_tag) begin
                        first_dword <= `_bs32(tlps_in.tdata[127:96]); //4st DW of CPLT packet
                        dw_received <= dw_received + 1;
                        if (dw_length <= 10'd1) begin
                            //complete read with only 1DW -> push to fifo
                            state <= RD_PACK_LAST;
                        end else begin
                            state <= RD_GET_DATA;
                        end
                    end
                end
                RD_GET_DATA: begin
                    if (tlps_in.tvalid) begin
                        fifo_wr_en <= 1'b1;
                        first_dword <= `_bs32(tlps_in.tdata[127:96]); //new 1st dword
                        fifo_wr_in[31:0]   <= first_dword; //old 1st dword
                        fifo_wr_in[63:32]  <= (dw_length - dw_received >= 1) ? `_bs32(tlps_in.tdata[31:0]) : 32'h0;
                        fifo_wr_in[95:64]  <= (dw_length - dw_received >= 2) ? `_bs32(tlps_in.tdata[63:32]) : 32'h0;
                        fifo_wr_in[127:96] <= (dw_length - dw_received >= 3) ? `_bs32(tlps_in.tdata[95:64]) : 32'h0;

                        if (dw_length - dw_received == 10'd4) begin
                            state <= RD_PACK_LAST;
                        end else if (dw_length - dw_received < 10'd4) begin
                            state <= RD_COMPLETE;
                        end else begin
                            dw_received <= dw_received + 10'd4;
                        end
                    end else begin
                        fifo_wr_en <= 1'b0;
                    end
                end
                RD_PACK_LAST: begin
                    fifo_wr_en <= 1'b1;
                    fifo_wr_in <= { 96'h0, first_dword };
                    state <= RD_COMPLETE;
                end
                RD_COMPLETE: begin
                    fifo_wr_en <= 1'b0;
                    if (!mem_rd_in.has_request) begin
                        state <= RD_CLEANUP;
                    end
                end
                RD_CLEANUP: begin
                    data_remaining <= 32'd0;
                    transfer_remaining <= 32'd0;
                    dwords_packet <= 3'd0;
                    read_address  <= 64'h0;
                    dw_received   <= 10'h0;
            
                    request_id <= request_id + 8'd1;

                    tlp_data <= 128'h0;
                    tkeepdw  <= 4'h0;
                    tfirst   <= 1'b0;
                    tlast    <= 1'b0;
                    tvalid   <= 1'b0;
                    has_data <= 1'b0;

                    first_dword <= 32'h0;

                    fifo_wr_in <= 128'h0;
                    fifo_wr_en <= 1'b0;

                    if (rst_timer == 3'd7) begin
                        state <= RD_IDLE;
                        fifo_rst <= 1'b0;
                        rst_timer <= 3'd0;
                    end else begin
                        fifo_rst <= 1'b1;
                        rst_timer <= rst_timer + 3'd1;
                    end
                end
            endcase
        end
    end
endmodule

module host_mem_read_2_mux (
    input                rst,
    input                clk_pcie,

    IfMemoryRead.source mux_source,
    IfMemoryRead.sink   rd_in_1,
    IfMemoryRead.sink   rd_in_2,
    IfMemoryRead.sink   rd_in_3,
    IfMemoryRead.sink   rd_in_4,
    IfMemoryRead.sink   rd_in_5
);
    reg [3:0] id;

    assign mux_source.address =     (id==1) ? rd_in_1.address :
                                    (id==2) ? rd_in_2.address : 
                                    (id==3) ? rd_in_3.address :
                                    (id==4) ? rd_in_4.address :
                                    (id==5) ? rd_in_5.address : 64'h0;
    
    assign mux_source.data_length = (id==1) ? rd_in_1.data_length :
                                    (id==2) ? rd_in_2.data_length :
                                    (id==3) ? rd_in_3.data_length :
                                    (id==4) ? rd_in_4.data_length :
                                    (id==5) ? rd_in_5.data_length : 32'h0;

    assign mux_source.has_request = (id==1) ? rd_in_1.has_request :
                                    (id==2) ? rd_in_2.has_request :
                                    (id==3) ? rd_in_3.has_request :
                                    (id==4) ? rd_in_4.has_request :
                                    (id==5) ? rd_in_5.has_request : 1'b0;

    assign mux_source.rd_en       = (id==1) ? rd_in_1.rd_en :
                                    (id==2) ? rd_in_2.rd_en :
                                    (id==3) ? rd_in_3.rd_en :
                                    (id==4) ? rd_in_4.rd_en :
                                    (id==5) ? rd_in_5.rd_en : 1'b0;

    assign rd_in_1.state = (id==1) ? mux_source.state : RD_IDLE;
    assign rd_in_2.state = (id==2) ? mux_source.state : RD_IDLE;
    assign rd_in_3.state = (id==3) ? mux_source.state : RD_IDLE;
    assign rd_in_4.state = (id==4) ? mux_source.state : RD_IDLE;
    assign rd_in_5.state = (id==5) ? mux_source.state : RD_IDLE;

    assign rd_in_1.dout = (id==1) ? mux_source.dout : 128'h0;
    assign rd_in_2.dout = (id==2) ? mux_source.dout : 128'h0;
    assign rd_in_3.dout = (id==3) ? mux_source.dout : 128'h0;
    assign rd_in_4.dout = (id==4) ? mux_source.dout : 128'h0;
    assign rd_in_5.dout = (id==5) ? mux_source.dout : 128'h0;

    wire [3:0] id_next_newsel = rd_in_1.has_request ? 1 :
                                rd_in_2.has_request ? 2 :
                                rd_in_3.has_request ? 3 :
                                rd_in_4.has_request ? 4 :
                                rd_in_5.has_request ? 5 : 0;
    
    wire [3:0] id_next        = mux_source.state == WR_IDLE ? id_next_newsel : id;

    always @ ( posedge clk_pcie ) begin
        id <= rst ? 0 : id_next;
    end
endmodule