`timescale 1ns / 1ps
`include "pcileech_header.svh"
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/01/26 12:18:30
// Design Name: 
// Module Name: bar0-controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module xhci_bar0_impl_2(
    input                 rst,
    input                 clk,
    // incoming BAR writes:
    input [31:0]          wr_addr,
    input [3:0]           wr_be,
    input [31:0]          wr_data,
    input                 wr_valid,
    // incoming BAR reads:
    input  [87:0]         rd_req_ctx,
    input  [31:0]         rd_req_addr,
    input                 rd_req_valid,

    // outgoing BAR read replies:
    output bit [87:0]     rd_rsp_ctx,
    output bit [31:0]     rd_rsp_data,
    output bit            rd_rsp_valid,
    xHCI_RtReg            if_rt_reg,
    IfEventRingRequest.source      event_ring_req_1,
    IfEventRingRequest.source      event_ring_req_2,
    IfEventRingRequest.source      event_ring_req_3,
    IfMemoryWrite.source          mem_wr_out_1,
    IfMemoryWrite.source          mem_wr_out_2,
    IfMemoryWrite.source          mem_wr_out_3,
    IfMemoryWrite.source          mem_wr_out_4, //Set Endpoint State
    IfMemoryRead.source           mem_rd_out_1,
    IfMemoryRead.source           mem_rd_out_2,
    IfMemoryRead.source           mem_rd_out_3,
    IfMemoryRead.source           mem_rd_out_4,
    IfMemoryRead.source           mem_rd_out_5, //Set Endpoint State
    IfInterrupterController.source interrupter_controller_out,
    IfDebugEventRing.sink         dbg_evt_ring
//    IfMsiXRequest.source  msix_req_out
);

    port_reg_t port_regs [8];

    xHCI_OpReg if_op_reg();
    xHCI_DbReg if_db_reg();
    xHCI_ExCap if_ex_cap();

    reg [87:0] drd_req_ctx;
    reg [31:0] drd_req_addr;
    reg        drd_req_valid;

    reg [31:0] dwr_addr;
    reg [3:0] dwr_be;
    reg [31:0] dwr_data;
    reg dwr_valid;

    reg command_ring_trigger;
    reg [1:0] command_ring_pointer_update;

    wire [31:0] offset = drd_req_addr & 32'hFFFF;
    wire [31:0] wr_offset = dwr_addr & 32'hFFFF;

    wire        rd_port_reg = offset >= 32'h480 && offset < 32'h500; //Port Register Read
    wire [31:0] rd_port_reg_index = rd_port_reg ? (offset - 32'h480) >> 4 : 32'h0; //Port Register Index
    wire [1:0]  rd_port_reg_data  = rd_port_reg ? ((offset - 32'h480) >> 2) & 2'b11 : 2'h0; //Port Register Data
    wire        wr_port_reg = wr_offset >= 32'h480 && wr_offset < 32'h500; //Port Register Write
    wire [31:0] wr_port_reg_index = wr_port_reg ? (wr_offset - 32'h480) >> 4 : 32'h0; //Port Register Index
    wire [1:0]  wr_port_reg_data  = wr_port_reg ? ((wr_offset - 32'h480) >> 2) & 2'b11 : 2'h0; //Port Register Data

    reg [12:0] mf_index_counter;

    //---------------------------------------------------------
    // Event Ring (TEST)
    //---------------------------------------------------------

    reg         event_ring_send;
    reg         event_ring_has_request;
    reg [2:0]   event_ring_interrupter_index;
    reg [127:0] event_ring_trb_data;

    assign event_ring_req_1.send              = event_ring_send;
    assign event_ring_req_1.has_request       = event_ring_has_request;
    assign event_ring_req_1.interrupter_index = event_ring_interrupter_index;
    assign event_ring_req_1.trb_data          = event_ring_trb_data;

    //---------------------------------------------------------
    // Memory Write FIFO TEST
    //---------------------------------------------------------

    reg [31:0] mwr_ctrl;
    reg [63:0] mwr_addr;
    reg [31:0] mwr_length;
    reg        mwr_has_data;

    reg [127:0] mwr_fifo_in;
    reg         mwr_fifo_en;
    reg         mwr_fifo_done;

    assign mem_wr_out_1.address     = mwr_addr;
    assign mem_wr_out_1.data_length = mwr_length;
    assign mem_wr_out_1.has_data    = mwr_has_data;
    assign mem_wr_out_1.din         = mwr_fifo_in;
    assign mem_wr_out_1.wr_en       = mwr_fifo_en;
    assign mem_wr_out_1.wr_done     = mwr_fifo_done;

    //---------------------------------------------------------
    // Memory Read Test Module
    //---------------------------------------------------------

    reg [31:0] mrd_ctrl;
    reg [63:0] mrd_addr;
    reg [31:0] mrd_length;
    reg        mrd_has_request;

    reg        mrd_fifo_en;
    
    reg [127:0] mrd_data_stack [8];

    assign mem_rd_out_3.address     = mrd_addr;
    assign mem_rd_out_3.data_length = mrd_length;
    assign mem_rd_out_3.has_request = mrd_has_request;
    assign mem_rd_out_3.rd_en       = mrd_fifo_en;

    //---------------------------------------------------------
    // Interrupter Register
    //---------------------------------------------------------
    wire [31:0] rd_interrupter_offset = offset - 32'h2020;
    wire [2:0]  rd_interrupter_index  = rd_interrupter_offset[7:5];
    wire [2:0]  rd_interrupter_data   = rd_interrupter_offset[4:2];
    wire [31:0] wr_interrupter_offset = wr_offset - 32'h2020;
    wire [2:0]  wr_interrupter_index  = wr_interrupter_offset[7:5];
    wire [2:0]  wr_interrupter_data   = wr_interrupter_offset[4:2];

    wire [4:0] command_ring_state;
    wire update_dequeue_pointer = (command_ring_pointer_update == 2'b10);

    //---------------------------------------------------------
    // Interrupter Controller (PREFIX: ic_)
    //---------------------------------------------------------
    // Interrupter Enable Bit => reset Enqueue Pointer
    reg       ic_interrupter_enabled;
    reg [2:0] ic_interrupter_index;

    assign interrupter_controller_out.interrupter_enabled = ic_interrupter_enabled;
    assign interrupter_controller_out.interrupter_index   = ic_interrupter_index;

    //---------------------------------------------------------
    // Device Connect 
    //---------------------------------------------------------

    localparam DC_START_CONNECT_COUNT = 32'd10000000;
    
    // STATE
    localparam DC_STATE_WAIT  = 6'h0;
    localparam DC_STATE_START = 6'h1;
    localparam DC_STATE_3     = 6'h2;
    localparam DC_STATE_4     = 6'h3;
    localparam DC_STATE_5     = 6'h4;
    localparam DC_STATE_6     = 6'h5;
    localparam DC_STATE_7     = 6'h6;
    
    reg [5:0]  dc_state;        //状態管理
    reg [31:0] dc_time_counter; //デバイス接続処理開始までのカウント

    reg dbg_have_hit_doorbell;
    reg dbg_have_update_cr;

    //---------------------------------------------------------
    // Enable Slot Task
    //---------------------------------------------------------

    IfEnableSlot if_enable_slot();

    wire [4:0] enable_slot_index = if_enable_slot.target_index * 4;

    //---------------------------------------------------------
    // Port Reset Handler
    //---------------------------------------------------------

    // STATE
    localparam PR_STATE_IDLE     = 5'd0;
    localparam PR_STATE_START    = 5'd1;
    localparam PR_STATE_ITR_LOCK = 5'd2;
    localparam PR_STATE_ITR_DATA = 5'd3;
    localparam PR_STATE_ITR_WAIT = 5'd4;
    
    reg [4:0] port_reset_state;
    reg [4:0] port_reset_index;

    wire [4:0]  control_enabled_slot;
    wire [67:0] ctrl_update_slot_ctx_ptr;

    wire [73:0] set_ep_tr_ptr_cmdr; //command ring -> slot_context
    wire [73:0] set_ep_tr_ptr_trsr; //transfer ring -> slot_context
    wire [73:0] set_ep_tr_ptr;      //mux -> slot_context

    //---------------------------------------------------------
    // debug
    //---------------------------------------------------------

    reg  [31:0] dbg_status;
    
    reg  [73:0] dbg_set_ep_tr_ptr;

    wire [63:0] dbg_cr_ptr;
    wire [63:0] dbg_tr_ptr;
    wire [63:0] dbg_ev_ptr = dbg_evt_ring.ptr;

    wire        dbg_cr_pending;
    wire        dbg_tr_pending;
    wire        dbg_ev_pending = dbg_evt_ring.pending_request;

    assign dbg_evt_ring.status = dbg_status[2];

    command_ring i_command_ring(
        .rst                      ( rst                      ),
        .clk                      ( clk                      ),
        .trigger                  ( command_ring_trigger     ),
        .read_out                 ( mem_rd_out_2             ),
        .read_out_2               ( mem_rd_out_5             ),
        .write_out                ( mem_wr_out_2             ),
        .write_out_2              ( mem_wr_out_4             ),
        .state                    ( command_ring_state       ),
        .update_dequeue_pointer   ( update_dequeue_pointer   ),
        .new_command_ring_pointer ( if_op_reg.crcr[63:6]     ),
        .dcbaap                   ( if_op_reg.dcbaap         ),
        .enable_slot_out          ( if_enable_slot.source    ),
        .event_ring_req           ( event_ring_req_2         ),
        .slot_ctx_in              ( if_slot_ctx.sink         ),
        .control_enabled_slot     ( control_enabled_slot     ),
        .ctrl_update_slot_ctx_ptr ( ctrl_update_slot_ctx_ptr ),
        .set_ep_tr_ptr_out        ( set_ep_tr_ptr_cmdr       ),
        .dbg_pending_request      ( dbg_cr_pending           ),
        .dbg_cr_ptr               ( dbg_cr_ptr               ),
        .dbg_status_cr            ( dbg_status[0]            )
    );

    //---------------------------------------------------------
    // Slot Context
    //---------------------------------------------------------

    IfSlotContext if_slot_ctx();

    slot_context i_slot_context(
        .rst                      ( rst                      ),
        .clk                      ( clk                      ),
        .slot_ctx_out             ( if_slot_ctx.source       ),
        .control_enabled_slot     ( control_enabled_slot     ),
        .ctrl_update_slot_ctx_ptr ( ctrl_update_slot_ctx_ptr ),
        .set_ep_tr_ptr_in         ( set_ep_tr_ptr            )
    );

    set_ep_tr_mux i_set_ep_tr_mux(
        .rst                      ( rst                      ),
        .clk                      ( clk                      ),
        .in_1                     ( set_ep_tr_ptr_cmdr       ),
        .in_2                     ( set_ep_tr_ptr_trsr       ),
        .out                      ( set_ep_tr_ptr            )
    );

    //---------------------------------------------------------
    // Transfer Ring
    //---------------------------------------------------------

    reg        tr_trigger;
    reg [7:0]  tr_slot_id;
    reg [7:0]  tr_ep_id;
    reg [15:0] tr_stream_id;

    wire [31:0] db_offset = wr_offset - 32'h3000;
    wire [7:0]  db_index  = db_offset[9:2];

    wire [3:0] dbg_tr_state;
    wire [3:0] dbg_tr_sub_task_type;
    wire [4:0] dbg_tr_sub_task_state;
    wire [7:0] dbg_ep_id;

    transfer_ring i_transfer_ring(
        .rst                      ( rst                    ),
        .clk                      ( clk                    ),
        .trigger                  ( tr_trigger             ),
        .read_out                 ( mem_rd_out_4           ),
        .write_out                ( mem_wr_out_3           ),
        .event_ring_out           ( event_ring_req_3       ),
        .slot_id_in               ( tr_slot_id             ),
        .ep_id_in                 ( tr_ep_id               ),
        .stream_id_in             ( tr_stream_id           ),
        .i_slot_ctx               ( if_slot_ctx.sink       ),
        .set_ep_tr_ptr_out        ( set_ep_tr_ptr_trsr     ),
        .dbg_pending_request      ( dbg_tr_pending         ),
        .dbg_tr_ptr               ( dbg_tr_ptr             ),
        .dbg_status_tr            ( dbg_status[1]          ),
        .dbg_state                ( dbg_tr_state           ),
        .dbg_sub_task_type        ( dbg_tr_sub_task_type   ),
        .dbg_sub_task_state       ( dbg_tr_sub_task_state  ),
        .dbg_ep_id                ( dbg_ep_id              )
    );

    task init_port_register;
        input integer index;
        begin

            //PORTSC
            port_regs[index].portsc.CCS <= 1'b0; //Current Connect Status - ROS
            port_regs[index].portsc.PED <= 1'b0; //Port Enabled/Disabled - RW1S
            port_regs[index].portsc.OCA <= 1'b0; //Over-current Active - RO
            port_regs[index].portsc.PR  <= 1'b0; //Port Reset - RW1S
            port_regs[index].portsc.PP  <= 1'b1; //Port Power - RWS
            port_regs[index].portsc.PLS <= 4'h5; // Port Link State - RWS
            port_regs[index].portsc.PortSpeed <= 4'h0; //Port Speed - ROS
            port_regs[index].portsc.PIC <= 2'h0; //Port Indicator Control - RWS
            port_regs[index].portsc.LWS <= 1'b0; //Port Link State Write Strobe - RW
            port_regs[index].portsc.CSC <= 1'b0; //Connect Status Change - RW1CS
            port_regs[index].portsc.PEC <= 1'b0; //Port ENabled/Disabled Change - RW1CS
            port_regs[index].portsc.WRC <= 1'b0; //Warm Port Reset Change - RW1CS/RsvdZ
            port_regs[index].portsc.OCC <= 1'b0; //Over-current Change - RW1CS
            port_regs[index].portsc.PRC <= 1'b0; //Port Reset Change - RW1CS
            port_regs[index].portsc.PLC <= 1'b0; //Port Link State - RW1CS
            port_regs[index].portsc.CEC <= 1'b0; //Port Config Error Change - RW1CS/RsvdZ
            port_regs[index].portsc.CAS <= 1'b0; //Cold Attach Status - RO
            port_regs[index].portsc.WCE <= 1'b0; //Wake on Connect Enable - RWS
            port_regs[index].portsc.WDE <= 1'b0; //Wake on Diconnect Enable - RWS
            port_regs[index].portsc.WOE <= 1'b0; //Wake on Over-current Enable - RWS
            port_regs[index].portsc.DR  <= 1'b0; //Device Removable - RO
            port_regs[index].portsc.WPR <= 1'b0; //Warm Port Reset - RW1S/RsvdZ

            //PORTPMSC
            if (index & 32'hF0) begin //USB3
                port_regs[index + 1].PORTPMSC[7:0]   <= 8'h0; //U1 Timeout - RWS
                port_regs[index + 1].PORTPMSC[15:8]  <= 8'h0; //U2 Timeout - RWS
                port_regs[index + 1].PORTPMSC[16]    <= 1'b0; //Force Link PM Accept - RW
            end else begin //USB2
                port_regs[index + 1].PORTPMSC[2:0]   <= 3'h0; //L1 Status - RO
                port_regs[index + 1].PORTPMSC[3]     <= 1'b0; //Remote Wake Enable - RW
                port_regs[index + 1].PORTPMSC[7:4]   <= 4'h0; //Best Effort Service Latency - RW
                port_regs[index + 1].PORTPMSC[15:8]  <= 8'h0; //L1 Device Slot - RW
                port_regs[index + 1].PORTPMSC[16]    <= 1'b0; //Hardware LPM Enable - RW
                port_regs[index + 1].PORTPMSC[31:28] <= 4'h0; //Port Teset Control - RW
            end

            //PORTLI
            if (index & 32'hF0) begin //USB3
                port_regs[index + 2].PORTLI[15:0]   <= 16'h0; //Link Error Count - RW
                port_regs[index + 2].PORTLI[19:16]  <= 4'h0; //Rx Lane Count - RO
                port_regs[index + 2].PORTLI[23:20]  <= 4'h0; //Tx Lane Count - RO
            end

            //PORTHLPMC
            if (!(index & 32'hF0)) begin //USB2
                port_regs[index + 3].PORTHLPMC[1:0]    <= 2'h0; //Host Initiated Resume Duration Mode - RWS
                port_regs[index + 3].PORTHLPMC[9:2]    <= 8'h0; //L1 Timeout - RWS
                port_regs[index + 3].PORTHLPMC[13:10]  <= 4'h0; //Best Effort Service Latency Deep - RWS
            end
        end
    endtask

    task reset_interrupter_register;
        input [2:0] index;
        begin
            //Interrupter Management Register
            if_rt_reg.interrupters[index].interrupt_pending <= 1'b0;
            if_rt_reg.interrupters[index].interrupt_enable  <= 1'b0;
            //Interrupter Moderation Register
            if_rt_reg.interrupters[index].interrupt_moderation_interval <= 16'h0;
            if_rt_reg.interrupters[index].interrupt_moderation_counter  <= 16'h0;
            //Event Ring Segment Table Size Register
            if_rt_reg.interrupters[index].event_ring_segment_table_size <= 16'h0;
            //Event Ring Segment Table Base Address Register
            if_rt_reg.interrupters[index].event_ring_segment_table_base_address <= 58'h0;
            //Event Ring Dequeue Pointer Register
            if_rt_reg.interrupters[index].dequeue_erst_segment_index <= 3'h0;
            if_rt_reg.interrupters[index].event_handler_busy         <= 1'b0;
            if_rt_reg.interrupters[index].event_ring_dequeue_pointer <= 60'h0;
        end
    endtask

    task reset_xhci;
        begin
            if_op_reg.usb_cmd <= 32'h0;
            if_op_reg.usb_sts <= 32'h1;
            if_op_reg.dnctrl <= 15'h0;
            if_op_reg.crcr = 64'h0;
            if_op_reg.dcbaap = 64'h0;
            if_op_reg.cfg = 32'h0;
            //Reset Port Registers (USB2)
            init_port_register(0);
            init_port_register(1);
            init_port_register(2);
            init_port_register(3);
            //Reset Port Registers (USB3)
            init_port_register(4);
            init_port_register(5);
            init_port_register(6);
            init_port_register(7);

            if_rt_reg.mf_index = 32'h0;
            if_db_reg.db_regs = 256'h0;

            //Clear Interrupter Registers
            for (int i = 0; i < 8; i++) begin
                reset_interrupter_register(i);
            end

            //Vendor Defined Capability + USB Debug Capability

        end
    endtask

    task init_vd_cap;
        begin
            if_ex_cap.vd_cap[0] <= 32'h00000CC1;
            if_ex_cap.vd_cap[1] <= 32'h03FFFFFF;
            if_ex_cap.vd_cap[2] <= 32'h00000000;
            if_ex_cap.vd_cap[3] <= 32'hFFFF0000;
            if_ex_cap.vd_cap[4] <= 32'h21931A00;
            if_ex_cap.vd_cap[5] <= 32'h00000000;
            if_ex_cap.vd_cap[6] <= 32'h3500AFFC;
            if_ex_cap.vd_cap[7] <= 32'h06C60000;
            if_ex_cap.vd_cap[8] <= 32'h2220505A;
            if_ex_cap.vd_cap[9] <= 32'h00000000;
            if_ex_cap.vd_cap[10] <= 32'h00000000;
            if_ex_cap.vd_cap[11] <= 32'h00000000;
            if_ex_cap.vd_cap[12] <= 32'h0000B0C0;
            if_ex_cap.vd_cap[13] <= 32'h00000000;
            if_ex_cap.vd_cap[14] <= 32'h00000000;
            if_ex_cap.vd_cap[15] <= 32'h00000000;
            if_ex_cap.vd_cap[16] <= 32'h00000000;
            if_ex_cap.vd_cap[17] <= 32'h00000000;
            if_ex_cap.vd_cap[18] <= 32'h0881F150;
            if_ex_cap.vd_cap[19] <= 32'h1300005D;
            if_ex_cap.vd_cap[20] <= 32'h4E1BD105;
            if_ex_cap.vd_cap[21] <= 32'h00008100;
            if_ex_cap.vd_cap[22] <= 32'h14003002;
            if_ex_cap.vd_cap[23] <= 32'h00000402;
            if_ex_cap.vd_cap[24] <= 32'h0003F80F;
            if_ex_cap.vd_cap[25] <= 32'h00000000;
            if_ex_cap.vd_cap[26] <= 32'h00000000;
            if_ex_cap.vd_cap[27] <= 32'h00000000;
            if_ex_cap.vd_cap[28] <= 32'h00000000;
            if_ex_cap.vd_cap[29] <= 32'h00000000;
            if_ex_cap.vd_cap[30] <= 32'h00000000;
            if_ex_cap.vd_cap[31] <= 32'h00000000;
            if_ex_cap.vd_cap[32] <= 32'h291780F0;
            if_ex_cap.vd_cap[33] <= 32'h004A4008;
            if_ex_cap.vd_cap[34] <= 32'h0001A01F;
            if_ex_cap.vd_cap[35] <= 32'h00014080;
            if_ex_cap.vd_cap[36] <= 32'h00032010;
            if_ex_cap.vd_cap[37] <= 32'h00000000;
            if_ex_cap.vd_cap[38] <= 32'h00000000;
            if_ex_cap.vd_cap[39] <= 32'h00000061;
            if_ex_cap.vd_cap[40] <= 32'h808DB1A0;
            if_ex_cap.vd_cap[41] <= 32'h00000000;
            if_ex_cap.vd_cap[42] <= 32'h00800080;
            if_ex_cap.vd_cap[43] <= 32'h08015C10;
            if_ex_cap.vd_cap[44] <= 32'h311803A0;
            if_ex_cap.vd_cap[45] <= 32'h80C40620;
            if_ex_cap.vd_cap[46] <= 32'hF865EB6B;
            if_ex_cap.vd_cap[47] <= 32'h00008003;
            if_ex_cap.vd_cap[48] <= 32'h00000010;
            if_ex_cap.vd_cap[49] <= 32'h12000000;
            if_ex_cap.vd_cap[50] <= 32'h00000081;
            if_ex_cap.vd_cap[51] <= 32'h00008020;
            if_ex_cap.vd_cap[52] <= 32'hF0FD838C;
            if_ex_cap.vd_cap[53] <= 32'h00000000;
            if_ex_cap.vd_cap[54] <= 32'h00000000;
            if_ex_cap.vd_cap[55] <= 32'h00000000;
            if_ex_cap.vd_cap[56] <= 32'h00000000;
            if_ex_cap.vd_cap[57] <= 32'h00000000;
            if_ex_cap.vd_cap[58] <= 32'h05647F42;
            if_ex_cap.vd_cap[59] <= 32'h0F425285;
            if_ex_cap.vd_cap[60] <= 32'h00004000;
            if_ex_cap.vd_cap[61] <= 32'h0C05140C;
            if_ex_cap.vd_cap[62] <= 32'h000C3C64;
            if_ex_cap.vd_cap[63] <= 32'h5B4190B8;
            if_ex_cap.vd_cap[64] <= 32'h0A000000;
            if_ex_cap.vd_cap[65] <= 32'h00000000;
            if_ex_cap.vd_cap[66] <= 32'h00000000;
            if_ex_cap.vd_cap[67] <= 32'h00000000;
            if_ex_cap.vd_cap[68] <= 32'h00000000;
            if_ex_cap.vd_cap[69] <= 32'hEB39060C;
            if_ex_cap.vd_cap[70] <= 32'h00083370;
            if_ex_cap.vd_cap[71] <= 32'h02000000;
            if_ex_cap.vd_cap[72] <= 32'h00007A00;
            if_ex_cap.vd_cap[73] <= 32'h00000001;
            if_ex_cap.vd_cap[74] <= 32'h20080404;
            if_ex_cap.vd_cap[75] <= 32'h00000000;
            if_ex_cap.vd_cap[76] <= 32'h00007FFA;
            if_ex_cap.vd_cap[77] <= 32'h00000000;
            if_ex_cap.vd_cap[78] <= 32'h00000000;
            if_ex_cap.vd_cap[79] <= 32'h00000000;
            if_ex_cap.vd_cap[80] <= 32'h00000000;
            if_ex_cap.vd_cap[81] <= 32'h00000000;
            if_ex_cap.vd_cap[82] <= 32'h00000000;
            if_ex_cap.vd_cap[83] <= 32'h00000000;
            if_ex_cap.vd_cap[84] <= 32'h00000000;
            if_ex_cap.vd_cap[85] <= 32'h00000000;
            if_ex_cap.vd_cap[86] <= 32'h00000000;
            if_ex_cap.vd_cap[87] <= 32'h00000000;
            if_ex_cap.vd_cap[88] <= 32'h00000002;
            if_ex_cap.vd_cap[89] <= 32'h00000000;
            if_ex_cap.vd_cap[90] <= 32'h00000400;
            if_ex_cap.vd_cap[91] <= 32'h00000000;
            if_ex_cap.vd_cap[92] <= 32'h00000000;
            if_ex_cap.vd_cap[93] <= 32'h00000000;
            if_ex_cap.vd_cap[94] <= 32'h00000000;
            if_ex_cap.vd_cap[95] <= 32'h00000000;
            if_ex_cap.vd_cap[96] <= 32'h00000000;
            if_ex_cap.vd_cap[97] <= 32'h00000000;
            if_ex_cap.vd_cap[98] <= 32'h00000000;
            if_ex_cap.vd_cap[99] <= 32'h00000000;
            if_ex_cap.vd_cap[100] <= 32'hC0C0C0C0;
            if_ex_cap.vd_cap[101] <= 32'h013641F4;
            if_ex_cap.vd_cap[102] <= 32'h001F4014;
            if_ex_cap.vd_cap[103] <= 32'h00000000;
            if_ex_cap.vd_cap[104] <= 32'h00000000;
            if_ex_cap.vd_cap[105] <= 32'h00000000;
            if_ex_cap.vd_cap[106] <= 32'h00000000;
            if_ex_cap.vd_cap[107] <= 32'h00000000;
            if_ex_cap.vd_cap[108] <= 32'h00000000;
            if_ex_cap.vd_cap[109] <= 32'h00000000;
            if_ex_cap.vd_cap[110] <= 32'h00000000;
            if_ex_cap.vd_cap[111] <= 32'h00000000;
            if_ex_cap.vd_cap[112] <= 32'h00000000;
            if_ex_cap.vd_cap[113] <= 32'h00000000;
            if_ex_cap.vd_cap[114] <= 32'h00000000;
            if_ex_cap.vd_cap[115] <= 32'h00000000;
            if_ex_cap.vd_cap[116] <= 32'h00000000;
            if_ex_cap.vd_cap[117] <= 32'h00000000;
            if_ex_cap.vd_cap[118] <= 32'h00000000;
            if_ex_cap.vd_cap[119] <= 32'h00000000;
            if_ex_cap.vd_cap[120] <= 32'h00000000;
            if_ex_cap.vd_cap[121] <= 32'h00000000;
            if_ex_cap.vd_cap[122] <= 32'h00000000;
            if_ex_cap.vd_cap[123] <= 32'h00000000;
            if_ex_cap.vd_cap[124] <= 32'h00000000;
            if_ex_cap.vd_cap[125] <= 32'h00000000;
            if_ex_cap.vd_cap[126] <= 32'h00000000;
            if_ex_cap.vd_cap[127] <= 32'h00000000;
            if_ex_cap.vd_cap[128] <= 32'h00000000;
            if_ex_cap.vd_cap[129] <= 32'h00000000;
            if_ex_cap.vd_cap[130] <= 32'h00000000;
            if_ex_cap.vd_cap[131] <= 32'h00000000;
            if_ex_cap.vd_cap[132] <= 32'h00000000;
            if_ex_cap.vd_cap[133] <= 32'h00000000;
            if_ex_cap.vd_cap[134] <= 32'h00000000;
            if_ex_cap.vd_cap[135] <= 32'h00000000;
            if_ex_cap.vd_cap[136] <= 32'h00000000;
            if_ex_cap.vd_cap[137] <= 32'h00000000;
            if_ex_cap.vd_cap[138] <= 32'h00000000;
            if_ex_cap.vd_cap[139] <= 32'h00000000;
            if_ex_cap.vd_cap[140] <= 32'h00000000;
            if_ex_cap.vd_cap[141] <= 32'h00000000;
            if_ex_cap.vd_cap[142] <= 32'h00000000;
            if_ex_cap.vd_cap[143] <= 32'h00000000;
            if_ex_cap.vd_cap[144] <= 32'h00000000;
            if_ex_cap.vd_cap[145] <= 32'h00000000;
            if_ex_cap.vd_cap[146] <= 32'h00000000;
            if_ex_cap.vd_cap[147] <= 32'h00000000;
            if_ex_cap.vd_cap[148] <= 32'h00000000;
            if_ex_cap.vd_cap[149] <= 32'h00000000;
            if_ex_cap.vd_cap[150] <= 32'h00000000;
            if_ex_cap.vd_cap[151] <= 32'h00000000;
            if_ex_cap.vd_cap[152] <= 32'h00000000;
            if_ex_cap.vd_cap[153] <= 32'h00000000;
            if_ex_cap.vd_cap[154] <= 32'h00000000;
            if_ex_cap.vd_cap[155] <= 32'h00000000;
            if_ex_cap.vd_cap[156] <= 32'h00000000;
            if_ex_cap.vd_cap[157] <= 32'h00000000;
            if_ex_cap.vd_cap[158] <= 32'h00000000;
            if_ex_cap.vd_cap[159] <= 32'h00000000;
            if_ex_cap.vd_cap[160] <= 32'h00000000;
            if_ex_cap.vd_cap[161] <= 32'h00000000;
            if_ex_cap.vd_cap[162] <= 32'h00000000;
            if_ex_cap.vd_cap[163] <= 32'h00000000;
            if_ex_cap.vd_cap[164] <= 32'h00000000;
            if_ex_cap.vd_cap[165] <= 32'h00000000;
            if_ex_cap.vd_cap[166] <= 32'h00000000;
            if_ex_cap.vd_cap[167] <= 32'h00000000;
            if_ex_cap.vd_cap[168] <= 32'h00000000;
            if_ex_cap.vd_cap[169] <= 32'h00000000;
            if_ex_cap.vd_cap[170] <= 32'h00000000;
            if_ex_cap.vd_cap[171] <= 32'h00000000;
            if_ex_cap.vd_cap[172] <= 32'h00000000;
            if_ex_cap.vd_cap[173] <= 32'h00000000;
            if_ex_cap.vd_cap[174] <= 32'h00000000;
            if_ex_cap.vd_cap[175] <= 32'h00000000;
            if_ex_cap.vd_cap[176] <= 32'h00000000;
            if_ex_cap.vd_cap[177] <= 32'h00000000;
            if_ex_cap.vd_cap[178] <= 32'h00000000;
            if_ex_cap.vd_cap[179] <= 32'h00000000;
            if_ex_cap.vd_cap[180] <= 32'h00000000;
            if_ex_cap.vd_cap[181] <= 32'h00000000;
            if_ex_cap.vd_cap[182] <= 32'h00000000;
            if_ex_cap.vd_cap[183] <= 32'h00000000;
            if_ex_cap.vd_cap[184] <= 32'h00000000;
            if_ex_cap.vd_cap[185] <= 32'h00000000;
            if_ex_cap.vd_cap[186] <= 32'h00000000;
            if_ex_cap.vd_cap[187] <= 32'h00000000;
            if_ex_cap.vd_cap[188] <= 32'h00000401;
            if_ex_cap.vd_cap[189] <= 32'h00000000;
            if_ex_cap.vd_cap[190] <= 32'h00000000;
            if_ex_cap.vd_cap[191] <= 32'h00000000;
            if_ex_cap.vd_cap[192] <= 32'h0007000A;
            if_ex_cap.vd_cap[193] <= 32'h00000000;
            if_ex_cap.vd_cap[194] <= 32'h00000000;
            if_ex_cap.vd_cap[195] <= 32'h00000000;
            if_ex_cap.vd_cap[196] <= 32'h00000000;
            if_ex_cap.vd_cap[197] <= 32'h00000000;
            if_ex_cap.vd_cap[198] <= 32'h00000000;
            if_ex_cap.vd_cap[199] <= 32'h00000000;
            if_ex_cap.vd_cap[200] <= 32'h00010000;
            if_ex_cap.vd_cap[201] <= 32'h00000000;
            if_ex_cap.vd_cap[202] <= 32'h00000000;
            if_ex_cap.vd_cap[203] <= 32'h00000000;
            if_ex_cap.vd_cap[204] <= 32'h00000000;
            if_ex_cap.vd_cap[205] <= 32'h00000000;
            if_ex_cap.vd_cap[206] <= 32'h00000000;
            if_ex_cap.vd_cap[207] <= 32'h00000000;
        end
    endtask

    task read_port_register;
        begin
            if (rd_port_reg_data == 2'h0) begin //PORTSC
                rd_rsp_data <= port_regs[rd_port_reg_index].portsc;
            end else if (rd_port_reg_data == 2'h1) begin
                rd_rsp_data <= port_regs[rd_port_reg_index].PORTPMSC;
            end else if (rd_port_reg_data == 2'h2) begin
                rd_rsp_data <= port_regs[rd_port_reg_index].PORTLI;
            end else if (rd_port_reg_data == 2'h3) begin
                rd_rsp_data <= port_regs[rd_port_reg_index].PORTHLPMC;
            end
        end
    endtask

    always @ ( posedge clk ) begin
        if (rst) begin
            drd_req_ctx   <= 0;
            drd_req_addr  <= 0;
            drd_req_valid <= 0;

            dwr_addr <= 0;
            dwr_be <= 0;
            dwr_data <= 0;
            dwr_valid <= 0;

            event_ring_send <= 1'b0;
            event_ring_has_request <= 1'b0;
            event_ring_interrupter_index <= 3'h0;
            event_ring_trb_data <= 128'h0;

            command_ring_trigger <= 0;
            command_ring_pointer_update <= 2'b0;

            mf_index_counter <= 13'h0;

            //---------------------------------------------------------
            // Memory Write FIFO TEST
            //---------------------------------------------------------
            mwr_ctrl      <= 32'h0;
            mwr_addr      <= 64'h0;
            mwr_length    <= 32'h0;
            mwr_has_data  <= 1'b0;
            mwr_fifo_in   <= 128'h0;
            mwr_fifo_en   <= 1'b0;
            mwr_fifo_done <= 1'b0;

            //---------------------------------------------------------
            // Memory Read Test Module
            //---------------------------------------------------------
            mrd_ctrl <= 32'h0;
            mrd_addr <= 64'h0;
            mrd_length <= 32'h0;
            mrd_has_request <= 1'b0;
            mrd_fifo_en <= 1'b0;

            for (int i = 0; i < 8; i++) begin
                mrd_data_stack[i] <= 128'h0;
            end

            //---------------------------------------------------------
            // Interrupter Controller (PREFIX: ic_)
            //---------------------------------------------------------
            ic_interrupter_enabled <= 1'b0;
            ic_interrupter_index <= 3'b0;

            //---------------------------------------------------------
            // Device Connect 
            //---------------------------------------------------------
            dc_state        <= DC_STATE_WAIT;
            dc_time_counter <= 32'h0;

            //---------------------------------------------------------
            // DEBUG
            //---------------------------------------------------------

            dbg_have_hit_doorbell <= 1'b0;
            dbg_have_update_cr    <= 1'b0;

            //---------------------------------------------------------
            // Port Reset Handler
            //---------------------------------------------------------

            port_reset_state <= PR_STATE_IDLE;
            port_reset_index <= 5'h0;

            //---------------------------------------------------------
            // Transfer Ring
            //---------------------------------------------------------

            tr_trigger   <= 1'b0;
            tr_slot_id   <= 8'h0;
            tr_ep_id     <= 8'h0;
            tr_stream_id <= 16'h0;

            reset_xhci();
            init_vd_cap();
        end else begin
            drd_req_valid <= rd_req_valid;
            drd_req_ctx  <= rd_req_ctx;
            drd_req_addr <= rd_req_addr;
            rd_rsp_ctx    <= drd_req_ctx;
            rd_rsp_valid  <= drd_req_valid;

            dwr_addr <= wr_addr;
            dwr_be <= wr_be;
            dwr_data <= wr_data;
            dwr_valid <= wr_valid;

            if (set_ep_tr_ptr[0]) begin
                dbg_set_ep_tr_ptr <= set_ep_tr_ptr;
                dbg_status[3] <= 1'b1;
            end

            if (if_op_reg.usb_cmd[0]) begin
                if (mf_index_counter == 13'd7813) begin
                    mf_index_counter <= 13'h0;
                    if (if_rt_reg.mf_index == 32'h3FFF) begin
                        if_rt_reg.mf_index <= 32'h0;
                    end else begin
                        if_rt_reg.mf_index <= if_rt_reg.mf_index + 32'h1;
                    end
                end else begin
                    mf_index_counter <= mf_index_counter + 13'h1;
                end
            end else begin
                if_rt_reg.mf_index <= 32'h0;
            end

            case (command_ring_pointer_update)
                2'b01: begin
                    //check Command Ring is not running
                    command_ring_pointer_update <= 2'b10;
                end
                2'b10: begin
                    dbg_have_update_cr <= 1'b1;
                    command_ring_pointer_update <= 2'b00;
                end
            endcase
            if (command_ring_trigger) begin
                if (command_ring_state != 5'h0) begin
                    command_ring_trigger <= 1'b0;
                end
            end

            if (interrupter_controller_out.set_event_interrupt) begin
                if_op_reg.usb_sts[3] <= 1'b1;
            end

            //---------------------------------------------------------
            // Memory Write FIFO TEST
            //---------------------------------------------------------
            //trigger bit
            if (mwr_ctrl[0]) begin
                mwr_ctrl[0] <= 1'b0;
                mwr_ctrl[1] <= 1'b1;
                //mwr_ctrl[5:3] <= 3'd1;
                mwr_ctrl[21:12] <= 10'd1;
                mwr_ctrl[10] <= 1'b1;
                mwr_has_data <= 1'b1;
            end
            //running bit
            if (mwr_ctrl[1]) begin
                if (mem_wr_out_1.state == WR_DATA_INIT) begin
                    if (mwr_ctrl[21:12] != 10'd0) begin
                        if (mwr_ctrl[21:12] == 10'd35) begin
                            mwr_fifo_en <= 1'b0;
                            mwr_fifo_in <= 128'h0;
                            mwr_fifo_done <= 1'b1;
                            mwr_ctrl[21:12] <= 10'd0;
                        end else begin
                            mwr_fifo_en <= 1'b1;
                            mwr_fifo_in <= { 118'hFFFF_FFFF_FFFF_FFFF_3FFF_FFFF_FFFF_FF, mwr_ctrl[21:12] };
                            mwr_ctrl[21:12] <= mwr_ctrl[21:12] + 10'd1;
                        end
                    end
                end

                if (mem_wr_out_1.state == WR_COMPLETE) begin
                    mwr_ctrl[1] <= 1'b0;
                    mwr_ctrl[2] <= 1'b1;
                    mwr_has_data <= 1'b0;
                end
            end
            mwr_ctrl[9:6] <= mem_wr_out_1.state;

            //---------------------------------------------------------
            // Memory Read TEST
            //---------------------------------------------------------

            if (mrd_ctrl[0]) begin
                mrd_ctrl[0] <= 1'b0;//start
                mrd_ctrl[1] <= 1'b1;//running
                
                mrd_has_request <= 1'b1;
            end
            if (mrd_ctrl[1]) begin
                if (mem_rd_out_3.state == RD_COMPLETE) begin
                    if (!mrd_ctrl[11]) begin //sent delay
                        mrd_fifo_en <= 1'b1;
                        mrd_ctrl[11] <= 1'b1;
                    end else begin
                        mrd_data_stack[mrd_ctrl[9:7]] <= mem_rd_out_3.dout;
                        mrd_ctrl[9:7] <= mrd_ctrl[9:7] + 3'd1;
                        if (mrd_ctrl[9:7] == 3'd7) begin
                            mrd_ctrl[1] <= 1'b0;
                            mrd_ctrl[2] <= 1'b1;
                            mrd_ctrl[11] <= 1'b0;
                            mrd_has_request <= 1'b0;
                            mrd_fifo_en <= 1'b0;
                        end
                    end
                end
            end
            if (mrd_ctrl[10]) begin
                for (int i = 0; i < 8; i++) begin
                    mrd_data_stack[i] <= 128'h0;
                end
            end
            mrd_ctrl[6:3] <= mem_rd_out_3.state;
            
            //---------------------------------------------------------
            // Interrupter Controller (PREFIX: ic_)
            //---------------------------------------------------------
            // Interrupter Enable Bit Clear
            if (ic_interrupter_enabled) begin
                ic_interrupter_enabled <= 1'b0;
                ic_interrupter_index <= 3'b0;
            end

            //---------------------------------------------------------
            // Device Connect
            //---------------------------------------------------------

            case (dc_state)
                DC_STATE_WAIT: begin
                    if (if_op_reg.usb_cmd[0] && if_op_reg.usb_cmd[2]) begin
                        // if (dc_time_counter >= DC_START_CONNECT_COUNT) begin
                        //     dc_time_counter <= 32'h0;
                        //     dc_state <= DC_STATE_START;
                        // end else begin
                        //     dc_time_counter <= dc_time_counter + 32'h1;
                        // end
                    end
                end
                DC_STATE_START: begin
                    if_op_reg.usb_sts[4] <= 1'b1; //Port Change Detect

                    port_regs[0].portsc.CCS     <= 1'b1; //Current Connect Status
                    port_regs[0].portsc.PLS    <= 4'h7; //Port Link Status
                    port_regs[0].portsc.PortSpeed <= 4'h1; //Port Speed
                    port_regs[0].portsc.CSC    <= 1'b1; //Connect Status Change
                    dc_state <= DC_STATE_3;
                end
                // Event Ring 使用競合 対策
                DC_STATE_3: begin
                    if (!event_ring_has_request) begin
                        event_ring_has_request <= 1'b1;
                        dc_state <= DC_STATE_4;
                    end
                end
                // Send Port Status Change Event, PortID: 1, Completion Code: 1
                DC_STATE_4: begin
                    if (event_ring_req_1.ready) begin
                        event_ring_interrupter_index <= 3'h0;
                        event_ring_trb_data[31:0]   <= 32'h01000000;
                        event_ring_trb_data[63:32]  <= 32'h00000000;
                        event_ring_trb_data[95:64]  <= 32'h01000000;
                        event_ring_trb_data[127:96] <= 32'h00008800;
                        event_ring_send <= 1'b1;
                        
                        dc_state <= DC_STATE_5;
                    end
                end
                DC_STATE_5: begin
                    if (event_ring_req_1.complete) begin
                        event_ring_send <= 1'b0;
                        event_ring_has_request <= 1'b0;
                        dc_state <= DC_STATE_6;
                    end
                end
            endcase

            //---------------------------------------------------------
            // Enable Slot Command
            //---------------------------------------------------------

            if (if_enable_slot.enable_slot) begin
                port_regs[enable_slot_index].portsc.PED <= 1'b1; // Port Enabled/Disabled
            end

            //---------------------------------------------------------
            // Port Reset Handle
            //---------------------------------------------------------

            case(port_reset_state)
                PR_STATE_START: begin
                    port_reset_state <= 1'b0;

                    if_op_reg.usb_sts[4] <= 1'b1; //Port Change Detect

                    port_regs[port_reset_index].portsc.PR  <= 1'b0;  //Port Reset
                    port_regs[port_reset_index].portsc.PRC <= 1'b1; //Port Reset Change
                    if (dc_state == DC_STATE_6) begin
                        port_regs[port_reset_index].portsc.PLS <= 4'd0;
                        port_regs[port_reset_index].portsc.PED <= 1'b1;
                        port_regs[port_reset_index].portsc.PortSpeed <= 4'd3;
                        dc_state <= DC_STATE_7;
                    end
                    if (if_slot_ctx.slot_context[port_reset_index[3:2]].slot_state != DISABLE) begin
                        port_regs[port_reset_index].portsc.PED <= 1'b1;
                        port_regs[port_reset_index].portsc.PLS <= 4'd0;
                    end

                    port_reset_state <= PR_STATE_ITR_LOCK;
                end
                // Event Ring 使用競合 対策
                PR_STATE_ITR_LOCK: begin
                    if (!event_ring_has_request) begin
                        event_ring_has_request <= 1'b1;
                        port_reset_state <= PR_STATE_ITR_DATA;
                    end
                end
                // Send Port Status Change Event, PortID: 1, Completion Code: 1
                PR_STATE_ITR_DATA: begin
                    if (event_ring_req_1.ready) begin
                        event_ring_interrupter_index <= 3'h0;
                        event_ring_trb_data[31:0]   <= 32'h01000000;
                        event_ring_trb_data[63:32]  <= 32'h00000000;
                        event_ring_trb_data[95:64]  <= 32'h01000000;
                        event_ring_trb_data[127:96] <= 32'h00008800;
                        event_ring_send <= 1'b1;

                        port_reset_state <= PR_STATE_ITR_WAIT;
                    end
                end
                PR_STATE_ITR_WAIT: begin
                    if (event_ring_req_1.complete) begin
                        event_ring_send <= 1'b0;
                        event_ring_has_request <= 1'b0;

                        port_reset_state <= PR_STATE_IDLE;
                    end
                end
            endcase

            //---------------------------------------------------------
            // Transfer Ring
            //---------------------------------------------------------

            if (tr_trigger) begin
                tr_trigger   <= 1'b0;
                tr_slot_id   <= 8'h0;
                tr_ep_id     <= 8'h0;
                tr_stream_id <= 16'h0;
            end

            //---------------------------------------------------------
            // debug
            //---------------------------------------------------------

            if (dbg_cr_pending) begin
                dbg_status[0] <= 1'b1;
            end
            if (dbg_tr_pending) begin
                dbg_status[1] <= 1'b1;
            end
            if (dbg_ev_pending) begin
                dbg_status[2] <= 1'b1;
            end

            if (drd_req_valid) begin
                case (offset)
                    //eXtensible Host Controller Capability Registers (0x0) RO
                    32'h00000000 : rd_rsp_data <= 32'h01000080; //Capability Register Length
                    32'h00000004 : rd_rsp_data <= 32'h08000820; //Structural Parameters 1
                    32'h00000008 : rd_rsp_data <= 32'hFC000054; //Structural Parameters 2
                    32'h0000000C : rd_rsp_data <= 32'h00040001; //Structural Parameters 3
                    32'h00000010 : rd_rsp_data <= 32'h200071E1; //Capability Parameters 1
                    32'h00000014 : rd_rsp_data <= 32'h00003000; //Doorbell Offset
                    32'h00000018 : rd_rsp_data <= 32'h00002000; //Runtime Register Space Offset
                    32'h0000001C : rd_rsp_data <= 32'h00000000; //Capability Parameters 2
                    //Host Controller Operational Registers (0x80)
                    32'h00000080 : rd_rsp_data <= if_op_reg.usb_cmd;
                    32'h00000084 : rd_rsp_data <= if_op_reg.usb_sts;
                    32'h00000088 : rd_rsp_data <= 32'h00000001; //Page Size Register : RO
                    32'h00000094 : rd_rsp_data <= if_op_reg.dnctrl;
                    32'h00000098 : rd_rsp_data <= if_op_reg.crcr[31:0];
                    32'h0000009C : rd_rsp_data <= if_op_reg.crcr[63:32];
                    32'h000000B0 : rd_rsp_data <= if_op_reg.dcbaap[31:0];
                    32'h000000B4 : rd_rsp_data <= if_op_reg.dcbaap[63:32];
                    32'h000000B8 : rd_rsp_data <= if_op_reg.cfg;
                    32'h00002000 : rd_rsp_data <= if_rt_reg.mf_index;

                    32'h00004000 : rd_rsp_data <= {
                        1'h0,
                        dbg_tr_sub_task_state, //[30:26]
                        dbg_tr_sub_task_type,  //[25:22]
                        dbg_tr_state,          //[21:18]
                        port_reset_state,      //[17:13]
                        dbg_have_update_cr,    //[12]
                        dbg_have_hit_doorbell, //[11]
                        command_ring_state,    //[10:6]
                        dc_state               //[5:0]
                    };
                    32'h00004004 : rd_rsp_data <= dc_time_counter;
                    32'h00004008 : rd_rsp_data <= dbg_status;
                    32'h0000400C : rd_rsp_data <= {
                        24'h0,
                        dbg_ep_id
                    };

                    32'h00004010 : rd_rsp_data <= dbg_cr_ptr[31:0];
                    32'h00004014 : rd_rsp_data <= dbg_cr_ptr[63:32];
                    32'h00004018 : rd_rsp_data <= dbg_tr_ptr[31:0];
                    32'h0000401C : rd_rsp_data <= dbg_tr_ptr[63:32];
                    32'h00004020 : rd_rsp_data <= dbg_ev_ptr[31:0];
                    32'h00004024 : rd_rsp_data <= dbg_ev_ptr[63:32];

                    32'h00004030 : rd_rsp_data <= dbg_set_ep_tr_ptr[31:0];
                    32'h00004034 : rd_rsp_data <= dbg_set_ep_tr_ptr[63:32];
                    32'h00004038 : rd_rsp_data <= {
                        22'h0,
                        dbg_set_ep_tr_ptr[73:64]
                    };

                    //Extended Capability Registers
                    //xHCI Supported Protocol Capability 1
                    32'h00008000 : rd_rsp_data <= 32'h02000802;
                    32'h00008004 : rd_rsp_data <= 32'h20425355;
                    32'h00008008 : rd_rsp_data <= 32'h30190401;
                    32'h0000800C : rd_rsp_data <= 32'h00000000;
                    32'h00008010 : rd_rsp_data <= 32'h000C0021;
                    32'h00008014 : rd_rsp_data <= 32'h05DC0012;
                    32'h00008018 : rd_rsp_data <= 32'h01E00023;
                    32'h0000801C : rd_rsp_data <= 32'h00000000;
                    //xHCI Supported Protocol Capability 2
                    32'h00008020 : rd_rsp_data <= 32'h03000802;
                    32'h00008024 : rd_rsp_data <= 32'h20425355;
                    32'h00008028 : rd_rsp_data <= 32'h10000405;
                    32'h0000802C : rd_rsp_data <= 32'h00000000;
                    32'h00008030 : rd_rsp_data <= 32'h00050134;
                    32'h00008034 : rd_rsp_data <= 32'h00000000;
                    32'h00008038 : rd_rsp_data <= 32'h00000000;
                    32'h0000803C : rd_rsp_data <= 32'h00000000;

                    default : rd_rsp_data <= 32'h0;
                endcase
                if ((32'h20 <= offset) && (offset < 32'h80))
                    rd_rsp_data <= 32'h0;
                else if ((32'h8C <= offset) && (offset < 32'h94))
                    rd_rsp_data <= 32'h0;
                else if ((32'hA0 <= offset) && (offset < 32'hB0))
                    rd_rsp_data <= 32'h0;
                else if ((32'hBC <= offset) && (offset < 32'h480))
                    rd_rsp_data <= 32'h0;
                else if (rd_port_reg)
                    read_port_register();
                else if ((32'h500 <= offset) && (offset < 32'h2000))
                    rd_rsp_data <= 32'h0;
                else if ((32'h2004 <= offset) && (offset < 32'h2020))
                    rd_rsp_data <= 32'h0;
                else if ((32'h2020 <= offset) && (offset < 32'h2120)) //Runtime Registers->Interrupter Register Set[index]
                    case (rd_interrupter_data)
                        3'd0: rd_rsp_data <= {
                                30'h0,
                                if_rt_reg.interrupters[rd_interrupter_index].interrupt_enable,
                                if_rt_reg.interrupters[rd_interrupter_index].interrupt_pending
                            };
                        3'd1: rd_rsp_data <= {
                                if_rt_reg.interrupters[rd_interrupter_index].interrupt_moderation_counter,
                                if_rt_reg.interrupters[rd_interrupter_index].interrupt_moderation_interval
                            };
                        3'd2: rd_rsp_data <= {
                                16'h0,
                                if_rt_reg.interrupters[rd_interrupter_index].event_ring_segment_table_size
                            };
                        3'd3: rd_rsp_data <= 32'h0;
                        3'd4: rd_rsp_data <= {
                                if_rt_reg.interrupters[rd_interrupter_index].event_ring_segment_table_base_address[25:0],
                                6'h0
                            };
                        3'd5: rd_rsp_data <= if_rt_reg.interrupters[rd_interrupter_index].event_ring_segment_table_base_address[57:26];
                        3'd6: rd_rsp_data <= {
                                if_rt_reg.interrupters[rd_interrupter_index].event_ring_dequeue_pointer[27:0],
                                if_rt_reg.interrupters[rd_interrupter_index].event_handler_busy,
                                if_rt_reg.interrupters[rd_interrupter_index].dequeue_erst_segment_index
                            };
                        3'd7: rd_rsp_data <= if_rt_reg.interrupters[rd_interrupter_index].event_ring_dequeue_pointer[59:28];
                    endcase
                else if ((32'h3000 <= offset) && (offset < 32'h3800)) //Doorbell Registears
                    rd_rsp_data <= 32'h0;
                else if ((32'h8040 <= offset) && (offset < 32'h8380))
                    rd_rsp_data <= if_ex_cap.vd_cap[(offset - 32'h8040) / 4];
            end

            if (dwr_valid) begin
                if (wr_offset == 32'h4000) begin
                    if (dwr_data[6]) begin
                        dc_state <= DC_STATE_START;
                    end
                end

                if (wr_offset == 32'h4008) begin
                    if (dwr_data[0]) begin
                        dbg_status[0] <= 1'b0;
                    end
                    if (dwr_data[1]) begin
                        dbg_status[1] <= 1'b0;
                    end
                    if (dwr_data[2]) begin
                        dbg_status[2] <= 1'b0;
                    end
                    if (dwr_data[3]) begin
                        dbg_status[3] <= 1'b0;
                    end
                end

                if (wr_offset == 32'h3000) begin
                    command_ring_trigger <= 1'b1;
                    dbg_have_hit_doorbell <= 1'b1;
                end

                if (wr_offset == 32'h3004) begin
                    tr_trigger   <= 1'b1;
                    tr_slot_id   <= db_index;
                    tr_ep_id     <= dwr_data[7:0];
                    tr_stream_id <= dwr_data[31:16];
                end

                if (wr_offset == 32'h80) begin //USB CMD
                    if ((if_op_reg.usb_cmd[0] == 1'h0) && dwr_data[0]) begin //RUN/STOP 0 => 1
                        if_op_reg.usb_sts[0] <= 1'h0;
                        if_op_reg.usb_cmd[0] <= 1'h1;
                    end else if (if_op_reg.usb_cmd[0] && (dwr_data[0] == 1'h0)) begin //RUN/STOP 1 =>0
                        if_op_reg.usb_sts[0] <= 1'h1;
                        if_op_reg.usb_cmd[0] <= 1'h0;
                    end
                    if ((if_op_reg.usb_cmd[1] == 1'h0) && dwr_data[1]) begin //Host Controller Reset
                        reset_xhci();
                    end
                    if_op_reg.usb_cmd[3:2] <= dwr_data[3:2]; //RW
                    if_op_reg.usb_cmd[7] <= dwr_data[7]; //RW
                    if_op_reg.usb_cmd[11:8] <= dwr_data[11:8]; //RW
                    if_op_reg.usb_cmd[15:13] <= dwr_data[15:13]; //RW
                    if_op_reg.usb_cmd[16] <= dwr_data[16]; //RW
                end else if (wr_offset == 32'h84) begin //USB STATUS
                    //if_op_reg.usb_sts <= (dwr_data & ~32'h1) | (if_op_reg.usb_sts & 32'h1);
                    if (dwr_data[2])//Host System Error(HSE) - RW1C
                        if_op_reg.usb_sts[2] <= 1'h0;
                    if (dwr_data[3])//Event Interrupt(EINT) - RW1C
                        if_op_reg.usb_sts[3] <= 1'h0;
                    if (dwr_data[4])//Port Change Detect(PCD) - RW1C
                        if_op_reg.usb_sts[4] <= 1'h0;
                    if (dwr_data[10])//Save/Restore Error(SRE) RW1C
                        if_op_reg.usb_sts[10] <= 1'h0;
                end else if (wr_offset == 32'h94) begin //Device Notification Control Register
                    if_op_reg.dnctrl[15:0] <= dwr_data[15:0];
                end else if (wr_offset == 32'h98) begin //Command Ring Control Register
                    //if_op_reg.crcr[31:0] <= dwr_data;
                    if_op_reg.crcr[0] <= dwr_data[0];
                    if (dwr_data[1]) //Command Stop - RW1S
                        if_op_reg.crcr[1] <= 1'h1;
                    if (dwr_data[2]) //Command Abort - RW1S
                        if_op_reg.crcr[2] <= 1'h1;
                    if (if_op_reg.crcr[31:6] != dwr_data[31:6]) begin
                        if_op_reg.crcr[31:6] <= dwr_data[31:6]; // Command Ring Pointer - RW
                        command_ring_pointer_update <= 2'b01;
                    end
                end else if (wr_offset == 32'h9C) begin //Command Ring Control Register LOWER
                    if_op_reg.crcr[63:32] <= dwr_data;
                    command_ring_pointer_update <= 2'b01;
                end else if (wr_offset == 32'hB0) begin
                    if_op_reg.dcbaap[31:6] <= dwr_data[31:6];
                end else if (wr_offset == 32'hB4) begin
                    if_op_reg.dcbaap[63:32] <= dwr_data;
                end else if (wr_offset == 32'hB8) begin //Configure Register
                    if_op_reg.cfg[9:0] <= dwr_data[9:0]; //RW
                end else if (wr_port_reg) begin //port registers
                    case (wr_port_reg_data)
                        2'h0: begin // PORTSC
                            if (dwr_data[1]) // Port Enable/Disabled - RW1CS
                                port_regs[wr_port_reg_index].portsc.PED <= 1'b0;
                            if (dwr_data[4]) begin // Port Reset - RW1S
                                //init_port_register(index);
                                port_regs[wr_port_reg_index].portsc.PR <= 1'b1; // Port Reset
                                port_reset_state <= PR_STATE_START;
                                port_reset_index <= wr_port_reg_index;
                            end
                            if (dwr_data[8:5] != 4'd0 && dwr_data[8:5] != 4'd7 && dwr_data[8:5] != 4'd8 && dwr_data[8:5] != 4'd10) begin
                                port_regs[wr_port_reg_index].portsc.PLS <= dwr_data[8:5]; // RW
                            end
                            port_regs[wr_port_reg_index].portsc.PP <= dwr_data[9]; // RW
                            port_regs[wr_port_reg_index].portsc.PIC <= dwr_data[15:14]; // RW
                            port_regs[wr_port_reg_index].portsc.LWS <= dwr_data[16]; // RW
                            if (dwr_data[17]) // Connect Status Change - RW1CS
                                port_regs[wr_port_reg_index].portsc.CSC <= 1'b0;
                            if (dwr_data[18]) // Port Enabled/Disabled Change - RW1CS
                                port_regs[wr_port_reg_index].portsc.PEC <= 1'b0;
                            if (dwr_data[19]) // Warm Port Reset Change - RW1CS/RsvdZ
                                port_regs[wr_port_reg_index].portsc.WRC <= 1'b0;
                            if (dwr_data[20]) // Over-current Change - RW1CS
                                port_regs[wr_port_reg_index].portsc.OCC <= 1'b0;
                            if (dwr_data[21]) // Port Reset Change - RW1CS
                                port_regs[wr_port_reg_index].portsc.PRC <= 1'b0;
                            if (dwr_data[22]) // Port Link State Change - RW1CS
                                port_regs[wr_port_reg_index].portsc.PLC <= 1'b0;
                            if (dwr_data[23]) // Port Config Error Change - RW1CS/RsvdZ
                                port_regs[wr_port_reg_index].portsc.CEC <= 1'b0;
                            port_regs[wr_port_reg_index].portsc.WCE <= dwr_data[25]; // Wake on Connect Enable - RWS
                            port_regs[wr_port_reg_index].portsc.WDE <= dwr_data[26]; // Wake on Disconnect Event - RWS
                            port_regs[wr_port_reg_index].portsc.WOE <= dwr_data[27]; // Wake on Over-current Enable - RWS
                            if (dwr_data[31]) // Warm Port Reset - RW1S/RsvdZ
                                port_regs[wr_port_reg_index].portsc.WPR <= 1'b1;
                        end
                        2'h1: begin // PORTPMSC
                            port_regs[wr_port_reg_index].PORTPMSC <= dwr_data;
                        end
                        2'h2: begin // PORTLI
                            port_regs[wr_port_reg_index].PORTLI <= dwr_data;
                        end
                        2'h3: begin // PORTHLPMC
                            port_regs[wr_port_reg_index].PORTHLPMC <= dwr_data;
                        end
                    endcase
                end else if ((32'h2020 <= wr_offset) && (wr_offset < 32'h2120)) begin //Runtime Registers->Interrupter Register Set[index]
                    case (wr_interrupter_data)
                        3'd0: begin
                            if (dwr_data[0])
                                if_rt_reg.interrupters[wr_interrupter_index].interrupt_pending <= 1'b0;   //Interrupt Pending - RW1C
                            
                            //Interrupt Enable
                            if (!if_rt_reg.interrupters[wr_interrupter_index].interrupt_enable && dwr_data[1]) begin
                                ic_interrupter_enabled <= 1'b1;
                                ic_interrupter_index   <= wr_interrupter_index;
                            end

                            if_rt_reg.interrupters[wr_interrupter_index].interrupt_enable <= dwr_data[1]; //Interrupt Enable  - RW
                        end
                        3'd1: begin
                            if_rt_reg.interrupters[wr_interrupter_index].interrupt_moderation_interval <= dwr_data[15:0];  //Interrupt Moderaion Interval - RW
                            if_rt_reg.interrupters[wr_interrupter_index].interrupt_moderation_counter  <= dwr_data[31:16]; //Interrupt Modearion Counter  - RW
                        end
                        3'd2: begin
                            if_rt_reg.interrupters[wr_interrupter_index].event_ring_segment_table_size <= dwr_data[15:0]; //Event Ring Segment Table Size - RW
                        end
                        3'd4: begin
                            if_rt_reg.interrupters[wr_interrupter_index].event_ring_segment_table_base_address[25:0] <= dwr_data[31:6];
                        end
                        3'd5: begin
                            if_rt_reg.interrupters[wr_interrupter_index].event_ring_segment_table_base_address[57:26] <= dwr_data;
                        end
                        3'd6: begin
                            if_rt_reg.interrupters[wr_interrupter_index].dequeue_erst_segment_index <= dwr_data[2:0];
                            if_rt_reg.interrupters[wr_interrupter_index].event_handler_busy <= dwr_data[3];
                            if_rt_reg.interrupters[wr_interrupter_index].event_ring_dequeue_pointer[27:0] <= dwr_data[31:4];
                        end
                        3'd7: begin
                            if_rt_reg.interrupters[wr_interrupter_index].event_ring_dequeue_pointer[59:28] <= dwr_data;
                        end
                    endcase

                end else if ((32'h3000 <= wr_offset) && (wr_offset < 32'h3020)) begin
                    if_db_reg.db_regs[((wr_offset - 32'h3000) * 8 + 31) : ((wr_offset - 32'h3000) * 8)] <= dwr_data;
                end else if ((32'h8040 <= wr_offset) && (wr_offset < 32'h8380)) begin
                    integer index = (wr_offset - 32'h8040) / 4;
                    if (dwr_be[0]) if_ex_cap.vd_cap[index][7:0]   <= dwr_data[7:0];
                    if (dwr_be[1]) if_ex_cap.vd_cap[index][15:8]  <= dwr_data[15:8];
                    if (dwr_be[2]) if_ex_cap.vd_cap[index][23:16] <= dwr_data[23:16];
                    if (dwr_be[3]) if_ex_cap.vd_cap[index][31:24] <= dwr_data[31:24];
                end 
            end
        end
    end
endmodule

module msix_table_bar2_impl(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,

    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid,

    IfMsiX.source       msix_out
);

    reg [87:0] drd_req_ctx;
    reg [31:0] drd_req_addr;
    reg        drd_req_valid;

    reg [31:0] dwr_addr;
    reg [3:0] dwr_be;
    reg [31:0] dwr_data;
    reg dwr_valid;

    msix_table_entry_t msix_table [8];

    genvar i;
    generate
        for (i = 0; i < 8; i++) begin : GEN_ASSIGN
            assign msix_out.msix_table[i] = msix_table[i];
        end
    endgenerate

    wire [31:0] rd_addr_offset = drd_req_addr & 32'hFFFF;
    wire [2:0] rd_index = rd_addr_offset[6:4];
    wire [1:0] rd_dword = rd_addr_offset[3:2];

    wire [31:0] wr_addr_offset = dwr_addr & 32'hFFFF;
    wire [2:0] wr_index = wr_addr_offset[6:4];
    wire [1:0] wr_dword = wr_addr_offset[3:2];
    
    always @ ( posedge clk ) begin
        if (rst) begin
            drd_req_ctx   <= 0;
            drd_req_addr  <= 0;
            drd_req_valid <= 0;

            dwr_addr <= 0;
            dwr_be <= 0;
            dwr_data <= 0;
            dwr_valid <= 0;
        end else begin
            drd_req_valid <= rd_req_valid;
            drd_req_ctx  <= rd_req_ctx;
            drd_req_addr <= rd_req_addr;
            rd_rsp_ctx    <= drd_req_ctx;
            rd_rsp_valid  <= drd_req_valid;

            dwr_addr <= wr_addr;
            dwr_be <= wr_be;
            dwr_data <= wr_data;
            dwr_valid <= wr_valid;

            if (drd_req_valid) begin
                if (rd_index < 8) begin
                    case (rd_dword)
                        2'd0: rd_rsp_data <= msix_table[rd_index].msg_addr[31:0];
                        2'd1: rd_rsp_data <= msix_table[rd_index].msg_addr[63:32];
                        2'd2: rd_rsp_data <= msix_table[rd_index].msg_data;
                        2'd3: rd_rsp_data <= msix_table[rd_index].vector_ctrl;
                    endcase
                end else begin
                    rd_rsp_data <= 32'h0;
                end
            end
            if (dwr_valid) begin
                if (wr_index < 8) begin
                    for (int i = 0; i < 4; i++) begin
                        if (dwr_be[i]) begin
                            case (wr_dword)
                                2'd0: msix_table[wr_index].msg_addr[31:0] <= dwr_data;
                                2'd1: msix_table[wr_index].msg_addr[63:32] <= dwr_data;
                                2'd2: msix_table[wr_index].msg_data <= dwr_data;
                                2'd3: msix_table[wr_index].vector_ctrl <= dwr_data;
                            endcase
                        end
                    end
                end
            end
        end
    end
endmodule

module msix_pba_bar4_impl(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,

    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);
    reg [87:0] drd_req_ctx;
    reg [31:0] drd_req_addr;
    reg        drd_req_valid;

    reg [31:0] dwr_addr;
    reg [3:0] dwr_be;
    reg [31:0] dwr_data;
    reg dwr_valid;

    bit [31:0] msix_pba [3:0];

    wire [31:0] rd_offset = drd_req_addr & 32'hFFFF;
    wire [31:0] wr_offset = dwr_addr & 32'hFFFF;

    always @ ( posedge clk ) begin
        if (rst) begin
            drd_req_ctx   <= 0;
            drd_req_addr  <= 0;
            drd_req_valid <= 0;

            dwr_addr <= 0;
            dwr_be <= 0;
            dwr_data <= 0;
            dwr_valid <= 0;
        end else begin
            drd_req_valid <= rd_req_valid;
            drd_req_ctx  <= rd_req_ctx;
            drd_req_addr <= rd_req_addr;
            rd_rsp_ctx    <= drd_req_ctx;
            rd_rsp_valid  <= drd_req_valid;

            dwr_addr <= wr_addr;
            dwr_be <= wr_be;
            dwr_data <= wr_data;
            dwr_valid <= wr_valid;

            if (drd_req_valid) begin
                if (rd_offset < 32'h10) begin
                    integer index = rd_offset / 4;
                    rd_rsp_data <= msix_pba[index];
                end else begin
                    rd_rsp_data <= 32'h0;
                end
            end
            if (dwr_valid) begin
                if (wr_offset < 32'h10) begin
                    integer index = wr_offset / 4;
                    if (dwr_be[0]) msix_pba[index][7:0] <= dwr_data[7:0];
                    if (dwr_be[1]) msix_pba[index][15:8] <= dwr_data[15:8];
                    if (dwr_be[2]) msix_pba[index][23:16] <= dwr_data[23:16];
                    if (dwr_be[3]) msix_pba[index][31:24] <= dwr_data[31:24];
                end
            end
        end
    end
endmodule