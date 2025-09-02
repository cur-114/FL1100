//
// PCILeech FPGA.
//
// PCIe controller module - TLP handling for Artix-7.
//
// (c) Ulf Frisk, 2018-2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_pcie_tlp_a7(
    input                   rst,
    input                   clk_pcie,
    input                   clk_sys,
    IfPCIeFifoTlp.mp_pcie   dfifo,
    
    // PCIe core receive/transmit data
    IfAXIS128.source        tlps_tx,
    IfAXIS128.sink_lite     tlps_rx,
    IfAXIS128.sink          tlps_static,
    IfShadow2Fifo.shadow    dshadow2fifo,
    input [15:0]            pcie_id
    );
    
    IfAXIS128 tlps_bar_rsp();
    IfAXIS128 tlps_cfg_rsp();

    IfAXIS128 tlps_mem_wr();
    IfAXIS128 tlps_mem_rd();

    xHCI_RtReg if_rt_reg();

    // ------------------------------------------------------------------------
    // Memory Write Module
    // ------------------------------------------------------------------------

    IfMemoryWrite mem_wr_root(); //Root
    IfMemoryWrite mem_wr_test(); //Test Module
    IfMemoryWrite mem_wr_evtr(); //Event Ring
    IfMemoryWrite mem_wr_cmdr(); //Command Ring
    IfMemoryWrite mem_wr_msix(); //MSI-X
    IfMemoryWrite mem_wr_trsr(); //Transfer Ring
    IfMemoryWrite mem_wr_set_ep_state(); //Set Endpoint State

    host_mem_write_2 i_host_mem_write_2(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .pcie_id        ( pcie_id                       ),
        .mem_wr_in      ( mem_wr_root.sink              ),
        .tlps_out       ( tlps_mem_wr.source            )
    );

    host_mem_write_2_mux i_host_mem_write_2_mux(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .mux_source     ( mem_wr_root.source            ),
        .wr_in_1        ( mem_wr_test.sink              ),
        .wr_in_2        ( mem_wr_evtr.sink              ),
        .wr_in_3        ( mem_wr_cmdr.sink              ),
        .wr_in_4        ( mem_wr_msix.sink              ),
        .wr_in_5        ( mem_wr_trsr.sink              ),
        .wr_in_6        ( mem_wr_set_ep_state.sink      )
    );

    // ------------------------------------------------------------------------
    // Memory Read Module
    // ------------------------------------------------------------------------

    IfMemoryRead mem_rd_root(); //Root
    IfMemoryRead mem_rd_uptr(); //Update Transfer Ring Dequeue Pointer
    IfMemoryRead mem_rd_evtr(); //Event Ring
    IfMemoryRead mem_rd_cmdr(); //Command Ring
    IfMemoryRead mem_rd_test(); //Test Module
    IfMemoryRead mem_rd_trsr(); //Transfer Ring
    IfMemoryRead mem_rd_set_ep_state(); //Set Endpoint State

    host_mem_read_2 i_host_mem_read_2(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .pcie_id        ( pcie_id                       ),
        .mem_rd_in      ( mem_rd_root.sink              ),
        .tlps_out       ( tlps_mem_rd.source            ),
        .tlps_in        ( tlps_rx                       )
    );

    host_mem_read_2_mux i_host_mem_read_2_mux(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .mux_source     ( mem_rd_root.source            ),
        .rd_in_1        ( mem_rd_uptr.sink              ),
        .rd_in_2        ( mem_rd_evtr.sink              ),
        .rd_in_3        ( mem_rd_cmdr.sink              ),
        .rd_in_4        ( mem_rd_test.sink              ),
        .rd_in_5        ( mem_rd_trsr.sink              ),
        .rd_in_6        ( mem_rd_set_ep_state.sink      )
    );

    // ------------------------------------------------------------------------
    // MSI-X interrupt:
    // ------------------------------------------------------------------------

    IfMsiX        if_msix();
    IfMsiXRequest msix_request();

    msix i_msix(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .pcie_id        ( pcie_id                       ),
        .msix_req_in    ( msix_request.sink             ),
        .msix_in        ( if_msix.sink                  ),
        .write_out      ( mem_wr_msix.source            )
    );

    // ------------------------------------------------------------------------
    // Event Ring:
    // ------------------------------------------------------------------------

    IfEventRingRequest event_ring_req_root();
    IfEventRingRequest event_ring_req_1();
    IfEventRingRequest event_ring_req_2();
    IfEventRingRequest event_ring_req_3(); //Transfer Ring
    IfInterrupterController if_interrupter_controller();
    wire enable_event_interrupt_bit;
    IfDebugEventRing if_dbg_event_ring();

    event_ring i_event_ring(
        .rst               ( rst                            ),
        .clk_pcie          ( clk_pcie                       ),
        .rt_reg_in         ( if_rt_reg.sink                 ),
        .event_ring_req_in ( event_ring_req_root.sink       ),
        .write_out         ( mem_wr_evtr.source             ),
        .read_out          ( mem_rd_evtr.source             ),
        .msix_req_out      ( msix_request.source            ),
        .interrupter_controller_in ( if_interrupter_controller.sink ),
        .dbg_out           ( if_dbg_event_ring.source       )
    );

    event_ring_mux i_event_ring_mux(
        .rst               ( rst                            ),
        .clk_pcie          ( clk_pcie                       ),
        .in_1              ( event_ring_req_1.sink          ),
        .in_2              ( event_ring_req_2.sink          ),
        .in_3              ( event_ring_req_3.sink          ),
        .out               ( event_ring_req_root.source     )
    );

    // ------------------------------------------------------------------------
    // Convert received TLPs from PCIe core and transmit onwards:
    // ------------------------------------------------------------------------
    IfAXIS128 tlps_filtered();
    
    pcileech_tlps128_bar_controller i_pcileech_tlps128_bar_controller(
        .rst                ( rst                           ),
        .clk                ( clk_pcie                      ),
        .bar_en             ( dshadow2fifo.bar_en           ),
        .pcie_id            ( pcie_id                       ),
        .tlps_in            ( tlps_rx                       ),
        .tlps_out           ( tlps_bar_rsp.source           ),
        .msix_out           ( if_msix.source                ),
        .if_rt_reg          ( if_rt_reg                     ),
        .event_ring_req_1   ( event_ring_req_1.source       ),
        .event_ring_req_2   ( event_ring_req_2.source       ),
        .event_ring_req_3   ( event_ring_req_3.source       ),
        .mem_wr_out_1       ( mem_wr_test.source            ),
        .mem_wr_out_2       ( mem_wr_cmdr.source            ),
        .mem_wr_out_3       ( mem_wr_trsr.source            ),
        .mem_wr_out_4       ( mem_wr_set_ep_state.source    ),
        .mem_rd_out_1       ( mem_rd_uptr.source            ),
        .mem_rd_out_2       ( mem_rd_cmdr.source            ),
        .mem_rd_out_3       ( mem_rd_test.source            ),
        .mem_rd_out_4       ( mem_rd_trsr.source            ),
        .mem_rd_out_5       ( mem_rd_set_ep_state.source    ),
        .interrupter_controller_out ( if_interrupter_controller.source ),
        .dbg_evt_ring       ( if_dbg_event_ring.sink        )
    );
    
    pcileech_tlps128_cfgspace_shadow i_pcileech_tlps128_cfgspace_shadow(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .clk_sys        ( clk_sys                       ),
        .tlps_in        ( tlps_rx                       ),
        .pcie_id        ( pcie_id                       ),
        .dshadow2fifo   ( dshadow2fifo                  ),
        .tlps_cfg_rsp   ( tlps_cfg_rsp.source           )
    );
    
    pcileech_tlps128_filter i_pcileech_tlps128_filter(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .alltlp_filter  ( dshadow2fifo.alltlp_filter    ),
        .cfgtlp_filter  ( dshadow2fifo.cfgtlp_filter    ),
        .tlps_in        ( tlps_rx                       ),
        .tlps_out       ( tlps_filtered.source_lite     )
    );
    
    pcileech_tlps128_dst_fifo i_pcileech_tlps128_dst_fifo(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .clk_sys        ( clk_sys                       ),
        .tlps_in        ( tlps_filtered.sink_lite       ),
        .dfifo          ( dfifo                         )
    );
    
    // ------------------------------------------------------------------------
    // TX data received from FIFO
    // ------------------------------------------------------------------------
    IfAXIS128 tlps_rx_fifo();
    
    pcileech_tlps128_src_fifo i_pcileech_tlps128_src_fifo(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .clk_sys        ( clk_sys                       ),
        .dfifo_tx_data  ( dfifo.tx_data                 ),
        .dfifo_tx_last  ( dfifo.tx_last                 ),
        .dfifo_tx_valid ( dfifo.tx_valid                ),
        .tlps_out       ( tlps_rx_fifo.source           )
    );
    
    pcileech_tlps128_sink_mux1 i_pcileech_tlps128_sink_mux1(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .tlps_out       ( tlps_tx                       ),
        .tlps_in1       ( tlps_cfg_rsp.sink             ),
        .tlps_in2       ( tlps_bar_rsp.sink             ),
        .tlps_in3       ( tlps_rx_fifo.sink             ),
        .tlps_in4       ( tlps_static                   ),
        .tlps_in5       ( tlps_mem_wr.sink              ),
        .tlps_in6       ( tlps_mem_rd.sink              )
    );

endmodule



// ------------------------------------------------------------------------
// TLP-AXI-STREAM destination:
// Forward the data to output device (FT601, etc.). 
// ------------------------------------------------------------------------
module pcileech_tlps128_dst_fifo(
    input                   rst,
    input                   clk_pcie,
    input                   clk_sys,
    IfAXIS128.sink_lite     tlps_in,
    IfPCIeFifoTlp.mp_pcie   dfifo
);
    
    wire         tvalid;
    wire [127:0] tdata;
    wire [3:0]   tkeepdw;
    wire         tlast;
    wire         first;
       
    fifo_134_134_clk2 i_fifo_134_134_clk2 (
        .rst        ( rst               ),
        .wr_clk     ( clk_pcie          ),
        .rd_clk     ( clk_sys           ),
        .din        ( { tlps_in.tuser[0], tlps_in.tlast, tlps_in.tkeepdw, tlps_in.tdata } ),
        .wr_en      ( tlps_in.tvalid    ),
        .rd_en      ( dfifo.rx_rd_en    ),
        .dout       ( { first, tlast, tkeepdw, tdata } ),
        .full       (                   ),
        .empty      (                   ),
        .valid      ( tvalid            )
    );

    assign dfifo.rx_data[0]  = tdata[31:0];
    assign dfifo.rx_data[1]  = tdata[63:32];
    assign dfifo.rx_data[2]  = tdata[95:64];
    assign dfifo.rx_data[3]  = tdata[127:96];
    assign dfifo.rx_first[0] = first;
    assign dfifo.rx_first[1] = 0;
    assign dfifo.rx_first[2] = 0;
    assign dfifo.rx_first[3] = 0;
    assign dfifo.rx_last[0]  = tlast && (tkeepdw == 4'b0001);
    assign dfifo.rx_last[1]  = tlast && (tkeepdw == 4'b0011);
    assign dfifo.rx_last[2]  = tlast && (tkeepdw == 4'b0111);
    assign dfifo.rx_last[3]  = tlast && (tkeepdw == 4'b1111);
    assign dfifo.rx_valid[0] = tvalid && tkeepdw[0];
    assign dfifo.rx_valid[1] = tvalid && tkeepdw[1];
    assign dfifo.rx_valid[2] = tvalid && tkeepdw[2];
    assign dfifo.rx_valid[3] = tvalid && tkeepdw[3];

endmodule



// ------------------------------------------------------------------------
// TLP-AXI-STREAM FILTER:
// Filter away certain packet types such as CfgRd/CfgWr or non-Cpl/CplD
// ------------------------------------------------------------------------
module pcileech_tlps128_filter(
    input                   rst,
    input                   clk_pcie,
    input                   alltlp_filter,
    input                   cfgtlp_filter,
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source_lite   tlps_out
);

    bit [127:0]     tdata;
    bit [3:0]       tkeepdw;
    bit             tvalid  = 0;
    bit [8:0]       tuser;
    bit             tlast;
    
    assign tlps_out.tdata   = tdata;
    assign tlps_out.tkeepdw = tkeepdw;
    assign tlps_out.tvalid  = tvalid;
    assign tlps_out.tuser   = tuser;
    assign tlps_out.tlast   = tlast;
    
    bit  filter = 0;
    wire first = tlps_in.tuser[0];
    wire is_tlphdr_cpl = first && (
                        (tlps_in.tdata[31:25] == 7'b0000101) ||      // Cpl:  Fmt[2:0]=000b (3 DW header, no data), Cpl=0101xb
                        (tlps_in.tdata[31:25] == 7'b0100101)         // CplD: Fmt[2:0]=010b (3 DW header, data),    CplD=0101xb
                      );
    wire is_tlphdr_cfg = first && (
                        (tlps_in.tdata[31:25] == 7'b0000010) ||      // CfgRd: Fmt[2:0]=000b (3 DW header, no data), CfgRd0/CfgRd1=0010xb
                        (tlps_in.tdata[31:25] == 7'b0100010)         // CfgWr: Fmt[2:0]=010b (3 DW header, data),    CfgWr0/CfgWr1=0010xb
                      );
    wire filter_next = (filter && !first) || (cfgtlp_filter && first && is_tlphdr_cfg) || (alltlp_filter && first && !is_tlphdr_cpl && !is_tlphdr_cfg);
                      
    always @ ( posedge clk_pcie ) begin
        tdata   <= tlps_in.tdata;
        tkeepdw <= tlps_in.tkeepdw;
        tvalid  <= tlps_in.tvalid && !filter_next && !rst;
        tuser   <= tlps_in.tuser;
        tlast   <= tlps_in.tlast;
        filter  <= filter_next && !rst;
    end
    
endmodule



// ------------------------------------------------------------------------
// RX FROM FIFO - TLP-AXI-STREAM:
// Convert 32-bit incoming data to 128-bit TLP-AXI-STREAM to be sent onwards to mux/pcie core. 
// ------------------------------------------------------------------------
module pcileech_tlps128_src_fifo (
    input                   rst,
    input                   clk_pcie,
    input                   clk_sys,
    input [31:0]            dfifo_tx_data,
    input                   dfifo_tx_last,
    input                   dfifo_tx_valid,
    IfAXIS128.source        tlps_out
);

    // 1: 32-bit -> 128-bit state machine:
    bit [127:0] tdata;
    bit [3:0]   tkeepdw = 0;
    bit         tlast;
    bit         first   = 1;
    wire        tvalid  = tlast || tkeepdw[3];
    
    always @ ( posedge clk_sys )
        if ( rst ) begin
            tkeepdw <= 0;
            tlast   <= 0;
            first   <= 1;
        end
        else begin
            tlast   <= dfifo_tx_valid && dfifo_tx_last;
            tkeepdw <= tvalid ? (dfifo_tx_valid ? 4'b0001 : 4'b0000) : (dfifo_tx_valid ? ((tkeepdw << 1) | 1'b1) : tkeepdw);
            first   <= tvalid ? tlast : first;
            if ( dfifo_tx_valid ) begin
                if ( tvalid || !tkeepdw[0] )
                    tdata[31:0]   <= dfifo_tx_data;
                if ( !tkeepdw[1] )
                    tdata[63:32]  <= dfifo_tx_data;
                if ( !tkeepdw[2] )
                    tdata[95:64]  <= dfifo_tx_data;
                if ( !tkeepdw[3] )
                    tdata[127:96] <= dfifo_tx_data;   
            end
        end
		
    // 2.1 - packet count (w/ safe fifo clock-crossing).
    bit [10:0]  pkt_count       = 0;
    wire        pkt_count_dec   = tlps_out.tvalid && tlps_out.tlast;
    wire        pkt_count_inc;
    wire [10:0] pkt_count_next  = pkt_count + pkt_count_inc - pkt_count_dec;
    assign tlps_out.has_data    = (pkt_count_next > 0);
    
    fifo_1_1_clk2 i_fifo_1_1_clk2(
        .rst            ( rst                       ),
        .wr_clk         ( clk_sys                   ),
        .rd_clk         ( clk_pcie                  ),
        .din            ( 1'b1                      ),
        .wr_en          ( tvalid && tlast           ),
        .rd_en          ( 1'b1                      ),
        .dout           (                           ),
        .full           (                           ),
        .empty          (                           ),
        .valid          ( pkt_count_inc             )
    );
	
    always @ ( posedge clk_pcie ) begin
        pkt_count <= rst ? 0 : pkt_count_next;
    end
        
    // 2.2 - submit to output fifo - will feed into mux/pcie core.
    //       together with 2.1 this will form a low-latency "packet fifo".
    fifo_134_134_clk2_rxfifo i_fifo_134_134_clk2_rxfifo(
        .rst            ( rst                       ),
        .wr_clk         ( clk_sys                   ),
        .rd_clk         ( clk_pcie                  ),
        .din            ( { first, tlast, tkeepdw, tdata } ),
        .wr_en          ( tvalid                    ),
        .rd_en          ( tlps_out.tready && (pkt_count_next > 0) ),
        .dout           ( { tlps_out.tuser[0], tlps_out.tlast, tlps_out.tkeepdw, tlps_out.tdata } ),
        .full           (                           ),
        .empty          (                           ),
        .valid          ( tlps_out.tvalid           )
    );

endmodule



// ------------------------------------------------------------------------
// RX MUX - TLP-AXI-STREAM:
// Select the TLP-AXI-STREAM with the highest priority (lowest number) and
// let it transmit its full packet.
// Each incoming stream must have latency of 1CLK. 
// ------------------------------------------------------------------------
module pcileech_tlps128_sink_mux1 (
    input                       clk_pcie,
    input                       rst,
    IfAXIS128.source            tlps_out,
    IfAXIS128.sink              tlps_in1,
    IfAXIS128.sink              tlps_in2,
    IfAXIS128.sink              tlps_in3,
    IfAXIS128.sink              tlps_in4,
    IfAXIS128.sink              tlps_in5,
    IfAXIS128.sink              tlps_in6
);
    bit [3:0] id = 0;
    
    assign tlps_out.has_data    = tlps_in1.has_data ||
                                  tlps_in2.has_data ||
                                  tlps_in3.has_data ||
                                  tlps_in4.has_data ||
                                  tlps_in5.has_data ||
                                  tlps_in6.has_data;
    
    assign tlps_out.tdata       = (id==1) ? tlps_in1.tdata :
                                  (id==2) ? tlps_in2.tdata :
                                  (id==3) ? tlps_in3.tdata :
                                  (id==4) ? tlps_in4.tdata : 
                                  (id==5) ? tlps_in5.tdata :
                                  (id==6) ? tlps_in6.tdata : 0;
    
    assign tlps_out.tkeepdw     = (id==1) ? tlps_in1.tkeepdw :
                                  (id==2) ? tlps_in2.tkeepdw :
                                  (id==3) ? tlps_in3.tkeepdw :
                                  (id==4) ? tlps_in4.tkeepdw : 
                                  (id==5) ? tlps_in5.tkeepdw :
                                  (id==6) ? tlps_in6.tkeepdw : 0;
    
    assign tlps_out.tlast       = (id==1) ? tlps_in1.tlast :
                                  (id==2) ? tlps_in2.tlast :
                                  (id==3) ? tlps_in3.tlast :
                                  (id==4) ? tlps_in4.tlast : 
                                  (id==5) ? tlps_in5.tlast :
                                  (id==6) ? tlps_in6.tlast : 0;
    
    assign tlps_out.tuser       = (id==1) ? tlps_in1.tuser :
                                  (id==2) ? tlps_in2.tuser :
                                  (id==3) ? tlps_in3.tuser :
                                  (id==4) ? tlps_in4.tuser :
                                  (id==5) ? tlps_in5.tuser :
                                  (id==6) ? tlps_in6.tuser : 0;
    
    assign tlps_out.tvalid      = (id==1) ? tlps_in1.tvalid :
                                  (id==2) ? tlps_in2.tvalid :
                                  (id==3) ? tlps_in3.tvalid :
                                  (id==4) ? tlps_in4.tvalid :
                                  (id==5) ? tlps_in5.tvalid :
                                  (id==6) ? tlps_in6.tvalid : 0;
    
    wire [3:0] id_next_newsel   = tlps_in1.has_data ? 1 :
                                  tlps_in2.has_data ? 2 :
                                  tlps_in3.has_data ? 3 :
                                  tlps_in4.has_data ? 4 : 
                                  tlps_in5.has_data ? 5 :
                                  tlps_in6.has_data ? 6 : 0;
    
    wire [3:0] id_next          = ((id==0) || (tlps_out.tvalid && tlps_out.tlast)) ? id_next_newsel : id;
    
    assign tlps_in1.tready      = tlps_out.tready && (id_next==1);
    assign tlps_in2.tready      = tlps_out.tready && (id_next==2);
    assign tlps_in3.tready      = tlps_out.tready && (id_next==3);
    assign tlps_in4.tready      = tlps_out.tready && (id_next==4);
    assign tlps_in5.tready      = tlps_out.tready && (id_next==5);
    assign tlps_in6.tready      = tlps_out.tready && (id_next==6);
    
    always @ ( posedge clk_pcie ) begin
        id <= rst ? 0 : id_next;
    end
    
endmodule
