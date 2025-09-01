//
// PCILeech FPGA.
//
// SystemVerilog Header File for PCILeech FPGA projects.
//
// (c) Ulf Frisk, 2019-2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`ifndef _pcileech_header_svh_
`define _pcileech_header_svh_

`define _bs16(v)   {{v}[7:0], {v}[15:8]}
`define _bs32(v)   {{v}[7:0], {v}[15:8], {v}[23:16], {v}[31:24]}

// ------------------------------------------------------------------------
// Interface connecting COM to FIFO module.
// ------------------------------------------------------------------------
interface IfComToFifo;
    wire [63:0]     com_dout;
    wire            com_dout_valid;
    wire [255:0]    com_din;
    wire            com_din_wr_en;
    wire            com_din_ready;

    modport mp_com (
        output com_dout, com_dout_valid, com_din_ready,
        input com_din, com_din_wr_en
    );

    modport mp_fifo (
        input com_dout, com_dout_valid, com_din_ready,
        output com_din, com_din_wr_en
    );
endinterface

// ------------------------------------------------------------------------
// Interface connecting PCIe to PCIe CFG module.
// ------------------------------------------------------------------------
interface IfPCIeSignals;
    // ------------------------------------------------------------------------
    // VALUES FROM PCIe TO module.
    // ------------------------------------------------------------------------
    wire    [7:0]       cfg_bus_number;
    wire    [4:0]       cfg_device_number;
    wire    [2:0]       cfg_function_number;
    
    wire    [15:0]      cfg_command;
    wire    [31:0]      cfg_mgmt_do;
    wire                cfg_mgmt_rd_wr_done;
    
    wire    [2:0]       pl_initial_link_width;
    wire                pl_phy_lnk_up;
    wire    [1:0]       pl_lane_reversal_mode;
    wire                pl_link_gen2_cap;
    wire                pl_link_partner_gen2_supported;
    wire                pl_link_upcfg_cap;
    wire                pl_sel_lnk_rate;
    wire    [1:0]       pl_sel_lnk_width;
    wire    [5:0]       pl_ltssm_state;
    wire    [1:0]       pl_rx_pm_state;
    wire    [2:0]       pl_tx_pm_state;
    wire                pl_directed_change_done;
    wire                pl_received_hot_rst;
    
    wire                cfg_aer_rooterr_corr_err_received;
    wire                cfg_aer_rooterr_corr_err_reporting_en;
    wire                cfg_aer_rooterr_fatal_err_received;
    wire                cfg_aer_rooterr_fatal_err_reporting_en;
    wire                cfg_aer_rooterr_non_fatal_err_received;
    wire                cfg_aer_rooterr_non_fatal_err_reporting_en;
    wire                cfg_bridge_serr_en;
    wire    [15:0]      cfg_dcommand;
    wire    [15:0]      cfg_dcommand2;
    wire    [15:0]      cfg_dstatus;
    wire    [15:0]      cfg_lcommand;
    wire    [15:0]      cfg_lstatus;
    wire    [2:0]       cfg_pcie_link_state;
    wire                cfg_pmcsr_pme_en;
    wire                cfg_pmcsr_pme_status;
    wire    [1:0]       cfg_pmcsr_powerstate;
    wire                cfg_received_func_lvl_rst;
    wire                cfg_root_control_pme_int_en;
    wire                cfg_root_control_syserr_corr_err_en;
    wire                cfg_root_control_syserr_fatal_err_en;
    wire                cfg_root_control_syserr_non_fatal_err_en;
    wire                cfg_slot_control_electromech_il_ctl_pulse;
    wire    [15:0]      cfg_status;
    wire                cfg_to_turnoff;
    wire    [5:0]       tx_buf_av;
    wire                tx_cfg_req;
    wire                tx_err_drop;
    wire    [6:0]       cfg_vc_tcvc_map;
    
    wire    [2:0]       cfg_interrupt_mmenable;
    wire                cfg_interrupt_msienable;
    wire                cfg_interrupt_msixenable;
    wire                cfg_interrupt_msixfm;
    wire                cfg_interrupt_rdy;
    wire    [7:0]       cfg_interrupt_do;
    
    // ------------------------------------------------------------------------
    // VALUES FROM module TO PCIe.
    // ------------------------------------------------------------------------
    
    wire                cfg_mgmt_rd_en;
    wire                cfg_mgmt_wr_en;
    
    wire    [63:0]      cfg_dsn;
    wire    [31:0]      cfg_mgmt_di;
    wire    [9:0]       cfg_mgmt_dwaddr;
    wire    [3:0]       cfg_mgmt_byte_en;
    wire                cfg_mgmt_wr_readonly;
    wire                cfg_mgmt_wr_rw1c_as_rw;
    
    wire    [1:0]       pl_directed_link_change;
    wire    [1:0]       pl_directed_link_width;
    wire                pl_directed_link_auton;
    wire                pl_directed_link_speed;
    wire                pl_upstream_prefer_deemph;
    wire                pl_transmit_hot_rst;
    wire                pl_downstream_deemph_source;
    
    wire    [7:0]       cfg_interrupt_di;
    wire    [4:0]       cfg_pciecap_interrupt_msgnum;
    wire                cfg_interrupt_assert;
    wire                cfg_interrupt;
    wire                cfg_interrupt_stat;
    
    wire    [1:0]       cfg_pm_force_state;
    wire                cfg_pm_force_state_en;
    wire                cfg_pm_halt_aspm_l0s;
    wire                cfg_pm_halt_aspm_l1;
    wire                cfg_pm_send_pme_to;
    wire                cfg_pm_wake;
    wire                cfg_trn_pending;
    wire                cfg_turnoff_ok;
    wire                rx_np_ok;
    wire                rx_np_req;
    wire                tx_cfg_gnt;
    
    modport mpm (
        input cfg_bus_number, cfg_device_number, cfg_function_number, cfg_command, cfg_mgmt_do, cfg_mgmt_rd_wr_done,
            pl_initial_link_width, pl_phy_lnk_up, pl_lane_reversal_mode, pl_link_gen2_cap, pl_link_partner_gen2_supported,
            pl_link_upcfg_cap, pl_sel_lnk_rate, pl_sel_lnk_width, pl_ltssm_state, pl_rx_pm_state,
            pl_tx_pm_state, pl_directed_change_done, pl_received_hot_rst,
            cfg_aer_rooterr_corr_err_received, cfg_aer_rooterr_corr_err_reporting_en, cfg_aer_rooterr_fatal_err_received, cfg_aer_rooterr_fatal_err_reporting_en,
            cfg_aer_rooterr_non_fatal_err_received, cfg_aer_rooterr_non_fatal_err_reporting_en, cfg_bridge_serr_en, cfg_dcommand, cfg_dcommand2, cfg_dstatus,
            cfg_lcommand, cfg_lstatus, cfg_pcie_link_state, cfg_pmcsr_pme_en, cfg_pmcsr_pme_status, cfg_pmcsr_powerstate, cfg_received_func_lvl_rst, cfg_root_control_pme_int_en,
            cfg_root_control_syserr_corr_err_en, cfg_root_control_syserr_fatal_err_en, cfg_root_control_syserr_non_fatal_err_en, cfg_slot_control_electromech_il_ctl_pulse,
            cfg_status, cfg_to_turnoff, tx_buf_av, tx_cfg_req, tx_err_drop, cfg_vc_tcvc_map,
            cfg_interrupt_mmenable, cfg_interrupt_msienable, cfg_interrupt_msixenable, cfg_interrupt_msixfm, cfg_interrupt_rdy, cfg_interrupt_do,
            
        output cfg_mgmt_rd_en, cfg_mgmt_wr_en, cfg_dsn, cfg_mgmt_di, cfg_mgmt_dwaddr, cfg_mgmt_wr_readonly, cfg_mgmt_wr_rw1c_as_rw, cfg_mgmt_byte_en, pl_directed_link_change, pl_directed_link_width, pl_directed_link_auton,
            pl_directed_link_speed, pl_upstream_prefer_deemph, pl_transmit_hot_rst, pl_downstream_deemph_source,
            cfg_interrupt_di, cfg_pciecap_interrupt_msgnum, cfg_interrupt_assert, cfg_interrupt, cfg_interrupt_stat, cfg_pm_force_state, cfg_pm_force_state_en, cfg_pm_halt_aspm_l0s,
            cfg_pm_halt_aspm_l1, cfg_pm_send_pme_to, cfg_pm_wake, cfg_trn_pending, cfg_turnoff_ok, rx_np_ok, rx_np_req, tx_cfg_gnt
    );
endinterface

// ------------------------------------------------------------------------
// Interface PCIe 128-bit RX stream
// ------------------------------------------------------------------------

interface IfAXIS128;
    wire [127:0]    tdata;
    wire [3:0]      tkeepdw;
    wire            tvalid;
    wire            tlast;
    wire [8:0]      tuser;      // [0] = first
                                // [1] = last
                                // [8:2] = BAR, 2=BAR0, 3=BAR1, .. 7=BAR5, 8=EXPROM
    
    wire            tready;
    wire            has_data;
    
    modport source(
        input  tready,
        output tdata, tkeepdw, tvalid, tlast, tuser, has_data
    );
    
    modport sink(
        output tready,
        input  tdata, tkeepdw, tvalid, tlast, tuser, has_data
    );
    
    modport source_lite(
        output tdata, tkeepdw, tvalid, tlast, tuser
    );
    
    modport sink_lite(
        input  tdata, tkeepdw, tvalid, tlast, tuser
    );
endinterface

// ------------------------------------------------------------------------
// Interface connecting PCIe CFG to FIFO
// ------------------------------------------------------------------------
interface IfPCIeFifoCfg;
    wire    [63:0]      tx_data;
    wire                tx_valid;
    wire    [31:0]      rx_data;
    wire                rx_valid;
    wire                rx_rd_en;

    modport mp_fifo (
        output tx_data, tx_valid, rx_rd_en,
        input rx_data, rx_valid
    );

    modport mp_pcie (
        input tx_data, tx_valid, rx_rd_en,
        output rx_data, rx_valid
    );
endinterface

// ------------------------------------------------------------------------
// Interface connecting PCIe TLP to FIFO
// ------------------------------------------------------------------------
interface IfPCIeFifoTlp;
    wire    [31:0]      tx_data;
    wire                tx_last;
    wire                tx_valid;   
    wire    [31:0]      rx_data[4];
    wire                rx_first[4];
    wire                rx_last[4];
    wire                rx_valid[4];
    wire                rx_rd_en;

    modport mp_fifo (
        output tx_data, tx_last, tx_valid, rx_rd_en,
        input rx_data, rx_first, rx_last, rx_valid
    );

    modport mp_pcie (
        input tx_data, tx_last, tx_valid, rx_rd_en,
        output rx_data, rx_first, rx_last, rx_valid
    );
endinterface

// ------------------------------------------------------------------------
// Interface connecting PCIe CORE config to FIFO
// ------------------------------------------------------------------------
interface IfPCIeFifoCore;
    // PCIe optional config
    wire                pcie_rst_core;
    wire                pcie_rst_subsys;
    // DRP config
    wire                drp_rdy;
    wire    [15:0]      drp_do;
    wire                drp_en;
    wire                drp_we;
    wire    [8:0]       drp_addr;
    wire    [15:0]      drp_di;
    
    modport mp_fifo (
        input drp_rdy, drp_do,
        output pcie_rst_core, pcie_rst_subsys, drp_en, drp_we, drp_addr, drp_di
    );

    modport mp_pcie (
        input pcie_rst_core, pcie_rst_subsys, drp_en, drp_we, drp_addr, drp_di,
        output drp_rdy, drp_do
    );
endinterface

interface IfShadow2Fifo;
    // SHADOW CONFIGURATION SPACE TO FIFO
    wire                rx_rden;
    wire                rx_wren;
    wire    [3:0]       rx_be;
    wire    [31:0]      rx_data;
    wire    [9:0]       rx_addr;
    wire                rx_addr_lo;
    wire                tx_valid;
    wire    [31:0]      tx_data;
    wire    [9:0]       tx_addr;
    wire                tx_addr_lo;
    wire                cfgtlp_wren;
    wire                cfgtlp_zero;
    wire                cfgtlp_en;
    wire                cfgtlp_filter;
    wire                alltlp_filter;
    wire                bar_en;
    
    modport fifo (
        output cfgtlp_wren, cfgtlp_zero, rx_rden, rx_wren, rx_be, rx_addr, rx_addr_lo, rx_data, cfgtlp_en, cfgtlp_filter, alltlp_filter, bar_en,
        input tx_valid, tx_addr, tx_addr_lo, tx_data
    );

    modport shadow (
        input cfgtlp_wren, cfgtlp_zero, rx_rden, rx_wren, rx_be, rx_addr, rx_addr_lo, rx_data, cfgtlp_en, cfgtlp_filter, alltlp_filter, bar_en,
        output tx_valid, tx_addr, tx_addr_lo, tx_data
    );
endinterface

// ------------------------------------------------------------------------
// Interface PCIe AXI RX / TX
// ------------------------------------------------------------------------
interface IfPCIeTlpRxTx;
    wire    [63:0]      data;
    wire    [7:0]       keep;
    wire                last;
    wire    [21:0]      user;
    wire                valid;
    wire                ready;
    
    modport source (
        output data, keep, last, user, valid,
        input ready
    );
    
    modport sink (
        input data, keep, last, user, valid,
        output ready
    );
endinterface

interface IfPCIeTlpRx128;
    wire    [127:0]     data;
    wire    [21:0]      user;
    wire                valid;
    wire                ready;
    
    modport source (
        output data, user, valid,
        input ready
    );
    
    modport sink (
        input data, user, valid,
        output ready
    );
endinterface

typedef enum logic[3:0] {
    WR_IDLE,
    WR_DATA_INIT,
    WR_CALC_DATA,
    WR_PREPARE_HEADER,
    WR_TRANSMIT_HEADER,
    WR_DEASSERT_HEADER,
    WR_GET_DATA_FIFO,
    WR_WAIT_DATA_FIFO,
    WR_PREPARE_DATA,
    WR_TRANSMIT_DATA,
    WR_DEASSERT_DATA,
    WR_COMPLETE,
    WR_CLEANUP
} memory_write_state_t;

interface IfMemoryWrite;
    // control
    logic [63:0] address;
    logic [31:0] data_length;
    logic        has_data;

    // write data fifo transfer
    logic [127:0] din;
    logic         wr_en;
    logic         wr_done;

    memory_write_state_t state;

    modport source (
        input  state,
        output address, data_length, has_data, din, wr_en, wr_done
    );

    modport sink (
        output state,
        input  address, data_length, has_data, din, wr_en, wr_done
    );
endinterface

typedef enum logic[3:0] {
    RD_IDLE,
    RD_CALC_DATA,
    RD_PREPARE_HEADER,
    RD_TRANSMIT_HEADER,
    RD_WAIT_CPLT,
    RD_GET_DATA,
    RD_PACK_LAST,
    RD_COMPLETE,
    RD_CLEANUP
} memory_read_state_t;

interface IfMemoryRead;
    // control
    logic [63:0] address;
    logic [31:0] data_length;
    logic        has_request;

    // read data fifo transfer
    logic [127:0] dout;
    logic         rd_en;

    memory_read_state_t state;

    modport source (
        input  state, dout,
        output address, data_length, has_request, rd_en//, rd_done
    );

    modport sink (
        output state, dout,
        input  address, data_length, has_request, rd_en//, rd_done
    );
endinterface

interface xHCI_OpReg;
    reg [31:0] usb_cmd; //USB Command
    reg [31:0] usb_sts; //USB Status
    reg [15:0] dnctrl; //Device Notification Control Register
    reg [63:0] crcr; //Command Ring Control Register
    reg [63:0] dcbaap; //Device Context Base Address Array Pointer Register
    reg [31:0] cfg; //Config Register
    //reg [511:0] port_regs; //Host Controaller USB Port Register Set(4) 
    //reg [31:0] port_regs [31:0]; //Host Controaller USB Port Register Set(4) 
endinterface

typedef struct packed {
    //0x00: Interrupter Management Register
    logic        interrupt_pending;
    logic        interrupt_enable;
    //0x04: Interrupter Moderation Register
    logic [15:0] interrupt_moderation_interval;
    logic [15:0] interrupt_moderation_counter;
    //0x08: Event Ring Segment Table Size Register
    logic [15:0] event_ring_segment_table_size;
    //0x0C: Reserved
    //0x10: Event Ring Segment Table Base Address Register
    logic [57:0] event_ring_segment_table_base_address;
    //0x18: Event Ring Dequeue Pointer Register
    logic [2:0]  dequeue_erst_segment_index;
    logic        event_handler_busy;
    logic [59:0] event_ring_dequeue_pointer;
} interrupter_register_t;

interface xHCI_RtReg;
    logic [31:0] mf_index;
    interrupter_register_t interrupters [8];

    modport sink (
        input mf_index, interrupters
    );
endinterface

interface xHCI_ExCap;
    reg [31:0] vd_cap [207:0];
endinterface

typedef struct packed {
    logic [63:0] msg_addr;
    logic [31:0] msg_data;
    logic [31:0] vector_ctrl;
} msix_table_entry_t;

interface IfMsiX;
    msix_table_entry_t msix_table [8];
    
    modport source (
        output msix_table
    );

    modport sink (
        input msix_table
    );
endinterface

interface IfMsiXRequest;
    wire fire;
    wire [2:0] interrupter_index;

    wire ready;
    wire complete;

    modport source (
        output fire, interrupter_index,
        input ready, complete
    );

    modport sink (
        input fire, interrupter_index,
        output ready, complete
    );
endinterface

interface IfEventRingRequest;
    wire         send;
    wire [2:0]   interrupter_index;
    wire [127:0] trb_data;
    wire         has_request;
    
    wire [3:0] state;
    wire ready;
    wire complete;

    modport source (
        output send, interrupter_index, trb_data, has_request,
        input state, ready, complete
    );

    modport sink (
        input send, interrupter_index, trb_data, has_request,
        output state, ready, complete
    );
endinterface

interface IfInterrupterController;
    
    // Interrupter Enable
    wire       interrupter_enabled;
    wire [2:0] interrupter_index;

    // Set USBSTS Event Interrupt
    wire        set_event_interrupt;

    modport source (
        output interrupter_enabled, interrupter_index,
        input set_event_interrupt
    );

    modport sink (
        input interrupter_enabled, interrupter_index,
        output set_event_interrupt
    );
endinterface

interface IfEnableSlot;

    logic       enable_slot;
    logic [2:0] target_index;
    
    modport source (
        output enable_slot, target_index
    );

    modport sink (
        input  enable_slot, target_index
    );
endinterface

typedef struct packed {
    logic [63:0] p_tr_dequeue; //Transfer Ring Dequeue Pointer
    logic        cycle_state;  //Transfer Ring Cycle State
} endpoint_context_t;

typedef enum logic [4:0] { DISABLE=0, ENABLE=1, DEFAULT=2, ADDRESSED=3, CONFIGURED=4 } slot_state_t;

typedef struct packed {
    logic [63:0] p_ctx;            //Slot Context Pointer
    logic [7:0]  slot_id;          //Slot ID
    logic        slot_enabled;     //is Slot Enabled by Enable Slot Command
    slot_state_t slot_state;       //Slot State
} slot_context_t;

interface IfSlotContext;
    slot_context_t slot_context [8]; //slot 8個
    endpoint_context_t ep_ctx[248];

    modport source (
        output slot_context, ep_ctx
    );

    modport sink (
        input slot_context, ep_ctx
    );
endinterface

interface IfSetEPState;
    logic       has_request;
    logic [2:0] slot_id;
    logic [4:0] ep_id;
    logic [2:0] state;
    logic       done;

    modport source (
        output has_request, slot_id, ep_id, state,
        input  done
    );

    modport sink (
        input  has_request, slot_id, ep_id, state,
        output done
    );
endinterface

interface IfDebugEventRing;
    logic        pending_request;
    logic [63:0] ptr;
    logic        status;

    modport source (
        output pending_request, ptr,
        input  status
    );

    modport sink (
        input  pending_request, ptr,
        output status
    );
endinterface

// ------------------------------------------------------------------------
// xHCI PORTSC (Port Status and Control) Register Structure
// See xHCI Spec 4.19.2 (Port Status and Control Register (PORTSC))
// ------------------------------------------------------------------------
typedef struct packed {
    logic           WPR;    // [31] Warm Port Reset (RW1S)
    logic           DR;     // [30] Device Removable (RO)
    logic [1:0]     Rsvd_1; // [29:28] Reserved
    logic           WOE;    // [27] Wake on Over-current Enable (RWS)
    logic           WDE;    // [26] Wake on Disconnect Enable (RWS)
    logic           WCE;    // [25] Wake on Connect Enable (RWS)
    logic           CAS;    // [24] Cold Attach Status (RO)
    logic           CEC;    // [23] Port Config Error Change (RW1C/RsvdZ)
    logic           PLC;    // [22] Port Link State Change (RW1C)
    logic           PRC;    // [21] Port Reset Change (RW1C)
    logic           OCC;    // [20] Over-current Change (RW1C)
    logic           WRC;    // [19] Warm Port Reset Change (RW1C/RsvdZ)
    logic           PEC;    // [18] Port Enabled/Disabled Change (RW1C)
    logic           CSC;    // [17] Connect Status Change (RW1C)
    logic           LWS;    // [16] Port Link State Write Strobe (RW)
    logic [1:0]     PIC;    // [15:14] Port Indicator Control (RWS)
    logic [3:0]     PortSpeed; // [13:10] Port Speed (ROS)
    logic           PP;     // [9] Port Power (RWS)
    logic [3:0]     PLS;    // [8:5] Port Link State (RWS)
    logic           PR;     // [4] Port Reset (RW1S)
    logic           OCA;    // [3] Over-current Active (RO)
    logic           Rsvd_0; // [2] Reserved
    logic           PED;    // [1] Port Enabled/Disabled (RW1C)
    logic           CCS;    // [0] Current Connect Status (RO)
} portsc_reg_t;

typedef struct packed {
    portsc_reg_t portsc;
    logic [31:0] PORTPMSC;  // Port Power Management Status and Control
    logic [31:0] PORTLI;    // Port Link Info
    logic [31:0] PORTHLPMC; // Port Hardware LPM Control
} port_reg_t;

typedef struct packed {
    logic        valid; //is exists in map
    logic [7:0]  bram_addr; //index of data BRAM
    logic [15:0] data_length; //length of data in bytes
} tr_ctx_lookup_t;

typedef struct packed {
    logic [31:0] max_wait_time;
    logic [31:0] wait_time;
    logic        waiting;

    logic        received;
    logic [63:0] trb_ptr;
    logic [63:0] buffer_pointer;
    logic [16:0] transfer_length;
    logic [4:0]  td_size;
    logic [9:0]  interrupter_target;
    logic        chain_bit;
    logic        interrupt_on_short;
    logic        interrupt_on_cplt;
    logic        immediate_data;
} tr_ctx_normal_t;

typedef struct packed {
    logic [63:0] event_data;
    logic [9:0]  interrupter_target;
} tr_ctx_event_t;

typedef struct packed {
    tr_ctx_lookup_t lookup;
    tr_ctx_normal_t normal;
    tr_ctx_event_t  evt;
} tr_ctx_t;

package settings;
    localparam [31:0] OFFSET_OP_REG  = 32'h0080;
    localparam [31:0] OFFSET_RT_REG  = 32'h2000;
    localparam [31:0] OFFSET_DB      = 32'h3000;
    localparam [31:0] OFFSET_EXT_CAP = 32'h8000;

    localparam [15:0] HCI_VERSION_NUM  = 16'h0100;
    localparam [7:0]  NUM_DEVICE_SLOTS =  8'h20;
    localparam [10:0] NUM_INTERRUPTERS = 11'd8;
    localparam [7:0]  NUM_PORTS        =  8'd8;

    localparam [31:0] OFFSET_PORT_REG_START = OFFSET_OP_REG + 32'h400;
    localparam [31:0] OFFSET_PORT_REG_END   = OFFSET_OP_REG + 32'h400 + 32'h20 * NUM_PORTS;
endpackage

`endif