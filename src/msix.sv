`timescale 1ns / 1ps
`include "pcileech_header.svh"

module msix(
    input                 rst,
    input                 clk_pcie,
    input [15:0]          pcie_id,
    IfMsiXRequest.sink    msix_req_in,
    IfMsiX.sink           msix_in,
    IfMemoryWrite.source write_out
);

    localparam IDLE            = 3'd1;
    localparam WRITE_DATA_FIFO = 3'd2;
    localparam WAIT_WRITE_CPLT = 3'd3;
    localparam COMPLETE        = 3'd4;
    reg [2:0] state;

    assign msix_req_in.ready = state == IDLE;
    assign msix_req_in.complete = state == COMPLETE;

    //---------------------------------------------------------
    // Memory Write
    //---------------------------------------------------------

    reg [63:0] mwr_addr;
    reg [31:0] mwr_length;
    reg        mwr_has_data;

    reg [127:0] mwr_fifo_in;
    reg         mwr_fifo_en;
    reg         mwr_fifo_done;

    assign write_out.address     = mwr_addr;
    assign write_out.data_length = mwr_length;
    assign write_out.has_data    = mwr_has_data;
    assign write_out.din         = mwr_fifo_in;
    assign write_out.wr_en       = mwr_fifo_en;
    assign write_out.wr_done     = mwr_fifo_done;

    always @(posedge clk_pcie) begin
        if (rst) begin
            state <= IDLE;
            
            mwr_addr      <= 64'h0;
            mwr_length    <= 32'h0;
            mwr_has_data  <= 1'b0;
            mwr_fifo_in   <= 128'h0;
            mwr_fifo_en   <= 1'b0;
            mwr_fifo_done <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    if (msix_req_in.fire) begin
                        mwr_addr     <= msix_in.msix_table[msix_req_in.interrupter_index].msg_addr;
                        mwr_length   <= 32'h4;
                        mwr_has_data <= 1'b1;

                        state <= WRITE_DATA_FIFO;
                    end
                end
                WRITE_DATA_FIFO: begin
                    if (write_out.state == WR_DATA_INIT) begin
                        mwr_fifo_en <= 1'b1;
                        mwr_fifo_in <= { 96'h0, msix_in.msix_table[msix_req_in.interrupter_index].msg_data };
                        state <= WAIT_WRITE_CPLT;
                    end
                end
                WAIT_WRITE_CPLT: begin
                    if (write_out.state == WR_COMPLETE) begin
                        mwr_addr      <= 64'h0;
                        mwr_length    <= 32'h0;
                        mwr_has_data  <= 1'b0;
                        mwr_fifo_done <= 1'b0;

                        state <= COMPLETE;
                    end else begin
                        mwr_fifo_en <= 1'b0;
                        mwr_fifo_in <= 128'h0;
                        mwr_fifo_done <= 1'b1;
                    end
                end
                COMPLETE: begin
                    if (!msix_req_in.fire) begin
                        state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule

module msix_mux(

);

endmodule