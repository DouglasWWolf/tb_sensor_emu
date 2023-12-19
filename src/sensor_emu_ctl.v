//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 25-Oct-23  DWW     1  Initial creation
//====================================================================================

/*
    The purpose of this module is to stream out byte patterns that have been recorded  
    in a FIFO.   There are two input FIFOs, and only one at a time can be streaming out
    its contents.

    Whenever an entry is extracted from a FIFO it is written back into that FIFO so that
    the vector of values in the FIFO can be re-used in a continuous loop.
*/


module sensor_emu_ctl #
(
    // This must be 8, 16, 32, or 64
    parameter PATTERN_WIDTH = 32    
)
(
    input clk, resetn,

    //================== This is an AXI4-Lite slave interface ==================
        
    // "Specify write address"              -- Master --    -- Slave --
    input[31:0]                             S_AXI_AWADDR,   
    input                                   S_AXI_AWVALID,  
    output                                                  S_AXI_AWREADY,
    input[2:0]                              S_AXI_AWPROT,

    // "Write Data"                         -- Master --    -- Slave --
    input[31:0]                             S_AXI_WDATA,      
    input                                   S_AXI_WVALID,
    input[3:0]                              S_AXI_WSTRB,
    output                                                  S_AXI_WREADY,

    // "Send Write Response"                -- Master --    -- Slave --
    output[1:0]                                             S_AXI_BRESP,
    output                                                  S_AXI_BVALID,
    input                                   S_AXI_BREADY,

    // "Specify read address"               -- Master --    -- Slave --
    input[31:0]                             S_AXI_ARADDR,     
    input                                   S_AXI_ARVALID,
    input[2:0]                              S_AXI_ARPROT,     
    output                                                  S_AXI_ARREADY,

    // "Read data back to master"           -- Master --    -- Slave --
    output[31:0]                                            S_AXI_RDATA,
    output                                                  S_AXI_RVALID,
    output[1:0]                                             S_AXI_RRESP,
    input                                   S_AXI_RREADY,
    //==========================================================================
 

    //=========================   The output stream   ==========================
    output [PATTERN_WIDTH-1:0] AXIS_OUT_TDATA,
    output                     AXIS_OUT_TVALID,
    input                      AXIS_OUT_TREADY
    //==========================================================================
);  

    // Any time the register map of this module changes, this number should be bumped
    localparam MODULE_VERSION = 1;

    //=========================  AXI Register Map  =============================
    localparam REG_MODULE_REV       = 0;  /*  RO   */
    localparam REG_FIFO_CTL         = 1;
        localparam BIT_F0_RESET = 0;      /*  R/W  */
        localparam BIT_F1_RESET = 1;      /*  R/W  */
    
    localparam REG_UPPER32          = 2;  /*  R/W  */
    localparam REG_LOAD_F0          = 3;  /*  R/W  */
    localparam REG_LOAD_F1          = 4;  /*  R/W  */
    
    localparam REG_START            = 5;            
        localparam BIT_F0_START = 0;      /*  R/W  */
        localparam BIT_F1_START = 1;      /*  R/W  */

    localparam REG_HARD_STOP        = 6;  /*  R/W  */
    //==========================================================================


    //==========================================================================
    // We'll communicate with the AXI4-Lite Slave core with these signals.
    //==========================================================================
    // AXI Slave Handler Interface for write requests
    wire[31:0]  ashi_waddr;     // Input:  Write-address
    wire[31:0]  ashi_windx;     // Input:  Write register-index
    wire[31:0]  ashi_wdata;     // Input:  Write-data
    wire        ashi_write;     // Input:  1 = Handle a write request
    reg[1:0]    ashi_wresp;     // Output: Write-response (OKAY, DECERR, SLVERR)
    wire        ashi_widle;     // Output: 1 = Write state machine is idle

    // AXI Slave Handler Interface for read requests
    wire[31:0]  ashi_raddr;     // Input:  Read-address
    wire[31:0]  ashi_rindx;     // Input:  Read register-index
    wire        ashi_read;      // Input:  1 = Handle a read request
    reg[31:0]   ashi_rdata;     // Output: Read data
    reg[1:0]    ashi_rresp;     // Output: Read-response (OKAY, DECERR, SLVERR);
    wire        ashi_ridle;     // Output: 1 = Read state machine is idle
    //==========================================================================

    // The state of the state-machines that handle AXI4-Lite read and AXI4-Lite write
    reg[3:0] ashi_write_state, ashi_read_state;

    // The AXI4 slave state machines are idle when in state 0 and their "start" signals are low
    assign ashi_widle = (ashi_write == 0) && (ashi_write_state == 0);
    assign ashi_ridle = (ashi_read  == 0) && (ashi_read_state  == 0);
   
    // These are the valid values for ashi_rresp and ashi_wresp
    localparam OKAY   = 0;
    localparam SLVERR = 2;
    localparam DECERR = 3;

    // An AXI slave is gauranteed a minimum of 128 bytes of address space
    // (128 bytes is 32 32-bit registers)
    localparam ADDR_MASK = 7'h7F;

    // When one of these counters is non-zero, the associated FIFO is held in reset
    reg[3:0] f0_reset_counter, f1_reset_counter;

    // These two wires control the reset input of the FIFOs
    wire f0_reset = (resetn == 0 || f0_reset_counter != 0);
    wire f1_reset = (resetn == 0 || f1_reset_counter != 0);

    // These signals are the input bus to fifo_0
    reg[PATTERN_WIDTH-1:0]  f0in_tdata;
    reg                     f0in_tvalid;
    wire                    f0in_tready;

    // These signals are the output bus of fifo_0
    wire[PATTERN_WIDTH-1:0] f0out_tdata;
    wire                    f0out_tvalid;
    wire                    f0out_tready;

    // These signals are the input bus to fifo_1
    reg[PATTERN_WIDTH-1:0]  f1in_tdata;
    reg                     f1in_tvalid;
    wire                    f1in_tready;

    // These signals are the output bus of fifo_1
    wire[PATTERN_WIDTH-1:0] f1out_tdata;
    wire                    f1out_tvalid;
    wire                    f1out_tready;

    // The number of data elements stored in each FIFO
    reg[14:0] f0_count, f1_count;

    // This is the value that will be loaded into a FIFO
    reg[63:0] input_value;

    // When one of these bits are strobed high, "input value" is loaded into that FIFO
    reg[1:0] fifo_load_strobe;

    // These bits indicate which FIFO will be output next
    reg[1:0] fifo_on_deck;

    // These bits indicate which FIFO is actively outputting data
    reg[1:0] active_fifo;

    // If this strobes high, the output-state-machine immediately goes idle
    reg hard_stop;

    // If we have an active FIFO, the frame generator can do its thing
    assign AXIS_OUT_TVALID = (active_fifo != 0);
    
    //==========================================================================
    // This state machine handles AXI4-Lite write requests
    //
    // Drives:
    //   f0_reset_counter (and therefore, f0_reset)
    //   f1_reset_counter (and therefore, f1_reset)
    //   f0_count and f1_count
    //   fifo_load_strobe
    //   input_value
    //   fifo_on_deck   
    //   hard_stop
    //==========================================================================
    always @(posedge clk) begin

        // The reset counters for the two FIFOs always count down to zero
        if (f0_reset_counter) f0_reset_counter <= f0_reset_counter - 1;
        if (f1_reset_counter) f1_reset_counter <= f1_reset_counter - 1;

        // When one of these bit strobes high, "input_value" is loaded into a FIFO
        fifo_load_strobe <= 0;

        // This only strobes high for a single cycle at a time
        hard_stop <= 0;

        // If we're in reset, initialize important registers
        if (resetn == 0) begin
            ashi_write_state <= 0;
            f0_reset_counter <= 0;
            f1_reset_counter <= 0;
            f0_count         <= 0;
            f1_count         <= 0;
            fifo_on_deck     <= 0;

        // If we're not in reset, and a write-request has occured...        
        end else case (ashi_write_state)
        
        0:  if (ashi_write) begin
       
                // Assume for the moment that the result will be OKAY
                ashi_wresp <= OKAY;              
            
                // Examine the register index to determine which register to write to
                case (ashi_windx)
                
                    REG_FIFO_CTL:
                        begin
                           
                            // If the user wants to clear fifo_0...
                            if (ashi_wdata[BIT_F0_RESET] && ~active_fifo[0]) begin
                                f0_count         <= 0;
                                f0_reset_counter <= -1;
                                ashi_write_state <= 1;    
                            end
                            
                            // If the user wants to clear fifo_1...
                            if (ashi_wdata[BIT_F1_RESET] && ~active_fifo[1]) begin
                                f1_count         <= 0;
                                f1_reset_counter <= -1;
                                ashi_write_state <= 1;
                            end   
                        
                        end

                    // Is the user storing the upper 32-bits of an input value?
                    REG_UPPER32:
                        input_value[63:32]<= ashi_wdata;

                    // Is the user loading an entry into fifo_0?
                    REG_LOAD_F0:
                        if (~active_fifo[0]) begin
                            input_value[31:0] <= ashi_wdata;
                            fifo_load_strobe  <= 1;
                            f0_count          <= f0_count + 1;
                        end

                    // Is the user loading an entry into fifo_1?
                    REG_LOAD_F1:
                        if (~active_fifo[1]) begin
                            input_value[31:0] <= ashi_wdata;
                            fifo_load_strobe  <= 2;
                            f1_count          <= f1_count + 1;
                        end


                    // Don't allow the user to attempt to start both FIFOs at once
                    REG_START:
                        if (ashi_wdata[1:0] == 2'b00) begin
                            fifo_on_deck <= 0;
                        end
                        
                        else if (ashi_wdata[1:0] == 2'b01 && f0_count) begin
                            fifo_on_deck <= 1;
                        end
                        
                        else if (ashi_wdata[1:0] == 2'b10 && f1_count) begin
                            fifo_on_deck <= 2;
                        end

                    // Allow the user to say "return the output to idle mode immediately"
                    REG_HARD_STOP:
                        begin
                            fifo_on_deck <= 0;
                            hard_stop    <= 1;
                        end
                    
                    // Writes to any other register are a decode-error
                    default: ashi_wresp <= DECERR;
                endcase
            end

        // In this state, we're just waiting for the FIFO reset counters 
        // to both go back to zero
        1:  if (f0_reset_counter == 0 && f1_reset_counter == 0)
                ashi_write_state <= 0;

        endcase
    end
    //==========================================================================





    //==========================================================================
    // World's simplest state machine for handling AXI4-Lite read requests
    //==========================================================================
    always @(posedge clk) begin

        // If we're in reset, initialize important registers
        if (resetn == 0) begin
            ashi_read_state <= 0;
        
        // If we're not in reset, and a read-request has occured...        
        end else if (ashi_read) begin
       
            // Assume for the moment that the result will be OKAY
            ashi_rresp <= OKAY;              
            
            // Examine the register index to determine what the user is trying to read
            case (ashi_rindx)
 
                // Allow a read from any valid register                
                REG_MODULE_REV:       ashi_rdata <= MODULE_VERSION;
                REG_FIFO_CTL:         ashi_rdata <= {f1_reset, f0_reset};
                REG_UPPER32:          ashi_rdata <= input_value[63:32];
                REG_LOAD_F0:          ashi_rdata <= f0_count;
                REG_LOAD_F1:          ashi_rdata <= f1_count;
                REG_START:            ashi_rdata <= active_fifo;
                REG_HARD_STOP:        ashi_rdata <= 0;

                // Reads of any other register are a decode-error
                default: ashi_rresp <= DECERR;
            endcase
        end
    end
    //==========================================================================



    //====================================================================================
    // This state machine controls the inputs to the FIFOs
    //
    // Drives:
    //    f0in_tvalid (TVALID line for input to fifo_0)
    //    f1in_tvalid (TVALID line for input to fifo_1)
    //    f0in_tdata  (TDATA lines for input to fifo_0)
    //    f1in_tdata  (TDATA lines for input to fifo_1)
    //====================================================================================
    always @(posedge clk) begin
        
        // By default, we're not writing to either FIFO
        f0in_tvalid <= 0;
        f1in_tvalid <= 0;

        // If F0 load_strobe is high, we write the input value to fif0_0
        if (fifo_load_strobe[0]) begin
            f0in_tdata  <= input_value;
            f0in_tvalid <= 1;
        end

        // If F1 load_strobe is high, we write the input value to fif0_1
        if (fifo_load_strobe[1]) begin
            f1in_tdata  <= input_value;
            f1in_tvalid <= 1;
        end

        // When an entry is output from fifo_0, feed it back to the input
        if (f0out_tvalid & f0out_tready) begin
            f0in_tdata  <= f0out_tdata;
            f0in_tvalid <= 1;
        end

        // When an entry is output from fifo_1, feed it back to the input
        if (f1out_tvalid & f1out_tready) begin
            f1in_tdata  <= f1out_tdata;
            f1in_tvalid <= 1;
        end
    end
    //====================================================================================



    //====================================================================================
    // This state machine moves data from a FIFO to the output stream
    //
    // Drives:
    //  AXIS_OUT_TDATA
    //  AXIS_OUT_TVALID
    //  active_fifo
    //  f0out_tready
    //  f1out_tready
    //
    // "osm" means "output state machine"
    //
    // When reading this state machine, keep in mind that when AXIS_OUT_TVALID is low,
    // it means we're not outputting any data to the output stream and we're waiting to
    // be told which FIFO to output from
    //====================================================================================
    reg       osm_state;
    reg[15:0] osm_counter;

    // The data being driven out on AXIS_OUT_TDATA is the output of one of the FIFOs
    assign AXIS_OUT_TDATA = (active_fifo == 1) ? f0out_tdata :
                            (active_fifo == 2) ? f1out_tdata : 8'h55;

    assign f0out_tready = (active_fifo == 1) ? AXIS_OUT_TREADY : 0;
    assign f1out_tready = (active_fifo == 2) ? AXIS_OUT_TREADY : 0;                                
    //====================================================================================
    always @(posedge clk) begin

        if (resetn == 0) begin
            osm_state   <= 0; 
            active_fifo <= 0;
        
        end else case(osm_state)

            
            0:  begin
                    active_fifo <= 0;
                    osm_state   <= 1;
                end
    
            1:  // If we've been told to return to idle...               
                if (hard_stop) begin
                    active_fifo <= 0;
                end 

                // If we're waiting for a start command or this data-cycle is a handshake on AXIS_OUT...
                else if (active_fifo == 0 || (AXIS_OUT_TVALID & AXIS_OUT_TREADY)) begin
                    
                    // Don't forget that this doesn't take effect until the end of the cycle!
                    osm_counter <= osm_counter - 1;

                    // If we're either waiting for a "start" or if we've output the entire FIFO already...
                    if (active_fifo == 0 || osm_counter == 1) begin

                        // If we've been told to start outputting from fifo_0...    
                        if (fifo_on_deck == 1) begin
                            active_fifo <= 1;
                            osm_counter <= f0_count;
                        end else

                        // If we've been told to start outputting from fifo_1....
                        if (fifo_on_deck == 2) begin
                            active_fifo <= 2;
                            osm_counter <= f1_count;
                        end else

                        begin
                            active_fifo <= 0;
                        end
                    end
                end
        endcase

    end
    //====================================================================================




    //==========================================================================
    // This connects us to an AXI4-Lite slave core
    //==========================================================================
    axi4_lite_slave # (ADDR_MASK) axil_slave
    (
        .clk            (clk),
        .resetn         (resetn),
        
        // AXI AW channel
        .AXI_AWADDR     (S_AXI_AWADDR),
        .AXI_AWVALID    (S_AXI_AWVALID),   
        .AXI_AWREADY    (S_AXI_AWREADY),
        
        // AXI W channel
        .AXI_WDATA      (S_AXI_WDATA),
        .AXI_WVALID     (S_AXI_WVALID),
        .AXI_WSTRB      (S_AXI_WSTRB),
        .AXI_WREADY     (S_AXI_WREADY),

        // AXI B channel
        .AXI_BRESP      (S_AXI_BRESP),
        .AXI_BVALID     (S_AXI_BVALID),
        .AXI_BREADY     (S_AXI_BREADY),

        // AXI AR channel
        .AXI_ARADDR     (S_AXI_ARADDR), 
        .AXI_ARVALID    (S_AXI_ARVALID),
        .AXI_ARREADY    (S_AXI_ARREADY),

        // AXI R channel
        .AXI_RDATA      (S_AXI_RDATA),
        .AXI_RVALID     (S_AXI_RVALID),
        .AXI_RRESP      (S_AXI_RRESP),
        .AXI_RREADY     (S_AXI_RREADY),

        // ASHI write-request registers
        .ASHI_WADDR     (ashi_waddr),
        .ASHI_WINDX     (ashi_windx),
        .ASHI_WDATA     (ashi_wdata),
        .ASHI_WRITE     (ashi_write),
        .ASHI_WRESP     (ashi_wresp),
        .ASHI_WIDLE     (ashi_widle),

        // ASHI read registers
        .ASHI_RADDR     (ashi_raddr),
        .ASHI_RINDX     (ashi_rindx),
        .ASHI_RDATA     (ashi_rdata),
        .ASHI_READ      (ashi_read ),
        .ASHI_RRESP     (ashi_rresp),
        .ASHI_RIDLE     (ashi_ridle)
    );
    //==========================================================================



//====================================================================================
// This FIFO holds a vector of cell patterns
//====================================================================================
xpm_fifo_axis #
(
   .FIFO_DEPTH(8192),              // DECIMAL
   .TDATA_WIDTH(PATTERN_WIDTH),    // DECIMAL
   .FIFO_MEMORY_TYPE("auto"),      // String
   .PACKET_FIFO("false"),          // String
   .USE_ADV_FEATURES("0000")       // String
)
fifo_0
(
    // Clock and reset
   .s_aclk   (clk   ),
   .m_aclk   (clk   ),
   .s_aresetn(~f0_reset),

    // The input bus to the FIFO
   .s_axis_tdata (f0in_tdata ),
   .s_axis_tvalid(f0in_tvalid),
   .s_axis_tready(f0in_tready),

    // The output bus of the FIFO
   .m_axis_tdata (f0out_tdata ),
   .m_axis_tvalid(f0out_tvalid),
   .m_axis_tready(f0out_tready),

    // Unused input stream signals
   .s_axis_tdest(),
   .s_axis_tid  (),
   .s_axis_tstrb(),
   .s_axis_tuser(),
   .s_axis_tkeep(),
   .s_axis_tlast(),

    // Unused output stream signals
   .m_axis_tdest(),
   .m_axis_tid  (),
   .m_axis_tstrb(),
   .m_axis_tuser(),
   .m_axis_tkeep(),
   .m_axis_tlast(),

    // Other unused signals
   .almost_empty_axis(),
   .almost_full_axis(),
   .dbiterr_axis(),
   .prog_empty_axis(),
   .prog_full_axis(),
   .rd_data_count_axis(),
   .sbiterr_axis(),
   .wr_data_count_axis(),
   .injectdbiterr_axis(),
   .injectsbiterr_axis()
);
//====================================================================================


//====================================================================================
// This FIFO holds a vector of 8-bit cell-values
//====================================================================================
xpm_fifo_axis # 
(
   .FIFO_DEPTH(8192),              // DECIMAL
   .TDATA_WIDTH(PATTERN_WIDTH),    // DECIMAL
   .FIFO_MEMORY_TYPE("auto"),      // String
   .PACKET_FIFO("false"),          // String
   .USE_ADV_FEATURES("0000")       // String
)
fifo_1
(
    // Clock and reset
   .s_aclk   (clk   ),
   .m_aclk   (clk   ),
   .s_aresetn(~f1_reset),

    // The input bus to the FIFO
   .s_axis_tdata (f1in_tdata),
   .s_axis_tvalid(f1in_tvalid),
   .s_axis_tready(f1in_tready),

    // The output bus of the FIFO
   .m_axis_tdata (f1out_tdata),
   .m_axis_tvalid(f1out_tvalid),
   .m_axis_tready(f1out_tready),

    // Unused input stream signals
   .s_axis_tdest(),
   .s_axis_tid  (),
   .s_axis_tstrb(),
   .s_axis_tuser(),
   .s_axis_tkeep(),
   .s_axis_tlast(),

    // Unused output stream signals
   .m_axis_tdest(),
   .m_axis_tid  (),
   .m_axis_tstrb(),
   .m_axis_tuser(),
   .m_axis_tkeep(),
   .m_axis_tlast(),

    // Other unused signals
   .almost_empty_axis(),
   .almost_full_axis(),
   .dbiterr_axis(),
   .prog_empty_axis(),
   .prog_full_axis(),
   .rd_data_count_axis(),
   .sbiterr_axis(),
   .wr_data_count_axis(),
   .injectdbiterr_axis(),
   .injectsbiterr_axis()
);
//====================================================================================

endmodule
