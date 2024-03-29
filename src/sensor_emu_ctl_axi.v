//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 09-Jan-24  DWW     1  Initial creation
//====================================================================================


module sensor_emu_ctl_axi
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


    //==========================================================================
    //          These ports are probably mapped to AXI registers
    //==========================================================================

    // These reset the FIFOs
    output o_FIFO_CTL_f0_reset,
    output o_FIFO_CTL_f1_reset,
    output o_FIFO_CTL_wstrobe,

    // Upper 32 bits of data for a FIFO
    output [31:0] o_UPPER32,
    
    // Loads a word into FIFO 0
    output [31:0] o_LOAD_F0,
    output        o_LOAD_F0_wstrobe,
    
    // Loads a word into FIFO 1
    output [31:0] o_LOAD_F1,
    output        o_LOAD_F1_wstrobe,

    // Activates one of the FIFOs
    output [1:0]  o_START,
    output        o_START_wstrobe,

    // Forces a hard-stop
    output        o_HARD_STOP_wstrobe,

    // Module revision number
    input [31:0]  i_MODULE_REV,

    // The cell-pattern width, in bytes
    input [3:0]   i_PATTERN_WIDTH,

    // FIFO status 
    input         i_FIFO_STAT_f0_ready,
    input         i_FIFO_STAT_f1_ready,

    // The number of entries in each FIFO
    input [31:0]  i_F0_COUNT,
    input [31:0]  i_F1_COUNT,

    // Which FIFO is active (if any)
    input [1:0]   i_ACTIVE_FIFO
    //==========================================================================

);  

    // The number of AXI register we have
    localparam REGISTER_COUNT = 13;

    // 32-bit AXI accessible registers
    reg [31:0] axi_reg[0:REGISTER_COUNT-1];
    
    // These bits are a "1" when an AXI read or write to a register occurs
    reg [REGISTER_COUNT-1:0] rstrobe, wstrobe;

    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
    //                Between these two markers is module-specific code
    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>

    //=========================  AXI Register Map  =============================
    localparam SREG_MODULE_REV       = 0;  
    localparam SREG_PATTERN_WIDTH    = 1;  
    localparam SREG_FIFO_STAT        = 2;  
    localparam SREG_F0_COUNT         = 3;  
    localparam SREG_F1_COUNT         = 4;  
    localparam SREG_ACTIVE_FIFO      = 5;
    
    // This is an alias for the first control register
    localparam CREG_FIRST            = 6;    
        
    localparam CREG_FIFO_CTL         = 7;   
    localparam CREG_UPPER32          = 8; 
    localparam CREG_LOAD_F0          = 9; 
    localparam CREG_LOAD_F1          = 10;  
    localparam CREG_START            = 11;            
    localparam CREG_HARD_STOP        = 12; 
    //==========================================================================


    //-------------------------------------------------------
    // Map output ports to registers
    //-------------------------------------------------------
    assign o_FIFO_CTL_f0_reset  = axi_reg[CREG_FIFO_CTL ][0];
    assign o_FIFO_CTL_f1_reset  = axi_reg[CREG_FIFO_CTL ][1];
    assign o_FIFO_CTL_wstrobe   = wstrobe[CREG_FIFO_CTL ];
    assign o_UPPER32            = axi_reg[CREG_UPPER32  ];
    assign o_LOAD_F0            = axi_reg[CREG_LOAD_F0  ];
    assign o_LOAD_F0_wstrobe    = wstrobe[CREG_LOAD_F0  ];
    assign o_LOAD_F1            = axi_reg[CREG_LOAD_F1  ];
    assign o_LOAD_F1_wstrobe    = wstrobe[CREG_LOAD_F1  ];
    assign o_START              = axi_reg[CREG_START    ];
    assign o_START_wstrobe      = wstrobe[CREG_START    ];
    assign o_HARD_STOP_wstrobe  = wstrobe[CREG_HARD_STOP];
    //-------------------------------------------------------


    //-----------------------------------------------------------------
    // Map registers to input ports
    //-----------------------------------------------------------------
    always @* axi_reg[SREG_MODULE_REV   ]    = i_MODULE_REV;
    always @* axi_reg[SREG_FIFO_STAT    ][0] = i_FIFO_STAT_f0_ready;
    always @* axi_reg[SREG_FIFO_STAT    ][1] = i_FIFO_STAT_f1_ready;
    always @* axi_reg[SREG_F0_COUNT     ]    = i_F0_COUNT;
    always @* axi_reg[SREG_F1_COUNT     ]    = i_F1_COUNT;
    always @* axi_reg[SREG_ACTIVE_FIFO  ]    = i_ACTIVE_FIFO;
    always @* axi_reg[SREG_PATTERN_WIDTH]    = i_PATTERN_WIDTH;
    //-----------------------------------------------------------------


    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
    //                         End of module-specific code
    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
    //<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>



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
    
    integer i;
    
    //==========================================================================
    // This state machine handles AXI4-Lite write requests
    //==========================================================================
    always @(posedge clk) begin

        // These bits strobe high for only a single cycle
        wstrobe <= 0;

        // If we're in reset, initialize important registers
        if (resetn == 0) begin
            ashi_write_state <= 0;

        // If we're not in reset, and a write-request has occured...        
        end else case (ashi_write_state)
        
        0:  if (ashi_write) begin
                  
                for (i = CREG_FIRST; i < REGISTER_COUNT; i=i+1) begin
                    if (ashi_windx == i) begin
                        axi_reg[i] <= ashi_wdata;
                        wstrobe[i] <= 1;
                    end
                end

                // The value of ashi_wresp depends on whether the user just 
                // wrote to a valid register    
                if (ashi_windx >= CREG_FIRST && ashi_windx < REGISTER_COUNT)
                    ashi_wresp <= OKAY;
                else
                    ashi_wresp <= SLVERR;
                                
            end

        endcase
    end
    //==========================================================================





    //==========================================================================
    // World's simplest state machine for handling AXI4-Lite read requests
    //==========================================================================
    always @(posedge clk) begin

        // These bits strobe high for only a single cycle
        rstrobe <= 0;

        // If we're in reset, initialize important registers
        if (resetn == 0) begin
            ashi_read_state <= 0;
        
        // If we're not in reset, and a read-request has occured...        
        end else if (ashi_read) begin
       
            // Assume for the moment that the result will be OKAY
            ashi_rresp <= OKAY;              
            
            // Examine the register index to determine what the user is trying to read
            if (ashi_rindx < REGISTER_COUNT) begin
                ashi_rdata          <= axi_reg[ashi_rindx];
                rstrobe[ashi_rindx] <= 1;
            end

             // Reads of any other register are a decode-error
            else ashi_rresp <= DECERR;

        end
    end
    //==========================================================================



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

    

endmodule
