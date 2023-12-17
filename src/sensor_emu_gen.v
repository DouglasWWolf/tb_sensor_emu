//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 16-Dec-23  DWW     1  Initial creation
//====================================================================================

/*

*/

module sensor_emu_gen #
(
    PATTERN_WIDTH      = 32,
    LVDS_WIDTH         = 512,
    SYNC_PULSE_LENGTH  = 16
)
(
    input clk, resetn,

    output[31:0] dbg_cycle_number, 
    output[5:0]  dbg_fsm_state,

    // We output our sync pulse only when this is high
    input ENABLE,

    // These both signal "start outputting a frame"
    input RS0, RS256,

    // The number of clock cycles per data-frame.  Must be even and at least 4
    input[31:0] CYCLES_PER_FRAME,

    // The bytes that are output during the idle pattern
    input[7:0] IDLE_0, IDLE_1,

    // A sync pulse with a period of 256 clock cycles
    output PA_SYNC,

    // The LVDS lines are the primary output of this module
    output[LVDS_WIDTH-1:0] LVDS,

    // Denotes start-of-frame and end-of-frame
    output SOF, EOF,

    //==================  The stream of input bit-patterns  ====================
    input[PATTERN_WIDTH-1:0] PATTERN_TDATA,
    input                    PATTERN_TVALID,
    output reg               PATTERN_TREADY
    //==========================================================================

);

// The number of times the the input pattern will be replicated across the LVDS lines
localparam PATTERN_REPS = LVDS_WIDTH / PATTERN_WIDTH;

// This is the current cell-data pattern being output on LVDS
reg[PATTERN_WIDTH-1:0] pattern_tdata;

//==========================================================================
// This is a free-running timer
//==========================================================================
reg[7:0] free_timer;
always @(posedge clk) begin
    if (resetn == 0)
        free_timer <= 0;
    else
        free_timer <= free_timer + 1;
end
//==========================================================================

// Our sync pulse goes out periodically
assign PA_SYNC = (ENABLE & free_timer < SYNC_PULSE_LENGTH);

// We only look for "begin outputting a frame" when the free-timer is 0 ot 1
wire frame_trigger = (RS0 | RS256) & (free_timer < 2);

// The state of our main state machine
reg[5:0] fsm_state;
localparam FSM_RESET     =  1;
localparam FSM_IDLE0     =  2;
localparam FSM_IDLE1     =  4;
localparam FSM_FRAME_FC  =  8;
localparam FSM_FRAME_DC  = 16;
localparam FSM_FRAME_LC  = 32;

// Provide "start of frame" and "end of frame" markers to ease debugging
assign SOF = (fsm_state == FSM_FRAME_FC);
assign EOF = (fsm_state == FSM_FRAME_LC);

// The data on the LVDS lines depends on what state we're in
assign LVDS =
    (fsm_state == FSM_IDLE0   ) ? {64{IDLE_0}}                  :
    (fsm_state == FSM_IDLE1   ) ? {64{IDLE_1}}                  :
    (fsm_state == FSM_FRAME_FC) ? {PATTERN_REPS{pattern_tdata}} :
    (fsm_state == FSM_FRAME_DC) ? {PATTERN_REPS{pattern_tdata}} :
    (fsm_state == FSM_FRAME_LC) ? {PATTERN_REPS{pattern_tdata}} : 0;


//==========================================================================
// This state machine manages the cycle_number and the pattern input stream
//==========================================================================
reg[31:0] cycle_number;
always @(posedge clk) begin
    
    // This strobes high for one cycle at a time
    PATTERN_TREADY <= 0;

    // Count clock cycles
    cycle_number <= cycle_number + 1;

    // If we see a valid frame trigger
    if ((fsm_state == FSM_IDLE1 || fsm_state == FSM_FRAME_LC)) begin
        if (frame_trigger) begin
            cycle_number   <= 1;             // Cycle number starts over
            pattern_tdata  <= PATTERN_TDATA; // Save the current data-pattern 
            PATTERN_TREADY <= 1;             // Advance to the next pattern 
        end
    end

end
//==========================================================================

//==========================================================================
// Our main state machine - generates idle cycles and data frames
//==========================================================================
always @(posedge clk) begin

    if (resetn == 0)
        fsm_state <= FSM_RESET;

    else case (fsm_state)

        // Are we just coming out of reset?    
        FSM_RESET:
            fsm_state <= FSM_IDLE0;

        // Are we outputting the first idle byte?
        FSM_IDLE0:
            fsm_state <= FSM_IDLE1;

        // Are we outputting the second idle byte?
        FSM_IDLE1:
            if (frame_trigger) begin
                fsm_state <= FSM_FRAME_FC;
            end else
                fsm_state <= FSM_IDLE0;

        // Are we outputting the frame's first cycle?
        FSM_FRAME_FC:
            fsm_state <= FSM_FRAME_DC;
        
        // Are we outputting a frame's ordinary data cycle?
        FSM_FRAME_DC:
            if (cycle_number == CYCLES_PER_FRAME-1) begin
                fsm_state  <= FSM_FRAME_LC;
            end

        // Are we outputting a frame's last cycle?
        FSM_FRAME_LC:
            if (frame_trigger) begin
                fsm_state <= FSM_FRAME_FC;
            end else
                fsm_state <= FSM_IDLE0;

    endcase
end
//==========================================================================

assign dbg_cycle_number = cycle_number; 
assign dbg_fsm_state    = fsm_state;;


endmodule