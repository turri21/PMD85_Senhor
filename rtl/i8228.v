//`default_nettype none

module i8228(
   output wire       memr_n,            // Memory Read
   output wire       memw_n,            // Memory Write
   output wire       ior_n,             // IO Read
   output wire       iow_n,             // IO Write
   output wire       inta_n,            // Interrupt Acknowledge
   input  wire [7:0] databus_from_cpu,  // from CPU to 8228
   output wire [7:0] databus_to_cpu,    // from 8228 to CPU
   output wire [7:0] databus_from_8228, // from 8228 to peripherials
   input  wire [7:0] databus_to_8228,   // from 8228 to peripherials
   input  wire       busen_n,           // Bus Enable Input
   input  wire       ststb_n,           // Status Strobe from 8224
   input  wire       dbin,              // Data Bus In Control from 8080
   input  wire       wr_n,              // WR from 8080
   input  wire       hlda,              // Hold Acknowledge from 8080

   // this is not present on original chip - instead if you need one level interrupt you were suppose to put 12V on inta_n pin. 
   // This is replacement of this behavior
   input  wire       inta_12V
);

reg[7:0] status; // CPU status

/*
Bidirectional data bus. The processor also transiently sets here the "processor state", providing information about what the processor is currently doing:

    D0 reading interrupt command. In response to the interrupt signal, the processor is reading and executing a single arbitrary command with this flag raised. 
         Normally the supporting chips provide the subroutine call command (CALL or RST), transferring control to the interrupt handling code.
    D1 reading (low level means writing)
    D2 accessing stack (probably a separate stack memory space was initially planned)
    D3 doing nothing, has been halted by the HLT instruction
    D4 writing data to an output port
    D5 reading the first byte of an executable instruction
    D6 reading data from an input port
    D7 reading data from memory
*/

// internal signals only

wire memr = ( ~status[0] &   status[1]              & ~status[3] & ~status[4] & ~status[6] &  status[7] ); // ~INT &  nWO          & ~HLTA & ~OUT & ~INP &  MEMR
wire memw = ( ~status[0] &  ~status[1]              & ~status[3] & ~status[4] & ~status[6] & ~status[7] ); // ~INT & ~nWO          & ~HLTA & ~OUT & ~INP & ~MEMR
wire ior  = ( ~status[0] &   status[1] & ~status[2] & ~status[3] & ~status[4] &  status[6] & ~status[7] ); // ~INT &  nWO & ~STACK & ~HLTA & ~OUT &  INP & ~MEMR
wire iow  = ( ~status[0] &  ~status[1] & ~status[2] & ~status[3] &  status[4] & ~status[6] & ~status[7] ); // ~INT & ~nWO & ~STACK & ~HLTA &  OUT & ~INP & ~MEMR
wire inta = ( status[0] );

assign memr_n            = ~( memr ); 
assign memw_n            = ~( ~wr_n & memw ); 
assign ior_n             = ~( ior ); 
assign iow_n             = ~( ~wr_n & iow ); 

assign inta_n            = ~( dbin & inta );
assign databus_to_cpu    = (inta & inta_12V) ? 8'hFF : databus_to_8228; // RST 7 or system bus
assign databus_from_8228 = databus_from_cpu;

initial 
begin
   status = 8'd0;
end


always @(negedge ststb_n) 
begin
   status <= databus_from_cpu;
end


endmodule // i8228