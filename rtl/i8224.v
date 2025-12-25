//`default_nettype none

module i8224(
   output wire osc,       // oscilator output - same as clk
   output wire phi1,      // phi 1+2 = 8080 Clocks
   output wire phi2,
   output wire ststb_n,   // Status STB
   output reg  reset,     // Reset output
   output reg  ready,     // Ready Output
   input  wire clk,       // clock input - instead of crystal used in real HW
   input  wire sync,
   input  wire resetin_n, // Reset in
   input  wire readyin    // Ready input from 8080
);

reg [8:0] phi1reg = 9'b110000000;
reg [8:0] phi2reg = 9'b001111100;

assign osc = clk;
assign ststb_n = ~( (phi1 & sync) | reset );
assign phi1 = phi1reg[0];
assign phi2 = phi2reg[0];

initial
begin
   reset = 0;
   ready = 1;
end


always @(posedge clk)
begin
   phi1reg <= { phi1reg[7:0], phi1reg[8] };
   phi2reg <= { phi2reg[7:0], phi2reg[8] };
end


always @(posedge phi2)
begin
   reset <= ~resetin_n;
   ready <= readyin;
end


endmodule //i8224

