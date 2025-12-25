//`default_nettype none

module i8255(
   input  wire [7:0] PA_In,
   output wire [7:0] PA_Out,
   input  wire [7:0] PB_In,
   output wire [7:0] PB_Out,
   input  wire [7:0] PC_In,
   output wire [7:0] PC_Out,

   input  wire [7:0] D_In,
   output wire [7:0] D_Out,

   input  wire       clk,
   input  wire       RD_n,
   input  wire       WR_n,
   input  wire [1:0] A,
   input  wire       RESET,
   input  wire       CS_n,
   
   output reg        PAisInput,
   output reg        PBisInput,
   output reg        PCLisInput,
   output reg        PCHisInput
);


reg [1:0] grpAmode; // port A + port C[7:4]
reg       grpBmode; // port B + port C[3:0]
reg       WR_n_last;

always @(posedge clk)
begin
   WR_n_last <= WR_n;
   
   if (~WR_n & WR_n_last)
   begin
      PA_WR           <= (A == 2'b00) & ~CS_n;
      PB_WR           <= (A == 2'b01) & ~CS_n;
      PC_Set          <= (A == 2'b10) & ~CS_n;
      PC_BitSet       <= (A == 2'b11) & (D_In[7] == 1'b0) & ~CS_n;
      ControlWord_Set <= (A == 2'b11) & (D_In[7] == 1'b1) & ~CS_n;
   end
   else if (WR_n & ~WR_n_last)
   begin
      PA_WR           <= 1'b0;
      PB_WR           <= 1'b0;
      PC_Set          <= 1'b0;
      PC_BitSet       <= 1'b0;
      ControlWord_Set <= 1'b0;
   end;
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
//  PA Port
reg [7:0] PA_In_Latch  = 8'h00;
reg [7:0] PA_Out_Latch = 8'h00;

wire PA_RD = (A == 2'b00) & ~CS_n & ~RD_n;
reg  PA_WR;

wire PA_MODE0 = (grpAmode == 2'b00);
wire PA_MODE1 = (grpAmode == 2'b01);
wire PA_MODE2 = (grpAmode[1]);

reg PA_RD_Last     = 1'b0;
reg PA_WR_Last     = 1'b0;
reg PA_ACK_n_Last  = 1'b0;
reg PA_STB_n_Last  = 1'b0;
reg PAisInput_Last = 1'b0;
reg [1:0] grpAmode_Last = 2'b11;

reg PA_OBF_n      = 1'b1;
reg PA_IBF        = 1'b0;
reg PA_INTRi      = 1'b0;  // interrupt for IBF
reg PA_INTRo      = 1'b0;  // interrupt for OBF
wire PA_INTEi     = PC_Out_Latch[4];  // interrupt enabled for IBF
wire PA_INTEo     = PC_Out_Latch[6];  // interrupt enabled for OBF

wire PA_INTR = PA_MODE2 ? (PA_INTRi & PA_INTEi) | (PA_INTRo & PA_INTEo) :
               PA_MODE1 ? (PAisInput ? (PA_INTRi & PA_INTEi) : (PA_INTRo & PA_INTEo)) : 1'b0;

wire PA_ACK_n = PA_MODE0 |  PAisInput | PC_In[6]; // only when NOT Mode0 & IS NOT Input & PC_In[6] 
wire PA_STB_n = PA_MODE0 | ~PAisInput | PC_In[4]; // only when NOT Mode0 & IS Input & PC_In[4] 

wire [7:0] PA_READ = (PA_MODE0) ? (PAisInput ? PA_In : PA_Out_Latch) : PA_In_Latch;

always @(posedge clk)
begin
   PA_RD_Last <= PA_RD;
   PA_WR_Last <= PA_WR;
   PA_ACK_n_Last <= PA_ACK_n;
   PA_STB_n_Last <= PA_STB_n;
   grpAmode_Last <= grpAmode;
   PAisInput_Last <= PAisInput;

   if (RESET | ~(grpAmode_Last == grpAmode) | (PAisInput == ~PAisInput_Last))
   begin
      PA_In_Latch  <= 8'h00;
      PA_Out_Latch <= 8'h00;
      PA_IBF       <= 1'b0;
      PA_OBF_n     <= 1'b1;
      PA_INTRi     <= 1'b0;
      PA_INTRo     <= 1'b1;
   end

// this part only when PA is OUTPUT

   // WR start (posedge PA_WR) => latch data + deactivate INT (PA_INTR <= 0)
   if (PA_WR & ~PA_WR_Last)
   begin
      PA_Out_Latch <= D_In;
      if (~PAisInput)
         PA_INTRo <= 1'b0;
   end

   // WR end (negedge PA_WR) => activate OBF (PA_OBF_n <= 0) if Mode 1 or 2 and ACK is not activated (PA_ACK_n == 1)
   if (~PA_WR & PA_WR_Last & ~PAisInput)
      PA_OBF_n <= ~(PA_MODE1 | PA_MODE2) | ~PA_ACK_n;

   // ACK start (negedge PA_ACK_n) => deactivate OBF (PA_OBF_n <= 1)
   if (~PA_ACK_n & PA_ACK_n_Last & ~PAisInput)
      PA_OBF_n <= 1'b1;
   // ACK end (posedge PA_ACK_n) => activate INT (PA_INTR <= 1) if Mode 1 or 2
   if (PA_ACK_n & ~PA_ACK_n_Last & ~PAisInput)
      PA_INTRo <= PA_MODE1 | PA_MODE2;


// this part only when PA is INPUT

   // STB start (negedge PA_STB_n) => atch data + activate IBF (PA_IBF <= 1) if Mode 1 or 2
   if (~PA_STB_n & PA_STB_n_Last)
   begin
      PA_In_Latch <= PA_In;
      if (PAisInput)
        PA_IBF <= (PA_MODE1 | PA_MODE2);
   end

   // STB end (posedge PA_STB_n) => activate INT (PA_INTR <= 1) if Mode 1 or 2 and PA_RD is not activated (PA_RD == 0)
   if (PA_STB_n & ~PA_STB_n_Last & PAisInput)
      PA_INTRi <= (PA_MODE1 | PA_MODE2) & ~PA_RD;

   // RD start (posedge PA_RD) => deactivate INT (PA_INTR <= 0)
   if (PA_RD & ~PA_RD_Last & PAisInput)
      PA_INTRi <= 1'b0;

   // RD end (negedge PA_RD) => deactivate IBF (PA_IBF <= 0)
   if (~PA_RD & PA_RD_Last & PAisInput)
      PA_IBF <= 1'b0 | ~PA_STB_n;
end

//---------------------------------------------------------------------------------------------------------------------------------------------------
//  PB Port
reg [7:0] PB_In_Latch  = 8'h00;
reg [7:0] PB_Out_Latch = 8'h00;

wire PB_RD  = (A == 2'b01) & ~CS_n & ~RD_n;
reg  PB_WR;

wire PB_MODE0 = ~grpBmode;
wire PB_MODE1 =  grpBmode;

reg PB_RD_Last     = 1'b0;
reg PB_WR_Last     = 1'b0;
reg PB_ACK_n_Last  = 1'b0;
reg PB_STB_n_Last  = 1'b0;
reg PBisInput_Last = 1'b0;
reg grpBmode_Last  = 1'b1;

reg  PB_OBF_n     = 1'b1;
reg  PB_IBF       = 1'b0;
reg  PB_INTRi     = 1'b0;  // interrupt for IBF
reg  PB_INTRo     = 1'b0;  // interrupt for OBF
wire PB_INTEi     = PC_Out_Latch[2];  // interrupt enabled for IBF
wire PB_INTEo     = PC_Out_Latch[2];  // interrupt enabled for OBF

wire PB_INTR = PB_MODE1 ? (PBisInput ? (PB_INTRi & PB_INTEi) : (PB_INTRo & PB_INTEo)) : 1'b0;
wire PB_ACK_n = PB_MODE0 |  PBisInput | PC_In[2]; // only when NOT Mode0 & IS NOT Input & PC_In[2]
wire PB_STB_n = PB_MODE0 | ~PBisInput | PC_In[2]; // only when NOT Mode0 & IS Input & PC_In[2]

wire [7:0] PB_READ = (PB_MODE0) ? (PBisInput ? PB_In : PB_Out_Latch) : PB_In_Latch;

always @(posedge clk)
begin
   PB_RD_Last     <= PB_RD;
   PB_WR_Last     <= PB_WR;
   PB_ACK_n_Last  <= PB_ACK_n;
   PB_STB_n_Last  <= PB_STB_n;
   grpBmode_Last  <= grpBmode;
   PBisInput_Last <= PBisInput;

   if (RESET | ~(grpBmode_Last == grpBmode) | (PBisInput == ~PBisInput_Last))
   begin
      PB_In_Latch <= 8'h00;
      PB_Out_Latch <= 8'h00;
      PB_IBF <= 1'b0;
      PB_OBF_n <= 1'b1;
      PB_INTRi <= 1'b0;
      PB_INTRo <= 1'b1;
   end

// this part only when PB is OUTPUT

   // WR start (posedge PB_WR) => latch data + deactivate INT (PB_INTR <= 0) 
   if (PB_WR & ~PB_WR_Last)
   begin
      PB_Out_Latch <= D_In;
      if (~PBisInput)
         PB_INTRo <= 1'b0;
   end

   // WR end (negedge PB_WR) => activate OBF (PB_OBF_n <= 0) if Mode 1 and ACK is not activated (PB_ACK_n == 1)
   if (~PB_WR & PB_WR_Last & ~PBisInput)
      PB_OBF_n <= ~PB_MODE1 | ~PB_ACK_n;

   // ACK start (negedge PB_ACK_n) => deactivate OBF (PB_OBF_n <= 1)
   if (~PB_ACK_n & PB_ACK_n_Last & ~PBisInput)
      PB_OBF_n <= 1'b1;

      // ACK end (posedge PB_ACK_n) => activate INT (PB_INTR <= 1) if Mode 1
   if (PB_ACK_n & ~PB_ACK_n_Last & ~PBisInput)
      PB_INTRo <= PB_MODE1;


// this part only when PB is INPUT
   
   // STB start (negedge PB_STB_n) => atch data + activate IBF (PB_IBF <= 1) if Mode 1
   if (~PB_STB_n & PB_STB_n_Last)
   begin
      PB_In_Latch <= PB_In;
      if (PBisInput)
        PB_IBF <= PB_MODE1;
   end
   
   // STB end (posedge PB_STB_n) => activate INT (PB_INTR <= 1) if Mode 1 and PB_RD is not activated (PB_RD == 0)
   if (PB_STB_n & ~PB_STB_n_Last & PBisInput)
      PB_INTRi <= PB_MODE1 & ~PB_RD;

   // RD start (posedge PB_RD) => deactivate INT (PB_INTR <= 0)
   if (PB_RD & ~PB_RD_Last & PBisInput)
      PB_INTRi <= 1'b0;
   
   // RD end (negedge PB_RD) => deactivate IBF (PB_IBF <= 0)
   if (~PB_RD & PB_RD_Last & PBisInput)
      PB_IBF <= 1'b0 | ~PB_STB_n;
end

//---------------------------------------------------------------------------------------------------------------------------------------------------
//  PC Port & Control Word
reg  [7:0] PC_Out_Latch;
reg        PC_Set;
reg        PC_BitSet;
reg        ControlWord_Set;
wire [7:0] mask = (D_In[3:1] == 3'd0) ? 8'b0000_0001 :
                  (D_In[3:1] == 3'd1) ? 8'b0000_0010 :
                  (D_In[3:1] == 3'd2) ? 8'b0000_0100 :
                  (D_In[3:1] == 3'd3) ? 8'b0000_1000 :
                  (D_In[3:1] == 3'd4) ? 8'b0001_0000 :
                  (D_In[3:1] == 3'd5) ? 8'b0010_0000 :
                  (D_In[3:1] == 3'd6) ? 8'b0100_0000 : 8'b1000_0000;


reg PC_Set_last;
reg PC_BitSet_last;

always @(posedge clk)
begin
   PC_Set_last    <= PC_Set;
   PC_BitSet_last <= PC_BitSet;

   if (RESET)
      PC_Out_Latch <= 8'h00;
   else
   if (PC_Set & ~PC_Set_last)
      PC_Out_Latch <= D_In;
else
   if (PC_BitSet & ~PC_BitSet_last)
      PC_Out_Latch <= D_In[0] ? (PC_Out_Latch | mask) : (PC_Out_Latch & ~mask);
end


//---------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge RESET or posedge ControlWord_Set)
begin
   if (RESET)
   begin      
      PAisInput  <= 1'b1;
      PBisInput  <= 1'b1;
      PCLisInput <= 1'b1;
      PCHisInput <= 1'b1;
      grpAmode   <= 2'b00;
      grpBmode   <= 1'b0;
   end
   else if (ControlWord_Set) begin
      // mode set flag      
      PCLisInput <= D_In[0];
      PBisInput  <= D_In[1];
      grpBmode   <= D_In[2];
      PCHisInput <= D_In[3];
      PAisInput  <= D_In[4];
      grpAmode   <= D_In[6:5];
   end 
end

//---------------------------------------------------------------------------------------------------------------------------------------------------

assign PA_Out = PA_Out_Latch;
assign PB_Out = PB_Out_Latch;


wire [7:3] PC_Out_H = (PA_MODE0) ? PC_Out_Latch[7:3] : 
                      (PA_MODE1) ? {(PAisInput) ? {PC_Out_Latch[7:6], PA_IBF, PC_Out_Latch[4], PA_INTR} : {PA_OBF_n, PC_Out_Latch[6:4], PA_INTR} } :
                    /*(PA_MODE2)*/ { PA_OBF_n, PC_Out_Latch[6], PA_IBF, PC_Out_Latch[4], PA_INTR };

wire [2:0] PC_Out_L = (PB_MODE0) ? PC_Out_Latch[2:0] : 
                    /*(PB_MODE1)*/ {(PBisInput) ? {PC_Out_Latch[2], PB_IBF, PB_INTR} : {PC_Out_Latch[2], PB_OBF_n, PB_INTR} };

assign PC_Out = {PC_Out_H, PC_Out_L};

//wire [7:0] PC_READ = {PC_Out_H, PC_Out_L};
wire [7:0] PC_READ = {PC_Out_H, PC_Out_L};

assign D_Out = ( A == 2'b00 ) ? PA_READ :
               ( A == 2'b01 ) ? PB_READ :
               ( A == 2'b10 ) ? PC_READ : 8'hFF;

endmodule // i8255
