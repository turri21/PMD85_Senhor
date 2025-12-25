// *******************************************************
// Verilog implementation of MHB8251 - clone of i8251
// 2022 Petr Mrzena
//
// Please note - only Async mode is implemented

//`default_nettype none

module i8251(
   
   input wire CLK,
   input wire RESET,
   input wire CS_n,
   input wire WR_n,
   input wire RD_n,
   input wire CD,         // 1 = Control, 0 = Data
   input wire [7:0] DIn,  // Data from CPU to UART
   output reg [7:0] DOut, // Data from UART to CPU
   
   input wire RxD,
   output reg RxRDY,
   input wire RxC_n,
// SYNDET???

   output reg  TxD,
   output wire TxRDY,      // = buffer is empty
   output reg  TxEMPTY,
   input wire  TxC_n,

   input wire  DSR_n,
   output reg  DTR_n,
   input wire  CTS_n,
   output reg  RTS_n
);

//
// Mode instruction     - Asynchronous mode
reg [1:0] S;    // D7+6 - Number of stop bits; 00 = invalid, 01 = 1 bit, 10 = 1.5 bit, 11 = 2 bits
reg EP;         // D5   - Even parity generation/check; 1 = even, 0 = odd
reg PEN;        // D4   - Parity enable; 1 = enable, 0 = disable
reg [1:0] L;    // D3+2 - Character lenght; 00 = 5 bits, 01 = 6 bits,  10 = 7 bits, 11 = 8 bits
reg [1:0] B;    // D1+0 - Baud rate factor; 00 = Sync Mode, 01 = 1x, 10 = 16x, 11 = 64x
//
// Mode instruction     - Synchronous mode
reg SCS;        // D7   - Single character sync; 1 = single, 0 = double character sync
reg ESD;        // D6   - External sync detect; 1 = SYNDET is an input, 0= output
//wire EP;      // D5   - Even parity generation/check; 1 = even, 0 = odd
//wire PEN      // D4   - Parity enable; 1 = enable, 0 = disable
//wire [1:0] L; // D3+2 - Character lenght; 00 = 5 bits, 01 = 6 bits,  10 = 7 bits, 11 = 8 bits
//wire [1:0] B; // D1+0 - Baud rate factor; 00 = Sync Mode!
//
// Command
//wire EH;      // D7   - Enter hunt mode; 1 = enable search for Sync Characters
//wire IR;      // D6   - Internal RESET - "high" returns 8251 to Mode Instruction Format
//wire RTD;     // D5   - Request to send - "high" will force /RTS output to zero
//wire ER;      // D4   - Error RESET - 1 = reset all error flags (PE, OE, FE)
reg SBRK;       // D3   - Send break character - 1 = forces TxD "low", 0 = normal operation
reg RxE;        // D2   - Receive enable, 1 = enable, 0 = disable
//wire DTR;     // D1   - Data terminal ready - "high" will force DTR output to zero
reg TxEN;       // D0   - Transmit enable - 1 = enable, 0 = disable
//
//
//// Status
reg DSR;        // D7   - Data set Ready
reg SYNDET;     // D6   - 
reg FE;         // D5   - Framing Error (Async Only), The FE is set when a valid StopBit is not detected at the end
                //        of every character. It is reset by the ER bit of the Command Instruction. FE does not inhibit the operation of the 8251.
reg OE;         // D4   - Overrun error - The OE flag is set when the CPU does not read a character before the next 
                //        one becomes available. It is reset by the ER bit of the Command Instruction. OE does not inhibit operation of the 8251;
                //        however the previously overrun character is lost
reg PE;         // D3   - Parity Error - The PE flag is set when a parity error is detected. It is reset by the ER bit if the Command Instruction.
                //        PE does not inhibit operation of the 8251
//wire TxE;     // D2   - Transmitter empty
//wire RxRDY;   // D1   - Receiver ready to be read by CPU from 8251
//wire TxRDY;   // D0   - Transmitter ready to accept a data character, 0 when character loaded from CPU

//----------------------------
wire       RD = ~(CS_n | RD_n);
wire       WR = ~(CS_n | WR_n);
reg        TxRDYStatus;              // 1=ready to get new data into register (while previous data might still be transmitted!)
reg        TxRDYStatusSet;
reg        TxRDYStatusReset;
reg          TxEMPTYSet;
reg          TxEMPTYReset;
assign     TxRDY = TxRDYStatus & TxEN & ~CTS_n;
reg        RxRDYSet;
reg        RxRDYReset;
reg        FESet;
reg        OESet;
reg        PESet;
reg        ErrorsReset;
reg        ErrorsResetRD;
reg  [1:0] mode = 2'd0; // 0 = Instruction, 1 = SyncChar1, 2 = SyncChar2, 3 = Command
reg  [7:0] syncChar1;
reg  [7:0] syncChar2;
wire [3:0] charLenBits        = (L == 2'b00) ? 4'd4 :
                                (L == 2'b01) ? 4'd5 :
                                (L == 2'b10) ? 4'd6 : 4'd7; // 00 = 5 bits, 01 = 6 bits,  10 = 7 bits, 11 = 8 bits
reg  [3:0] RESET_Internal_cnt = 0;
wire RESET_Internal = RESET | (RESET_Internal_cnt != 4'd0);


always @(posedge TxRDYStatusSet or posedge TxRDYStatusReset)
begin
   if (TxRDYStatusReset)
      TxRDYStatus <= 1'b0;
   else
      TxRDYStatus <= 1'b1;
end

always @(posedge TxEMPTYSet or posedge TxEMPTYReset)
begin
   if (TxEMPTYReset)
      TxEMPTY <= 1'b0;
   else
      TxEMPTY <= 1'b1;
end

always @(posedge RxRDYSet or posedge RxRDYReset)
begin
   if (RxRDYReset)
      RxRDY <= 1'b0;
   else
      RxRDY <= 1'b1; 
end

always @(posedge FESet or posedge OESet or posedge PESet or posedge ErrorsReset or posedge ErrorsResetRD)
begin
   if (ErrorsReset | ErrorsResetRD)
   begin
      FE <= 1'b0;
      OE <= 1'b0;
      PE <= 1'b0;
   end

   else
   begin
      if (FESet)
         FE <= 1'b1;
      if (OESet)
         OE <= 1'b1;
      if (PESet)
         PE <= 1'b1;
   end
end


// *********************************************************************************************
// Instruction, command and data read from CPU
//
reg [7:0] TxDataBuffer;
reg [7:0] RxDataBuffer;
reg       WR_last;

always @(posedge CLK)
begin
   if (RESET_Internal)
   begin
      if (RESET_Internal_cnt != 4'd0)
         RESET_Internal_cnt <= RESET_Internal_cnt - 1'b1;
      mode         <= 2'd0;
      RTS_n        <= 1'b1; // Request to send - "high" will force /RTS output to zero
      RxE          <= 1'b0; // Receive enable, 1 = enable, 0 = disable
      DTR_n        <= 1'b1; // Data terminal ready - "high" will force DTR output to zero
      TxEN         <= 1'b0; // Transmit enable - 1 = enable, 0 = disable
      ErrorsReset  <= 1'b1;
      TxDataBuffer <= 8'd0;
   end

   else
   begin
      TxRDYStatusReset <= 1'b0;
      TxEMPTYReset     <= 1'b0;
      ErrorsReset      <= 1'b0;
      WR_last <= WR;
      if ((WR) && (WR == ~WR_last))
      begin
         if (CD == 1) // Instruction & Command
         begin
            if (mode == 2'd0) // Instruction
            begin
               // Common
               EP    <= DIn[5];   // Even parity generation/check; 1 = even, 0 = odd
               PEN   <= DIn[4];   // Parity enable; 1 = enable, 0 = disable
               L     <= DIn[3:2]; // Character lenght; 00 = 5 bits, 01 = 6 bits,  10 = 7 bits, 11 = 8 bits
               B     <= DIn[1:0]; // Baud rate factor; 00 = Sync Mode, 01 = 1x, 10 = 16x, 11 = 64x

               // Mode instruction  - Asynchronous mode
               S     <= DIn[7:6]; // Number of stop bits; 00 = invalid, 01 = 1 bit, 10 = 1.5 bit, 11 = 2 bits

               // Mode instruction  - Synchronous mode
               SCS   <= DIn[7];   // Single character sync; 1 = single, 0 = double character sync
               ESD   <= DIn[6];   // External sync detect; 1 = SYNDET is an input, 0= output

               mode  <= (DIn[1:0] == 2'b00) ? 2'd1 : 2'd3;
            end

            else if (mode == 2'd1) // SyncChar1
            begin
               syncChar1 <= DIn;
               mode <= (SCS) ? 2'd3 : 2'd2;
            end

            else if (mode == 2'd2) // SyncChar2
            begin
               syncChar2 <= DIn;
               mode <= 2'd3;
            end

            else //if (mode == 2'd3) // Command
            begin
               //wire EH;         // D7 - Enter hunt mode; 1 = enable search for Sync Characters
               if (DIn[6])        // IR ... D6 - Internal RESET - "high" returns 8251 to Mode Instruction Format
                  RESET_Internal_cnt <= 4'b0111;
               RTS_n <= ~DIn[5];  // Request to send - "high" will force /RTS output to zero
               if (DIn[4])        // ER ... D4 - Error RESET - 1 = reset all error flags (PE, OE, FE)
                  ErrorsReset <= 1'b1;
               SBRK <= DIn[3];    // Send break character - 1 = forces TxD "low", 0 = normal operation
               RxE <= DIn[2];     // Receive enable, 1 = enable, 0 = disable
               DTR_n <= ~DIn[1];  // Data terminal ready - "high" will force DTR output to zero
               TxEN <= DIn[0];    // Transmit enable - 1 = enable, 0 = disable
            end
         end
         else if (CD == 0) // data
            begin
               TxDataBuffer <= DIn;
               TxRDYStatusReset <= 1'b1; // resetuj TxRDY
               TxEMPTYReset     <= 1'b1;
            end
      end
   end
end


// *********************************************************************************************
// Status read by CPU
//
wire [7:0] status = {
     ~DSR_n,      // Data set Ready
      1'd0,       // SYNDET
      FE,         // Framing Error
      OE,         // Overrun error
      PE,         // Parity Error
      TxEMPTY,    // Transmitter empty
      RxRDY,      // Receiver ready to be read by CPU from 8251
      TxRDYStatus // Transmitter ready to accept a data character, 0 when character loaded from CPU
      };

always @(posedge RD)
   DOut <= ((CD) ? status : RxDataBuffer);

reg DSR_d1;
reg DSR_d2;
always @(posedge CLK)
begin
   DSR_d1 <= ~DSR_n;
   DSR_d2 <= DSR_d1;
end

reg RD_last;
always @(posedge RESET_Internal or posedge CLK)
begin
   if (RESET_Internal)
   begin
      RxRDYReset    <= 1'b1;
      ErrorsResetRD <= 1'b0;
   end
   else
   begin
      RxRDYReset    <= 1'b0;
      ErrorsResetRD <= 1'b0;
      RD_last <= RD;
      if ((RD) && (RD == ~RD_last))
      begin
         if (~CD)
         begin
            RxRDYReset    <= 1'b1;
            ErrorsResetRD <= 1'b1;
         end
      end
   end
end


// *********************************************************************************************
// Transmitter 
//    inspired by code on http://www.nandland.com
//
parameter s_IDLE       = 3'b000;
parameter s_START_BIT  = 3'b001;
parameter s_DATA_BITS  = 3'b010;
parameter s_PARITY_BIT = 3'b011;
parameter s_STOP_BIT1  = 3'b100;
parameter s_STOP_BIT2  = 3'b101;


reg [7:0] TxData;
reg [2:0] Tx_State     = 0;
reg [3:0] Tx_Bits      = 0; // Number of bits to be transmitted
reg       Tx_Parity    = 0;

reg [5:0] TxC_DIV = 0;
always @(negedge TxC_n)
   TxC_DIV <= TxC_DIV + 1'b1;

wire TxC_internal_n = (B == 2'b00) ? TxC_n : // Baud rate factor; 00 = Sync Mode,
                      (B == 2'b01) ? TxC_n :      // 01 =  1x
                      (B == 2'b10) ? TxC_DIV[3] : // 10 = 16x
                                     TxC_DIV[5];  // 11 = 64x
                                     
always @(posedge RESET_Internal or negedge TxC_internal_n)
begin 
   if (RESET_Internal)
   begin
      Tx_State       <= s_IDLE;
      TxEMPTYSet     <= 1'b1;
      TxD            <= 1'b1;
      TxRDYStatusSet <= 1'b1;
   end
   else
   begin
      TxRDYStatusSet <= 1'b0;
      TxEMPTYSet     <= 1'b0;
      case (Tx_State)
         s_IDLE :
            begin
               TxD        <= (SBRK) ? 1'b0 : 1'b1;   // Send break character - 1 = forces TxD "low", 0 = normal operation
               Tx_Bits    <= charLenBits;
               Tx_Parity  <= ~EP;                    // Even parity generation/check; 1 = even, 0 = odd
               if (TxRDYStatus)
                  TxEMPTYSet <= 1'b1;                // Empty means no data in register and no data in transmittion

               if ((~TxRDYStatus) & (TxEN) & (~CTS_n))
               begin
                  TxRDYStatusSet <= 1'b1;            // set TXRDY = 1
                  TxData         <= TxDataBuffer;
                  TxD            <= 1'b0;            // Send out Start Bit. Start bit = 0
                  Tx_State        <= s_DATA_BITS;               
               end
               else
               Tx_State <= s_IDLE;
            end // case: s_IDLE

         s_DATA_BITS :
            begin
               TxD <= TxData[0];
               if (TxData[0])
                  Tx_Parity <= ~Tx_Parity;
               if (Tx_Bits == 4'd0)
                  // PEN = Parity enable; 1 = enable, 0 = disable
                  Tx_State  <= (PEN) ? s_PARITY_BIT : s_STOP_BIT1;
               else
               begin
                  Tx_Bits  <= Tx_Bits - 1'b1;
                  TxData   <= {1'b0, TxData[7:1]};
                  Tx_State <= s_DATA_BITS;
               end
            end // case: s_DATA_BITS

            // Send out Parity bit.
         s_PARITY_BIT:
            begin
               TxD      <= Tx_Parity;
               Tx_State <= s_STOP_BIT1;
            end // case: s_PARITY_BIT 

            // Send out Stop bit(s).  Stop bit(s) = 1, 1.5, 2
         s_STOP_BIT1 :
            begin
               TxD      <= 1'b1;
               Tx_State <= (S == 2'b01) ? s_IDLE : s_STOP_BIT2;
            end // case: s_STOP_BIT1

         s_STOP_BIT2 :
            begin
               TxD      <= 1'b1;
               Tx_State <= s_IDLE;
            end // case: s_STOP_BIT2

         default :
            Tx_State <= s_IDLE;
         endcase
   end
end

// *********************************************************************************************
// Receiver 
//    inspired by code on http://www.nandland.com
//
reg [7:0] RxData;
reg [2:0] Rx_State     = 3'd0;
reg [3:0] Rx_Bits      = 4'd0;
reg       Rx_Parity    = 0;
reg       RxD_DR;
reg       RxD_DR2;

// Purpose: Double-register the incoming data.
// This allows it to be used in the UART RX Clock Domain.
// (It removes problems caused by metastability)
always @(posedge RxC_n)
begin
   RxD_DR2 <= RxD;
   RxD_DR  <= RxD_DR2;
end

reg  [5:0] RxC_DIV_Cnt  = 0;
wire [5:0] RxC_DIV_Full = (B == 2'b10) ? 15 : // 10 = 16x                          
                          (B == 2'b11) ? 63 : // 11 = 64x
                                         0;   // Baud rate factor; 00 = Sync Mode or 01 =  1x                                        

wire [5:0] RxC_DIV_Half = (B == 2'b10) ?  7 : // 10 = 16x                          
                          (B == 2'b11) ? 32 : // 11 = 64x
                                         0;   // Baud rate factor; 00 = Sync Mode or 01 =  1x                                        
                                     

reg RxC_mark = 0;                                     
                                     
always @(posedge RESET_Internal or posedge RxC_n)
begin
   if (RESET_Internal)
   begin
      Rx_State     <= s_IDLE;
      RxDataBuffer <= 8'd0;
   end
   else
   begin
      FESet    <= 1'b0;
      OESet    <= 1'b0;
      PESet    <= 1'b0;
      RxRDYSet <= 1'b0;
      case (Rx_State)
         s_IDLE :
            begin
RxC_mark <= ~RxC_mark;
               RxData    <= 8'd0;
               Rx_Bits   <= charLenBits;
               Rx_Parity <= ~EP;  // Even parity generation/check; 1 = even, 0 = odd
               if ((RxD_DR == 1'b0) & (B[1]))  // Start bit detected, x16 or x64
               begin
                  RxC_DIV_Cnt <= RxC_DIV_Half;
                  Rx_State    <= s_START_BIT;
               end
               else if ((RxD_DR == 1'b0) & (~B[1]))  // Start bit detected, x1
               begin
                  RxC_DIV_Cnt <= RxC_DIV_Full;
                  Rx_State    <= s_DATA_BITS;
               end
            end

         s_START_BIT :
            begin
               if (RxC_DIV_Cnt == 0)
               begin
RxC_mark <= ~RxC_mark;
                  if (RxD_DR == 1'b0)
                  begin
                     RxC_DIV_Cnt <= RxC_DIV_Full;
                     Rx_State    <= s_DATA_BITS;
                  end
                  else
                     Rx_State    <= s_IDLE;
               end
               else
                  RxC_DIV_Cnt <= RxC_DIV_Cnt - 1;
            end
            
         // Sample serial data
         s_DATA_BITS :
            begin
               if (RxC_DIV_Cnt == 0)
               begin
RxC_mark <= ~RxC_mark;
                  RxData       <= {RxD_DR, RxData[7:1]};
                  if (RxD_DR)
                     Rx_Parity <= ~Rx_Parity;
                  RxC_DIV_Cnt <= RxC_DIV_Full;
                  // Check if we have received all bits
                  if (Rx_Bits == 4'd0)
                     // PEN = Parity enable; 1 = enable, 0 = disable
                     Rx_State  <= (PEN) ? s_PARITY_BIT : s_STOP_BIT1;
                  else
                  begin
                     Rx_Bits  <= Rx_Bits - 1'b1;
                     Rx_State <= s_DATA_BITS;
                  end
               end
               else
                  RxC_DIV_Cnt <= RxC_DIV_Cnt - 1;
            end // case: s_DATA_BITS

         // Receive Parity bit.
         s_PARITY_BIT:
            begin
               if (RxC_DIV_Cnt == 0)
               begin
RxC_mark <= ~RxC_mark;
                  RxC_DIV_Cnt <= RxC_DIV_Full;
                  Rx_State <= s_STOP_BIT1;
                  if (RxD_DR != Rx_Parity)
                    PESet  <= 1'b1;
               end
               else
                  RxC_DIV_Cnt <= RxC_DIV_Cnt - 1;                 
            end // case: s_PARITY_BIT

         // Receive Stop bit.  Stop bit = 1
         s_STOP_BIT1:
            begin
               if (RxC_DIV_Cnt == 0)
               begin            
RxC_mark <= ~RxC_mark;
                  RxC_DIV_Cnt   <= 0;
                  Rx_State      <= s_IDLE;
                  RxDataBuffer  <= (L == 2'b00) ? {3'd0, RxData[7:3]} : 
                                   (L == 2'b01) ? {2'd0, RxData[7:2]} : 
                                   (L == 2'b10) ? {1'd0, RxData[7:1]} : RxData; // 00 = 5 bits, 01 = 6 bits,  10 = 7 bits, 11 = 8 bits
                  RxRDYSet      <= 1'b1;
                  if (RxD_DR != 1'b1)
                     FESet <= 1'b1;
                  if (RxRDY)
                     OESet <= 1'b1;
               end
               else
                  RxC_DIV_Cnt <= RxC_DIV_Cnt - 1;
            end // case: s_STOP_BIT1

         default :
            Rx_State <= s_IDLE;
      endcase 
   end
end

endmodule //i8251
