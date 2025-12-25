//`default_nettype none


// PMD85 Memory map
// ****************
//                   PMD85 v.1                PMD85 v.2A           PMD85 v.3
// 0x0000 - 0x7FFF   RAM                      RAM                  RAM
// 0x8000 - 0x8FFF   ROM (Monitor)            ROM / RAM (all Ram)  RAM / ROM copy in compatibility mode (JUMP FFF0)
// 0x9000 - 0x9FFF   not used                 RAM                  RAM
// 0xA000 - 0xAFFF   ROM (mirror 8000-8FFF)   ROM / RAM (all Ram)  RAM
// 0xB000 - 0xBFFF   not used                 RAM                  RAM
// 0xC000 - 0xDFFF   Video RAM                Video RAM            Video RAM
// 0xE000 - 0xFFFF   Video RAM                Video RAM            Video RAM / ROM Monitor (shadow)

// Video
// *****
//
// Resolution in
//   - pixels:    288 columns (289-384 invisible) × 256 rows (257-320 invisible)
//   - characters: 48 columns (49-64 invisible)   x  32 rows (33-40 invisible)
//
// 1 micro row ... 48 byte + 16 byte unused = 64 byte
//
// micro row  1 ... 0xC000 - 0xC030   (r = 0x0000 - 0x0030)
// micro row  2 ... 0xC040 - 0xC070
// ...
// row 32 ... 0xFFC0 - 0xFFF0

// PMD 85-3 dôjde pri prechode do režimu kompatibility s PMD 85-2 príkazom JUMP FFF0. 

 
module PMD85_core
(
   input  wire        clk_50M, // 50MHz main clock   
   input  wire        clk_sys, // PMD85 system clock (for 8224) is 18.432MHz
   input  wire        reset_main,
   input  wire [10:0] ps2_key,
   input  wire [24:0] ps2_mouse,
   input  wire  [4:0] joystick,     // FIRE LEFT RIGHT UP DOWN   
   
   output wire        clk_video,     // pixel is valid on negedge clk_video (being changed on posedge)
   output wire        SR_n,
   output wire        SD_n,
   output wire        ZAT_n,
   output reg         ZAT_n_XXX,
   output wire        pixel,
   output wire  [7:0] VGA_R,
   output wire  [7:0] VGA_G,
   output wire  [7:0] VGA_B,   
    
   input  wire        lineIn,        // Cassette line in for loading data
   output wire        lineOut,       // Cassette line out for recording data
   input  wire        RxD,
   output wire        TxD,
    
   input  wire        PMD_version,   // 0 = PMD85 2A, 1 = PMD85 v3
   input  wire        mouseEnabled,  // 0 = disabled, 1 = port K2 
   input  wire  [1:0] joystickPort,  // 00 = None, 01 = port K3, 10 = port K4
   input  wire  [1:0] audioMode,     // 00 = Beeper, 01 = Beeper + MIF85, 10 = Beeper + Musica
   input  wire  [1:0] ColorMode,   
   input  wire        RomPackType,   // 0 = clasic rom pack implemented in BRAM, 1 = MEGA Rom Pack in SDRAM
   input  wire        ROMPackEject,
   
   output wire        beeper,
   output wire  [7:0] MIF85_left_out,
   output wire  [7:0] MIF85_right_out,
   output wire  [2:0] musica_out,
   output wire        led_yellow,
   output wire        led_red,        
   
   output wire  [7:0] sdram_in,
   input  wire  [7:0] sdram_out,
   output wire [24:0] sdram_a,
   output wire        sdram_rd,
   output wire        sdram_we,
   input  wire        sdram_ready,
    
   input  wire        ioctl_download , // signal indicating an active download
   input  wire  [7:0] ioctl_index,     // menu index used to upload the file
   input  wire        ioctl_wr,
   input  wire [26:0] ioctl_addr,      // in WIDE mode address will be incremented by 2
   input  wire  [7:0] ioctl_dout,
   input  wire [31:0] ioctl_file_ext,
   output wire        ioctl_wait  
);



//---------------------------------------------- i8224 + i8080 + i8228 ---------------------------------------------------------
//
wire        osc;
wire        phi1;
wire        phi2;
wire        sync;
wire        reset; // reset from i8224 to 8080
wire        reset_from_keyboard;
wire        ststb_n; // strobe from 8224 to 8228
wire        hold          = 0; //ioctl_download; //0; // 8080 hold pin
wire [15:0] address_bus;
wire        int8080_n     = mouseEnabled ? mouseInt_n  : MIF85_Int;
wire        int8080Enable;
wire        wait8080;
wire        dbin;
wire        wr_n;
wire        hlda;

wire        memr_n;
wire        memw_n;
wire        ior_n;
wire        iow_n;
wire        inta_n ;

reg         cpu_status_reading;  // reading (low level means writing)
wire        ready8080     = cpu_status_reading ^ VIDEO;

wire [7:0] databus_from_8228;
wire [7:0] databus_to_8228;
wire [7:0] databus_from_cpu;
wire [7:0] databus_to_cpu;

always @(posedge ststb_n) begin
   cpu_status_reading <= databus_from_cpu[1];
end


i8224 i8224
(
   .osc(osc),
   .phi1(phi1),
   .phi2(phi2), 
   .ststb_n(ststb_n), 
   .reset(reset), 
   .clk(clk_sys), 
   .sync(sync), 
   .resetin_n(~reset_main & ~reset_from_keyboard), 
   .readyin(1'b1)
);

i8228 i8228
(
   .memr_n(memr_n), 
   .memw_n(memw_n), 
   .ior_n(ior_n), 
   .iow_n(iow_n), 
   .inta_n(inta_n), 
   .inta_12V(1'b1),
   .databus_from_cpu(databus_from_cpu), 
   .databus_to_cpu(databus_to_cpu),  
   .databus_from_8228(databus_from_8228),
   .databus_to_8228(databus_to_8228),
   .busen_n(VIDEO),
   .ststb_n(ststb_n), 
   .dbin(dbin), 
   .wr_n(wr_n), 
   .hlda(hlda)
);

(* keep *) wire pin_aena;
   
vm80a_core cpu ( 
   .pin_clk(~clk_sys),

   .pin_f1(phi1), 
   .pin_f2(phi2), 
   .pin_reset(reset), 
   .pin_hold(hold), 
   .pin_ready(ready8080), 
   .pin_int(~int8080_n), 
   .pin_inte(int8080Enable), 
   .pin_a(address_bus), 
   .pin_aena(pin_aena),
   .pin_dout(databus_from_cpu),
   .pin_din(databus_to_cpu),
   .pin_dbin(dbin), 
   .pin_wr_n(wr_n), 
   .pin_hlda(hlda), 
   .pin_sync(sync), 
   .pin_wait(wait8080) 
);
      
assign databus_to_8228 =
        (~ifc_CS1_n      & ~ior_n)                ? ifc_i8251_data_out :  // interface i8251
        (~address_bus[3] & ~ior_n)                ? dataKey8255        :  // Keyboard      
        (~address_bus[2] & ~ior_n & ~RomPackType) ? ROMPack_C_data_out :  // ROM Module - small/classic
        (~address_bus[2] & ~ior_n &  RomPackType) ? ROMPack_M_data_out :  // ROM Module - MEGA Rom Pack
        (~ifc_CS4_n      & ~ior_n)                ? ifc_i8255_data_out :  // interface i8255 
        (~ifc_CS5_n      & ~ior_n)                ? ifc_i8253_data_out :  // interface i8253 
        (ifc_k2_OE       & ~ior_n & mouseEnabled) ? MouseData_OUT      :  // Mouse data
      (isEprom         & ~memr_n)               ? data_EPROM_out     :    // EPROM
      (~isEprom        & ~memr_n)               ? data_RAM_out       :    // RAM
        8'hFF;
      
      
//------------------------------- Interface board (ifc) ------------------------------------------------
//
//
reg clk_1Hz; // 1Hz clock - replacement for original U114/MHA1116
reg [25:0] cnt_1Hz;

always @(posedge clk_sys) begin
    if (cnt_1Hz == 26'd0) begin    
      clk_1Hz <= ~clk_1Hz;
      cnt_1Hz <= 26'd12_500_000;
   end else
      cnt_1Hz <= cnt_1Hz - 26'd1;      
end

// I/O range 0x[0-7]0 - 0x[0-7]B if forbidden - collision with internal I/O spaces => only 0x[0-7]C - 0x[0-7]F is usable
// see https://pmd85.borik.net/wiki/Obsadenie_vstupno_v%C3%BDstupn%C3%BDch_adries_PMD_85
wire ifc_CS1_n = ~(address_bus[7:4] == 4'b0001);  // 0x1C - 0x1F ... interface board i8251 - UART
wire ifc_CS4_n = ~(address_bus[7:4] == 4'b0100);  // 0x4C - 0x4F ... interface board i8255 - ports K3 + K4
wire ifc_CS5_n = ~(address_bus[7:4] == 4'b0101);  // 0x5C - 0x5F ... Interface board i8253 - counters 
wire ifc_CS7_n = ~(address_bus[7:4] == 4'b0111);  // 0x7C - 0x7F ... Interface board i8255 - port K5 (not implemented)
wire ifc_k2_OE = address_bus[2] & address_bus[3] & address_bus[7];
wire [7:0] data_bus_K2;
wire ifc_clk0  = mouseEnabled ? ifc_out1 : 
                 MIF85Enabled ? ~phi2    : 1'b0;
wire ifc_gate0 = 1'b1;
wire ifc_out0;
wire ifc_gate1 = 1'b1;
wire ifc_out1;
wire RTS_CTS;

wire [7:0] ifc_PA_K34;
wire [7:0] ifc_PB_K34;
wire [7:0] ifc_PC_K34;
wire [7:0] ifc_i8251_data_out;

i8251 ifc_i8251
(
   .CLK(~phi2),
   .RESET(reset), 
   .CS_n(ifc_CS1_n), 
   .WR_n(iow_n), 
   .RD_n(ior_n), 
   .CD(address_bus[0]), 
   .DIn(databus_from_cpu),
   .DOut(ifc_i8251_data_out),
   .TxD(TxD),
   .TxC_n(ifc_out1),
   .RxD(RxD),
   .RxC_n(ifc_out1),
   .DSR_n(~lineIn),
   .CTS_n(RTS_CTS),
   .RTS_n(RTS_CTS)
);

assign lineOut = ifc_out1 ^ TxD;

(* keep *) wire [7:0] ifc_i8253_data_out;
k580vi53 ifc_i8253
(
   .clk_sys(clk_50M),
   .reset(reset),
   .addr(address_bus[1:0]),
   .din(databus_from_cpu),
   .dout(ifc_i8253_data_out),
   .wr(~iow_n & ~ifc_CS5_n),
   .rd(~ior_n & ~ifc_CS5_n),
   .clk_timer( { clk_1Hz, phi2, ifc_clk0 } ),
   .gate( { 1'b1, ifc_gate1, ifc_gate0 } ),
   .out( { ifc_out1, ifc_out0 } )
);

wire [7:0] ifc_i8255_data_out;
i8255 ifc_i8255_K34
(
   .PA_In(ifc_PA_K34),   
   .PB_In(ifc_PB_K34),   
   .PC_Out(ifc_PC_K34),   
   .D_In(databus_from_cpu), 
   .D_Out(ifc_i8255_data_out),
   .clk(clk_50M), //clk_sys),
   .RD_n(ior_n),
   .WR_n(iow_n),
   .A(address_bus[1:0]),
   .RESET(reset),
   .CS_n(ifc_CS4_n)
);

   
//-------------------------------------------------------------------------------   
// Joystick[4:0] ... FIRE LEFT RIGHT UP DOWN
//
assign ifc_PA_K34 = (ifc_PC_K34[4] & (joystickPort == 2'b01)) ? {3'b111, joystick } : 8'hFF;  //K3 GPIO 0
assign ifc_PB_K34 = (ifc_PC_K34[4] & (joystickPort == 2'b10)) ? {3'b111, joystick } : 8'hFF;  //K3 GPIO 1


//-------------------------------------------------------------------------------   
// Mouse
//
wire [7:0] MouseData_OUT = {MouseRButton, MouseLButton, 2'b00, MouseX2, MouseX1, MouseY1, MouseY2};
wire       MouseLButton;
wire       MouseRButton;
wire       MouseX1;
wire       MouseX2;
wire       MouseY1;
wire       MouseY2;
wire       mouseInt_n    = mouseEnabled ? ifc_out0 : 1'b1;  
 
Mouse602 mouse //#(CLK_FREQ = 26'd18_432_000)
(
   .clk(clk_sys),
   .ps2_mouse(ps2_mouse),
   .Left_Button(MouseLButton),   
   .Right_Button(MouseRButton),
   .Ax(MouseX1),
   .Bx(MouseX2),   
   .Ay(MouseY1),
   .By(MouseY2) 
);

 
//-------------------------------------------------------------------------------   
// Musica - Sound card
//
wire musica_CS_n    = ~(address_bus[7:2] == 6'b1111_11);      // 0xFC - 0xFF
wire musica_Enabled = (audioMode == 2'b10);

k580vi53 musica_i8253
(
   .clk_sys(clk_sys), // clk_sys), 
   .reset(reset),
   .addr(address_bus[1:0]),
   .din(databus_from_cpu),
//   .dout(),
   .wr(~iow_n & ~musica_CS_n & musica_Enabled),
   .rd(1'b0),
   .clk_timer( { ~phi2, ~phi2, ~phi2 } ),
   .gate(3'b111),
   .out(musica_out)
);

//-------------------------------------------------------------------------------   
// MIF85 - Sound card
//
wire       MIF85_CS_n        = ~(address_bus[7:1] == 7'b1110_111);           // 0xEE - 0xEF
wire       MIF85_IntWrite_CS =  (address_bus[7:0] == 8'b1110_1100) & ~iow_n; // 0xEC
reg        MIF85_IntEnable   = 1'b1;
wire       MIF85_Int         = (MIF85Enabled & MIF85_IntEnable) ? (MIF85_cnt == 8'd0) : 1'b1;
reg  [7:0] MIF85_cnt         = 8'd0;
wire       MIF85Enabled      = (audioMode == 2'b01);

 
reg clk_MIF; 
always @(posedge clk_sys) 
   clk_MIF <= ~clk_MIF;
   
always @(posedge clk_sys)  // 12us pulse for INT
begin
   if (~ifc_out0 & MIF85_IntEnable & (MIF85_cnt == 8'd0))
      MIF85_cnt <= 8'd221;
   else
   if (MIF85_cnt != 8'd0)
      MIF85_cnt <= MIF85_cnt - 8'd1;
end

always @(posedge MIF85_IntWrite_CS)
   MIF85_IntEnable <= databus_from_cpu[0];    
  
  
saa1099 MIF85
(
   .clk_sys(clk_sys),
   .ce(clk_MIF),      
   .rst_n(~reset),
   .cs_n(MIF85_CS_n),
   .a0(address_bus[0]),
   .wr_n(iow_n),   
   .din(databus_from_cpu),
   .out_l(MIF85_left_out),
   .out_r(MIF85_right_out)
);  

  
//-------------------------------------------------------------------------------
//  Keyboard + Beeper
//
wire [7:0] PA;
wire [7:0] PB;
wire [7:0] PC;
wire       PCHisInput;
wire [7:0] dataKey8255; 
 
i8255 Key8255
(
   .PA_Out(PA),   
   .PB_In(PB),   
   .PC_Out(PC),   
   .D_In(databus_from_8228),
   .D_Out(dataKey8255),
   .PCHisInput(PCHisInput),
   .clk(clk_50M),
   .RD_n(ior_n),
   .WR_n(iow_n),
   .A(address_bus[1:0]),
   .RESET(reset),
   .CS_n(address_bus[3] )
);
   
keyboard keyboard
(
   .reset(reset), 
   .clk(clk_sys),
   .ps2_key(ps2_key), 
   .row(PA[3:0]), 
   .columns(PB[6:0])
);

assign beeper     = PC[0] & r[9] | PC[1] & r[7] | PC[2];
assign led_red    = allRam_n;

assign led_yellow = beeper;
wire allRam_n     = PCHisInput ? 1'b1 : PC[4] ; // PMD85 v2+3    ... 1 .. ROM, 0 .. RAM only(ALL RAM)
wire ROM_Mirror   = PCHisInput ? 1'b1 : PC[5] ; // PMD85 v3 only ... 1 .. ROM is mirrored over all data RAM space, 0 .. data RAM spave is accessible for read

reg [28:0] resetFromKBD_cnt;
assign     reset_from_keyboard = (resetFromKBD_cnt != 29'd0);

always @(posedge clk_sys)
begin
   if (~(PB[6] | PB[5])) // PB[6] = STOP, PB[5] = SHIFT, STOP+SHIFT=>RESET
      resetFromKBD_cnt <= 29'd18_432_000 / 29'd2; // 500ms reset pulse
   else if (resetFromKBD_cnt != 29'd0)
      resetFromKBD_cnt <= resetFromKBD_cnt - 29'd1;
end


//-------------------------------------------------------------------------------
//  ROMPack module
//
//
wire [15:0] ROMPack_address;
//wire  [7:0] ROMPack_data_out;
wire  [7:0] ROMPack_C_data_out;
(* keep *) wire  [7:0] ROMPack_M_data_out;
wire  [7:0] ROMPack_EPROM_data;


i8255 ROMPack8255
(
   .PA_In(isROMPackEjected ? 8'hFF : ROMPack_EPROM_data), 
   .PB_Out(ROMPack_address[7:0]),   
   .PC_Out(ROMPack_address[15:8]),   
   .D_In(databus_from_cpu), 
   .D_Out(ROMPack_C_data_out),
   .clk(clk_sys),
   .RD_n(ior_n),
   .WR_n(iow_n),
   .A(address_bus[1:0]),
   .RESET(reset),
   .CS_n(address_bus[2])
);
 
dpram #(.ADDRWIDTH(16)) myROMPack
(
   .clock(clk_50M),
   .address_a(ROMPack_address),   
   .wren_a(0),
   .q_a(ROMPack_EPROM_data),
   
   .address_b(ioctl_addr[15:0]),
   .data_b(ioctl_dout),
   .wren_b((ioctl_index[5:0] == 6'd1) & ioctl_wr & ioctl_download & ~RomPackType)
);
 
//-------------------------------------------------------------------------------
//  ROMPack MEGA Module = MM
//   32 pages x 32kB per page = 1MB max size of MEGA ROM file
// 
reg   [7:0] ROMPack_SDRAM_data;
wire  [7:0] MM_PB;
wire  [7:0] MM_PC;
reg   [4:0] MM_Page           = 5'd0;     // 2^5 = 32 pages
wire [19:0] MM_ROMPackAddress = {MM_Page, MM_PC[6:0], MM_PB} ;   // 19:15 = Page, 14:0 address within 32Kb page
wire        ROMPack_read      = (address_bus[2:0] == 3'b000) & ~ior_n;
wire        ROMPack_write     = (address_bus[2:0] == 3'b000) & ~iow_n;
wire        MM_Control        = (address_bus[7:0] == 8'b0110_1111) & ~iow_n; // = 0x6F
reg         isROMPackEjected  = 1'b1;
wire        ROMPackLoad       = (ioctl_wr & ioctl_download);

always @(posedge ROMPackLoad or posedge ROMPackEject)
begin
   if (ROMPackEject)
      isROMPackEjected <= 1'b1;
   else 
      isROMPackEjected <= 1'b0;
end

always @(posedge reset or posedge MM_Control)
begin
   if (reset)
      MM_Page <= 5'd0;
    else
      MM_Page <= databus_from_cpu[4:0];
end

i8255 MM_ROMPack8255
(
   .PA_In(isROMPackEjected ? 8'hFF : sdram_out),
   .PB_Out(MM_PB),
   .PC_Out(MM_PC),
   .D_In(databus_from_cpu), 
   .D_Out(ROMPack_M_data_out),
   .clk(clk_sys),
   .RD_n(ior_n),
   .WR_n(iow_n),
   .A( {address_bus[1], address_bus[0]} ),
   .RESET(reset),
   .CS_n(address_bus[2])
); 
 
//-------------------------------------------------------------------------------
// RAS + CAS + AMUX + STB + VIDEO signal generator - originaly with 74LS164
//
reg [7:0] clk_shift;
reg       VIDEO  = 1'b0;               //  VIDEO = 0 => cpu address; VIDEO = 1 => video address (+refresh)
wire      RAS    = clk_shift[2];
wire      CAS    = clk_shift[4];
wire      STB    = ( VIDEO & clk_shift[4] & clk_shift[6] );
wire      AMUX   = clk_shift[7];
assign clk_video = ~( ~( clk_shift[1] & clk_shift[7] ) & // IC38D
                      ~( clk_shift[1] & clk_shift[4] ) & // IC38B
                      ~( clk_shift[3] & clk_shift[7] )); // IC38C

always @(posedge clk_sys) 
   clk_shift <= { clk_shift[6:0], phi2 };  
   
always @(negedge CAS)
   VIDEO <= ~VIDEO; 

   
//-------------------------------------------------------------------------------
// refresh (+ video) address generator - originaly with 4x 74LS93 (sensitive on negative edge)
//
reg [14:0] r      = 15'd0; // refresh + video mem address
reg [14:0] rNext;

//**** debug purposes only **//
wire [5:0] video_column    = r [5:0];
wire [8:0] video_micro_row = r[14:5];
//**** debug purposes only **//

always @(negedge STB)
begin
   if (rNext[12] & rNext[14]) begin
      r <= 15'd0;
      rNext <= 15'd1;
   end
   else begin
      r <= rNext;
      rNext <= rNext + 15'd1;
   end      
end


//-------------------------------------------------------------------------------
// CS + CAS7 coder - originaly made with 2x 3205
//


wire CAS7_n_v2A  = ~(( ~memr_n | ~memw_n | VIDEO ) & CAS & ~isEprom_v2A ); 
wire isEprom_v2A = ( ~memr_n & ~address_bus[12] & ~address_bus[14] & allRam_n & ~VIDEO & ( address_bus[15] | postReset ) );
wire isEprom_v3  = ~( (~(address_bus[15:13] == 3'b111) & ~ROM_Mirror) | memr_n | VIDEO | ~memw_n | (~allRam_n & ~ROM_Mirror) );
wire CAS7_n_v3   = ~(CAS & ECAS);
wire ECAS_wire   = ( (~(address_bus[15:13] == 3'b111) & ~ROM_Mirror & ~memr_n & memw_n) | 
                     ( memr_n & ~memw_n     & ~VIDEO ) | 
                     ( memr_n & ~ROM_Mirror &  VIDEO ) |
                     ( memw_n & ~ROM_Mirror &  VIDEO ) |
                     (~memr_n &  memw_n     & ~allRam_n & ~ROM_Mirror) );
reg ECAS;
always @(posedge clk_shift[3])
   ECAS <= ECAS_wire;
wire CAS7_n  = PMD_version ? CAS7_n_v3  : CAS7_n_v2A;
wire isEprom = PMD_version ? isEprom_v3 : isEprom_v2A;

reg  postReset = 1'b1 ;           
always @(posedge clk_sys) 
begin
   if (reset)
      postReset <= 1;
   else if (~iow_n) 
      postReset <= 0;
end   


//------------------------------- ADDRESS MULTIPLEXER + SWITCHER ------------------------------------------------
//
// RAM addresses mux - originaly made with 4x 74LS153
// AMUX = 0 => cols; AMUX = 1 => rows
// VIDEO = 0 => cpu address; VIDEO = 1 => refresh + video address
//wire [7:0] addrRam = // this is address shown to DRAM module
//    (( {AMUX, VIDEO} ) == 2'b00) ? address_bus[14:7] :  // address cols
//    (( {AMUX, VIDEO} ) == 2'b01) ? { 1'b1, r[13:7] } :  // refresh + video cols
//    (( {AMUX, VIDEO} ) == 2'b10) ? { address_bus[15], address_bus[6:0] } :  // address rows
//    (( {AMUX, VIDEO} ) == 2'b11) ? { 1'b1, r[6:0] } : 8'bzzzzzzz;  //refresh + video cols


wire [15:0] addrRamMAX = // this is address shown to RAM module
   (VIDEO) ? { 2'b11, r[13:0] } :  // refresh + video cols
             address_bus[15:0] ;   // CPU address
    
//-------------------------------------------------------------------------------------------------------------
// VIDEOPROCESSOR
//
// MOD = Modularní videosignál = pixel
// SD_n = Vertical Sync   .... 20ms perioda .... aktivní v 0 ....
// SR_n = Horizontal Sync .... 64us perioda .... aktivní v 0 .... 4us=0 + 60us=1
// ZAT_n = blank signal (ZAT_n ... 1 = display ON, 0 = display off) 
wire      blink          = blinkCounter[9];
wire      rowActiveReset = (  r[5] & r[4] & r[0] ); // this is when row is beyond display area ... from 49 character/byte
wire      rowActiveSet   = ( ~r[5]        & r[0] ); // this is when row begins to be active    ... from 65 character/byte
reg       rowActive      = 1'b0;
wire      rowActive_n    = ~rowActive;
reg [5:0] pixelBuffer; // 6 pixels to be rolled out with video clock
reg [1:0] pixelFunction; // attributes for these 6 pixels; pixelFunction[1] = F2, pixelFunction[0] = F1


assign SD_n  = ~( r[14] & (r[11:8] == 4'b1000) );   //  SD_n = 0 .... 0x4800 - 0x4900 Video RAM = r counter
assign SR_n  = ~( (r[3:2] == 2'b01) & rowActive_n );//  SR_n = 0 ...
assign ZAT_n =  ( ~r[14] & rowActive );

always @(posedge rowActiveReset or posedge rowActiveSet)
begin
   if (rowActiveReset)
      rowActive <= 0;
   else if (rowActiveSet) 
      rowActive <= 1;
end   


// blink signal for pixel function bits (should be 500ms)
reg [13:0] blinkCounter;
always @(posedge r[9]) begin
   blinkCounter <= blinkCounter + 1'b1;
end

always @(posedge clk_video)
begin
   if (STB)   
   begin
      pixelBuffer           <= data_VRAM_out[5:0];
      pixelFunction         <=  data_VRAM_out[7:6];
      colorAcePixelFunction <= colorAceRAMData[7:6];

      if ((video_column == 6'd0) & (~r[14]))
        ZAT_n_XXX <= 1;
      else if (video_column == 6'd48)
        ZAT_n_XXX <= 0;
   end     
   else
      pixelBuffer           <= { 1'b0, pixelBuffer[5:1] };
end

assign pixel = pixelBuffer[0] & ZAT_n_XXX;

//------------------------final color assignment----------------------------
assign VGA_R = (ColorMode == 2'b00) ? ColorGreen_R : 
               (ColorMode == 2'b01) ? ColorTV_R : 
               (ColorMode == 2'b10) ? ColorRGB_R : 
               ColorAce_R;
 
assign VGA_G = (ColorMode == 2'b00) ? ColorGreen_G : 
               (ColorMode == 2'b01) ? ColorTV_G : 
               (ColorMode == 2'b10) ? ColorRGB_G : 
               ColorAce_G;
               
assign VGA_B = (ColorMode == 2'b00) ? ColorGreen_B : 
               (ColorMode == 2'b01) ? ColorTV_B : 
               (ColorMode == 2'b10) ? ColorRGB_B : 
               ColorAce_B;

//-----------------------------Color Green----------------------------------
wire [7:0] ColorGreen_R = 8'h00;
wire [7:0] ColorGreen_G = (pixel & (pixelFunction[1] ? blink : 1'b1)) ? (pixelFunction[0] ? 8'h80 : 8'hFF): 8'h00;
wire [7:0] ColorGreen_B = 8'h00;

//-----------------------------Color TV-------------------------------------
//7 6    TV                         TV - PMD 85-3       RGB
//--------------------------------------------------------------------------
//0 0    normálny jas               #FFFFFF            #008000 zelený
//0 1    znízený jas                #DFDFDF            #FF0000 cervený
//1 0    normálny jas s blikaním    #BFBFBF            #0000FF modrý
//1 1    znízený jas s blikaním     #9F9F9F            #FF00FF ruzový
wire [7:0] ColorTV_R = ~pixel                  ? 8'h00 : 
                        pixelFunction == 2'b00 ? 8'hFF : 
                        pixelFunction == 2'b01 ? 8'hDF :
                        pixelFunction == 2'b10 ? 8'hBF : 8'h9F;                     
wire [7:0] ColorTV_G = ColorTV_R;
wire [7:0] ColorTV_B = ColorTV_R;

//-----------------------------Color RGB------------------------------------
wire [7:0] ColorRGB_R = pixel & pixelFunction[0]         ? 8'hFF : 8'h00;
wire [7:0] ColorRGB_G = pixel & (pixelFunction == 2'b00) ? 8'h80 : 8'h00;
wire [7:0] ColorRGB_B = pixel & pixelFunction[1]         ? 8'hFF : 8'h00;

//----------------------------- Color ACE ----------------------------------
wire [15:0] colorAceAddr              = {addrRamMAX[15:7], ~addrRamMAX[6], addrRamMAX[5:0]};
wire  [7:0] colorAceRAMData;
reg   [1:0] colorAcePixelFunction;
reg   [1:0] colorAcePixelFunctionRAM;

wire  [7:0] ColorAce_R = ((pixelFunction[0] | colorAcePixelFunction[0]) & pixel)                 ? 8'hFF : 8'h00;
wire  [7:0] ColorAce_G = (((pixelFunction == 2'b00) | (colorAcePixelFunction == 2'b00)) & pixel) ? 8'hFF : 8'h00;
wire  [7:0] ColorAce_B = ((pixelFunction[1] | colorAcePixelFunction[1]) & pixel)                 ? 8'hFF : 8'h00;

//-------------------------------------- EPROM -----------------------------
//
wire [7:0] data_EPROM_out = PMD_version ? data_EPROM_out_v3 : data_EPROM_out_v2A;
wire [7:0] data_EPROM_out_v2A;
wire [7:0] data_EPROM_out_v3;

dpram #(.ADDRWIDTH(12), .MEM_INIT_FILE("../ROM/monit2A.mif")) myEPPROM_v2A
(
   .clock(clk_50M), //clk_sys),
   .address_a(address_bus[11:0]),
   .wren_a(0),
   .q_a(data_EPROM_out_v2A)
);

dpram #(.ADDRWIDTH(13), .MEM_INIT_FILE("../ROM/monit3.mif")) myEPPROM_v3
(
   .clock(clk_50M),
   .address_a(address_bus[12:0]),
   .wren_a(0),
   .q_a(data_EPROM_out_v3)
);


//-------------------------------------- RAM -------------------------------
//
wire [7:0] data_RAM_out;
wire [7:0] data_VRAM_out = data_RAM_out;
 
//
wire memr_RAS_CAS = RAS & CAS & ~memr_n; // & ~VIDEO; // tohle funguje jaks taks
wire memw_RAS_CAS = RAS & CAS & ~memw_n & ~VIDEO;  

dpram #(.ADDRWIDTH(16)) myRam
(
   .clock(clk_50M),
   .address_a(addrRamMAX),
   .data_a(databus_from_8228),
   .wren_a(memw_RAS_CAS),
   .q_a(data_RAM_out),

   // data for ColorACE
   .address_b(colorAceAddr),
   .q_b(colorAceRAMData),
   .wren_b(0)
);


//*************************************************************************************

assign sdram_in     = ioctl_dout;
assign sdram_a      = ioctl_download ? {4'b0010, ioctl_addr[19:0],  1'b0} :
                                       {4'b0010, MM_ROMPackAddress, 1'b0};
assign sdram_we     = ioctl_wr & ioctl_download & RomPackType;
assign sdram_rd     = ~ioctl_download & ROMPack_read & RomPackType;
assign ioctl_wait = (ioctl_download & ~sdram_ready & RomPackType);

endmodule // PMD85_core