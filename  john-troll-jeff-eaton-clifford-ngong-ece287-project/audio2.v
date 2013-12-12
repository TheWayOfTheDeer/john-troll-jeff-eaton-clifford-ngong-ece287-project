module audio2(

	// credit to John Loomis for the majority of the code in this project
	// LCD Control Code - http://www.johnloomis.org/digitallab/lcdlab/lcdlab3/lcdlab3.html
	// Audio Codec Code - http://www.johnloomis.org/digitallab/audio/audio2/audio2.html
	// PS/2 Keyboard Code - http://www.johnloomis.org/digitallab/ps2lab1/ps2lab1.html
	
	// Jeff Eaton + John Troll + Clifford Ngong
	
  // Clock Input (50 MHz)
  input CLOCK_50, // 50 MHz
  input CLOCK_27, // 27 MHz
  //  Push Buttons
  input  [3:0]  KEY,
  //  DPDT Switches 
  input  [17:0]  SW,
  //  7-SEG Displays
  output  [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
  //  LEDs
  output  [8:0]  LEDG,  //  LED Green[8:0]
  output  [17:0]  LEDR, //  LED Red[17:0]
  // TV Decoder
  output TD_RESET, // TV Decoder Reset
  // I2C
  inout  I2C_SDAT, // I2C Data
  output I2C_SCLK, // I2C Clock
  // Audio CODEC
  output/*inout*/ AUD_ADCLRCK, // Audio CODEC ADC LR Clock
  input	 AUD_ADCDAT,  // Audio CODEC ADC Data
  output /*inout*/  AUD_DACLRCK, // Audio CODEC DAC LR Clock
  output AUD_DACDAT,  // Audio CODEC DAC Data
  inout	 AUD_BCLK,    // Audio CODEC Bit-Stream Clock
  output AUD_XCK,     // Audio CODEC Chip Clock
  
  //	LCD Module 16X2
  output LCD_ON,	// LCD Power ON/OFF
  output LCD_BLON,	// LCD Back Light ON/OFF
  output LCD_RW,	// LCD Read/Write Select, 0 = Write, 1 = Read
  output LCD_EN,	// LCD Enable
  output LCD_RS,	// LCD Command/Data Select, 0 = Command, 1 = Data
  inout [7:0] LCD_DATA,	// LCD Data bus 8 bits
  
  //  GPIO Connections
  inout [35:0]  GPIO_0, GPIO_1,
  
  output reg [7:0] period,
  
  // kb stuff
  input PS2_CLK, PS2_DAT,
  
  output reg[7:0] ScanCode,
  output reg skipNextBit,
  output reg ScanRdy
);


//	All inout port turn to tri-state
assign	GPIO_0		=	36'hzzzzzzzzz;
assign	GPIO_1		=	36'hzzzzzzzzz;

wire [6:0] myclock; // idk if this is used

wire RST;
assign RST = KEY[0];

// reset delay gives some time for peripherals to initialize
wire DLY_RST;
Reset_Delay r0(	.iCLK(CLOCK_50),.oRESET(DLY_RST) );

// Send switches to red leds 
assign LEDR = SW;

// Turn off green leds
assign LEDG = 0;

// Turn off 7-segment displays
parameter BLANK = 7'h7f;
assign HEX0 = BLANK;
assign HEX1 = BLANK;
assign HEX2 = BLANK;
assign HEX3 = BLANK;
assign HEX4 = BLANK;
assign HEX5 = BLANK;
assign HEX6 = BLANK;
assign HEX7 = BLANK;


assign	TD_RESET = 1'b1;  // Enable 27 MHz

// ----------------- AUDIO STUFF -----------------
VGA_Audio_PLL 	p1 (	
	.areset(~DLY_RST),
	.inclk0(CLOCK_27),
	.c0(VGA_CTRL_CLK),
	.c1(AUD_CTRL_CLK),
	.c2(VGA_CLK)
);

I2C_AV_Config u3(	
//	Host Side
  .iCLK(CLOCK_50),
  .iRST_N(KEY[0]),
//	I2C Side
  .I2C_SCLK(I2C_SCLK),
  .I2C_SDAT(I2C_SDAT)	
);

assign	AUD_ADCLRCK	=	AUD_DACLRCK;
assign	AUD_XCK		=	AUD_CTRL_CLK;

audio_clock u4(	
//	Audio Side
   .oAUD_BCK(AUD_BCLK),
   .oAUD_LRCK(AUD_DACLRCK),
//	Control Signals
  .iCLK_18_4(AUD_CTRL_CLK),
   .iRST_N(DLY_RST)	
);

audio_converter u5(
	// Audio side
	.AUD_BCK(AUD_BCLK),       // Audio bit clock
	.AUD_LRCK(AUD_DACLRCK), // left-right clock
	.AUD_ADCDAT(AUD_ADCDAT),
	.AUD_DATA(AUD_DACDAT),
	// Controller side
	.iRST_N(DLY_RST),  // reset
	.AUD_outL(audio_outL),
	.AUD_outR(audio_outR),
	.AUD_inL(audio_inL),
	.AUD_inR(audio_inR)
);

always @(negedge AUD_DACLRCK)
		if (index<period-1'b1 && keyPressed)
			index <= index + 1'b1;
		else
			index <= 8'h00;

assign audio_outL = 15'h0000; // only use the right channel

wire [15:0] audio_inL, audio_inR;
wire [15:0] audio_outL, audio_outR;
wire [15:0] signal;

reg [7:0] index;

sine_table sig1(
	.index(index),
	.signal(audio_outR)
);

// ------------------ KEYBOARD STUFF -------------------

reg Clock;
reg keyPressed;

always @(posedge CLOCK_27) // you'd think you could just use the PS2_CLK but that doesn't work
	Clock = PS2_CLK;

reg[3:0] BitCount;
reg StartBitDetected;

parameter LCD_CHAR_Z = 8'h5A,
		LCD_CHAR_S = 8'h53,
		LCD_CHAR_X = 8'h58,
		LCD_CHAR_D = 8'h44,
		LCD_CHAR_C = 8'h43,
		LCD_CHAR_V = 8'h56,
		LCD_CHAR_G = 8'h47,
		LCD_CHAR_B = 8'h42,
		LCD_CHAR_H = 8'h48,
		LCD_CHAR_N = 8'h4E,
		LCD_CHAR_J = 8'h4A,
		LCD_CHAR_M = 8'h4D,
		LCD_CHAR_COMMA = 8'h2C,
		
		KEY_Z = 8'h1A,
		KEY_S = 8'h1B,
		KEY_X = 8'h22,
		KEY_D = 8'h23,
		KEY_C = 8'h21,
		KEY_V = 8'h2A,
		KEY_G = 8'h34,
		KEY_B = 8'h32,
		KEY_H = 8'h33,
		KEY_N = 8'h31,
		KEY_J = 8'h3B,
		KEY_M = 8'h3A,
		KEY_COMMA = 8'h41;

always @(posedge Clock or negedge RST) // the if-else tabs got messed up somewhere in here
begin
	if(~RST)
	begin
		BitCount <= 0;
		StartBitDetected <= 0;
		ScanCode <= 0;
	end
	else // if reset is off
	begin
		if(PS2_DAT == 0 && StartBitDetected == 0) // if it sees the start data bit
		begin
			StartBitDetected <= 1;
			ScanRdy <= 0;
		end
		else
			if(StartBitDetected)
			begin
				if(skipNextBit==1 && BitCount < 8) // we want to skip the code after a 8'hF0 is received
				begin
					BitCount <= BitCount + 1'b1;
					ScanCode <=  {1'b0, ScanCode[7:1]};
				end
				else
				if(skipNextBit==0 && BitCount < 8)
				begin
					BitCount <= BitCount + 1'b1;
					ScanCode <= {PS2_DAT, ScanCode[7:1]};
				end
				else
				begin
					if(ScanCode == 8'hF0 && BitCount == 8) // 8'hF0 means a key was released
					begin
						skipNextBit <= 1;
						keyPressed <= 0;
					end
					else
					begin // this is where we set the period of the audio signal and enable/disable playing the sound
						skipNextBit <= 0;
						case(ScanCode)
							KEY_Z: // C4
							begin
								period <= 8'd179;
								keyPressed <= 1;
							end
							KEY_S: // C#4
							begin
								period <= 8'd169;
								keyPressed <= 1;
							end
							KEY_X: // D4
							begin
								period <= 8'd160;
								keyPressed <= 1;
							end
							KEY_D: // D#4
							begin
								period <= 8'd151;
								keyPressed <= 1;
							end
							KEY_C: // E4
							begin
								period <= 8'd142;
								keyPressed <= 1;
							end
							KEY_V: // F4
							begin
								period <= 8'd134;
								keyPressed <= 1;
							end
							KEY_G: // F#4
							begin
								period <= 8'd127;
								keyPressed <= 1;
							end
							KEY_B: // G4
							begin
								period <= 8'd120;
								keyPressed <= 1;
							end
							KEY_H: // G#4
							begin
								period <= 8'd113;
								keyPressed <= 1;
							end
							KEY_N: // A4
							begin
								period <= 8'd107;
								keyPressed <= 1;
							end
							KEY_J: // A#4
							begin
								period <= 8'd101;
								keyPressed <= 1;
							end
							KEY_M: // B4
							begin
								period <= 8'd95;
								keyPressed <= 1;
							end
							KEY_COMMA: // C5
							begin
								period <= 8'd90;
								keyPressed <= 1;
							end
							default: // shouldn't happen
							begin
								period <= 8'd179;
								keyPressed <= 0;
							end
						endcase
					end
					
					StartBitDetected <= 0;
					BitCount <= 0;
					ScanRdy <= 1;
				end
			end
		end
	end

// ----------------- LCD DISPLAY STUFF -----------------
assign	LCD_ON = 1'b1;
assign	LCD_BLON	= 1'b1;

reg [7:0] expectedCode;
reg [5:0] noteIndex;
reg [3:0] hex1, hex0;
reg [5:0] numNotes;

LCD_Display u1(
// Host Side
   .iCLK_50MHZ(CLOCK_50),
   .iRST_N(DLY_RST),
   .hex0(hex0),
   .hex1(hex1),
// LCD Side
   .DATA_BUS(LCD_DATA),
   .LCD_RW(LCD_RW),
   .LCD_E(LCD_EN),
   .LCD_RS(LCD_RS),
	.song(SW[1:0])
);

// ----------------- SONG STUFF -----------------
always @(posedge keyPressed or negedge RST)
begin
	if(~RST)
		noteIndex <= 0;
	else
	begin
		// if correct note
		if(ScanCode == expectedCode)
			if(noteIndex < numNotes)
				noteIndex <= noteIndex + 1;
			else
				noteIndex <= 0;
	end
end

always @(*)
begin
	case(SW[1:0])
		2'b00: // twinkle twinkle little star
		begin
		numNotes = 6'd41;
		case(noteIndex)
			6'd0: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // twin
			6'd1: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // kle
			6'd2: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // twin
			6'd3: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // kle
			6'd4: {hex1, hex0, expectedCode} <= {LCD_CHAR_N, KEY_N}; // lit
			6'd5: {hex1, hex0, expectedCode} <= {LCD_CHAR_N, KEY_N}; // tle
			6'd6: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // star
			6'd7: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // how
			6'd8: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // I
			6'd9: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // won
			6'd10: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // der
			6'd11: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // what
			6'd12: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // you
			6'd13: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // are
			6'd14: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // up
			6'd15: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // a
			6'd16: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // bove
			6'd17: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // the
			6'd18: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // world
			6'd19: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // so
			6'd20: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // high
			6'd21: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // like
			6'd22: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // a
			6'd23: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // dia
			6'd24: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // mond
			6'd25: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // in
			6'd26: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // the
			6'd27: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // sky
			6'd28: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // twin
			6'd29: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // kle
			6'd30: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // twin
			6'd31: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // kle
			6'd32: {hex1, hex0, expectedCode} <= {LCD_CHAR_N, KEY_N}; // lit
			6'd33: {hex1, hex0, expectedCode} <= {LCD_CHAR_N, KEY_N}; // tle
			6'd34: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // star
			6'd35: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // how
			6'd36: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // I
			6'd37: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // won
			6'd38: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // der
			6'd39: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // what
			6'd40: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // you
			6'd41: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // are
			default: // shouldn't happen
				{hex1, hex0, expectedCode} <= {LCD_CHAR_COMMA,  KEY_COMMA}; // twin
		endcase
		end
		
		2'b01: // happy birthday
		begin
		numNotes = 6'd24;
		case(noteIndex)
			6'd0: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // hap
			6'd1: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // py
			6'd2: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // birth
			6'd3: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // day
			6'd4: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // to
			6'd5: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // you
			6'd6: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // hap
			6'd7: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // py
			6'd8: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // birth
			6'd9: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // day
			6'd10: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // to
			6'd11: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // you
			6'd12: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // hap
			6'd13: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // py
			6'd14: {hex1, hex0, expectedCode} <= {LCD_CHAR_COMMA, KEY_COMMA}; // birth
			6'd15: {hex1, hex0, expectedCode} <= {LCD_CHAR_N, KEY_N}; // day
			6'd16: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // dear
			6'd17: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // naaa
			6'd18: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // aaame,
			6'd19: {hex1, hex0, expectedCode} <= {LCD_CHAR_J, KEY_J}; // hap
			6'd20: {hex1, hex0, expectedCode} <= {LCD_CHAR_J, KEY_J}; // py
			6'd21: {hex1, hex0, expectedCode} <= {LCD_CHAR_N, KEY_N}; // birth
			6'd22: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // day
			6'd23: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // to
			6'd24: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // you
			default: // shouldn't happen
				{hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // hap
		endcase
		end
		
		2'b10: // mary had a little lamb
		begin
		numNotes = 6'd25;
		case(noteIndex)
			6'd0: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // ma
			6'd1: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // ry
			6'd2: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // had
			6'd3: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // a
			6'd4: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // lit
			6'd5: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // tle
			6'd6: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // lamb,
			6'd7: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // lit
			6'd8: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // tle
			6'd9: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // lamb,
			6'd10: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // lit
			6'd11: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // tle
			6'd12: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // lamb,
			6'd13: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // ma
			6'd14: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // ry
			6'd15: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // had
			6'd16: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // a
			6'd17: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // lit
			6'd18: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // tle
			6'd19: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // lamb
			6'd20: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // whose
			6'd21: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // fleece
			6'd22: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // was
			6'd23: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // white
			6'd24: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // as
			6'd25: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // snow

			default: // shouldn't happen
				{hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // ma
		endcase
		end
		
		2'b11: // jingle bells
		begin
		numNotes = 6'd50;
		case(noteIndex)
			6'd0: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // jin
			6'd1: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // gle
			6'd2: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // bells,
			6'd3: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // jin
			6'd4: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // gle
			6'd5: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // bells,
			6'd6: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // jin
			6'd7: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // gle
			6'd8: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // all
			6'd9: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // the
			6'd10: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // wayyyyy,
			6'd11: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // oh
			6'd12: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // what
			6'd13: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // fun
			6'd14: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // it
			6'd15: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // is
			6'd16: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // to
			6'd17: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // ride
			6'd18: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // on
			6'd19: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // a
			6'd20: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // one
			6'd21: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // horse
			6'd22: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // op
			6'd23: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // en
			6'd24: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // sleigh,
			6'd25: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // hey!
			6'd26: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // jin
			6'd27: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // gle
			6'd28: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // bells,
			6'd29: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // jin
			6'd30: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // gle
			6'd31: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // bells,
			6'd32: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // jin
			6'd33: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // gle
			6'd34: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // all
			6'd35: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // the
			6'd36: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // wayyyyy,
			6'd37: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // oh
			6'd38: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // what
			6'd39: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // fun
			6'd40: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // it
			6'd41: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // is
			6'd42: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // to
			6'd43: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // ride
			6'd44: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // on
			6'd45: {hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // a
			6'd46: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // one
			6'd47: {hex1, hex0, expectedCode} <= {LCD_CHAR_B, KEY_B}; // horse
			6'd48: {hex1, hex0, expectedCode} <= {LCD_CHAR_V, KEY_V}; // op
			6'd49: {hex1, hex0, expectedCode} <= {LCD_CHAR_X, KEY_X}; // en
			6'd50: {hex1, hex0, expectedCode} <= {LCD_CHAR_Z, KEY_Z}; // sleigh
			default: // shouldn't happen
				{hex1, hex0, expectedCode} <= {LCD_CHAR_C, KEY_C}; // jin
		endcase
		end
		
		// all 4 possible cases are covered so I'm not putting a default
		
	endcase
end

endmodule