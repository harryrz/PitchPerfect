
module game_mode_test(CLOCK_50,resetn, ps2_key_data, ps2_key_pressed, speed, CounterValue, Note, current_state, aud_en, score, game_str, notes);
	input CLOCK_50,resetn; 
	input [7:0]ps2_key_data; 
	input ps2_key_pressed;
	input [1:0] speed;
	wire [7:0]keyPressedCode;
	output [11:0]current_state;
	output aud_en;
	wire sel_note_en;
	wire load_str_en;
	output [63:0] notes;
	wire gameCheckDone;
	wire EnableDC;
	wire [30:0]downcount;
	output [63:0]game_str; // to be fed to comparison module
	wire game_check_en;
	output [3:0]score;
	wire [3:0]counter;
	output [3:0]CounterValue;
	output [7:0]Note;
	

	controlpath p1(CLOCK_50, resetn, ps2_key_data, ps2_key_pressed, keyPressedCode, current_state, aud_en, sel_note_en, notes, gameCheckDone, load_str_en, game_check_en);
	RateDivider r1(CLOCK_50, resetn, speed, sel_note_en, EnableDC, downcount);
	GameSelectNotes g1(CLOCK_50, resetn, EnableDC, sel_note_en, notes, CounterValue, Note);
	load_str l1(CLOCK_50, resetn, load_str_en, ps2_key_data, ps2_key_pressed, game_str, counter);
	game_check_str g2(CLOCK_50, game_check_en, notes, game_str, gameCheckDone, score);
endmodule



module RateDivider
#(parameter CLOCK_FREQUENCY = 50000000) (
input ClockIn,
input Resetn,
input [1:0] Speed,
input sel_note_en,
output Enable,
output reg [30:0] downcount
);
    always @ (posedge ClockIn)
    begin
	 if(sel_note_en)
	 begin
	if((!Resetn) || (downcount == 0))
			begin
				if(Speed == 2'b00)
				downcount <= 0;
				else if(Speed == 2'b01)
				downcount <=  24999999;
				else if(Speed == 2'b10)
				downcount <= CLOCK_FREQUENCY - 1;
				else if(Speed == 2'b11)
				downcount <= CLOCK_FREQUENCY * 2 - 1;
			end
	else downcount <= downcount - 1;
	end
    end
	assign Enable = (downcount == 0)?1'b1:1'b0;
endmodule


module GameSelectNotes (
input Clock,
input Resetn,
input EnableDC,
input selNoteEn,
input [63:0] notes,
output [3:0] CounterValue,
output reg[7:0] Note
);
	reg [3:0] counter;
	
	always @ (posedge Clock)
	begin
	    if(!Resetn) counter <= 4'b0000;
            if(selNoteEn)
            begin
                if(counter == 4'd8)
                    counter <= 4'b0000;
		else if(EnableDC)
			begin
			counter <= counter + 1'b1;
			begin
         		if(counter == 4'd0) Note <= notes[63:56];
			else if(counter == 4'd1) Note <= notes[55:48];
			else if(counter == 4'd2) Note <= notes[47:40];
			else if(counter == 4'd3) Note <= notes[39:32];
			else if(counter == 4'd4) Note <= notes[31:24];
			else if(counter == 4'd5) Note <= notes[23:16];
			else if(counter == 4'd6) Note <= notes[15:8];
			else if(counter == 4'd7) Note <= notes[7:0];
                    	end
			end
    		else counter <= counter;
            end
        end
	assign CounterValue = counter;
endmodule



module controlpath(CLOCK_50, resetn, ps2_key_data, ps2_key_pressed, keyPressedCode, current_state, aud_en, sel_note_en, notes, gameCheckDone, load_str_en, game_check_en);
    input CLOCK_50, resetn;
				// this is a register that stores the CODEWORD for 7 different notes (8 bits each), this traindataset will
                               // be fed into an audio output module when in the TR_FIRST_NOTE state and output the audio
    output reg [11:0] current_state;
    reg [11:0] next_state;

    input		    [7:0] ps2_key_data;
    input			ps2_key_pressed;
    input           gameCheckDone;


    output reg [7:0] keyPressedCode; //register to store data
    output reg aud_en, sel_note_en, load_str_en, game_check_en;
    output reg [63:0] notes;


    localparam HOME_SCREEN          = 12'd0,
               TRAIN_MODE           = 12'd1,
               GAME_MODE            = 12'd2,
               TR_FIRST_NOTE        = 12'd3,
               TR_LOAD_FIRST_NOTE   = 12'd4,
               TR_CHECK_FIRST       = 12'd5,
               TR_SECOND_NOTE       = 12'd6,
               TR_LOAD_SECOND_NOTE  = 12'd7,
               TR_CHECK_SECOND      = 12'd8,
               TR_THIRD_NOTE        = 12'd9,
               TR_LOAD_THIRD_NOTE   = 12'd10,
               TR_CHECK_THIRD       = 12'd11,
               TR_FOURTH_NOTE       = 12'd12,
               TR_LOAD_FOURTH_NOTE  = 12'd13,
               TR_CHECK_FOURTH      = 12'd14,
               TR_FIFTH_NOTE        = 12'd15,
               TR_LOAD_FIFTH_NOTE   = 12'd16,
               TR_CHECK_FIFTH       = 12'd17,
               TR_SIXTH_NOTE        = 12'd18,
               TR_LOAD_SIXTH_NOTE   = 12'd19,
               TR_CHECK_SIXTH       = 12'd20,
               TR_SEVENTH_NOTE      = 12'd21,
               TR_LOAD_SEVENTH_NOTE = 12'd22,
               TR_CHECK_SEVENTH     = 12'd23,

               GAME_FIRST_ROUND     = 12'd24,
               GAME_LOAD_FIRST_STR  = 12'd25,
               GAME_CHECK_FIRST     = 12'd26,
					GAME_SECOND_ROUND    = 12'd27,
					GAME_LOAD_SECOND_STR = 12'd28,
               GAME_CHECK_SECOND    = 12'd29,
					GAME_THIRD_ROUND     = 12'd30,
					GAME_LOAD_THIRD_STR  = 12'd31,
               GAME_CHECK_THIRD     = 12'd32,
					GAME_FOURTH_ROUND    = 12'd33,
					GAME_LOAD_FOURTH_STR = 12'd34,
               GAME_CHECK_FOURTH    = 12'd35,
					GAME_FIFTH_ROUND     = 12'd36,
					GAME_LOAD_FIFTH_STR  = 12'd37,
               GAME_CHECK_FIFTH     = 12'd38;



               



    always @ (*)
    begin: state_table
            case(current_state)
                HOME_SCREEN: 
                    begin
                    if(keyPressedCode == 8'b00101100) next_state = TRAIN_MODE; //key = 2c = T
                    else if(keyPressedCode == 8'b00110100) next_state = GAME_MODE; //key = 34 = G
                    else next_state = HOME_SCREEN;
                    end
                TRAIN_MODE: next_state = (keyPressedCode == 8'b00101001)? TR_FIRST_NOTE : TRAIN_MODE; // key = 29 = space key
                TR_FIRST_NOTE: next_state = TR_LOAD_FIRST_NOTE;
                TR_LOAD_FIRST_NOTE: next_state = (ps2_key_pressed)? TR_CHECK_FIRST: TR_LOAD_FIRST_NOTE;
                TR_CHECK_FIRST: next_state = (keyPressedCode == 8'b00100001)? TR_SECOND_NOTE: TR_FIRST_NOTE; // key = 21 = C
                TR_SECOND_NOTE: next_state = TR_LOAD_SECOND_NOTE;
                TR_LOAD_SECOND_NOTE: next_state = (ps2_key_pressed)? TR_CHECK_SECOND: TR_LOAD_SECOND_NOTE;
                TR_CHECK_SECOND: next_state = (keyPressedCode == 8'b00100011)? TR_THIRD_NOTE: TR_SECOND_NOTE; // key = 23 = D 
                TR_THIRD_NOTE: next_state = TR_LOAD_THIRD_NOTE;
                TR_LOAD_THIRD_NOTE: next_state = (ps2_key_pressed)? TR_CHECK_THIRD: TR_LOAD_THIRD_NOTE;
                TR_CHECK_THIRD: next_state = (keyPressedCode == 8'b00100100)? TR_FOURTH_NOTE: TR_THIRD_NOTE; // key = 24 = E
                TR_FOURTH_NOTE: next_state = TR_LOAD_FOURTH_NOTE;
                TR_LOAD_FOURTH_NOTE: next_state = (ps2_key_pressed)? TR_CHECK_FOURTH: TR_LOAD_FOURTH_NOTE;
                TR_CHECK_FOURTH: next_state = (keyPressedCode == 8'b00101011)? TR_FIFTH_NOTE: TR_FOURTH_NOTE; // key = 2B = F
                TR_FIFTH_NOTE: next_state = TR_LOAD_FIFTH_NOTE;
                TR_LOAD_FIFTH_NOTE: next_state = (ps2_key_pressed)? TR_CHECK_FIFTH: TR_LOAD_FIFTH_NOTE;
                TR_CHECK_FIFTH: next_state = (keyPressedCode == 8'b00110100)? TR_SIXTH_NOTE: TR_FIFTH_NOTE; //key = 34 = G
                TR_SIXTH_NOTE: next_state = TR_LOAD_SIXTH_NOTE;
                TR_LOAD_SIXTH_NOTE: next_state = (ps2_key_pressed)? TR_CHECK_SIXTH: TR_LOAD_SIXTH_NOTE;
                TR_CHECK_SIXTH: next_state = (keyPressedCode == 8'b00011100)? TR_SEVENTH_NOTE: TR_SIXTH_NOTE; //key = 1c = A 
                TR_SEVENTH_NOTE: next_state = TR_LOAD_SEVENTH_NOTE;
                TR_LOAD_SEVENTH_NOTE: next_state = (ps2_key_pressed)? TR_CHECK_SEVENTH: TR_LOAD_SEVENTH_NOTE;
                TR_CHECK_SEVENTH: next_state = (keyPressedCode == 8'b00110010)? HOME_SCREEN: TR_SEVENTH_NOTE; //key = 32 = B
                GAME_MODE: next_state = (keyPressedCode == 8'b00101001)? GAME_FIRST_ROUND: GAME_MODE;
                GAME_FIRST_ROUND: next_state = GAME_LOAD_FIRST_STR;
                GAME_LOAD_FIRST_STR: next_state = (keyPressedCode == 8'b00110001) ? GAME_CHECK_FIRST: GAME_LOAD_FIRST_STR; // key = 31 = N for next
                GAME_CHECK_FIRST: next_state = (keyPressedCode == 8'b00101001) ? GAME_SECOND_ROUND: GAME_CHECK_FIRST;
					 GAME_SECOND_ROUND: next_state = GAME_LOAD_SECOND_STR;
					 GAME_LOAD_SECOND_STR: next_state = (keyPressedCode == 8'b00110001) ? GAME_CHECK_SECOND: GAME_LOAD_SECOND_STR;
                GAME_CHECK_SECOND: next_state = (keyPressedCode == 8'b00101001) ? GAME_THIRD_ROUND: GAME_CHECK_SECOND;
					 GAME_THIRD_ROUND: next_state = GAME_LOAD_THIRD_STR;
					 GAME_LOAD_THIRD_STR: next_state = (keyPressedCode == 8'b00110001) ? GAME_CHECK_THIRD: GAME_LOAD_THIRD_STR;
                GAME_CHECK_THIRD: next_state = (keyPressedCode == 8'b00101001) ? GAME_FOURTH_ROUND: GAME_CHECK_THIRD;
					 GAME_FOURTH_ROUND: next_state = GAME_LOAD_FOURTH_STR;
					 GAME_LOAD_FOURTH_STR: next_state = (keyPressedCode == 8'b00110001) ? GAME_CHECK_FOURTH: GAME_LOAD_FOURTH_STR;
                GAME_CHECK_FOURTH: next_state = (keyPressedCode == 8'b00101001) ? GAME_FIFTH_ROUND: GAME_CHECK_FOURTH;
					 GAME_FIFTH_ROUND: next_state = GAME_LOAD_FIFTH_STR;
					 GAME_LOAD_FIFTH_STR: next_state = (keyPressedCode == 8'b00110001) ? GAME_CHECK_FIFTH: GAME_LOAD_FIFTH_STR;
                GAME_CHECK_FIFTH: next_state = HOME_SCREEN;
					 

                default: next_state = HOME_SCREEN;
	     endcase
     end

    always @ (*)
    begin: enable_signals
	 
	
				
      case(current_state)
		HOME_SCREEN:
		begin
		notes = 0;
		aud_en = 0;
		sel_note_en = 0;
		load_str_en = 0;
		game_check_en = 0;
		end

                TR_FIRST_NOTE: 
                begin
                aud_en = 1;
		sel_note_en = 1;
		notes = 64'h2121212121212121;
                //delay = 19'd191109; //C4 Note: 261.63Hz
                end
		
		TR_CHECK_FIRST:
		begin
		aud_en = 0;
		sel_note_en = 0;
		notes = 0;
		end

                TR_SECOND_NOTE:
                begin
                aud_en = 1;
		sel_note_en = 1;
		notes = 64'h2323232323232323;
                //delay = 19'd170265;  //D4 Note: 293.66Hz
                end
		
		TR_CHECK_SECOND:
		begin
		aud_en = 0;
		sel_note_en = 0;
		notes = 0;
		end

                TR_THIRD_NOTE:
                begin
                aud_en = 1;
		sel_note_en = 1;
		notes = 64'h2424242424242424;
                //delay = 19'd151685;  //E4 Note: 326.63Hz
                end

		TR_CHECK_THIRD:
		begin
		aud_en = 0;
		sel_note_en = 0;
		notes = 0;
		end

                TR_FOURTH_NOTE:
                begin
                aud_en = 1;
		sel_note_en = 1;
		notes = 64'h2B2B2B2B2B2B2B2B;
                //delay = 19'd143172;  //F4 Note: 349.23Hz
                end

		TR_CHECK_FOURTH:
		begin
		aud_en = 0;
		sel_note_en = 0;
		notes = 0;
		end


                TR_FIFTH_NOTE:
                begin
                aud_en = 1;
		sel_note_en = 1;
		notes = 64'h3434343434343434;
                //delay = 19'd125771;  //G4 Note: 392.00Hz
                end
		
		TR_CHECK_FIFTH:
		begin
		aud_en = 0;
		sel_note_en = 0;
		notes = 0;
		end

                TR_SIXTH_NOTE:
                begin
                aud_en = 1;
		sel_note_en = 1;
		notes = 64'h1C1C1C1C1C1C1C1C;
                //delay = 19'd113636; //A4 Note: 440.00Hz
                end

		TR_CHECK_SIXTH:
		begin
		aud_en = 0;
		sel_note_en = 0;
		notes = 0;
		end

      TR_SEVENTH_NOTE:
      begin
      aud_en = 1;
		sel_note_en = 1;
		notes = 64'h3232323232323232;
      //delay = 19'd101239;  //B4 Note: 493.88Hz
      end
		
		TR_CHECK_SEVENTH:
		begin
		aud_en = 0;
		sel_note_en = 0;
		notes = 0;
		end

      GAME_FIRST_ROUND:
      begin
      aud_en = 1;
      sel_note_en = 1;
      notes = 64'h2123242B341C3221;
      end
		
		GAME_LOAD_FIRST_STR:
		begin
		load_str_en = 1;
		end
		
		GAME_CHECK_FIRST:
		begin
		aud_en = 0;
		sel_note_en = 0;
		load_str_en = 0;
		game_check_en = 1;
		end
			
		GAME_SECOND_ROUND:
		begin
		aud_en = 1;
		sel_note_en = 1;
		notes = 64'h342B24232123212B;
		end
		
		GAME_LOAD_SECOND_STR:
		begin
		load_str_en = 1;
		end
		
		GAME_CHECK_SECOND:
		begin
		aud_en = 0;
		sel_note_en = 0;
		load_str_en = 0;
		game_check_en = 1;
		end
		
		GAME_THIRD_ROUND:
		begin
		aud_en = 1;
		sel_note_en = 1;
		notes = 64'h23322B34212B3234;
		end
		
		GAME_LOAD_THIRD_STR:
		begin
		load_str_en = 1;
		end
		
		GAME_CHECK_THIRD:
		begin
		aud_en = 0;
		sel_note_en = 0;
		load_str_en = 0;
		game_check_en = 1;
		end
		
		GAME_FOURTH_ROUND:
		begin
		aud_en = 1;
		sel_note_en = 1;
		notes = 64'h23241C321C322124;
		end
		
		GAME_LOAD_FOURTH_STR:
		begin
		load_str_en = 1;
		end
		
		GAME_CHECK_FOURTH:
		begin
		aud_en = 0;
		sel_note_en = 0;
		load_str_en = 0;
		game_check_en = 1;
		end
		
		GAME_FIFTH_ROUND:
		begin
		aud_en = 1;
		sel_note_en = 1;
		notes = 64'h1C2B342334232132;
		end
		
		GAME_LOAD_FIFTH_STR:
		begin
		load_str_en = 1;
		end
		
		GAME_CHECK_FIFTH:
		begin
		aud_en = 0;
		sel_note_en = 0;
		load_str_en = 0;
		game_check_en = 1;
		end
		
		
	   endcase
		//cdefgab 1234567 gfedcdc
    end

                

     always @ (posedge CLOCK_50)
        if(!resetn)
            begin
                keyPressedCode <= 0;
					 //notes <= 0;
                current_state <= HOME_SCREEN;
            end
        else if(ps2_key_pressed)
            begin
                keyPressedCode <= ps2_key_data;
                current_state <= next_state;
            end
        else current_state <= next_state;
               




    
endmodule

module load_str(CLOCK_50, resetn, load_str_en, ps2_key_data, ps2_key_pressed, game_str, counter);
	input CLOCK_50, resetn, load_str_en;
	input [7:0] ps2_key_data;
	input ps2_key_pressed;
	output reg [63:0] game_str;
	output reg [3:0] counter;
	always @(posedge CLOCK_50)
		if(!resetn)
			begin
			game_str <= 0;
			counter <= 0;
			end
		else if(load_str_en)
			begin
			if(counter == 0 && ps2_key_pressed)
				begin
					if(ps2_key_data != 8'h31)
					begin
						game_str[63:56] <= ps2_key_data;
						counter <= counter + 1;
					end
				end
			else if(counter == 1 && ps2_key_pressed)
				begin
					if(ps2_key_data != game_str[63:56])
					begin
						game_str[55:48] <= ps2_key_data;
						counter <= counter + 1;
					end
				end
			else if(counter == 2 && ps2_key_pressed)
				begin
					if(ps2_key_data != game_str[55:48])
						begin
							game_str[47:40] <= ps2_key_data;
							counter <= counter +1;
						end
				end
			else if(counter == 3 && ps2_key_pressed)
				begin
					if(ps2_key_data != game_str[47:40])
						begin
							game_str[39:32] <= ps2_key_data;
							counter <= counter +1;
						end
				end
			else if(counter == 4 && ps2_key_pressed)
				begin
					if(ps2_key_data != game_str[39:32])
						begin
							game_str[31:24] <= ps2_key_data;
							counter <= counter +1;
						end
				end
			else if(counter == 5 && ps2_key_pressed)
				begin
					if(ps2_key_data != game_str[31:24])
						begin
							game_str[23:16] <= ps2_key_data;
							counter <= counter +1;
						end
				end
			else if(counter == 6 && ps2_key_pressed)
				begin
					if(ps2_key_data != game_str[23:16])
						begin
							game_str[15:8] <= ps2_key_data;
							counter <= counter +1;
						end
				end
			else if(counter == 7 && ps2_key_pressed)
				begin
					if(ps2_key_data != game_str[15:8])
						begin
							game_str[7:0] <= ps2_key_data;
							counter <= counter +1;
						end
				end
			else if(counter == 8)
				begin
					counter <= 0;
				end
			end


endmodule

module game_check_str(CLOCK_50, game_check_en, notes, game_str, game_check_done, score);
	input game_check_en, CLOCK_50;
	input [63:0] notes, game_str;
	output reg game_check_done;
	output reg score;
	always @(posedge CLOCK_50)
		if(game_check_en)
			begin
				if(notes == game_str)
					begin
						game_check_done = 1;
						score = 1;
					end
				else
					begin
						game_check_done = 1;
						score = 0;
					end
			end

endmodule



// External module with modifications made

module DE1_SoC_Audio_Example (
	// Inputs
	CLOCK_50,
	KEY,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	FPGA_I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT,

	FPGA_I2C_SCLK,
	SW,
	PS2_CLK,
	PS2_DAT,
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,
	HEX6,
	HEX7
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				CLOCK_50;
input		[3:0]	KEY;
input		[9:0]	SW;

input				AUD_ADCDAT;
//input		[7:0] Note;
//input 	aud_en;


inout				PS2_CLK;
inout				PS2_DAT;

// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

inout				FPGA_I2C_SDAT;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;

output				FPGA_I2C_SCLK;
output      [6:0] HEX0;
output      [6:0] HEX1;
output		[6:0]	HEX2;
output		[6:0]	HEX3;
output		[6:0]	HEX4;
output		[6:0]	HEX5;
output		[6:0]	HEX6;
output		[6:0]	HEX7;

/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;


wire     [7:0] keyPressedCode;
wire     [19:0] current_state;
wire     [3:0] CounterValue;
wire     [7:0] Note;
wire  	aud_en;
wire		[3:0]score;


wire 		[7:0] ps2_key_data;
wire 		ps2_key_pressed;
wire 	   [63:0] game_str;
wire		[63:0] notes;
// Internal Registers

reg [18:0] delay_cnt;
reg [18:0] delay;



reg snd;

//assign HEX2 = 7'h7F;
//assign HEX3 = 7'h7F;
//assign HEX4 = 7'h7F;
//assign HEX5 = 7'h7F;
assign HEX6 = 7'h7F;
assign HEX7 = 7'h7F;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/
always @(*)
	begin
	if(Note == 8'h21)
		delay = {SW[3:0], 19'd191109};
	else if(Note == 8'h23)
		delay = {SW[3:0], 19'd170265};
	else if(Note == 8'h24)
		delay = {SW[3:0], 19'd151685};
	else if(Note == 8'h2B)
		delay = {SW[3:0], 19'd143172};
	else if(Note == 8'h34)
		delay = {SW[3:0], 19'd125771};
	else if(Note == 8'h1C)
		delay = {SW[3:0], 19'd113636};
	else if(Note == 8'h32)
		delay = {SW[3:0], 19'd101239};
		
	end

/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/

always @(posedge CLOCK_50)
	if(aud_en)
	begin
		if(delay_cnt == delay) begin
			delay_cnt <= 0;
			snd <= !snd;
		end else delay_cnt <= delay_cnt + 1;
	end

/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

//assign delay = {SW[3:0], 19'd191109};

wire [31:0] sound = (SW == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;


assign read_audio_in			= audio_in_available & audio_out_allowed;

assign left_channel_audio_out	= left_channel_audio_in+sound;
assign right_channel_audio_out	= right_channel_audio_in+sound;
assign write_audio_out			= audio_in_available & audio_out_allowed;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/
 
PS2_Demo demoio(
	// Inputs
	CLOCK_50,
	KEY[3:0],

	// Bidirectionals
	PS2_CLK,
	PS2_DAT,
	
	// Outputs
	SW[9:0],
	ps2_key_data,
	ps2_key_pressed
);
game_mode_test game(.CLOCK_50(CLOCK_50),.resetn(KEY[1]), .ps2_key_data(ps2_key_data), .ps2_key_pressed(ps2_key_pressed), .speed(SW[9:8]), .CounterValue(CounterValue), .Note(Note), .current_state(current_state), .aud_en(aud_en), .score(score), .game_str(game_str), .notes(notes));


Hexadecimal_To_Seven_Segment Segment0 (
	// Inputs
	.hex_number			(score[3:0]),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX0)
);
Hexadecimal_To_Seven_Segment Segment1 (
	// Inputs
	.hex_number			(4'b0),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX1)
);


Hexadecimal_To_Seven_Segment Segment2 (
	// Inputs
	.hex_number			(current_state[3:0]),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX2)
);
Hexadecimal_To_Seven_Segment Segment3 (
	// Inputs
	.hex_number			(current_state[7:4]),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX3)
);
Hexadecimal_To_Seven_Segment Segment4 (
	// Inputs
	.hex_number			(game_str[3:0]),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX4)
);
Hexadecimal_To_Seven_Segment Segment5 (
	// Inputs
	.hex_number			(game_str[7:4]),

	// Bidirectional

	// Outputs
	.seven_seg_display	(HEX5)
);


Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);

avconf #(.USE_MIC_INPUT(1)) avc (
	.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
	.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
	.CLOCK_50					(CLOCK_50),
	.reset						(~KEY[0])
);

endmodule






