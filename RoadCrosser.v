/**
* RoadCrosser 2037
*
* Created by: 
* 
* First Name: Xiao Lei
* Last Name: Zhang
* Student Number: 998923820
* UofT E-mail Address: excel.zhang@mail.utoronto.ca
*
* First Name: Won Tae
* Last Name: Jung
* Student Number: 1002433208
* UofT E-mail Address: won.jung@mail.utoronto.ca
*
* First Name: Joseph
* Last Name: Dong
* Student Number: 100 334 9272
* UofT E-mail Address: joseph.dong@mail.utoronto.ca
* 
* Roadcrosser 2037 is a 2D pixel based action game implemented using Verilog. 
*
* Basic Controls
* Player is controlled using KEY[3:0]. KEY[3] is up; KEY[2] is down; KEY[1] is left; and KEY[0] is 
* right. SW[9] is an active low game reset. 
*
* Starting a game
* Before game starts, player is allowed to set number of lives between 1 and 15 (Setting life to 0
* will set life to 1 instead.). This is followed by number of cars of type 1, type 2, and type 3 
* respectively. Type 1 cars are the fastest moving cars and type 3 cars are the slowest moving cars.
* Number of cars of each type can be set between 0 and 15. Numbers are set using SW[3:0]. After each 
* setting, player needs to press and release KEY[0] to proceed to setting nextnumber. LEDR light 
* status indicators have been added as a bonus feature to aid player in setting each game option.
*
* Gameplay
* This game starts after setting lives, number of cars of type 1, number of cars of type 2, and number
* of cars of type 3. During this game, the goal of player is to move to the top of the screen while 
* maintaining positive life points. Each collision with a car will decrease the player's life by 1. 
* The game ends either when the player reaches the top of the screen or loses all life points. Moving
* left or right to either end of the screen will make the player appear at the other end. This looped
* motion is supported as long as player is above the VGA on screen HEX display panels.
*
* Score and Lives Display
* Number of lives of player is displayed on HEX2. Score of the player is displayed on HEX0 and HEX1.
* Score is based on current distance of player away from bottom of the screen. This distance is between 
* 0h and 77h. Once game ends, player is allowed to start a new game by setting the game settings mentioned
* above again. Score from previous game is not cleared until next game begins. Score and lives are also 
* displayed on screen at bottom right corner using VGA HEX display panels.
**/

/**
   Global Constant Declarations
**/

// Defines Max coords
`define MAX_X 159
`define MAX_Y 119

// Defines Y range of safe zone
`define SAFE_Y_MIN 100
`define SAFE_Y_MAX 119

// Defines Y range of War zone
`define WAR_Y_MIN 0
`define WAR_Y_MAX 99

// Defines Score display panels constants
`define HEX0_X 157
`define HEX0_Y 114

`define HEX1_X 153
`define HEX1_Y 114

// Defines Life display panel constants
`define HEX2_X 149
`define HEX2_Y 114

`define HEX_PANEL_MIN_X 149
`define HEX_PANEL_MAX_X 159
`define HEX_PANEL_MIN_Y 114
`define HEX_PAENL_MAX_Y 119

// Defines Player spawn location
`define PLAYER_SPAWN_X 80
`define PLAYER_SPAWN_Y 115

// Defines Player color
`define PLAYER_COLOR 1

// Defines Periods of Cars
`define CAR1_CYCLES 26'd799999
`define CAR2_CYCLES 26'd899999
`define CAR3_CYCLES 26'd999999

// Defines Update period of player
`define PLAYER_CYCLES 26'd3099999

// Defines Period of collision grace
`define COLLISION_GRACE_PERIOD 26'd3100000

/**
Inputs: SW, KEY, CLOCK_50

Outputs: HEX0, HEX1, HEX2, VGA module outputs

This module serves as the top module for the RoadCrosser
game. SW[9] is an active low synchronous reset. SW[3:0] are
used for input of lives and number of mobs of each type.
Before game starts, player can press KEY[0] as the go button
after setting the parameters in order: # of Lives, # of cars
of type 1, # of cars of type 2 # of cars of type 3 (which speeds
are from fast to slow.)  After game starts, KEY[3:0] are used
for movements. The game ends either player reaches the top of 
the screen or when player's lives falls down to 0. Each collision
decreases the player's life by 1. Score is output to HEX0 and HEX1.
Number of lives is output to HEX2. The score is not reset on the
display panels after the game ends. It is reset once the next game 
begins.
**/
module RoadCrosser
	(
		CLOCK_50,						//	On Board 50 MHz

		// Your inputs and outputs here
        	KEY,
        	SW,

		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,                                                  //	VGA Blue[9:0]
 		HEX0, HEX1, HEX2, LEDR			
	);

	input	CLOCK_50;				        	//	50 MHz
	input  [9:0] SW;
	input  [3:0] KEY;
   	output [9:0] LEDR;
   	output [6:0] HEX0;  						// declares output to score HEX panel d0
        output [6:0] HEX1;  						// declares output to score HEX panel d1
        output [6:0] HEX2;  						// declares output to lives HEX panel d0

	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				        //	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				        //	VGA Green[9:0]
	output	[9:0]	VGA_B;   				        //	VGA Blue[9:0]
	
        // declares wire for active low reset of this game
	wire resetn;
	assign resetn = SW[9];
        
	// declares the colour, x, y and writeEn wires that are vga inputs from vga buffer module output.
	wire [2:0] colour;
	wire [7:0] x;
	wire [7:0] y;
	wire writeEn;
        
        // declares wire for output of startGame value from master controlpath
        wire startGameOut;

	// Creates an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y[6:0]),
			.plot(writeEn),
			// Signals for the DAC to drive the monitor. 
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
    
     // declares load parameters from master control to RAM
     wire w_load_num_cars;
     wire w_load_num_cars1;
     wire w_load_num_cars2;
     wire w_load_num_cars3;
     wire w_load_lives, w_load_score, w_init_cars_data, w_init_player_data;

     // decalres wire for score reset in memory (1'b1 => reset)
     wire w_reset_score; 

     // decalres wires for load objects parameters from RAM to the corresponding object controls
     wire w_load_car1, w_load_car2, w_load_car3, w_load_player;

     // // declares wires for score output from RAM (digits: LSB at 0; MSB at 7)
     // score is based on Y coordinate
     wire [7:0] w_score; 

     // declares wires for score input into RAM from master control
     wire [7:0] w_score_ram_in;
     
     // declares wires for lives output from ram
     wire [3:0] w_lives;
     
     // declares wires for lives input into ram
     wire [3:0] w_lives_ram_in;
     
     // declares wires for number of cars output from ram
     wire [3:0] w_n_car1_ram_out;
     wire [3:0] w_n_car2_ram_out;
     wire [3:0] w_n_car3_ram_out;

     // declares wires for number of cars inputs into RAM from master control output
     wire [3:0] w_n_car1_ram_in;
     wire [3:0] w_n_car2_ram_in;
     wire [3:0] w_n_car3_ram_in;

     // declares wires for output of car data from RAM to mater control
     wire [359:0] w_x_ram_out;
     wire [359:0] w_y_ram_out;
     wire [134:0] w_color_ram_out;
     
     // decalres wire for reset signal input to ram from master control
     wire w_mem_reset_in;
     
     // declares wire for signal status for start reset processing from control master
     wire w_start_reset_processing;
     
     // declares wires to input SW into master control
     // Resetn for master control is controlled by SW[9] separately without using this wire
     wire [9:0] SW_master_control_in;
     
     // declares wire to input the go singnal into master control
     wire go_master_control_in;
     
     // enables go and SW inputs  iff game is not resetting or running the level
     assign go_master_control_in = (w_start_reset_processing || startGameOut) ? 1'b0 : ~KEY[0];
     assign SW_master_control_in = (w_start_reset_processing || startGameOut) ? 1'b0 : SW;
     
     // declares wires player data outputs from RAM
     wire [7:0] w_player_x_ram_out;
     wire [7:0] w_player_y_ram_out;
     wire [2:0] w_player_color_ram_out;
     
     // declares wires for player data input into RAM from cPlayer control
     wire [7:0] w_player_x_ram_in;
     wire [7:0] w_player_y_ram_in;
     wire [2:0] w_player_color_ram_in;
     
     // declares wires for Car1 data outputs from RAM
     wire [7:0] w_car1_x_ram_out;
     wire [119:0] w_car1_y_ram_out;
     wire [44:0] w_car1_color_ram_out;
     
     // declares wires for Car2 data outputs from RAM
     wire [7:0] w_car2_x_ram_out;
     wire [119:0] w_car2_y_ram_out;
     wire [44:0] w_car2_color_ram_out;

     // declares wires for Car3 data outputs from RAM
     wire [7:0] w_car3_x_ram_out;
     wire [119:0] w_car3_y_ram_out;
     wire [44:0] w_car3_color_ram_out;

     // assigns ram outputs to wires for cars control inputs
     assign w_car1_x_ram_out = w_x_ram_out[7:0];
     assign w_car1_y_ram_out = w_y_ram_out[119:0];
     assign w_car1_color_ram_out = w_color_ram_out[44:0];
     
     assign w_car2_x_ram_out = w_x_ram_out[127:120];
     assign w_car2_y_ram_out = w_y_ram_out[239:120];
     assign w_car2_color_ram_out = w_color_ram_out[89:45];

     assign w_car3_x_ram_out = w_x_ram_out[247:240];
     assign w_car3_y_ram_out = w_y_ram_out[359:240];
     assign w_car3_color_ram_out = w_color_ram_out[134:90];
     

     // declares wires for Car1 data inputs into RAM
     wire [7:0] w_car1_x_ram_in;
     wire [119:0] w_car1_y_ram_in;
     wire [44:0] w_car1_color_ram_in;
    
     // declares wires for Car2 data inputs into RAM
     wire [7:0] w_car2_x_ram_in;
     wire [119:0] w_car2_y_ram_in;
     wire [44:0] w_car2_color_ram_in;

     // declares wires for Car3 data inputs into RAM
     wire [7:0] w_car3_x_ram_in;
     wire [119:0] w_car3_y_ram_in;
     wire [44:0] w_car3_color_ram_in;
     
     // declares wires from divider reset/enable to corresponding cars/player divider reset/enable
     wire car1D_reset, car2D_reset, car3D_reset;
     wire car1D_enable, car2D_enable, car3D_enable;     
     wire playerD_enable, playerD_reset;
      
     // declares wires from divider pulse to cars/player control pulse input
     wire car1D_pulse, car2D_pulse, car3D_pulse, playerD_pulse;

     // declares object reset wire from master control to other controls (active low reset)
     wire w_objects_reset;

     // declares wires connecting collision grace counter module and cMaster (resets are active low)
     wire w_reset_cgrace_pulse1, w_reset_cgrace, w_cgrace_over_pulse;

     // declares wires connecting 90 bit random number generator to memory module
     wire [89:0] rand_to_mem; 
   
     // declares wires from cMaster to VGApath for loading vga values
     wire w_load_vga;
     
     // declares wires for data inputs into VGApath from cMaster
     wire [7:0] w_vgaPath_x_in;
     wire [7:0] w_vgaPath_y_in;
     wire [2:0] w_vgaPath_color_in;
     
     // declares score data output from HEX VGA decoders to master control path
     wire [119:0] w_HEX0_X;
     wire [119:0] w_HEX0_Y;
     wire [0:44] w_HEX0_COLOR;
     
     wire [119:0] w_HEX1_X;
     wire [119:0] w_HEX1_Y;
     wire [0:44] w_HEX1_COLOR;

     wire [119:0] w_HEX2_X;
     wire [119:0] w_HEX2_Y;
     wire [0:44] w_HEX2_COLOR;
    
     // Module instance declartions with wiring are listed below:     
     // declares wiring and instance for master control module
     controlMaster cMaster (.clock(CLOCK_50), .reset_n(resetn), .start_game(startGameOut),
     .load_num_cars(w_load_num_cars), .load_num_cars1(w_load_num_cars1), .load_num_cars2(w_load_num_cars2),
     .load_num_cars3(w_load_num_cars3), .load_lives(w_load_lives), .load_vga(w_load_vga),
     .load_score(w_load_score), .reset_score(w_reset_score), .init_cars_data(w_init_cars_data),
     .init_player_data(w_init_player_data), .n_car1(w_n_car1_ram_out), .n_car2(w_n_car2_ram_out),
     .n_car3(w_n_car3_ram_out), .n_car1_out(w_n_car1_ram_in), .n_car2_out(w_n_car2_ram_in),
     .n_car3_out(w_n_car3_ram_in), .x(w_x_ram_out), .y(w_y_ram_out), .color(w_color_ram_out),
     .playerX(w_player_x_ram_out), .playerY(w_player_y_ram_out), .playerColor(w_player_color_ram_out),
     .score(w_score), .lives(w_lives), .lives_out(w_lives_ram_in), .score_out(w_score_ram_in),
     .go(go_master_control_in), .plot(writeEn), .vga_color(w_vgaPath_color_in), .vga_x(w_vgaPath_x_in),
     .vga_y(w_vgaPath_y_in), .SW_in(SW_master_control_in), .memReset(w_mem_reset_in),
     .start_reset_processing(w_start_reset_processing), .objects_reset(w_objects_reset),
     .collision_grace_over_pulse(w_cgrace_over_pulse), .collision_grace_counter_reset_n(w_reset_cgrace),
     .collision_grace_counter_resetn_pulse1(w_reset_cgrace_pulse1), .HEX0_X(w_HEX0_X), .HEX0_Y(w_HEX0_Y),
     .HEX0_COLOR(w_HEX0_COLOR), .HEX1_X(w_HEX1_X), .HEX1_Y(w_HEX1_Y), .HEX1_COLOR(w_HEX1_COLOR), 
     .HEX2_X(w_HEX2_X), .HEX2_Y(w_HEX2_Y), .HEX2_COLOR(w_HEX2_COLOR), .LEDR(LEDR));
     
     // declares instance for vga data buffer module
     displayOut vgaPath (.clock(CLOCK_50), .reset_n(1'b1), .x(w_vgaPath_x_in), .y(w_vgaPath_y_in),
     .load(w_load_vga), .color(w_vgaPath_color_in), .x_out(x), .y_out(y), .color_out(colour));
     
     // declares instance for player control module
     controlPlayer cPlayer(.clock(CLOCK_50), .reset_n(w_objects_reset), .start_game(startGameOut),
     .reset_divider(playerD_reset), .divider_enable(playerD_enable), .pulse_in(playerD_pulse),
     .up(KEY[3]), .down(KEY[2]), .left(KEY[1]), .right(KEY[0]), .x(w_player_x_ram_out), .y(w_player_y_ram_out),
     .color(w_player_color_ram_out), .load_player(w_load_player), .x_out(w_player_x_ram_in),
     .y_out(w_player_y_ram_in), .color_out(w_player_color_ram_in));

     // declares instance for car control modules of different movement speeds
     controlCar cCar1 (.clock(CLOCK_50), .reset_n(w_objects_reset), .start_game(startGameOut),
     .reset_divider(car1D_reset), .divider_enable(car1D_enable), .pulse_in(car1D_pulse),
     .x(w_car1_x_ram_out), .y(w_car1_y_ram_out), .color(w_car1_color_ram_out), .dir(1'b1),
     .n_cars(w_n_car1_ram_out), .load_car(w_load_car1), .x_out(w_car1_x_ram_in),
     .y_out(w_car1_y_ram_in), .color_out(w_car1_color_ram_in));

     controlCar cCar2 (.clock(CLOCK_50), .reset_n(w_objects_reset), .start_game(startGameOut),
     .reset_divider(car2D_reset), .divider_enable(car2D_enable), .pulse_in(car2D_pulse),
     .x(w_car2_x_ram_out), .y(w_car2_y_ram_out), .color(w_car2_color_ram_out), .dir(1'b0),
     .n_cars(w_n_car2_ram_out), .load_car(w_load_car2), .x_out(w_car2_x_ram_in),
     .y_out(w_car2_y_ram_in), .color_out(w_car2_color_ram_in));

     controlCar cCar3 (.clock(CLOCK_50), .reset_n(w_objects_reset), .start_game(startGameOut),
     .reset_divider(car3D_reset), .divider_enable(car3D_enable), .pulse_in(car3D_pulse), .x(w_car3_x_ram_out),
     .y(w_car3_y_ram_out), .color(w_car3_color_ram_out), .dir(1'b1), .n_cars(w_n_car3_ram_out),
     .load_car(w_load_car3), .x_out(w_car3_x_ram_in), .y_out(w_car3_y_ram_in), .color_out(w_car3_color_ram_in));
     
     // declares instance for memory module with simultaneous loads
     memory RAM(.clock(CLOCK_50), .reset_n(w_mem_reset_in), .x(w_x_ram_out), .y(w_y_ram_out),
     .color(w_color_ram_out), .playerX(w_player_x_ram_out), .playerY(w_player_y_ram_out),
     .playerColor(w_player_color_ram_out), .score(w_score), .lives(w_lives), .n_car1_out(w_n_car1_ram_out),
     .n_car2_out(w_n_car2_ram_out), .n_car3_out(w_n_car3_ram_out), .n_car1_in(w_n_car1_ram_in),
     .n_car2_in(w_n_car2_ram_in), .n_car3_in(w_n_car3_ram_in), .car1_x_in(w_car1_x_ram_in),
     .car2_x_in(w_car2_x_ram_in), .car3_x_in(w_car3_x_ram_in), .car1_y_in(w_car1_y_ram_in),
     .car2_y_in(w_car2_y_ram_in), .car3_y_in(w_car3_y_ram_in), .car1_color_in(w_car1_color_ram_in),
     .car2_color_in(w_car2_color_ram_in), .car3_color_in(w_car3_color_ram_in), .player_x_in(w_player_x_ram_in),
     .player_y_in(w_player_y_ram_in), .player_color_in(w_player_color_ram_in), .lives_in(w_lives_ram_in),
     .score_in(w_score_ram_in), .load_car1(w_load_car1), .load_car2(w_load_car2), .load_car3(w_load_car3),
     .load_num_cars(w_load_num_cars), .load_num_cars1(w_load_num_cars1), .load_num_cars2(w_load_num_cars2),
     .load_num_cars3(w_load_num_cars3), .load_player(w_load_player), .load_lives(w_load_lives),
     .load_score(w_load_score), .reset_score(w_reset_score), .init_cars_data(w_init_cars_data),
     .init_player_data(w_init_player_data), .rand_in(rand_to_mem));
     
     // declares instance for score display on HEX0 and HEX1
     HEXDisplay score1 (.HEX(HEX0[6:0]), .c(w_score[3:0]));
     HEXDisplay score2 (.HEX(HEX1[6:0]), .c(w_score[7:4]));

     // declares instance for number of lives display on HEX2
     HEXDisplay livesHEX (.HEX(HEX2[6:0]), .c(w_lives));
     
     // declares instances for vga hex panel decoders for score
     HEX_VGA d0(.xArray(w_HEX0_X), .yArray(w_HEX0_Y), .offsetX(`HEX0_X), .offsetY(`HEX0_Y),
     .colorArray(w_HEX0_COLOR), .in(w_score[3:0]) );
     HEX_VGA d1(.xArray(w_HEX1_X), .yArray(w_HEX1_Y), .offsetX(`HEX1_X), .offsetY(`HEX1_Y),
     .colorArray(w_HEX1_COLOR), .in(w_score[7:4]) );
     
     // declares instance for vga hex panel decoders for in game life display panel
     HEX_VGA d2(.xArray(w_HEX2_X), .yArray(w_HEX2_Y), .offsetX(`HEX2_X), .offsetY(`HEX2_Y),
     .colorArray(w_HEX2_COLOR), .in(w_lives) );
     
     // declares instances for rate dividers controlling different speeds of 
     // cars and the update rate of the player movements
     RateDivider car1D (.clock(CLOCK_50), .reset_n(car1D_reset), .enable(car1D_enable),
     .period(`CAR1_CYCLES), .pulse(car1D_pulse));  

     RateDivider car2D (.clock(CLOCK_50), .reset_n(car2D_reset), .enable(car2D_enable),
     .period(`CAR2_CYCLES), .pulse(car2D_pulse));  

     RateDivider car3D (.clock(CLOCK_50), .reset_n(car3D_reset), .enable(car3D_enable),
     .period(`CAR3_CYCLES), .pulse(car3D_pulse));  
     
     RateDivider playerD (.clock(CLOCK_50), .reset_n(playerD_reset), .enable(playerD_enable),
     .period(`PLAYER_CYCLES), .pulse(playerD_pulse));  
     
     // declares instance for counter used in collision grace period in master control module
     counter collisionGraceCounter (.clock(CLOCK_50), .reset_n(w_reset_cgrace),
     .reset_n_pulse_1(w_reset_cgrace_pulse1) ,
     .pulse(w_cgrace_over_pulse), .limit(`COLLISION_GRACE_PERIOD));
     
     // declares instance for 90 bit random generator module inspired by online sources
     fibonacci_lfsr_90bit rand(.clk(CLOCK_50),  .rst_n(1'b1),  .data(rand_to_mem));
endmodule

/**
Inputs: clock, reset_n, x, y, SW_in, collision_grace_over_pulse, n_car1,
        n_car2, n_car3, color, playerX, playerY, playerColor, lives,
        score, go, HEX0_X, HEX0_Y, HEX0_COLOR, HEX1_X, HEX1_Y, HEX1_COLOR,
        HEX2_X, HEX2_Y, HEX2_COLOR, LEDR
        
Outputs: plot, cga_color, vga_x, vga_y, memReset, load_num_cars, load_lives, 
         load_vga, load_score, reset_score, init_cars_data, init_player_data, 
         collision_grace_counter_reset_n, collision_grace_counter_resetn_pulse1,
         objects_reset, start_game, start_reset_processing, n_car1_out, n_car2_out,
         n_car3_out, lives_out, score_out, load_num_cars1, load_num_cars2,
         load_num_cars3

This module creates a master control path for this game.
It is driven by CLOCK_50 and has an active low reset. 
Numerous inputs and outputs are avaiable to read or
write to the memory module. Player inputs before the
game starts are also processed here. Once the game
starts, the control path performs updates iff changes
to the objects' positions in memory are observed. The updates
happen in the following order: Clear Previous Objects
On Screen -> Plot New Objects On Screen -> Detect Collision
-> Check Winning Condition. If the game ends during this
process or reset_n is set to 0, the control will jump to
the S_RESET1 state where all other controls and the memory
unit is reset and the screen is cleared for the next game.
Score will not be reset till next game starts to allow
the player to view the score after the game ends. 
**/
module controlMaster(clock, reset_n, start_game, load_num_cars, load_num_cars1,
load_num_cars2 , load_num_cars3, load_lives, load_vga, load_score, reset_score,
init_cars_data, init_player_data, n_car1, n_car2, n_car3, n_car1_out, n_car2_out,
n_car3_out, x, y, color, playerX, playerY, playerColor, score, lives, lives_out,
score_out, go, plot, vga_color, vga_x, vga_y, SW_in, memReset, start_reset_processing,
objects_reset, collision_grace_over_pulse, collision_grace_counter_reset_n, 
collision_grace_counter_resetn_pulse1, HEX0_X, HEX0_Y, HEX0_COLOR, HEX1_X, 
HEX1_Y, HEX1_COLOR, HEX2_X, HEX2_Y, HEX2_COLOR, LEDR);

    input clock, reset_n;
    input [9:0] SW_in;    

    // stores output data to vga module
    output plot;
    output reg [2:0] vga_color;
    output reg [7:0] vga_x;
    output reg [7:0] vga_y;
	 
    // declares LED outputs for input status
    output [9:0] LEDR;
    
    // declares output for controlling memory reset
    output reg memReset;
    
    // inputs 1 iff collision grace period is over
    input collision_grace_over_pulse;
    
    // declares reset output signal to collision grace counter
    output reg collision_grace_counter_reset_n;
    output reg collision_grace_counter_resetn_pulse1;
    
    // temporary variables to store car coord/color during collision detection
    reg [7:0] checkX;
    reg [7:0] checkY;
    reg [2:0] checkColor;

    // declares loading-codes to memory
    output reg load_num_cars, load_lives, load_score, reset_score,
    init_cars_data, init_player_data, load_vga;
    output reg load_num_cars1, load_num_cars2 , load_num_cars3;

    // Updates current player data on the next posedge
    // of the clock
    reg load_currPlayer;
    
    // Updates current car data based on car_index on the positions of current car
    // in the update on the next posedge of the clock
    reg load_currCarData;
    
    // Updates all current cars data based on the value of t_curr_x and t_curr_y
    reg load_currCarsData;

    // declares n-edge triggered reset control output signal to game objects
    output reg objects_reset;    

    // controls start of the game
    output reg start_game;
    
    // declares temp variables storing game statuses and their load values
    reg t_start_game;
    reg load_start_game_status;

    reg t_start_reset_processing;
    reg load_start_reset_processing_status;

    reg t_observed_changes;
    reg load_observed_changes_status;

    // controls start of reset processing
    output reg start_reset_processing;

    // declares input for number of cars of each type from memory
    input [3:0] n_car1;
    input [3:0] n_car2;
    input [3:0] n_car3;

    // declares output of number of cars of each type to memory
    output reg [3:0] n_car1_out;
    output reg [3:0] n_car2_out;
    output reg [3:0] n_car3_out;

    // declares input for x,y coords of each car from memory
    input [359:0] x;
    input [359:0] y;
    
    // declares input for color for each car from memory
    input [134:0] color;

    // input for player data from memory
    input [7:0] playerX;
    input [7:0] playerY;
    input [2:0] playerColor;
    
    // stores coordinates since last graphic update
    reg [359:0] curr_x;
    reg [359:0] curr_y;

    // declares temporary variables for setting the current corrdinates
    // these coordinates are assigned to the current coordinates
    // registers on the next posedge of the clock
    reg [359:0] t_curr_x;
    reg [359:0] t_curr_y;

    // declares temporary variables to store position for current car
    // The corresponding car data in current car data registers
    // car be updated on the next posedge of the clock by setting
    // load_currData to 1'b1
    reg [7:0] t_curr_car_x;
    reg [7:0] t_curr_car_y;

    // stores coordinates since last graphic update
    reg [7:0] curr_playerX;
    reg [7:0] curr_playerY;
    
    // declares temprary registers to store current player coordinates
    // The actual current player coordinate register are
    // updated on the next posedge of the clock.
    reg [7:0] t_curr_playerX;
    reg [7:0] t_curr_playerY;

    // declares input for player lives and score from memory
    input [3:0] lives;
    input [7:0] score;
    
    // declares output for player lives and score to memory
    output reg [3:0] lives_out;
    output reg [7:0] score_out;

    // decalres inputs from HEX Display panel decoders
    input [119:0] HEX0_X;
    input [119:0] HEX0_Y;
    input [0:44] HEX0_COLOR;

    input [119:0] HEX1_X;
    input [119:0] HEX1_Y;
    input [0:44] HEX1_COLOR;

    input [119:0] HEX2_X;
    input [119:0] HEX2_Y;
    input [0:44] HEX2_COLOR;
    
    // declares go button during lives and number of cars selection states
    input go;
    
    // declares state registers
    reg [6:0] current_state;
    reg [6:0] next_state;
    
    // stores index of car during graphic update 
    // (car_index for drawing cars; car_index2 for clearing cars)
    integer car_index;
    integer car_index2;
    
    // declares hex display pixel display counters
    integer HEXD0_CLEAR_INDEX;
    integer HEXD1_CLEAR_INDEX;
    integer HEXD2_CLEAR_INDEX;
    integer HEXD0_INDEX;
    integer HEXD1_INDEX;
    integer HEXD2_INDEX;
    
    // declares loop accumulation variables
    integer i;
    integer j;
    integer k;
    integer l;
    integer m;
    integer o;
    integer p;
    integer q;
    integer r;
    integer s;
    integer t;
    integer u;

    // generates coordinates for y and x (8 bit each) to clear the entire screen
    // during reset
    reg [15:0] counter;

    // controls game update decision based on whether changes to data in memory
    // ocurred
    reg observed_changes;
 
    localparam
               S_LIVES_INPUT            = 7'd0,
               S_LIVES_INPUT_WAIT       = 7'd1,
               S_N_CARS1_INPUT          = 7'd2,
               S_N_CARS1_INPUT_WAIT     = 7'd3,
               S_N_CARS2_INPUT          = 7'd4,
               S_N_CARS2_INPUT_WAIT     = 7'd5,
               S_N_CARS3_INPUT          = 7'd6,
               S_N_CARS3_INPUT_WAIT     = 7'd7,
               S_INIT_DATA              = 7'd8,
               S_INIT_DATA_WAIT         = 7'd9,
               S_UPDATE_GRAPHICS        = 7'd10,
               S_UPDATE_GRAPHICS_CLEAR = 7'd11,
               S_UPDATE_GRAPHICS_CLEAR_CYCLE1 = 7'd24,
               S_UPDATE_GRAPHICS_CLEAR_CYCLE2 = 7'd25,
               S_UPDATE_GRAPHICS_CLEAR_CYCLE3 = 7'd65,
               S_UPDATE_GRAPHICS_CLEAR_CARS_END = 7'd33,
               S_UPDATE_GRAPHICS_CLEAR_PLAYER = 7'd31,
               S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE1 = 7'd32,
               S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE2 = 7'd34, 
               S_UPDATE_GRAPHICS_CLEAR_PLAYER_END = 7'd43,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0 = 7'd44,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE1 = 7'd45,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE2 = 7'd46,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE3 = 7'd47,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_END = 7'd48,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1 = 7'd49,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE1 = 7'd50,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE2 = 7'd51,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE3 = 7'd52,
               S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_END = 7'd53,
               S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES = 7'd66,
               S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE1 = 7'd67, 
               S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE2 = 7'd68,
               S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE3 = 7'd69,
               S_UPDATE_GRAPHICS_CLEAR_END = 7'd18,
               S_UPDATE_GRAPHICS_CARS  = 7'd19,
               S_UPDATE_GRAPHICS_CARS_CYCLE1 = 7'd26,
               S_UPDATE_GRAPHICS_CARS_CYCLE2 = 7'd27,
               S_UPDATE_GRAPHICS_CARS_CYCLE3 = 7'd41,
               S_UPDATE_GRAPHICS_CARS_END  = 7'd20,
               S_UPDATE_GRAPHICS_PLAYER = 7'd12,
               S_UPDATE_GRAPHICS_PLAYER_CYCLE1 = 7'd28,
               S_UPDATE_GRAPHICS_PLAYER_END = 7'd64,
               S_UPDATE_GRAPHICS_HEX_SCORE_D0 = 7'd54,
               S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE1 = 7'd55,
               S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE2 = 7'd56,
               S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE3 = 7'd57,
               S_UPDATE_GRAPHICS_HEX_SCORE_D0_END = 7'd58,
               S_UPDATE_GRAPHICS_HEX_SCORE_D1 = 7'd59,
               S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE1 = 7'd60,
               S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE2 = 7'd61,
               S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE3 = 7'd62,
               S_UPDATE_GRAPHICS_HEX_SCORE_D1_END = 7'd63,
               S_UPDATE_GRAPHICS_HEX_LIVES = 7'd70,
               S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE1 = 7'd71,
               S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE2 = 7'd72,
               S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE3 = 7'd73,
               S_UPDATE_GRAPHICS_HEX_LIVES_END = 7'd74,
               S_UPDATE_GRAPHICS_WAIT   = 7'd13,
               S_COLLISION_DETECTION    = 7'd14,
               S_COLLISION_DETECTION_CYCLE1 = 7'd35,
               S_COLLISION_DETECTION_CYCLE2 = 7'd36,
               S_COLLISION_DETECTION_END = 7'd15,
               S_WIN_DETECTION = 7'd16,
               S_WIN_DETECTION_END = 7'd21,
               S_RESET1 = 7'd17,
               S_RESET1_CYCLE1 = 7'd42, 
               S_CLEAR_SCREEN = 7'd22,
               S_CLEAR_SCREEN_CYCLE1 = 7'd29,
               S_CLEAR_SCREEN_CYCLE2 = 7'd30,
               S_CLEAR_SCREEN_END = 7'd23,
               S_INIT_DATA_WAIT2 = 7'd37,
               S_OBSERVE_INIT = 7'd40,
               S_OBSERVE_CHANGES = 7'd38,
               S_MAKE_DECISION_BASED_ON_OBSERVATIONS = 7'd39;
    
    // initializes registers and variables for master controlpath module           
    initial
    begin
       start_game = 1'b0;
       start_reset_processing = 1'b0;
       collision_grace_counter_reset_n = 1'b1;
       collision_grace_counter_resetn_pulse1 = 1'b1;
       vga_x = 8'b0000_0000;
       vga_y = 8'b0000_0000;
       vga_color = 3'b000;
       car_index = 0;
       car_index2 = 0;
       counter = {16{1'b0}};
       observed_changes = 1'b0;
       i = 0;
       j = 0;
       k = 0;
       l = 0;
       m = 0;
       o = 0;
       p = 0;
       q = 0;
       r = 0;
       s = 0;
       t = 0;
       u = 0;
       HEXD0_CLEAR_INDEX = 0;
       HEXD1_CLEAR_INDEX = 0;
       HEXD2_CLEAR_INDEX = 0;
       HEXD0_INDEX = 0;
       HEXD1_INDEX = 0;
       HEXD2_INDEX = 0; 
    end       
               
    always @(*)
    begin: state_table
       case (current_state)
              S_LIVES_INPUT : next_state = go ? S_LIVES_INPUT_WAIT: S_LIVES_INPUT;
              S_LIVES_INPUT_WAIT: next_state = go ?  S_LIVES_INPUT_WAIT : S_N_CARS1_INPUT;
              S_N_CARS1_INPUT: next_state = go ? S_N_CARS1_INPUT_WAIT : S_N_CARS1_INPUT;
              S_N_CARS1_INPUT_WAIT: next_state = go ? S_N_CARS1_INPUT_WAIT : S_N_CARS2_INPUT;
              S_N_CARS2_INPUT: next_state = go ? S_N_CARS2_INPUT_WAIT : S_N_CARS2_INPUT;
              S_N_CARS2_INPUT_WAIT: next_state = go ? S_N_CARS2_INPUT_WAIT : S_N_CARS3_INPUT;
              S_N_CARS3_INPUT: next_state = go ? S_N_CARS3_INPUT_WAIT : S_N_CARS3_INPUT;
              S_N_CARS3_INPUT_WAIT: next_state = go ? S_N_CARS3_INPUT_WAIT : S_INIT_DATA;
              S_INIT_DATA: next_state = S_INIT_DATA_WAIT;
              S_INIT_DATA_WAIT: next_state = S_INIT_DATA_WAIT2;
              S_INIT_DATA_WAIT2: next_state =  S_UPDATE_GRAPHICS; 
              S_OBSERVE_INIT: next_state = S_OBSERVE_CHANGES;
              S_OBSERVE_CHANGES: next_state = S_MAKE_DECISION_BASED_ON_OBSERVATIONS;
              S_MAKE_DECISION_BASED_ON_OBSERVATIONS : next_state =
                observed_changes ? S_UPDATE_GRAPHICS : S_OBSERVE_INIT;
              S_UPDATE_GRAPHICS: next_state = S_UPDATE_GRAPHICS_CLEAR; 
              S_UPDATE_GRAPHICS_CLEAR: next_state =  S_UPDATE_GRAPHICS_CLEAR_CYCLE1;
              S_UPDATE_GRAPHICS_CLEAR_CYCLE1: next_state = S_UPDATE_GRAPHICS_CLEAR_CYCLE2;
              S_UPDATE_GRAPHICS_CLEAR_CYCLE2: next_state = S_UPDATE_GRAPHICS_CLEAR_CYCLE3;
              S_UPDATE_GRAPHICS_CLEAR_CYCLE3: next_state =
                (car_index2 == 45) ? S_UPDATE_GRAPHICS_CLEAR_CARS_END:S_UPDATE_GRAPHICS_CLEAR;
              S_UPDATE_GRAPHICS_CLEAR_CARS_END: next_state = S_UPDATE_GRAPHICS_CLEAR_PLAYER; 
              S_UPDATE_GRAPHICS_CLEAR_PLAYER: next_state = S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE1;
              S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE1: next_state = S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE2;
              S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE2: next_state =  S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0; 
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE1;
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE1: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE2;
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE2: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE3;
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE3: next_state =
                (HEXD0_CLEAR_INDEX == 15) ? S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_END : S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0;
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_END: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1;
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE1;
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE1: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE2;
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE2: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE3;
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE3: next_state =
                (HEXD1_CLEAR_INDEX == 15) ? S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_END : S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1;
              S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_END: next_state =  S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES;
              S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE1;
	      S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE1: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE2;
              S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE2: next_state = S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE3;
              S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE3: next_state =
                (HEXD2_CLEAR_INDEX == 15) ? S_UPDATE_GRAPHICS_CLEAR_END : S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES;
              S_UPDATE_GRAPHICS_CLEAR_END:  next_state = S_UPDATE_GRAPHICS_CARS;
              S_UPDATE_GRAPHICS_CARS: next_state = S_UPDATE_GRAPHICS_CARS_CYCLE1;
              S_UPDATE_GRAPHICS_CARS_CYCLE1: next_state = S_UPDATE_GRAPHICS_CARS_CYCLE2;
              S_UPDATE_GRAPHICS_CARS_CYCLE2: next_state = S_UPDATE_GRAPHICS_CARS_CYCLE3;
              S_UPDATE_GRAPHICS_CARS_CYCLE3: next_state =
                (car_index == 45) ?  S_UPDATE_GRAPHICS_CARS_END : S_UPDATE_GRAPHICS_CARS;
              S_UPDATE_GRAPHICS_CARS_END: next_state = S_UPDATE_GRAPHICS_PLAYER;
              S_UPDATE_GRAPHICS_PLAYER: next_state = S_UPDATE_GRAPHICS_PLAYER_CYCLE1;
              S_UPDATE_GRAPHICS_PLAYER_CYCLE1: next_state = S_UPDATE_GRAPHICS_PLAYER_END;
              S_UPDATE_GRAPHICS_PLAYER_END : next_state = S_UPDATE_GRAPHICS_HEX_SCORE_D0; 
              S_UPDATE_GRAPHICS_HEX_SCORE_D0: next_state = S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE1;
              S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE1: next_state = S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE2;
              S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE2: next_state = S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE3;
              S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE3: next_state =
                (HEXD0_INDEX == 15) ? S_UPDATE_GRAPHICS_HEX_SCORE_D0_END : S_UPDATE_GRAPHICS_HEX_SCORE_D0;
              S_UPDATE_GRAPHICS_HEX_SCORE_D0_END:  next_state = S_UPDATE_GRAPHICS_HEX_SCORE_D1;
              S_UPDATE_GRAPHICS_HEX_SCORE_D1: next_state = S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE1;
              S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE1: next_state = S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE2;
              S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE2: next_state = S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE3;
              S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE3: next_state =
                (HEXD1_INDEX == 15) ? S_UPDATE_GRAPHICS_HEX_SCORE_D1_END : S_UPDATE_GRAPHICS_HEX_SCORE_D1;
              S_UPDATE_GRAPHICS_HEX_SCORE_D1_END: next_state = S_UPDATE_GRAPHICS_HEX_LIVES;
              S_UPDATE_GRAPHICS_HEX_LIVES: next_state = S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE1;
	      S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE1: next_state = S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE2;
              S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE2: next_state = S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE3;
              S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE3: next_state =
                (HEXD2_INDEX == 15) ? S_UPDATE_GRAPHICS_HEX_LIVES_END : S_UPDATE_GRAPHICS_HEX_LIVES;
              S_UPDATE_GRAPHICS_HEX_LIVES_END: next_state = S_COLLISION_DETECTION;
              S_COLLISION_DETECTION: next_state = S_COLLISION_DETECTION_CYCLE1;
              S_COLLISION_DETECTION_CYCLE1: next_state = S_COLLISION_DETECTION_CYCLE2;
              S_COLLISION_DETECTION_CYCLE2: next_state = S_COLLISION_DETECTION_END;
              S_COLLISION_DETECTION_END: next_state = (start_game) ?  S_WIN_DETECTION : S_RESET1;
              S_WIN_DETECTION: next_state = S_WIN_DETECTION_END;
              S_WIN_DETECTION_END: next_state = start_game ?  S_OBSERVE_INIT : S_RESET1;
              S_RESET1: next_state = S_RESET1_CYCLE1;
              S_RESET1_CYCLE1: next_state = S_CLEAR_SCREEN;
              S_CLEAR_SCREEN: next_state = S_CLEAR_SCREEN_CYCLE1;
              S_CLEAR_SCREEN_CYCLE1: next_state = S_CLEAR_SCREEN_CYCLE2;
              S_CLEAR_SCREEN_CYCLE2: next_state =
                (counter == 16'b1111_1111_1111_1111) ? S_CLEAR_SCREEN_END : S_CLEAR_SCREEN;
              S_CLEAR_SCREEN_END: next_state = S_LIVES_INPUT;
              default: next_state = S_LIVES_INPUT;         
       endcase
    end

   always @(*)
   begin

      // sets default values for load enable values
      // sets module active low resets to 1 by default
      // Note: reset_score is not an active low reset signal
      load_num_cars = 1'b0;
      load_num_cars1 = 1'b0;
      load_num_cars2 = 1'b0; 
      load_num_cars3 = 1'b0;
      load_lives = 1'b0;
      load_score = 1'b0;
      load_vga = 1'b0;
      load_currPlayer = 1'b0;
      load_currCarData = 1'b0;
      load_currCarsData = 1'b0;
      reset_score = 1'b0;
      init_cars_data = 1'b0;
      init_player_data = 1'b0;
      memReset = 1'b1;
      objects_reset = 1'b1;
      collision_grace_counter_resetn_pulse1 = 1'b1;
      collision_grace_counter_reset_n = 1'b1;
      
      // sets default values for temp game status registers and their load enable values
      t_start_game = 1'b0;
      load_start_game_status = 1'b0;
      t_start_reset_processing = 1'b0;
      load_start_reset_processing_status = 1'b0;
      t_observed_changes = 1'b0;
      load_observed_changes_status = 1'b0;

      // sets default values for VGA buffer inputs
      vga_x = 8'b0000_0000;
      vga_y = 8'b0000_0000;
      vga_color = 3'b000;

      //set default values for temp data registers
      checkX = 8'b0000_0000;
      checkY = 8'b0000_0000;
      checkColor = 3'b000;
      t_curr_car_x = 8'b0000_0000;
      t_curr_car_y = 8'b0000_0000;
      t_curr_x = {360{1'b0}};
      t_curr_y = {360{1'b0}};
      t_curr_playerX = 8'b0000_0000;
      t_curr_playerY = 8'b0000_0000;

      // sets default values for temp outputs to ram module
      lives_out = 4'b0000;
      score_out = 8'b0000_0000;
      n_car1_out = 4'b0000;
      n_car2_out = 4'b0000;
      n_car3_out = 4'b0000;

      // sets default values for loop index variables
      i = 0;
      j = 0;
      k = 0;
      l = 0;
      m = 0;
      o = 0;
      p = 0;
      q = 0;
      r = 0;
      s = 0;
      t = 0;
      u = 0;

      case (current_state)
           S_LIVES_INPUT: begin
                             
                             // loads life input which is between 1 and 15 decimal in base 2
                             lives_out = (SW_in[3:0] == 0) ? 4'b0001 : SW_in[3:0];
                             load_lives = 1'b1;
                          end
           S_LIVES_INPUT_WAIT: begin

                                  // Add code here for future development
                               end
           S_N_CARS1_INPUT: begin
                               n_car1_out = SW_in[3:0];
                               load_num_cars1 = 1'b1;
                            end
           S_N_CARS1_INPUT_WAIT: begin

                                    // Add code here for future development
                                 end
           S_N_CARS2_INPUT: begin
                               n_car2_out = SW_in[3:0];
                               load_num_cars2 = 1'b1;     
                            end 
           S_N_CARS2_INPUT_WAIT: begin

                                    // Add code here for future development
                                 end
           S_N_CARS3_INPUT: begin
                               n_car3_out = SW_in[3:0];
                               load_num_cars3 = 1'b1;
                            end
           S_N_CARS3_INPUT_WAIT: begin
                                    
                                 end
           S_INIT_DATA: begin
                           reset_score = 1'b1;
                           init_cars_data =1'b1;
                           init_player_data = 1'b1;
                        end
           S_INIT_DATA_WAIT: begin
                                // Add code here for future development							  
                             end
           S_INIT_DATA_WAIT2: begin

                                 // sets start_game status to 1
                                 t_start_game = 1'b1;
                                 load_start_game_status = 1'b1;

                                 // Sets current object positions to initial positions
                                 t_curr_x = x;
                                 t_curr_y = y;
                                 t_curr_playerX = playerX;
                                 t_curr_playerY = playerY;
                                 load_currPlayer = 1'b1;
                                 load_currCarsData = 1'b1;
                              end
           S_OBSERVE_INIT: begin

                               // Add code here for future development
                            end
           S_OBSERVE_CHANGES: begin

                                 // sets observed_changes to 1 iff changes 
                                 // in the coordinates or colors are observed
                                 if(curr_x != x || curr_y != y || curr_playerX != playerX 
                                 || curr_playerY != playerY)
                                 begin
                                    t_observed_changes = 1'b1;
                                    load_observed_changes_status = 1'b1;
                                 end
                                 
                              end
           S_MAKE_DECISION_BASED_ON_OBSERVATIONS : begin

                                                      // Add code here for future development
                                                   end
           S_UPDATE_GRAPHICS: begin
                                 t_observed_changes = 1'b0;
                                 load_observed_changes_status = 1'b1;
                              end
           S_UPDATE_GRAPHICS_CLEAR: begin

                                         // prepares coord x,y, color for plot black clear pixel
                                         for (k = 0; k <= 7; k = k + 1)
                                         begin
                                            vga_x[k] = curr_x[car_index2*8 + k];
                                            vga_y[k] = curr_y[car_index2*8 + k];
                                         end
                                         
                                         vga_color = 3'b000;
                                         load_vga = 1'b1;

                                    end
          S_UPDATE_GRAPHICS_CLEAR_CYCLE1: begin

                                             // Add code here for future development
                                             // car_index = car_index + 1 in this state;
                                          end  
          S_UPDATE_GRAPHICS_CLEAR_CYCLE2: begin

                                             // Add code here for future development
                                          end
          S_UPDATE_GRAPHICS_CLEAR_CYCLE3: begin

                                          end
          S_UPDATE_GRAPHICS_CLEAR_CARS_END: begin

                                               // Add code here for future development
                                            end   
          S_UPDATE_GRAPHICS_CLEAR_PLAYER: begin

                                             // clears current player object on the screen
                                             vga_x = curr_playerX;
                                             vga_y = curr_playerY;
                                             vga_color = 3'b000;
                                             load_vga = 1'b1;
                                          end  
          S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE1: begin

                                                    // Add code here for future development
                                                 end
          S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE2: begin

                                                    // Add code here for future development
                                                 end
          S_UPDATE_GRAPHICS_CLEAR_PLAYER_END: begin
                                                
                                              end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0: begin

                                                     // Clears first digit of the score on the screen
                                                     // pixel by pixel
                                                     for(p = 0; p <= 7; p = p + 1)
                                                     begin
                                                          vga_x[p] = HEX0_X[8 * HEXD0_CLEAR_INDEX + p];
                                                          vga_y[p] = HEX0_Y[8 * HEXD0_CLEAR_INDEX + p];
                                                     end
                                                     
                                                     vga_color = 3'b000;
                                                     load_vga = 1'b1;
                                                end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE1: begin

                                                          // Add code here for future development
                                                       end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE2: begin

                                                          // Add code here for future development
                                                       end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE3: begin

                                                          // Add code here for future development
                                                       end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_END: begin

                                                       // Add code here for future development
                                                    end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1: begin

                                                   // clears second digit of the score on screen
                                                   // pixel by pixel
                                                   for(q = 0; q <= 7; q = q + 1)
                                                   begin
                                                        vga_x[q] = HEX1_X[8 * HEXD1_CLEAR_INDEX + q];
                                                        vga_y[q] = HEX1_Y[8 * HEXD1_CLEAR_INDEX + q];
                                                   end
                                                   vga_color = 3'b000;
                                                   load_vga = 1'b1;
                                                end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE1: begin

                                                          // Add code here for future development
                                                       end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE2: begin

                                                          // Add code here for future development
                                                       end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE3: begin

                                                          // Add code here for future development
                                                       end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_END: begin

                                                       // Add code here for future development
                                                    end 
          S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES: begin

                                                // clears HEX Lives Panel on screen pixel by pixel
                                                for(t = 0; t <= 7; t = t + 1)
                                                begin
                                                     vga_x[t] = HEX2_X[8 * HEXD2_CLEAR_INDEX + t];
                                                     vga_y[t] = HEX2_Y[8 * HEXD2_CLEAR_INDEX + t];
                                                end
                                                vga_color = 3'b000;
                                                load_vga = 1'b1;
                                             end
	  S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE1: begin
                                                       
                                                       // Add code here for future development
                                                    end 
	  S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE2: begin
                                                       
                                                       // Add code here for future development
                                                    end
	  S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE3: begin
                                                       
                                                       // Add code here for future development
                                                    end
          S_UPDATE_GRAPHICS_CLEAR_END: begin

                                          // initializes car index for updating cars on the screen
                                          // car_index = 0 in this state
                                          // HEXD2_CLEAR_INDEX = 0 in this state
                                          // Add code here for future development
                                       end
          S_UPDATE_GRAPHICS_CARS: begin
                                     
                                     // updates cars on the screen one by one based on car_index
                                     for (i = 0; i <= 7; i = i + 1)
                                     begin

					// stores coordinate plotted for clearing and observation later
                                        t_curr_car_x[i] = x[car_index * 8 + i];
                                        t_curr_car_y[i] = y[car_index * 8 + i];
                                     end
                                     
                                     // outputs car coordinate to vga module
                                     vga_x = t_curr_car_x;
                                     vga_y = t_curr_car_y;
                                     
                                     // outputs car color to vga module for display bit by bit
                                     for (j = 0; j <= 2; j = j + 1)
                                     begin
                                          vga_color[j] = color[car_index * 3 + j]; 
                                     end
                                     
                                     // loads new data for current car into currCar register
                                     // enables writing to vga buffer
                                     load_currCarData = 1'b1;
                                     load_vga = 1'b1;
                                  end
          S_UPDATE_GRAPHICS_CARS_CYCLE1: begin

                                            // Add code here for future development
                                         end
          S_UPDATE_GRAPHICS_CARS_CYCLE2: begin

                                              // car_index = car_index + 1; in this state
                                              // Add code here for future development
                                         end
          S_UPDATE_GRAPHICS_CARS_CYCLE3: begin

                                            // Add code here for future development
                                         end
          S_UPDATE_GRAPHICS_CARS_END: begin

                                         // car_index = 0; in this state
                                         // Add code here for future development
                                      end
          S_UPDATE_GRAPHICS_PLAYER: begin

                                       // stores current player position for clearing and observation purposes
                                       // in temporary register
                                       t_curr_playerX = playerX;
                                       t_curr_playerY = playerY;
                                       
                                       // outputs current player data to vga module for display
                                       vga_x = playerX;
                                       vga_y = playerY;
                                       vga_color = playerColor;
                                       
                                       // loads vga output and currPlayer data
                                       load_vga = 1'b1;
                                       load_currPlayer = 1'b1;
                                    end
          S_UPDATE_GRAPHICS_PLAYER_CYCLE1: begin

                                              // updates the score
                                              score_out = `MAX_Y - curr_playerY;
                                              load_score = 1'b1;
                                           end
          S_UPDATE_GRAPHICS_PLAYER_END: begin
                                           
                                        end
          S_UPDATE_GRAPHICS_HEX_SCORE_D0: begin
                                             
                                             // updates one pixel of the first digit of score 
                                             for(r = 0; r <= 7; r = r + 1)
                                             begin
                                                  vga_x[r] = HEX0_X[8 * HEXD0_INDEX + r];
                                                  vga_y[r] = HEX0_Y[8 * HEXD0_INDEX + r];
                                             end
                                             
                                             // outputs new color to VGA buffer module 
                                             vga_color[0] = HEX0_COLOR[3 * HEXD0_INDEX];
                                             vga_color[1] = HEX0_COLOR[3 * HEXD0_INDEX + 1];
                                             vga_color[2] = HEX0_COLOR[3 * HEXD0_INDEX + 2];
                                             load_vga = 1'b1;
                                          end
          S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE1: begin
                                                    
 						    // Add code here for future development
                                                 end
          S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE2: begin

						    // Add code here for future development
                                                 end
          S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE3: begin

						    // Add code here for future development
                                                 end
          S_UPDATE_GRAPHICS_HEX_SCORE_D0_END: begin

						 // Add code here for future development
                                              end
          S_UPDATE_GRAPHICS_HEX_SCORE_D1: begin
                                             
                                             // updates one pixel of the second digit of the score
                                             for(s = 0; s <= 7; s = s + 1)
                                             begin
                                                  vga_x[s] = HEX1_X[8 * HEXD1_INDEX + s];
                                                  vga_y[s] = HEX1_Y[8 * HEXD1_INDEX + s];
                                             end
                                             
                                             // outputs new color to VGA buffer module    
                                             vga_color[0] = HEX1_COLOR[3 * HEXD1_INDEX];
                                             vga_color[1] = HEX1_COLOR[3 * HEXD1_INDEX + 1];
                                             vga_color[2] = HEX1_COLOR[3 * HEXD1_INDEX + 2];
                                             load_vga = 1'b1;
                                          end
          S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE1: begin
                                                    
                                                    // Add code here for future development
                                                 end
          S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE2: begin
                                                    
                                                    // Add code here for future development
                                                 end
          S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE3: begin
                                                    
                                                    // Add code here for future development
                                                 end
          S_UPDATE_GRAPHICS_HEX_SCORE_D1_END: begin
                                                 
                                                 // Add code here for future development
                                              end
          S_UPDATE_GRAPHICS_HEX_LIVES:begin

                                         // clears one pixel of the lives HEX Panel on the screen
                                         for(u = 0; u <= 7; u = u + 1)
                                         begin
                                              vga_x[u] = HEX2_X[8 * HEXD2_INDEX + u];
                                              vga_y[u] = HEX2_Y[8 * HEXD2_INDEX + u];
                                         end
                                         
                                         // outputs new color to VGA buffer module 
                                         vga_color[0] = HEX2_COLOR[3 * HEXD2_INDEX];
                                         vga_color[1] = HEX2_COLOR[3 * HEXD2_INDEX + 1];
                                         vga_color[2] = HEX2_COLOR[3 * HEXD2_INDEX + 2];
                                         load_vga = 1'b1;      
                                      end
	  S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE1: begin
                                                  
                                                 // Add code here for future development
                                              end
          S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE2: begin
                                                 
                                                 // Add code here for future development
                                              end
          S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE3: begin
                                                 
                                                 // Add code here for future development
                                              end
          S_UPDATE_GRAPHICS_HEX_LIVES_END: begin
                                                 
                                              // Add code here for future development
                                           end
          S_COLLISION_DETECTION: begin

                                    // Processes collision detection
                                    for (l = 0; l <= 44; l = l + 1)
                                    begin
                                       
                                       // Prepares car coordinate to be checked for collison with player
                                       for (m = 0; m<=7; m=m+1)
                                       begin
                                          checkX[m] = curr_x[8 * l + m];
                                          checkY[m] = curr_y[8 * l + m];
                                       end
 
                                       // Prepares current car's color to be checked
                                       for (o = 0; o<=2; o=o+1)
                                       begin
                                          checkColor[o] = color[3 * l + o];
                                       end

                                       // Allows collision damage iff collision occurs and car
                                       // is not invisible(color black) and collision grace period is over
                                       // Car can be invisible depending on the number of cars set for 
                                       // the game by the player
                                       if(checkX == curr_playerX && checkY == curr_playerY 
                                       && checkColor != 3'b000 && collision_grace_over_pulse)
                                       begin
                                             lives_out = lives - 4'b0001;
                                             load_lives = 1'b1;
                                             
                                             // resets collision grace period counter if collision is detected
                                             if(collision_grace_counter_reset_n == 1'b1)
                                             begin
                                             	collision_grace_counter_reset_n = 1'b0;
                                             end
                                       end
                                    end
                                 end
           S_COLLISION_DETECTION_CYCLE1: begin
			  
                                            // Add code here for future development
                                         end
           S_COLLISION_DETECTION_CYCLE2: begin
                                            
                                            // ends game iff lives register reaches 0
                                            if(lives == 4'b0000)
                                            begin
                                               t_start_game = 1'b0;
                                               load_start_game_status = 1'b1;
                                            end
                                         end
           S_COLLISION_DETECTION_END: begin
                                         // Add code here for future development
                                      end
           S_WIN_DETECTION: begin
                               
                               // ends game if player has reached top of the screen
                               if(curr_playerY == 0)
                               begin
                                  t_start_game = 1'b0;
                                  load_start_game_status = 1'b1;
                               end
 
                            end
           S_WIN_DETECTION_END: begin
                                   // Winning is decided in this state.
                                   // Add code here for future development
                                end
           S_RESET1: begin

                        // counter = 0; in this state

                        // sets start game status to 0      
                        t_start_game = 1'b0;
                        load_start_game_status = 1'b1;
                        
                        // sets start reset processing status to 1
                        t_start_reset_processing = 1'b1;
                        load_start_reset_processing_status = 1'b1;
                        
                        // sets observed changes status to 0
                        t_observed_changes = 1'b0;
                        load_observed_changes_status = 1'b1;
                     end
           S_RESET1_CYCLE1: begin
                               
                               // resets all objects in this game
                               objects_reset = 1'b0;
                               
                               // resets collision grace counter with pulse reset to 1
                               collision_grace_counter_resetn_pulse1 = 1'b0;
                               collision_grace_counter_reset_n = 1'b1;
                            end
           S_CLEAR_SCREEN: begin

                              // prepares black pixel to be plotted
                              // counter[7:0] is for x; counter[15:8] is for y
                              if(counter[7:0] <= `MAX_X && counter[15:8] <= `MAX_Y)
                              begin
                                 vga_x = counter[7:0];
                                 vga_y = counter [15:8];
                                 vga_color = 3'b000;
                              end
                              else
                              begin
                                  vga_x = 8'b0000_0000;
                                  vga_y = 8'b0000_0000;
                                  vga_color = 3'b000;
                              end

                              // enables writing to vga buffer module
                              load_vga = 1'b1;
                           end
         S_CLEAR_SCREEN_CYCLE1: begin
                                   // counter = counter + 1; in this state
                                   // Add code here for future development
                                end
         S_CLEAR_SCREEN_CYCLE2: begin
                                   // Add code here for future development
                                end
         S_CLEAR_SCREEN_END: begin

                                // counter = 0; in this state
                                collision_grace_counter_resetn_pulse1 = 1'b1;
                                memReset = 1'b0;
                                
                                // resets all game status registers below: 
                                t_start_reset_processing = 1'b0;
                                load_start_reset_processing_status = 1'b1;

                                t_start_game = 1'b0;
                                load_start_game_status = 1'b1;
                        
                                t_observed_changes = 1'b0;
                                load_observed_changes_status = 1'b1;
                             end
      endcase
   end
   
   always@(posedge clock)
   begin: state_FFs
        if(!reset_n)
            current_state <= S_RESET1;
        else
            current_state <= next_state;
        
        // Updates counting variables based on current_state
        case (current_state)
             S_INIT_DATA_WAIT2: begin 
                                    HEXD0_CLEAR_INDEX <= 0;
                                    HEXD1_CLEAR_INDEX <= 0;
                                    HEXD2_CLEAR_INDEX <= 0;
                                    HEXD0_INDEX <= 0;
                                    HEXD1_INDEX <= 0; 
                                    HEXD2_INDEX <= 0; 
                                    counter <= {16{1'b0}};
                                    car_index2 <= 0;
                                    car_index <= 0;
                               end
             S_UPDATE_GRAPHICS_CLEAR_CYCLE2: car_index2 <= car_index2 + 1;
             S_UPDATE_GRAPHICS_CARS_CYCLE2: car_index <= car_index + 1;
             S_CLEAR_SCREEN_CYCLE1: counter <= counter + 1;
             S_UPDATE_GRAPHICS: car_index2 <= 0;
             S_UPDATE_GRAPHICS_CARS_END: car_index <= 0;
             S_UPDATE_GRAPHICS_CLEAR_CARS_END : car_index2 <= 0;
             S_UPDATE_GRAPHICS_CLEAR_END: begin
                                               car_index <= 0;
                                               HEXD2_CLEAR_INDEX <= 0;
                                          end
             S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE2: HEXD0_CLEAR_INDEX <= HEXD0_CLEAR_INDEX + 1;
             S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE2: HEXD1_CLEAR_INDEX <= HEXD1_CLEAR_INDEX + 1;
             S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE2: HEXD2_CLEAR_INDEX <= HEXD2_CLEAR_INDEX + 1;
             S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE2: HEXD0_INDEX <= HEXD0_INDEX + 1;
             S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE2: HEXD1_INDEX <= HEXD1_INDEX + 1;
             S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE2: HEXD2_INDEX <= HEXD2_INDEX + 1;
             S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_END: HEXD0_CLEAR_INDEX <= 0;
             S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_END: HEXD1_CLEAR_INDEX <= 0;
             S_UPDATE_GRAPHICS_HEX_SCORE_D0_END: HEXD0_INDEX <= 0;
             S_UPDATE_GRAPHICS_HEX_SCORE_D1_END: HEXD1_INDEX <= 0;
             S_UPDATE_GRAPHICS_HEX_LIVES_END: HEXD2_INDEX <= 0;
             S_RESET1: counter <= 0;
             S_CLEAR_SCREEN_END: counter <= 0;
        endcase

        // Updates current player positions registers if load is enabled or during reset
        if(load_currPlayer || current_state == S_RESET1_CYCLE1)
        begin
            curr_playerX <= t_curr_playerX;
            curr_playerY <= t_curr_playerY;
        end
        
        // Updates position of current car in update loop
        if(load_currCarData)
        begin
            curr_x[car_index*8] <= t_curr_car_x[0];
            curr_x[car_index*8+1] <= t_curr_car_x[1];
            curr_x[car_index*8+2] <= t_curr_car_x[2];
            curr_x[car_index*8+3] <= t_curr_car_x[3];
            curr_x[car_index*8+4] <= t_curr_car_x[4];
            curr_x[car_index*8+5] <= t_curr_car_x[5];
            curr_x[car_index*8+6] <= t_curr_car_x[6];
            curr_x[car_index*8+7] <= t_curr_car_x[7];
            
            curr_y[car_index*8] <= t_curr_car_y[0];
            curr_y[car_index*8+1] <= t_curr_car_y[1];
            curr_y[car_index*8+2] <= t_curr_car_y[2];
            curr_y[car_index*8+3] <= t_curr_car_y[3];
            curr_y[car_index*8+4] <= t_curr_car_y[4];
            curr_y[car_index*8+5] <= t_curr_car_y[5];
            curr_y[car_index*8+6] <= t_curr_car_y[6];
            curr_y[car_index*8+7] <= t_curr_car_y[7];
        end
        
        // updates position of all current cars if load is enabled or during reset
        if(load_currCarsData || current_state == S_RESET1_CYCLE1)
        begin
           curr_x <= t_curr_x;
           curr_y <= t_curr_y;
        end
        
        // loads new start game status if load is enabled
        if(load_start_game_status)
        begin
           start_game <= t_start_game;
        end
        
        // loads new start reset processing status if load is enabled
        if(load_start_reset_processing_status)
        begin
           start_reset_processing <= t_start_reset_processing;
        end
        
        // loads new observed changes status if load is enabled
        if(load_observed_changes_status)
        begin
           observed_changes <= t_observed_changes;
        end
   end 
   
   // enables VGA plot based on current_state of the fsm
   assign plot = (current_state == S_UPDATE_GRAPHICS_CLEAR_CYCLE2 
   || current_state == S_UPDATE_GRAPHICS_CARS_CYCLE2 
   || current_state == S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE2 
   || current_state == S_UPDATE_GRAPHICS_PLAYER_CYCLE1 
   || current_state == S_CLEAR_SCREEN_CYCLE1 
   || current_state == S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D0_CYCLE2 
   || current_state == S_UPDATE_GRAPHICS_CLEAR_HEX_SCORE_D1_CYCLE2
   || current_state == S_UPDATE_GRAPHICS_CLEAR_HEX_LIVES_CYCLE2 
   || current_state == S_UPDATE_GRAPHICS_HEX_SCORE_D0_CYCLE2
   || current_state == S_UPDATE_GRAPHICS_HEX_SCORE_D1_CYCLE2
   || current_state == S_UPDATE_GRAPHICS_HEX_LIVES_CYCLE2) ? 1'b1 : 1'b0;
	
   // Assigns LEDs' values for setting status based on current state
   assign LEDR[0] = (current_state == S_N_CARS1_INPUT 
        || current_state == S_N_CARS1_INPUT_WAIT 
	|| current_state == S_N_CARS2_INPUT_WAIT 
        || current_state == S_N_CARS3_INPUT_WAIT
	|| current_state == S_N_CARS2_INPUT 
        || current_state == S_N_CARS3_INPUT) ? 1'b1 : 1'b0;
   assign LEDR[1] = (current_state == S_N_CARS2_INPUT 
        || current_state == S_N_CARS2_INPUT_WAIT
	|| current_state == S_N_CARS3_INPUT_WAIT
	|| current_state == S_N_CARS3_INPUT) ? 1'b1 : 1'b0;
   assign LEDR[2] = (current_state == S_N_CARS3_INPUT 
	|| current_state == S_N_CARS3_INPUT_WAIT) ? 1'b1 : 1'b0;
endmodule

/**
Inputs: clock, reset_n, x, y, load, color

Outputs: x_out, y_out, color_out

This module creates a data buffer for feeding outputs
to the vga module. New vga outputs are loaded on the
next positive edge of the clock if load is enabled.
Load acts as the write enable signal in the buffer.
This module is driven by CLOCK_50 and has an active-low
reset feature. Outputs to VGA are allowed to last until
the next write enable signal or reset_n = 1'b0;
**/
module displayOut(clock, reset_n, x, y, load, color, x_out, y_out, color_out);
   
   // declares display inputs to be loaded
   input [7:0] x;
   input [7:0] y;
   input [2:0] color;
   
   // declares load parameter
   input load;
   
   // declares clock and active-low reset inputs
   input clock, reset_n;

   // declares output data registers to vga module
   output reg [7:0] x_out;
   output reg [7:0] y_out;
   output reg [2:0] color_out;

   // initializes outputs to vga module
   initial
   begin
      x_out = 8'b0000_0000;
      y_out = 8'b0000_0000;
      color_out = 3'b000;
   end
   
   // processes loading or reset during the positive edge of the clock
   always @(posedge clock)
   begin
      if(!reset_n)
      begin
         x_out <= 8'b0000_0000;
         y_out <= 8'b0000_0000;
         color_out <= 3'b000;
      end
      else if(load)
      begin
    
         // allows new values to pass if load is enabled
         x_out <= x;
         y_out <= y;
         color_out <= color;
      end
   end
endmodule

/**
Inputs: x, y, color, start_game, up, down, left, right

Outputs: reset_divider, divider_enable, x_out, y_out, color_out, load_player

This module serves as a controlpath for the player in the game.
It is driven by CLOCK_50 and has an active low reset. Once
game starts, the control will wait for a pulse from the divider.
Player movements are enabled once this control path receives 
a pulse from the divider module. Player movements are controlled
by KEY[3:0]. Player can only move in one direction at a time. 
This control unit updates the player's positions by writing
to the memory module. Data about the player (x, y, and color) can
also be read from the memory module.
**/
module controlPlayer(clock, reset_n, start_game, reset_divider, divider_enable,
pulse_in, up, down, left, right, x, y, color, load_player, x_out, y_out, color_out); 

    // declares clock, active-low reset and divider pulse input signals
    input clock, reset_n,  pulse_in;
    
    // declares inputs from memory output
    input [7:0] x;
    input [7:0] y;
    input [2:0] color;
    
    // declares divider enable and reset signals
    output reset_divider, divider_enable;
    
    // declares game start signal
    input start_game;     

    // declares output values to memory
    output reg [7:0] x_out;
    output reg [7:0] y_out;
    output reg [2:0] color_out;
    
    // declares load player output signal
    output reg load_player;

    // declares movement inputs from keys
    input up, down, left, right;
    
    reg [4:0] current_state, next_state;

    // loop index variable
    integer i;

    localparam
               S_WAIT          = 5'd0,
               S_WAIT_FOR_PULSE = 5'd1,
               S_UPDATE_INFO   = 5'd2;

    always @(*)
    begin: state_table
       case (current_state)
            S_WAIT:  next_state = start_game ? S_WAIT_FOR_PULSE : S_WAIT;
            S_WAIT_FOR_PULSE: next_state = pulse_in ? S_UPDATE_INFO : S_WAIT_FOR_PULSE;
            S_UPDATE_INFO: next_state = S_WAIT_FOR_PULSE;
            default:     next_state = S_WAIT;
       endcase
    end

    always @(*)
    begin

       // makes all our signals 0 by default
       load_player = 1'b0;
       x_out =8'b0000_0000;
       y_out =8'b0000_0000;
       color_out =3'b000;
       i = 0;
       
       case (current_state)
           S_UPDATE_INFO: begin
                             
                             // updates player position based on key pressed
                             if(!up)
                             begin
                                if(y != 8'b0000_0000)
                                begin
                                   x_out = x;
                                   y_out = y - 8'b0000_0001;
                                   color_out = color;
                                   load_player = 1'b1;
                                end
                            
                             end
                             else if(!down)
                             begin
                                if (x < `HEX_PANEL_MIN_X-1 || y < `HEX_PANEL_MIN_Y-1)
                                begin
                                     if(y != `MAX_Y)
                                     begin
                                          x_out = x;
                                          y_out = y + 8'b0000_0001;
                                          color_out = color;
                                          load_player = 1'b1;
                                     end
                                end
                             end
                             else if(!left)
                             begin
                                if(x != 8'b0000_0000)
                                begin
                                   x_out = x - 8'b0000_0001;
                                   y_out = y;
                                   color_out = color;
                                   load_player = 1'b1;
                                end
				else if(y < `HEX_PANEL_MIN_Y-1)
				begin
                                   
                                   // allows looped motion if player is above vga hex panels
				   x_out = `MAX_X;
                                   y_out = y;
                                   color_out = color;
                                   load_player = 1'b1;
			        end
                             end
                             else if(!right)
                             begin
                                if (x < `HEX_PANEL_MIN_X-1 
                                || y < `HEX_PANEL_MIN_Y-1)
                                begin
                                     if(x != `MAX_X)
                                     begin
                                          x_out = x + 8'b0000_0001;
                                          y_out = y;
                                          color_out = color;
                                          load_player = 1'b1;
                                     end
				     else
				     begin
					  x_out = 8'b0000_0000;
                                          y_out = y;
                                          color_out = color;
                                          load_player = 1'b1;
				     end
                                end
                             end
                        end
       endcase
    end

    // updates current_state register
    always@(posedge clock)
    begin: state_FFs
        if(!reset_n)
            current_state <= S_WAIT;
        else
            current_state <= next_state;
    end // state_FFS
    
    // divider is enabled and not reset iff game starts
    assign divider_enable = start_game ? 1'b1 : 1'b0;
    assign reset_divider = start_game ? 1'b1 : 1'b0;

endmodule

/**
Inputs: clock, reset_n, x, y, color, dir,  n_cars, start-game

Outputs: reset_divider, divider_enable, x_out, y_out, color_out,
         load_car

This module creates a controlpath for car objects in this game.
Each such module controls cars of one speed type. All cars of one 
speed type have the same x position but different y positions on
the screen. This module is driven by CLOCK_50 and has an active-low
reset. Once the game starts, this module will wait for a pulse from
the divider. Updates to the car's positions are performed once pulse
is received from the divider. This module is also connected to the
memory module to allow read/write of cars' data from/to memory. 
Direction of movement can be set using the input dir. The car moves
right if and only if dir = 1'b1;
**/
module controlCar(clock, reset_n, start_game, reset_divider, divider_enable,
pulse_in, x, y, color, dir, n_cars, load_car, x_out, y_out, color_out);

    // declares clock, active-low reset, divider pulse input signals
    input clock, reset_n,  pulse_in;

    // declares inputs for car data from memory output
    input [7:0] x;
    input [119:0] y;
    input [44:0] color;
    input [3:0] n_cars;

    // declares start game input signal from master control unit
    input start_game;
    
    // declares car movement direction: dir = 1 iff car moves right
    input dir;    

    // decalres reset and enable outputs to the divider
    output reset_divider, divider_enable;

    // declares output car data to memory
    output reg [7:0] x_out;
    output reg [119:0] y_out;
    output reg [44:0] color_out;
    
    // declares car1 load signal
    output reg load_car;
    
    reg [4:0] current_state, next_state;
    
    // loop index variable
    integer i;

    localparam  
                S_WAIT          = 5'd0,
                S_WAIT_FOR_PULSE = 5'd1,
                S_UPDATE_INFO   = 5'd2;

    always @(*)
    begin: state_table
       case (current_state)
            S_WAIT:  next_state = start_game ? S_WAIT_FOR_PULSE : S_WAIT;
            S_WAIT_FOR_PULSE: next_state = pulse_in ? S_UPDATE_INFO : S_WAIT_FOR_PULSE;
            S_UPDATE_INFO: next_state = S_WAIT_FOR_PULSE;
            default: next_state = S_WAIT;
       endcase
    end
    
    always @(*)
    begin

       // makes all our inputs 0 by default
       load_car = 1'b0;
       x_out = 0;
       y_out = 0;
       color_out = 0;
       i = 0;

       case (current_state)
           S_UPDATE_INFO: begin
                             if(dir)
                             begin
                             	  if (x != `MAX_X)
                                  begin
                                
                                       // moves car to the right if MAX_X is not reached
                                       x_out = x + 8'b0000_0001;
                                       y_out = y;
                                       color_out = color;
                                       load_car = 1'b1;
                                  end
                                  else
                                  begin

                                       // moves car across the screen and back to x=0 position if 
                                       // MAX_X is reached.
                                       x_out = 8'b0000_0000;
                                       y_out = y;
                                       color_out = color;
                                       load_car = 1'b1;
                                  end
                             end
                             else
                             begin
                                  if (x != 8'b0000_0000)
                                  begin
                                
                                       // moves car to the left if x = 0 is not reached
                                       x_out = x - 8'b0000_0001;
                                       y_out = y;
                                       color_out = color;
                                       load_car = 1'b1;
                                   end
                                   else
                                   begin

                                       // moves car across the screen and back to MAX_X position if 
                                       // x = 0 is reached
                                       x_out = `MAX_X;
                                       y_out = y;
                                       color_out = color;
                                       load_car = 1'b1;
                                   end
                             end
                          end
       endcase
    end

    always@(posedge clock)
    begin: state_FFs
        if(!reset_n)
            current_state <= S_WAIT;
        else
            current_state <= next_state;
    end 
    
    // Divider is enabled and not reset iff game starts
    assign divider_enable = start_game ? 1'b1 : 1'b0;
    assign reset_divider = start_game ? 1'b1 : 1'b0;
   
endmodule

/**
Inputs: clock, reset_n, load_car1, load_car2, load_car3, load_num_cars, load_num_cars1,
        load_num_cars2, load_player, load_lives, load_score, reset_score, init_cars_data,
        init_player_data, n_car1_in, n_car2_in, n_car3_in, car1_x_in, car1_y_in, car2_x_in,
        car2_y_in, car3_x_in, car3_y_in, car1_color_in, car2_color_in, car3_color_in,
        player_x_in, player_y_in, player_color_in, lives_in, score_in, rand_in

Outputs: n_car1_out, n_car2_out, n_car3_out, x, y, color, playerX, playerY, playerColor,
         score, lives

This module creates a memory unit for our game. It also functions as a datapath where
operations allow data to be updated. Data stored inside here can be read and written
by the controlpath modules including master control, car controls, and player control.
This memory unit is driven by CLOCK_50 and has an active-low reset. Multiple pieces of data
can be loaded simultaneously during one clock cycle. Score is not reset by reset_n to allow
the player to view the score after the game ends. Score can be reset by the operation 
reset_score. 8 bit binary numbers are taken from rand_in to initialize the three 8 bit
x coordinates of the 3 car types when game starts.
**/
module memory(clock, reset_n, x, y, color, playerX, playerY, playerColor, score, lives,
 n_car1_out, n_car2_out, n_car3_out, n_car1_in, n_car2_in, n_car3_in, car1_x_in, car2_x_in,
 car3_x_in, car1_y_in, car2_y_in, car3_y_in, car1_color_in, car2_color_in, car3_color_in,
 player_x_in, player_y_in, player_color_in, lives_in, score_in, load_car1, load_car2,
 load_car3, load_num_cars, load_num_cars1, load_num_cars2, load_num_cars3, load_player,
 load_lives, load_score, reset_score, init_cars_data, init_player_data, rand_in);

     // declares CLOCK_50 and active low reset inputs
     input clock, reset_n;
     
     // declares operation signal inputs
     input load_car1, load_car2, load_car3, load_num_cars, load_player, load_lives, load_score,
     reset_score, init_cars_data, init_player_data;
     input load_num_cars1, load_num_cars2, load_num_cars3;

     // declares number of each type of car objects (15 max) input
     input [3:0] n_car1_in;
     input [3:0] n_car2_in;
     input [3:0] n_car3_in;
     
     // declares output of number of cars of each type
     output reg [3:0] n_car1_out;
     output reg [3:0] n_car2_out;
     output reg [3:0] n_car3_out;
     
     // declares temp number of cars of each type to be loaded
     reg [3:0] t_n_car1_out;
     reg [3:0] t_n_car2_out;
     reg [3:0] t_n_car3_out;
     
     // declares inputs for car positions of each type
     input [7:0] car1_x_in;
     input [119:0] car1_y_in;
     
     input [7:0] car2_x_in;
     input [119:0] car2_y_in;

     input [7:0] car3_x_in;
     input [119:0] car3_y_in;
     
     // declares inputs for car colors of each type
     input [44:0] car1_color_in;
     input [44:0] car2_color_in;
     input [44:0] car3_color_in;
     
     // declares player data inputs
     input [7:0] player_x_in;
     input [7:0] player_y_in;
     input [2:0] player_color_in;
     
     // declares number of lives and score inputs
     input [3:0] lives_in;
     input [7:0] score_in;

     // declares 90 bit random number input
     // from random number generator module
     input [89:0] rand_in;
     
     // declares car position outputs
     // There are 15 car1 + 15 car2 + 15 car3 (8bits each)
     output reg [359:0] x;
     output reg [359:0] y;
     
     // declares temp registers for storing car positions to be
     // loaded into x and y regs on next positive clock edge
     // during car position init
     reg [359:0] t_x;
     reg [359:0] t_y;

     // declares temp registers for car positions and colors to be 
     // loaded into x and y regs on next positive lock edge
     // during the corresponding loading of the type of cars
     reg [7:0] t_car1_x;
     reg [119:0] t_car1_y;
     reg [7:0] t_car2_x;
     reg [119:0] t_car2_y;
     reg [7:0] t_car3_x;
     reg [119:0] t_car3_y;
     reg [44:0] t_car1_color;
     reg [44:0] t_car2_color;
     reg [44:0] t_car3_color;
      
     // declares car color outputs
     // Cars' colors: 3bit each * 45 cars = 135 bits
     output reg [134:0] color;
     reg [134:0] t_color;  
   
     // declares player color and coords outputs
     output reg [7:0] playerX;
     output reg [7:0] playerY;
     output reg [2:0] playerColor;
     
     // declares temp registers for player color and coords 
     reg [7:0] t_playerX;
     reg [7:0] t_playerY;
     reg [2:0] t_playerColor;
  
     // declares score and lives outputs (15 lives max)
     output reg [7:0] score;
     output reg [3:0] lives; 
     
     // declares temp registers for score and lives
     reg [7:0] t_score;
     reg [3:0] t_lives;

     // declares temp registers for coordinate processing
     reg [7:0] tempX;
     reg [7:0] tempY;
     reg [2:0] tempColor;
     
     // declares loop index variables
     integer i;
     integer j;
     
     // initializes all output registers
     initial
     begin
        x = {360{1'b0}};
        y = {360{1'b0}};
        color = {135{1'b0}};
        playerX = 8'b0000_0000;
        playerY = 8'b0000_0000;
        playerColor = 3'b000;
        score = 8'b0000_0000;
        lives = 4'b0001;
        i = 0;
        j = 0;
     end

     always @(*)
     begin

        // sets default values for each register
        t_x = {360{1'b0}};
        t_y = {360{1'b0}};
        t_color = {135{1'b0}};
        t_playerX = 8'b0000_0000;
        t_playerY = 8'b0000_0000;
        t_playerColor = 3'b000;
   	t_lives = 4'b0001;
        t_n_car1_out = 4'b0000;
        t_n_car2_out = 4'b0000;
        t_n_car3_out = 4'b0000;
        t_score = 8'b0000_0000;
        t_car1_x = 8'b0000_0000;
        t_car1_y = {120{1'b0}};
        t_car2_x = 8'b0000_0000;
        t_car2_y = {120{1'b0}};
        t_car3_x = 8'b0000_0000;
        t_car3_y = {120{1'b0}};
        t_car1_color = {45{1'b0}};
        t_car2_color = {45{1'b0}};
        t_car3_color = {45{1'b0}};
        tempX = 8'b0000_0000;
        tempY = 8'b0000_0000;
        tempColor = 3'b000;
        i = 0;
        j = 0;

        if(!reset_n)
        begin
            
            // resets this memory module
            // score is not reset here
            t_x = {360{1'b0}};
            t_y = {360{1'b0}};
            t_color = {135{1'b0}};
            t_playerX = 8'b0000_0000;
            t_playerY = 8'b0000_0000;
            t_playerColor = 3'b000;
   	      t_lives = 4'b0001;
            t_n_car1_out = 4'b0000;
            t_n_car2_out = 4'b0000;
            t_n_car3_out = 4'b0000;
            t_car1_x = 8'b0000_0000;
            t_car1_y = {120{1'b0}};
            t_car2_x = 8'b0000_0000;
            t_car2_y = {120{1'b0}};
            t_car3_x = 8'b0000_0000;
            t_car3_y = {120{1'b0}};
            t_car1_color = {45{1'b0}};
            t_car2_color = {45{1'b0}};
            t_car3_color = {45{1'b0}};
            tempX = 8'b0000_0000;
            tempY = 8'b0000_0000;
            tempColor = 3'b000;
        end
        else
        begin
               // simultaneous loadings are supported
               if(load_car1)
               begin

                    // updates Car1 data
                    t_car1_x = car1_x_in;
                    t_car1_y = car1_y_in;
                    t_car1_color = car1_color_in;
               end
               if(load_car2)
               begin

                    // updates Car2 data
                    t_car2_x = car2_x_in;
                    t_car2_y = car2_y_in;
                    t_car2_color = car2_color_in;
               end

               if(load_car3)
               begin

                    // updates Car3 data
                    t_car3_x = car3_x_in;
                    t_car3_y = car3_y_in;
                    t_car3_color = car3_color_in;
               end
                
               if(load_player)
               begin

                     // updates player data
                     t_playerX = player_x_in;
                     t_playerY = player_y_in;
                     t_playerColor = player_color_in;
               end
               
               if(load_lives)
               begin 

                    // loads new number of lives
                    t_lives = lives_in;
               end

               if(reset_score)
               begin

                    // resets score
                    t_score = 8'b0000_0000;
               end
               
               if(load_num_cars)
               begin

                    // updates car numbers of each type
                    t_n_car1_out = n_car1_in;
                    t_n_car2_out = n_car2_in;
                    t_n_car3_out = n_car3_in;
               end
               
               if(load_num_cars1)
               begin
                  
                  // loads number of cars of type 1
                  t_n_car1_out = n_car1_in;
               end
               
               if(load_num_cars2)
               begin

                  // loads number of cars of type 2
                  t_n_car2_out = n_car2_in;
               end

               if(load_num_cars3)
               begin

                  // loads number of cars of type 3
                  t_n_car3_out = n_car3_in;
               end

               if(load_score)
               begin

                  // loads score
                  t_score = score_in;
               end
               
               // initializes car data
               if(init_cars_data)
               begin
                  
                   // initializes data for car1
                  tempX = rand_in[7:0] % `MAX_X;
                  for (i = 0; i <= 14; i = i + 1)
                  begin
                     
                     tempY = i * 6 + 2;
                     tempColor = (i < n_car1_out) ? 3'b100:3'b000;
                     
                     for (j = 0; j <= 7; j = j + 1)
                     begin
                        t_x[i * 8 + j] = tempX[j];
                        t_y[i * 8 + j] = tempY[j];
                     end
                     for (j = 0; j <= 2; j = j + 1) 
                     begin
                        t_color[i * 3 + j] = tempColor[j];
                     end
                  end
                  
                  // Initializes data for car2
                  tempX = rand_in[17:10] % `MAX_X;
                  for (i = 15; i <= 29; i = i + 1)
                  begin
                     
                     tempY = (i - 15) * 6 + 4;
                     tempColor = (i - 15 < n_car2_out) ? 3'b010 : 3'b000;
                     
                     for (j = 0; j <= 7; j = j + 1)
                     begin
                        t_x[i * 8 + j] = tempX[j];
                        t_y[i * 8 + j] = tempY[j];
                     end
                     for (j = 0; j <= 2; j = j + 1) 
                     begin
                        t_color[i * 3 + j] = tempColor[j];
                     end
                  end
                  
                  // Initializes data for car3
                  tempX = rand_in[57:50] % `MAX_X;
                  for (i = 30; i <= 44; i = i + 1)
                  begin
                     
                     tempY = (i - 30)*6 + 6;
                     tempColor = (i - 30 < n_car3_out) ? 3'b101 : 3'b000;
                     
                     for (j = 0; j <= 7; j = j + 1)
                     begin
                        t_x[i * 8 + j] = tempX[j];
                        t_y[i * 8 + j] = tempY[j];
                     end
                     for (j = 0; j <= 2; j = j + 1) 
                     begin
                        t_color[i * 3 + j] = tempColor[j];
                     end
                  end
               end
               
               if(init_player_data)
               begin
 
                  // initializes player data
                  t_playerX = `PLAYER_SPAWN_X;
                  t_playerY = `PLAYER_SPAWN_Y;
                  t_playerColor =`PLAYER_COLOR;
               end         
        end
     end

     always @(posedge clock)
     begin

        // initializes new car data
        if(init_cars_data || !reset_n)
        begin
           x <= t_x;
           y <= t_y;
           color <= t_color;
        end
        
        // loads new data for cars of type 1
        if(load_car1)
        begin
           x[119:0] <= {15{t_car1_x}};
           y[119:0] <= t_car1_y;
           color[44:0] <= t_car1_color; 
        end

        // loads new data for cars of type 2
        if(load_car2)
        begin
           x[239:120] <= {15{t_car2_x}};
           y[239:120] <= t_car2_y;
           color[89:45] <= t_car2_color; 
        end

        // loads new data for cars of type 3
        if(load_car3)
        begin
           x[359:240] <= {15{t_car3_x}};
           y[359:240] <= t_car3_y;
           color[134:90] <= t_car3_color; 
        end
        
        // loads new data for the player
        if(init_player_data || load_player || !reset_n)
        begin
           playerX <= t_playerX;
           playerY <= t_playerY;
           playerColor <= t_playerColor;  
        end

        // loads new data for the number of cars of each type 
        if(load_num_cars || !reset_n)
        begin
           n_car1_out <= t_n_car1_out;
           n_car2_out <= t_n_car2_out;
           n_car3_out <= t_n_car3_out;
        end

        // loads new data for the number of cars of type 1
        if(load_num_cars1)
        begin
           n_car1_out <= t_n_car1_out;
        end

        // loads new data for the number of cars of type 2
        if(load_num_cars2)
        begin
           n_car2_out <= t_n_car2_out;
        end
 
        // loads new data for the number of cars of type 3
        if(load_num_cars3)
        begin
           n_car3_out <= t_n_car3_out;
        end
        
        // loads new score
        if(reset_score || load_score)
        begin
           score <= t_score;
        end

        // loads new number of lives for the player
        if(load_lives || !reset_n)
        begin
           lives <= t_lives;
        end
     end
endmodule

/**
Inputs: clock, Clear_b, period
Outputs: q, pulse

This module implements a rate divider for the objects
in this game. Period sets the number of cycles of the clock
before the divider resets. If you want the period to be P,
then set period to be P-1. The Pth cycle is when q==period
and q resets. Pulse is generated throughout the last 
cycle. Clear_b resets the counter to start counting from
0 and sets pulse to 0. The divider will function iff enable
is 1'b1. Reset will work regardless of the value of enable.
**/
module RateDivider (clock, reset_n, enable, period, pulse);  

    // declares clock and active low reset inputs
    input clock;
    input reset_n;
    
    // declares period input  
    input [25:0] period;

    // declares enable signal input
    input enable;

    // declares pulse output
    output reg pulse;

    // declares counter register
    reg [26:0] q; 
    
    // initializes pulse and counter output registers
    initial
    begin
       pulse <= 0;
       q <= 0;
    end
    
    always@(posedge clock)   
    begin
   
        if(!reset_n)
        begin
           pulse <= 0;
       	   q <= 0;
        end
        else if(enable)
        begin
                // peforms normal counting and pulsing if enabled
        	if (q == period) 
       	        begin
            		// resets q to 0
            		q <= 0; 

            		// generates pulse
            		pulse <= 1'b1;
        	end
        	else if (clock == 1'b1) 
        	begin
            		// increments q
                        pulse <= 1'b0;
            		q <= q + 1'b1;  
        	end
    	end
   end
endmodule

/**
Input: clock, reset_n, limit, reset_n_pulse_1
Output: pulse

This module implements a counter that counts up
to this limit value. Once limit is reached pulse
will be generated on the next posedge of clock.
Reset_n resets the q value to 0 to allow the 
counter to count again. reset_n_pulse is also an
active low reset, but sets pulse to 1 instad of 0.
**/
module counter(clock, reset_n, reset_n_pulse_1 , pulse, limit);

   // declares clock and active low reset inputs
   input clock, reset_n, reset_n_pulse_1;
   input [25:0] limit;
   
   // declares pulse output 
   output reg pulse;
   reg [25:0] q;
   
   // initializes q and pulse value
   initial
   begin
      q = 0;
      pulse = 1;
   end
   
   always @(posedge clock)
   begin
      if(!reset_n)
      begin
         
         // resets q and sets pulse to 0
         q <= 0;
         pulse <= 0;
      end
      
      if(!reset_n_pulse_1)
      begin

         // resets q and sets pulse to 0
         q <= 0;
         pulse <= 1;
      end
      
      if(reset_n && reset_n_pulse_1)
      begin
         
         // increases q until limit is reached
         // sets pulse to 1 when q reaches limit
         if(q == limit)
         begin
            pulse <= 1;
         end
         else
         begin
            q <= q + 1;
         end
      end
   end
endmodule 

/**
Input: c
Output: HEX

This module implements a 7-segment HEX display decoder.
c[3:0] are the input bits. HEX[6:0] are
the output bits. A segment of this display
is on iff its corresponding output bit is set to 0.
**/
module HEXDisplay(HEX, c);

        // declares output to HEX display
  	output reg[6:0] HEX;
        
        // declares input value
	input [3:0] c;
	
        // writes output to HEX based on c
	always @(c)
 	  case (c)
 		4'h0: HEX = 7'b1000000;
 		4'h1: HEX = 7'b1111001;
 		4'h2: HEX = 7'b0100100;
 		4'h3: HEX = 7'b0110000;
 		4'h4: HEX = 7'b0011001;
 		4'h5: HEX = 7'b0010010;
 		4'h6: HEX = 7'b0000010;
 		4'h7: HEX = 7'b1111000;
 		4'h8: HEX = 7'b0000000;
 		4'h9: HEX = 7'b0010000;
 		4'hA: HEX = 7'b0001000;
 		4'hB: HEX = 7'b0000011;
 		4'hC: HEX = 7'b1000110;
 		4'hD: HEX = 7'b0100001;
 		4'hE: HEX = 7'b0000110;
 		4'hF: HEX = 7'b0001110;
                default: HEX = 7'b0000110;
          endcase
endmodule 

/**
input: clk, rst_n 
output: data

Given clk, rst_n, this module
outputs a 90-bit random number per
cycle of clk(on positive edge). rst_n
is a synchronous active low reset. 
The idea for this random number
generator module was inspired by
the following post on StackOverflow:
https://stackoverflow.com/questions/14497877/how-to-implement-a-pseudo-hardware-random-number-generator
**/
module fibonacci_lfsr_90bit(

  // declares clock and active-low reset inputs
  input clk,
  input rst_n,

  // declares 90 bit random number output
  output reg [89:0] data
);

	reg [89:0] data_next;
	integer i;

	// initializes output value
	initial
	begin
   	     data =
             90'b1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111;
	end

	// computes next number to be generated 
	always @* 
        begin
  	     data_next[89] = data[89]^data[1];
  	     data_next[88] = data[88]^data[0];

  	     for (i=87; i>=0; i=i-1)
  	     begin
     	     	  data_next[i]=data[i]^data_next[i+2];
  	     end
	end

	// processes reset and updates output
	always @(posedge clk or negedge rst_n)
        begin
  	    if(!rst_n)
    		data <=
                90'b1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111;
  	    else
    		data <= data_next;
        end
endmodule

/**
 * Inputs:
 * in is a 4 bit binary input that represents values 0 to F
 * offsetX is an 8 bit offset input applied to each 8-bit x coordinate in
 *        the output xArray
 * offsetY is an 8 bit offset input applied to each 8-bit y coordinate in
 *        the output yArray
 *
 * Outputs: 
 * xArray is a 120 bit output that represents the x coordinates of a 3x5
 *        matrix of pixels, in row by row order, each coordinate is 8 bits
 * yArray is a 120 bit output that represents the y coordinates of a 3x5
 *        matrix of pixels, in row by row order, each coordinate is 8 bits
 * colorArray is a 45 bit output that represents the rgb values of a 3x5
 *        matrix of pixels, in row by row order, each pixel is 3 bits
 *
 * This module creates a decoder for displaying HEX digits on the screen.
 * in is connected to the scores or lives output from the memory module. 
 * offsetX and offsetY can be used to set offsets for each coordinate 
 * stored in xArray and yArray. xArray, yArray, and colorArray can be 
 * feeded into the master controlpath for outputs to the vga buffer module.
 */
module HEX_VGA(xArray, yArray, offsetX, offsetY, colorArray, in);

   // declares HEX number and offset inputs
   input [3:0] in;
   input [7:0] offsetX;
   input [7:0] offsetY;

   // declares x, y, and color array outputs
   output reg [119:0] xArray;
   output reg [119:0] yArray;
   output reg [0:44] colorArray;
   
   always @(in or offsetX or offsetY)
   begin

        // sets default values for the output registers based on offsetX and offsetY
        xArray = {120{1'b0}};
        yArray = {120{1'b0}};
        colorArray = {45{1'b0}};
        
        // outputs new coordinate and color arrays based on in
        if(in == 4'b0000)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111_111_111_111_000_111_111_000_111_111_000_111_111_111_111;
        end
        else if (in == 4'b0001)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111000000111000000111000000111000111111111;
        end
        else if (in == 4'b0010)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111000000111111111111111000000111111111;
        end
        else if (in == 4'b0011)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111000000111111111111000000111111111111;
        end
        else if (in == 4'b0100)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111000111111000111111111111000000111000000111;
        end
        else if (in == 4'b0101)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111111000000111111111000000111111111111;
        end
        else if (in == 4'b0110)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
            colorArray = 45'b111111111111000000111111111111000111111111111;
        end
        else if (in == 4'b0111)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111000000111000000111000000111000000111;
        end
        else if (in == 4'b1000)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111111000111111111111111000111111111111;
        end
        else if (in == 4'b1001)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111111000111111111111000000111000000111;
        end
        else if (in == 4'b1010)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111111000111111111111111000111111000111;
        end
        else if (in == 4'b1011)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111000000111000000111111111111000111111111111;
        end
        else if (in == 4'b1100)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111111000000111000000111000000111111111;
        end
        else if (in == 4'b1101)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b000000111000000111111111111111000111111111111;
        end
        else if (in == 4'b1110)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111111000000111111111111000000111111111;
        end
        else if (in == 4'b1111)
        begin
             xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
             yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
             colorArray = 45'b111111111111000000111111111111000000111000000;
        end
        else
        begin
           
           // outputs data for number 0 by default
           xArray = {56'b00000010_00000001_00000000_00000010_00000001_00000000_00000010,
                       64'b00000001_00000000_00000010_00000001_00000000_00000010_00000001_00000000};
           yArray = {56'b00000100_00000100_00000100_00000011_00000011_00000011_00000010,
                       64'b00000010_00000010_00000001_00000001_00000001_00000000_00000000_00000000};
           colorArray = 45'b111_111_111_111_000_111_111_000_111_111_000_111_111_111_111;
        end
        
        // applies offsets to position of each color
        xArray = xArray + {15{offsetX}};
        yArray = yArray + {15{offsetY}};
   end
endmodule 