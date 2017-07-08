
// Max coords
`define MAX_X 159
`define MAX_Y 119

// Y range of safe zone
`define SAFE_Y_MIN 100
`define SAFE_Y_MAX 119

// Y range of War zone
`define WAR_Y_MIN 0
`define WAR_Y_MAX 99

// Player spawn spot
`define PLAYER_SPAWN_X 80
`define PLAYER_SPAWN_Y 115

// Player color
`define PLAYER_COLOR 1

// Periods of Cars
`define CAR1_CYCLES 26'd9999999//26'd49999999//26'd9999999
`define CAR2_CYCLES 26'd19999999//26'd49999999//26'd19999999
`define CAR3_CYCLES 26'd24999999//26'd49999999//26'd24999999
`define PLAYER_CYCLES 26'd14999999//26'd49999999//26'd14999999

// Period of collision grace
`define COLLISION_GRACE_PERIOD 26'd16000000
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
 		HEX0, HEX1, HEX2				
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;

        output [6:0] HEX0;  // score
        output [6:0] HEX1;  // score
        output [6:0] HEX2;  // lives

	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	wire resetn;
	assign resetn = SW[9];
        
	
	// Create the colour, x, y and writeEn wires that are vga inputs from controller output.
	wire [2:0] colour;
	wire [7:0] x;
	wire [7:0] y;
	wire writeEn;
        
        // Output of startGame value
        wire startGameOut;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	/**vga_adapter VGA(
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
		defparam VGA.BACKGROUND_IMAGE = "black.mif";**/
    
     // load parameters from master control to RAM
     wire w_load_num_cars;
     wire w_load_lives, w_load_score, w_init_cars_data, w_init_player_data;

     // resets score in memory (1 => reset)
     wire w_reset_score; 

     // wires for load objects parameters from RAM to the corresponding object controls
     wire w_load_car1, w_load_car2, w_load_car3, w_load_player;

     // wires for score output from RAM (digits: LSB at 0; MSB at 7)
     // score is based on Y coordinate
     wire [7:0] w_score; 

     // wire for score input into RAM from master control
     wire [7:0] w_score_ram_in;
     
     // wire for lives output from ram
     wire [3:0] w_lives;
     
     // wire for lives input into ram
     wire [3:0] w_lives_ram_in;
     
     // wire for number of cars output from ram
     wire [3:0] w_n_car1_ram_out;
     wire [3:0] w_n_car2_ram_out;
     wire [3:0] w_n_car3_ram_out;

     // wire for number of cars inputs into RAM from master control output
     wire [3:0] w_n_car1_ram_in;
     wire [3:0] w_n_car2_ram_in;
     wire [3:0] w_n_car3_ram_in;

     // wires for output of car data from RAM to mater control
     wire [359:0] w_x_ram_out;
     wire [359:0] w_y_ram_out;
     wire [134:0] w_color_ram_out;
     
     // wire for reset signal input to ram from master control
     wire w_mem_reset_in;
     
     // wire for signal status for start reset processing from control master
     wire w_start_reset_processing;
     
     // wire to input SW into master control
     // Resetn for master control is controlled by SW[9] separately without using this wire
     wire [9:0] SW_master_control_in;
     
     // wire for to input the go singnal into master control
     wire go_master_control_in;
     
     // go and SW inputs are eanbled iff game is not resetting or running the level
     assign go_master_control_in = (w_start_reset_processing || startGameOut) ? 0 : ~KEY[0];
     assign SW_master_control_in = (w_start_reset_processing || startGameOut) ? 0 : SW;
     
     // Player data output from RAM
     wire [7:0] w_player_x_ram_out;
     wire [7:0] w_player_y_ram_out;
     wire [2:0] w_player_color_ram_out;
     
     // Player data input into RAM from cPlayer control
     wire [7:0] w_player_x_ram_in;
     wire [7:0] w_player_y_ram_in;
     wire [2:0] w_player_color_ram_in;
     
     // Car1 data output from RAM
     wire [7:0] w_car1_x_ram_out;
     wire [119:0] w_car1_y_ram_out;
     wire [44:0] w_car1_color_ram_out;
     
     // Car2 data output from RAM
     wire [7:0] w_car2_x_ram_out;
     wire [119:0] w_car2_y_ram_out;
     wire [44:0] w_car2_color_ram_out;

     // Car3 data output from RAM
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
     

     // Car1 data input into RAM
     wire [7:0] w_car1_x_ram_in;
     wire [119:0] w_car1_y_ram_in;
     wire [44:0] w_car1_color_ram_in;
    
     // Car2 data input into RAM
     wire [7:0] w_car2_x_ram_in;
     wire [119:0] w_car2_y_ram_in;
     wire [44:0] w_car2_color_ram_in;

     // Car3 data input into RAM
     wire [7:0] w_car3_x_ram_in;
     wire [119:0] w_car3_y_ram_in;
     wire [44:0] w_car3_color_ram_in;
     
     // wire from divider reset/enable to corresponding cars/player divider reset/enable
     wire car1D_reset, car2D_reset, car3D_reset;
     wire car1D_enable, car2D_enable, car3D_enable;     
     wire playerD_enable, playerD_reset;
      
     // wire from divider pulse to cars/player control pulse input
     wire car1D_pulse, car2D_pulse, car3D_pulse, playerD_pulse;

     // object reset wire from master control to other controls (active low reset)
     wire w_objects_reset;

     // wires connecting collision grace counter module and cMaster (resets are active low)
     wire w_reset_cgrace_pulse1, w_reset_cgrace, w_cgrace_over_pulse;
   
     controlMaster cMaster (.clock(CLOCK_50), .reset_n(resetn), .start_game(startGameOut), .load_num_cars(w_load_num_cars), .load_lives(w_load_lives),
     .load_score(w_load_score), .reset_score(w_reset_score), .init_cars_data(w_init_cars_data), .init_player_data(w_init_player_data), .n_car1(w_n_car1_ram_out), .n_car2(w_n_car2_ram_out), .n_car3(w_n_car3_ram_out), .n_car1_out(w_n_car1_ram_in), .n_car2_out(w_n_car2_ram_in), .n_car3_out(w_n_car3_ram_in), .x(w_x_ram_out), .y(w_y_ram_out), .color(w_color_ram_out),
     .playerX(w_player_x_ram_out), .playerY(w_player_y_ram_out), .playerColor(w_player_color_ram_out), .score(w_score), .lives(w_lives), .lives_out(w_lives_ram_in), .score_out(w_score_ram_in), .go(go_master_control_in), .plot(writeEn), .vga_color(colour), .vga_x(x), .vga_y(y), .SW_in(SW_master_control_in), .memReset(w_mem_reset_in), .start_reset_processing(w_start_reset_processing), .objects_reset(w_objects_reset)
     , .collision_grace_over_pulse(w_cgrace_over_pulse), .collision_grace_counter_reset_n(w_reset_cgrace), .collision_grace_counter_resetn_pulse1(w_reset_cgrace_pulse1));

     controlPlayer cPlayer(.clock(CLOCK_50), .reset_n(w_objects_reset), .start_game(startGameOut), .reset_divider(playerD_reset), .divider_enable(playerD_enable), .pulse_in(playerD_pulse), .up(KEY[3]), .down(KEY[2]), .left(KEY[1]), .right(KEY[0]), .x(w_player_x_ram_out), .y(w_player_y_ram_out), .color(w_player_color_ram_out), .load_player(w_load_player), .x_out(w_player_x_ram_in), .y_out(w_player_y_ram_in), .color_out(w_player_color_ram_in));

     controlCar cCar1 (.clock(CLOCK_50), .reset_n(w_objects_reset), .start_game(startGameOut), .reset_divider(car1D_reset), .divider_enable(car1D_enable), .pulse_in(car1D_pulse), .x(w_car1_x_ram_out), .y(w_car1_y_ram_out), .color(w_car1_color_ram_out), .n_cars(w_n_car1_ram_out), .load_car(w_load_car1), .x_out(w_car1_x_ram_in), .y_out(w_car1_y_ram_in), .color_out(w_car1_color_ram_in));
     controlCar cCar2 (.clock(CLOCK_50), .reset_n(w_objects_reset), .start_game(startGameOut), .reset_divider(car2D_reset), .divider_enable(car2D_enable), .pulse_in(car2D_pulse), .x(w_car2_x_ram_out), .y(w_car2_y_ram_out), .color(w_car2_color_ram_out), .n_cars(w_n_car2_ram_out), .load_car(w_load_car2), .x_out(w_car2_x_ram_in), .y_out(w_car2_y_ram_in), .color_out(w_car2_color_ram_in));
     controlCar cCar3 (.clock(CLOCK_50), .reset_n(w_objects_reset), .start_game(startGameOut), .reset_divider(car3D_reset), .divider_enable(car3D_enable), .pulse_in(car3D_pulse), .x(w_car3_x_ram_out), .y(w_car3_y_ram_out), .color(w_car3_color_ram_out), .n_cars(w_n_car3_ram_out), .load_car(w_load_car3), .x_out(w_car3_x_ram_in), .y_out(w_car3_y_ram_in), .color_out(w_car3_color_ram_in));

     memory RAM(.clock(CLOCK_50), .reset_n(w_mem_reset_in), .x(w_x_ram_out), .y(w_y_ram_out), .color(w_color_ram_out), .playerX(w_player_x_ram_out), .playerY(w_player_y_ram_out), .playerColor(w_player_color_ram_out), .score(w_score), .lives(w_lives), .n_car1_out(w_n_car1_ram_out), .n_car2_out(w_n_car2_ram_out), .n_car3_out(w_n_car3_ram_out), .n_car1_in(w_n_car1_ram_in), .n_car2_in(w_n_car2_ram_in), .n_car3_in(w_n_car3_ram_in), .car1_x_in(w_car1_x_ram_in), .car2_x_in(w_car2_x_ram_in), .car3_x_in(w_car3_x_ram_in), .car1_y_in(w_car1_y_ram_in),
     .car2_y_in(w_car2_y_ram_in), .car3_y_in(w_car3_y_ram_in), .car1_color_in(w_car1_color_ram_in), .car2_color_in(w_car2_color_ram_in), .car3_color_in(w_car3_color_ram_in), .player_x_in(w_player_x_ram_in), .player_y_in(w_player_y_ram_in), .player_color_in(w_player_color_ram_in), .lives_in(w_lives_ram_in), .score_in(w_score_ram_in), .load_car1(w_load_car1), .load_car2(w_load_car2), .load_car3(w_load_car3), .load_num_cars(w_load_num_cars), .load_player(w_load_player), .load_lives(w_load_lives),
     .load_score(w_load_score), .reset_score(w_reset_score), .init_cars_data(w_init_cars_data), .init_player_data(w_init_player_data));

     HEXDisplay score1 (.HEX(HEX0[6:0]), .c(w_score[3:0]));
     HEXDisplay score2 (.HEX(HEX1[6:0]), .c(w_score[7:4]));
     HEXDisplay livesHEX (.HEX(HEX2[6:0]), .c(w_lives));

     RateDivider car1D (.clock(CLOCK_50), .reset_n(car1D_reset), .enable(car1D_enable), .period(`CAR1_CYCLES), .pulse(car1D_pulse));  
     RateDivider car2D (.clock(CLOCK_50), .reset_n(car2D_reset), .enable(car2D_enable), .period(`CAR2_CYCLES), .pulse(car2D_pulse));  
     RateDivider car3D (.clock(CLOCK_50), .reset_n(car3D_reset), .enable(car3D_enable), .period(`CAR3_CYCLES), .pulse(car3D_pulse));  
     RateDivider playerD (.clock(CLOCK_50), .reset_n(playerD_reset), .enable(playerD_enable), .period(`PLAYER_CYCLES), .pulse(playerD_pulse));  

     counter collisionGraceCounter (.clock(CLOCK_50), .reset_n(w_reset_cgrace), .reset_n_pulse_1(w_reset_cgrace_pulse1) , .pulse(w_cgrace_over_pulse), .limit(`COLLISION_GRACE_PERIOD));
endmodule

//module datapath(clock, reset_n,);

//module control(clock, reset_n, );

module controlMaster(clock, reset_n, start_game, load_num_cars, load_lives,
 load_score, reset_score, init_cars_data, init_player_data, n_car1, n_car2, n_car3, n_car1_out, n_car2_out, n_car3_out, x, y, color,
 playerX, playerY, playerColor, score, lives, lives_out, score_out, go, plot, vga_color, vga_x, vga_y, SW_in, memReset, start_reset_processing, objects_reset, collision_grace_over_pulse, collision_grace_counter_reset_n
, collision_grace_counter_resetn_pulse1);

    input clock, reset_n;
    input [9:0] SW_in;    

    // stores output data to vga module
    output plot;
    output reg [2:0] vga_color;
    output reg [7:0] vga_x;
    output reg [7:0] vga_y;
    
    //Controls memory reset
    output reg memReset;
    
    // inputs 1 iff collision grace period is over
    input collision_grace_over_pulse;
    
    // reset output signal to collision grace counter
    output reg collision_grace_counter_reset_n;
    output reg collision_grace_counter_resetn_pulse1;
    

    // temporary variables to store car coord/color during collision detection
    reg [7:0] checkX;
    reg [7:0] checkY;
    reg [2:0] checkColor;

    // loading codes to memory
    output reg load_num_cars, load_lives, load_score, reset_score, init_cars_data, init_player_data;

    // code n-edge triggered reset control to game objects
    output reg objects_reset;    

    // controls start of the game
    output reg start_game;
    output reg start_reset_processing;

    // input for number of cars of each type from memory
    input [3:0] n_car1;
    input [3:0] n_car2;
    input [3:0] n_car3;

    // output of number of cars of each type to memory
    output reg [3:0] n_car1_out;
    output reg [3:0] n_car2_out;
    output reg [3:0] n_car3_out;

    // input for x,y coords of each car from memory
    input [359:0] x;
    input [359:0] y;
    
    // input for color for each car from memory
    input [134:0] color;

    // input for player data from memory
    input [7:0] playerX;
    input [7:0] playerY;
    input [2:0] playerColor;
    
    
    // stores coordinates since last graphic update
    reg [359:0] curr_x;
    reg [359:0] curr_y;

    // stores coordinates since last graphic update
    reg [7:0] curr_playerX;
    reg [7:0] curr_playerY;

    // input for player lives and score from memory
    input [3:0] lives;
    input [7:0] score;
    
    // output for player lives and score to memory
    output reg [3:0] lives_out;
    output reg [7:0] score_out;
    
    // Go button during lives and number of cars selection states
    input go;
    
    // state registers
    reg [6:0] current_state;
    reg [6:0] next_state;
    
    // stores index of car during graphic update
    integer car_index;
    
    integer i;
    integer j;

    reg [15:0] counter;
 
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
               S_UPDATE_GRAPHICS_CLEAR_CARS_END = 7'd33,
               S_UPDATE_GRAPHICS_CLEAR_PLAYER = 7'd31,
               S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE1 = 7'd32,
               S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE2 = 7'd34, 
               S_UPDATE_GRAPHICS_CLEAR_END = 7'd18,
               S_UPDATE_GRAPHICS_CARS  = 7'd19,
               S_UPDATE_GRAPHICS_CARS_CYCLE1 = 7'd26,
               S_UPDATE_GRAPHICS_CARS_CYCLE2 = 7'd27,
               S_UPDATE_GRAPHICS_CARS_END  = 7'd20,
               S_UPDATE_GRAPHICS_PLAYER = 7'd12,
               S_UPDATE_GRAPHICS_PLAYER_CYCLE1 = 7'd28,
               S_UPDATE_GRAPHICS_WAIT   = 7'd13,
               S_COLLISION_DETECTION    = 7'd14,
               S_COLLISION_DETECTION_CYCLE1 = 7'd35,
               S_COLLISION_DETECTION_CYCLE2 = 7'd36,
               S_COLLISION_DETECTION_END = 7'd15,
               S_WIN_DETECTION = 7'd16,
               S_WIN_DETECTION_END = 7'd21,
               S_RESET1 = 7'd17,
               S_CLEAR_SCREEN = 7'd22,
               S_CLEAR_SCREEN_CYCLE1 = 7'd29,
               S_CLEAR_SCREEN_CYCLE2 = 7'd30,
               S_CLEAR_SCREEN_END = 7'd23,
               S_INIT_DATA_WAIT2 = 7'd37;
    initial
    begin
       start_game = 0;
       start_reset_processing = 0;
       collision_grace_counter_reset_n = 1;
       collision_grace_counter_resetn_pulse1 = 1;
       vga_x = 0;
       vga_y = 0;
       vga_color = 0;
       car_index = 0;
       counter = 0;

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
              S_INIT_DATA_WAIT2: next_state = S_UPDATE_GRAPHICS;
              S_UPDATE_GRAPHICS: next_state = S_UPDATE_GRAPHICS_CLEAR_END; // changed for testing
              S_UPDATE_GRAPHICS_CLEAR: next_state =  S_UPDATE_GRAPHICS_CLEAR_CYCLE1;
              S_UPDATE_GRAPHICS_CLEAR_CYCLE1: next_state = S_UPDATE_GRAPHICS_CLEAR_CYCLE2;
              S_UPDATE_GRAPHICS_CLEAR_CYCLE2: next_state = (car_index == 45) ? S_UPDATE_GRAPHICS_CLEAR_CARS_END  : S_UPDATE_GRAPHICS_CLEAR;
              S_UPDATE_GRAPHICS_CLEAR_CARS_END: next_state = S_UPDATE_GRAPHICS_CLEAR_PLAYER; 
              S_UPDATE_GRAPHICS_CLEAR_PLAYER: next_state = S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE1;
              S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE1: next_state = S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE2;
              S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE2: next_state =  S_UPDATE_GRAPHICS_CLEAR_END;
              S_UPDATE_GRAPHICS_CLEAR_END:  next_state = S_UPDATE_GRAPHICS_CARS;
              S_UPDATE_GRAPHICS_CARS: next_state = S_UPDATE_GRAPHICS_CARS_CYCLE1;
              S_UPDATE_GRAPHICS_CARS_CYCLE1: next_state = S_UPDATE_GRAPHICS_CARS_CYCLE2;
              S_UPDATE_GRAPHICS_CARS_CYCLE2: next_state = (car_index == 45) ?  S_UPDATE_GRAPHICS_CARS_END : S_UPDATE_GRAPHICS_CARS;
              S_UPDATE_GRAPHICS_CARS_END: next_state = S_UPDATE_GRAPHICS_PLAYER;
              S_UPDATE_GRAPHICS_PLAYER: next_state = S_UPDATE_GRAPHICS_PLAYER_CYCLE1;
              S_UPDATE_GRAPHICS_PLAYER_CYCLE1: next_state = S_COLLISION_DETECTION; // changed for testing
              S_COLLISION_DETECTION: next_state = S_COLLISION_DETECTION_CYCLE1;
              S_COLLISION_DETECTION_CYCLE1: next_state = S_COLLISION_DETECTION_CYCLE2;
              S_COLLISION_DETECTION_CYCLE2: next_state = S_COLLISION_DETECTION_END;
              S_COLLISION_DETECTION_END: next_state = (start_game) ?  S_WIN_DETECTION : S_RESET1;
              S_WIN_DETECTION: next_state = S_WIN_DETECTION_END;
              S_WIN_DETECTION_END: next_state = start_game ?  S_UPDATE_GRAPHICS : S_RESET1;
              S_RESET1: next_state = S_CLEAR_SCREEN;
              S_CLEAR_SCREEN: next_state = S_CLEAR_SCREEN_CYCLE1;
              S_CLEAR_SCREEN_CYCLE1: next_state = S_CLEAR_SCREEN_CYCLE2;
              S_CLEAR_SCREEN_CYCLE2: next_state = (counter == 16'b1111_1111_1111_1111) ? S_CLEAR_SCREEN_END : S_CLEAR_SCREEN;
              S_CLEAR_SCREEN_END: next_state = S_LIVES_INPUT;
              default: next_state = S_LIVES_INPUT;         
       endcase
    end

   always @(*)
   begin
      // resets all output to memory instructions by default
      load_num_cars = 1'b0;
      load_lives = 1'b0;
      load_score = 1'b0;
      reset_score = 1'b0;
      init_cars_data = 1'b0;
      init_player_data = 1'b0;
      memReset = 1'b1;
      objects_reset = 1'b1;
      collision_grace_counter_resetn_pulse1 = 1'b1;
      collision_grace_counter_reset_n = 1'b1;
      
      case (current_state)
           S_LIVES_INPUT: begin
                             
                             lives_out = (SW_in[3:0] == 0) ? 1 : SW_in[3:0];
                             load_lives = 1'b1;
                          end
           S_N_CARS1_INPUT: begin
                               n_car1_out = SW_in[3:0];
                            end
           S_N_CARS2_INPUT: begin
                               n_car2_out = SW_in[3:0];
                            end 
           S_N_CARS3_INPUT: begin
                               n_car3_out = SW_in[3:0];
                               load_num_cars = 1'b1;
                            end
           S_INIT_DATA: begin
                           init_cars_data =1'b1;
                           init_player_data = 1'b1;
                           reset_score = 1'b1;
                        end
           S_INIT_DATA_WAIT: begin
                                
										  
                             end
           S_INIT_DATA_WAIT2: begin
                                 start_game = 1'b1;
                                 
                                 // Sets current object positions to initial positions
                                 curr_x = x;
                                 curr_y = y;
                                 curr_playerX = playerX;
                                 curr_playerY = playerY;
                              end
           S_UPDATE_GRAPHICS: begin
                                 // car_index = 0;
                                 
                              end
           S_UPDATE_GRAPHICS_CLEAR: begin

                                         // prepares coord x,y, color for plot black clear pixel
                                         for (i=0; i<=7; i=i+1)
                                         begin
                                            vga_x[i] = curr_x[car_index*8 + i];
                                            vga_y[i] = curr_y[car_index*8 + i];
                                         end
                                         vga_color = 3'b000;
                                    end
          S_UPDATE_GRAPHICS_CLEAR_CYCLE1: begin
                                            
                                             // car_index = car_index + 1;
                                          end  
          S_UPDATE_GRAPHICS_CLEAR_CARS_END: begin
                                               // car_index = 0;
                                            end   
          S_UPDATE_GRAPHICS_CLEAR_PLAYER: begin
                                             vga_x = curr_playerX;
                                             vga_y = curr_playerY;
                                             vga_color = playerColor;
                                          end  
          S_UPDATE_GRAPHICS_CLEAR_END: begin
                                          // Initializes car index for updating cars on the screen
                                          // car_index = 0;
                                       end
          S_UPDATE_GRAPHICS_CARS: begin
                                     for (i=0; i<=7; i=i+1)
                                     begin
												    // Stores coordinate plotted for clearing later
                                        curr_x[car_index*8+i] = x[car_index*8 + i];
                                        curr_y[car_index*8+i] = y[car_index*8 + i];
													 
                                        vga_x[i] = x[car_index*8 + i];
                                        vga_y[i] = y[car_index*8 + i];
                                        
                                        
                                     end
                                     
                                     for (i=0; i<=2; i=i+1)
                                     begin
                                          vga_color[i] = color[car_index*3 + i]; 
                                     end
                                  end
          S_UPDATE_GRAPHICS_CARS_CYCLE1: begin
                                           
                                              // car_index = car_index + 1;
                                         end
          S_UPDATE_GRAPHICS_CARS_END: begin
                                         // car_index = 0;
                                      end
          S_UPDATE_GRAPHICS_PLAYER: begin
                                       curr_playerX = playerX;
                                       curr_playerY = playerY;
                                       vga_x = curr_playerX;
                                       vga_y = curr_playerY;
                                       vga_color = playerColor;
                                    end
          S_UPDATE_GRAPHICS_PLAYER_CYCLE1: begin

                                              // Updates the score
                                              score_out = `MAX_Y - curr_playerY;
                                              load_score = 1'b1;

                                             
                                                
                                           end
          S_COLLISION_DETECTION: begin
                                    for (i = 0; i<=44; i=i+1)
                                    begin
                                       for (j = 0; j<=7; j=j+1)
                                       begin
                                          checkX[j] = curr_x[8*i + j];
                                          checkY[j] = curr_y[8*i + j];
                                       end
                                       for (j = 0; j<=2; j=j+1)
                                       begin
                                          checkColor[j] = color[3*i + j];
                                       end
                                       if(checkX == curr_playerX && checkY == curr_playerY && checkColor!= 3'b000 && collision_grace_over_pulse)
                                       begin
                                             lives_out = lives - 4'b0001;
                                             load_lives = 1'b1;
                                             collision_grace_counter_reset_n = 0;
                                             
                                       end
                                    end
                                 end
           S_COLLISION_DETECTION_CYCLE2: begin
                                            collision_grace_counter_reset_n = 1;
                                            if(lives == 0)
                                            begin
                                               start_game = 1'b0;
                                            end
                                         end
           S_WIN_DETECTION: begin
                               if(curr_playerY == 0)
                               begin
                                  start_game = 1'b0;
                               end
                            end
           S_RESET1: begin
                        // counter = 0;
                        start_game = 1'b0;
                        start_reset_processing = 1'b1;
                        objects_reset = 1'b0;
                        collision_grace_counter_resetn_pulse1 = 0;
                        collision_grace_counter_reset_n = 1;
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
                                  vga_x = 0;
                                  vga_y = 0;
                                  vga_color = 3'b000;
                              end
                           end
         
         S_CLEAR_SCREEN_CYCLE1: begin
                                  
                                   // counter = counter + 1;
                                end
         S_CLEAR_SCREEN_END: begin
                                // counter = 0;
                                collision_grace_counter_resetn_pulse1 = 1;
                                memReset = 1'b0;
                                start_reset_processing = 1'b0;
                             end
         
         
      endcase
      
   end
   
   always@(posedge clock)
   begin: state_FFs
        if(!reset_n)
            current_state = S_RESET1;
        else
            current_state = next_state;
        
        
        case (current_state)
             S_UPDATE_GRAPHICS_CLEAR_CYCLE1: car_index = car_index + 1;
             S_UPDATE_GRAPHICS_CARS_CYCLE1: car_index = car_index + 1;
             S_CLEAR_SCREEN_CYCLE1: counter = counter + 1;
             S_UPDATE_GRAPHICS: car_index = 0;
             S_UPDATE_GRAPHICS_CARS_END: car_index = 0;
             S_UPDATE_GRAPHICS_CLEAR_CARS_END : car_index = 0;
             S_UPDATE_GRAPHICS_CLEAR_END: car_index = 0;
             S_RESET1: counter = 0;
             S_CLEAR_SCREEN_END: counter = 0;
             
        endcase

   end // state_FFS
                
   assign plot = (current_state == S_UPDATE_GRAPHICS_CLEAR_CYCLE1 || current_state == S_UPDATE_GRAPHICS_CARS_CYCLE1 || current_state == S_UPDATE_GRAPHICS_CLEAR_PLAYER_CYCLE1 ||
   current_state == S_UPDATE_GRAPHICS_PLAYER_CYCLE1 || current_state == S_CLEAR_SCREEN_CYCLE1) ? 1'b1 : 1'b0;
 
endmodule

module controlPlayer(clock, reset_n, start_game, reset_divider, divider_enable, pulse_in, up, down, left, right, x, y, color, load_player, x_out, y_out, color_out); 

    input clock, reset_n,  pulse_in;
    
    // input from memory output
    input [7:0] x;
    input [7:0] y;
    input [2:0] color;
    
    output reset_divider, divider_enable;
    
    // game start signal
    input start_game;     

    // output values to memory
    output reg [7:0] x_out;
    output reg [7:0] y_out;
    output reg [2:0] color_out;

    output reg load_player;

    // Movement inputs from keys
    input up, down, left, right;

    reg [4:0] current_state, next_state;
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
       // By default make all our signals 0

       load_player = 1'b0;
       x_out =0;
       y_out =0;
       color_out =0;
       
       case (current_state)
           S_UPDATE_INFO: begin
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
                                if(y != `MAX_Y)
                                begin
                                   x_out = x;
                                   y_out = y + 8'b0000_0001;
                                   color_out = color;
                                   load_player = 1'b1;
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
                             end
                             else if(!right)
                             begin
                                if(x != `MAX_X)
                                begin
                                   x_out = x + 8'b0000_0001;
                                   y_out = y;
                                   color_out = color;
                                   load_player = 1'b1;
                                end
                             end
                          end
       endcase
       
    
    end

    // current_state registers
    always@(posedge clock)
    begin: state_FFs
        if(!reset_n)
            current_state <= S_WAIT;
        else
            current_state <= next_state;
    end // state_FFS
    
    
    assign divider_enable = start_game ? 1'b1 : 1'b0;
    assign reset_divider = start_game ? 1'b1 : 1'b0;

endmodule

/**
**/
module controlCar(clock, reset_n, start_game, reset_divider, divider_enable, pulse_in, x, y, color, n_cars, load_car, x_out, y_out, color_out);

    input clock, reset_n,  pulse_in;

    // input from memory output
    input [7:0] x;
    input [119:0] y;
    input [44:0] color;
    input [3:0] n_cars;

    input start_game;
     
    output reset_divider, divider_enable;

    // output values to memory
    output reg [7:0] x_out;
    output reg [119:0] y_out;
    output reg [44:0] color_out;
    
    // loads car1s' x's, y's, and colors into memory on next
    // posedge of clock iff load_car1 = 1'b1
    output reg load_car;
    
    reg [4:0] current_state, next_state;
    integer i;

    // MAX_X = 159d
    // MAX_Y = 119d
    // SAFE_Y_MIN = 100d
    // SAFE_Y_MAX = 119d
    // WAR_Y_MIN = 0d
    // WAR_Y_MAX = 99d
    localparam  
                //MAX_X           = 8'b1001_1111,
                //MAX_Y           = 8'b0111_0111,
                //SAFE_Y_MIN      = 8'b0110_0100,
                //SAFE_Y_MAX      = 8'b0111_0111,
                //WAR_Y_MIN       = 8'b0000_0000,
               // WAR_Y_MAX       = 8'b0110_0011,
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
       // By default make all our signals 0

       load_car = 1'b0;
       x_out =0;
       y_out =0;
       color_out =0;
       
       case (current_state)
           S_UPDATE_INFO: begin
                             // Moves car across the screen and back to x=0 position if MAX_X is reached.
                             if (x != `MAX_X)
                             begin
                                x_out = x + 8'b0000_0001;
                                y_out = y;
                                color_out = color;
                                load_car = 1'b1;
                             end
                             else
                             begin
                                x_out = 8'b0000_0000;
                                y_out = y;
                                color_out = color;
                                load_car = 1'b1;
                             end
                          end
       endcase
       
    
    end

    // current_state registers
    always@(posedge clock)
    begin: state_FFs
        if(!reset_n)
            current_state <= S_WAIT;
        else
            current_state <= next_state;
    end // state_FFS
    
    
    assign divider_enable = start_game ? 1'b1 : 1'b0;
    assign reset_divider = start_game ? 1'b1 : 1'b0;
   
endmodule


module memory(clock, reset_n, x, y, color, playerX, playerY, playerColor, score, lives, n_car1_out, n_car2_out, n_car3_out, n_car1_in, n_car2_in, n_car3_in, car1_x_in, car2_x_in, car3_x_in, car1_y_in,
 car2_y_in, car3_y_in, car1_color_in, car2_color_in, car3_color_in, player_x_in, player_y_in, player_color_in, lives_in, score_in, load_car1, load_car2, load_car3, load_num_cars, load_player, load_lives,
 load_score, reset_score, init_cars_data, init_player_data);

     input clock, reset_n;
     
     // memory operation types
     //input [3:0] op;
     
     // new op code inputs
     input load_car1, load_car2, load_car3, load_num_cars, load_player, load_lives, load_score, reset_score, init_cars_data, init_player_data;

     // number of each objects (15 max)
     input [3:0] n_car1_in;
     input [3:0] n_car2_in;
     input [3:0] n_car3_in;
     
     output reg [3:0] n_car1_out;
     output reg [3:0] n_car2_out;
     output reg [3:0] n_car3_out;

     input [7:0] car1_x_in;
     input [119:0] car1_y_in;

     input [7:0] car2_x_in;
     input [119:0] car2_y_in;

     input [7:0] car3_x_in;
     input [119:0] car3_y_in;
    
     input [44:0] car1_color_in;
     input [44:0] car2_color_in;
     input [44:0] car3_color_in;

     input [7:0]player_x_in;
     input [7:0]player_y_in;
     input [2:0]player_color_in;

     input [3:0] lives_in;
     input [7:0] score_in;

     // There are 15 car1 + 15 car2 + 15 car3 (8bits each)
     output reg [359:0] x;
     output reg [359:0] y;

     // Cars' colors: 3bit each *45 = 135 bits
     output reg [134:0] color;

     // Player color and coords
     output reg [7:0] playerX;
     output reg [7:0] playerY;
     output reg [2:0] playerColor;
     
     // Score and lives output
     output reg [7:0] score;
     output reg [3:0] lives; // max 15 lives

     // temp registers for coordinate processing
     reg [7:0] tempX;
     reg [7:0] tempY;
     reg [2:0] tempColor;

     integer i;
     integer j;
     
     // Initializes all registers
     initial
     begin
        for (i=0; i<=44; i=i+1)
        begin
           for (j=8*i; j<=8*i+7; j=j+1)
           begin
              x[j] = 1'b0;
              y[j] = 1'b0;
           end
           
           for (j=3*i; j<=3*i+2; j=j+1)
           begin
              color[j] = 1'b0;
           end
        end
        playerX = 8'b0000_0000;
        playerY = 8'b0000_0000;
        playerColor = 3'b000;
        score = 8'b0000_0000;
        lives = 4'b0001;
     end

     always @(posedge clock)
     begin
        if(!reset_n)
        begin
            
            // Score is not reset here. It is reset at the start of the game.
           for (i=0; i<=44; i=i+1)
           begin
               for (j=8*i; j<=8*i+7; j=j+1)
               begin
                    x[j] = 1'b0;
                    y[j] = 1'b0;
               end
           
               for (j=3*i; j<=3*i+2; j=j+1)
               begin
                    color[j] = 1'b0;
               end
           end
            playerX = 8'b0000_0000;
            playerY = 8'b0000_0000;
            playerColor = 3'b000;
   	      lives = 4'b0001;
        end
        else
        begin
             // Now allows simultaneous load updates
             if(load_car1)
             begin
                          // Updates Car1 data
                          for (i=0; i<=14; i=i+1)
								  begin
                               for (j= 8*i; j<= 8*i+7; j=j+1)
                               begin
                                  x[j] = car1_x_in[j-8*i];
											 y[j] = car1_y_in[j];
                               end
           		                for (j=3*i; j<=3*i+2; j=j+1)
                               begin
                                  color[j] = car1_color_in[j];
                               end   
           			
       			           end
               end
               if(load_car2)
               begin

                          // Updates Car2 data
                          for (i=15; i<=29; i=i+1)
								  begin
                               for (j= 8*i; j<= 8*i+7; j=j+1)
                               begin
                                  x[j] = car2_x_in[j-8*i];
           			                y[j] = car2_y_in[j-120];
                               end
           		                for (j=3*i; j<=3*i+2; j=j+1)
                               begin
                                  color[j] = car2_color_in[j-45];
                               end   
           			//x[8*i+7:8*i] <= car2_x_in;
           			//y[8*i+7:8*i] <= car2_y_in[8*(i-15)+7:8*(i-15)];
           			//color[3*i+2:3*i] <= car2_color_in[3*(i-15)+2:3*(i-15)];
									end
                      
               end

               if(load_car3)
               begin
                           // Updates Car3 data
                          for (i=30; i<=44; i=i+1)
        		  begin
                               for(j=8*i; j<=8*i+7; j=j+1)
                               begin
                                  x[j] = car3_x_in[j-8*i];
                                  y[j] = car3_y_in[j-240];
                               end
                               
           		       for (j=3*i; j<=3*i+2; j=j+1)
                               begin
                                  color[j] = car3_color_in[j-90];
                               end          
           			//x[8*i+7:8*i] <= car3_x_in;
           			//y[8*i+7:8*i] <= car3_y_in[8*(i-30)+7:8*(i-30)];
           			//color[3*i+2:3*i] <= car3_color_in[3*(i-44)+2:3*(i-44)];
       			  end
               end
                
               if(load_player)
               begin
                           // Updates player data
                           playerX = player_x_in;
                           playerY = player_y_in;
                           playerColor = player_color_in;
               end
               
               if(load_lives)
               begin 
                         lives = lives_in;
               end

               if(reset_score)
               begin
                         // resets score
                         score = 8'b0000_0000;
               end
               
               if(load_num_cars)
               begin
                           // Updates car numbers for each type
                           n_car1_out = n_car1_in;
                           n_car2_out = n_car2_in;
                           n_car3_out = n_car3_in;
               end
               if(load_score)
               begin
                           score = score_in;
               end
               
               // initializes car data
               if(init_cars_data)
               begin
                   
                  // Initializes data for car1
                  for (i=0; i<=14; i=i+1)
                  begin
                     tempX = 0;
                     tempY = i*2;
                     tempColor = (i<n_car1_out) ? 4:0;
                     
                     for (j=0; j<=7; j=j+1)
                     begin
                        x[i*8+j] = tempX[j];
                        y[i*8+j] = tempY[j];
                     end
                     for (j=0; j<=2; j=j+1) 
                     begin
                        color[i*3+j] = tempColor[j];
                     end
                  end
                  
                  // Initializes data for car2
                  for (i=15; i<=29; i=i+1)
                  begin
                     tempX = 0;
                     tempY = i*2;
                     tempColor = (i-15<n_car2_out) ? 2 : 0;
                     
                     for (j=0; j<=7; j=j+1)
                     begin
                        x[i*8+j] = tempX[j];
                        y[i*8+j] = tempY[j];
                     end
                     for (j=0; j<=2; j=j+1) 
                     begin
                        color[i*3+j] = tempColor[j];
                     end
                  end

                  // Initializes data for car3
                  for (i=30; i<=44; i=i+1)
                  begin
                     tempX = 0;
                     tempY = i*2;
                     tempColor = (i-30 < n_car3_out) ? 5 : 0;
                     
                     for (j=0; j<=7; j=j+1)
                     begin
                        x[i*8+j] = tempX[j];
                        y[i*8+j] = tempY[j];
                     end
                     for (j=0; j<=2; j=j+1) 
                     begin
                        color[i*3+j] = tempColor[j];
                     end
                  end
                  
               end
               
               if(init_player_data)
               begin
                  playerX = `PLAYER_SPAWN_X;
                  playerY = `PLAYER_SPAWN_Y;
                  playerColor =`PLAYER_COLOR;
               end
                       
        end
     end
     
endmodule

/**
Input: clock, Clear_b, period
Output: q, pulse

This module implements a rate divider for the objects
in the game. period sets the number of cycles of the clock
before the divider resets. If you want the period to be P,
then set period to be P-1. The Pth cycle is when q==period
and q resets. Pulse is generated in throughout the last 
cycle. Clear_b resets the counter to start countring from
0 and sets pulse to 0. The divider will function iff enable
is 1'b1.  Reset will work regardless of the value of enable.
**/
module RateDivider (clock, reset_n, enable, period, pulse);  
    input [0:0] clock;
    input [0:0] reset_n;
    input [25:0] period;
    input enable;
    output reg pulse;

    // declares q
    reg [26:0] q; 
    
    // declares d, not needed
    //wire [27:0] d; 
    initial
    begin
       pulse <= 0;
       q <= 0;
    end
    
    // triggered every time clock rises
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
            		// q reset to 0
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

This module implements a counter
that counts up to the limit value. 
Once limit is reached pulse will be
generated on the next posedge of clock.
reset_n resets the q value
to 0 to allow the counter to count again.
reset_n_pulse is also an active low reset, but
sets pulse to 1 instad of 0.
**/
module counter(clock, reset_n, reset_n_pulse_1 , pulse, limit);

   input clock, reset_n, reset_n_pulse_1;
   input [25:0] limit;

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
         q = 0;
         pulse = 0;
      end
      
      if(!reset_n_pulse_1)
      begin
         q = 0;
         pulse = 1;
      end
      
      if(reset_n && reset_n_pulse_1)
      begin
         
         if(q == limit)
         begin
            pulse = 1;
         end
         else
         begin
            q = q + 1;
         end
      end
   end
endmodule 

/**
Input: c
Output: HEX

This module implements a 7 segment HEX display.
c[3:0] are the input bits. HEX[6:0] are
the output bits. An segment of the display
is on iff its corresponding output bit is set to 0.
**/
module HEXDisplay(HEX,c);
  	output reg[6:0] HEX;
	input [3:0] c;
	
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
outputs a 90 bit random number per
cycle of clk(on positive edge). rst_n
is a synchronous active low reset. 
**/
module fibonacci_lfsr_90bit(
  input clk,
  input rst_n,

  output reg [89:0] data
);

reg [89:0] data_next;
integer i;

// Initializes output value
initial
begin
   data = 90'b1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111;
end

// Computes next number to be generated 
always @* begin
  data_next[89] = data[89]^data[1];
  data_next[88] = data[88]^data[0];
  for (i=87; i>=0; i=i-1)
  begin
     data_next[i]=data[i]^data_next[i+2];
  end
end

// Processes reset and updates output
always @(posedge clk or negedge rst_n)
  if(!rst_n)
    data <= 90'b1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111;
  else
    data <= data_next;

endmodule
