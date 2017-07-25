// Part 2 skeleton

module part2
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
		VGA_B   						//	VGA Blue[9:0]
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;

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
	assign resetn = KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	/*vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			
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
		defparam VGA.BACKGROUND_IMAGE = "black.mif";*/
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
    
    // Instansiate datapath
	// datapath d0(...);

    // Instansiate FSM control
    // control c0(...);
    
endmodule

module datapath(clock, reset, deg, coefficient,  color, ld_coefficient, ld_deg, ld_color, ld_alu_out, ld_result, alu_op, xout, yout, color_out, status);
        input clock, reset;
        input [3:0] deg;
        input [10:0] coefficient;
        input [2:0] color;
        input ld_coefficient, ld_deg, ld_color, ld_alu_out, ld_result;
        input [4:0] alu_op;
        
        output reg [] xout, yout;
        output reg [2:0] color_out;
        output reg [3:0] status;
      
        

endmodule;

module control(clock, reset, go, plot, status_in, ld_coefficient, ld_deg, ld_color, ld_alu_out, ld_result, alu_op);

	input clock, reset, go;
        input [3:0] status_in;

        output reg ld_coefficient, ld_deg, ld_coord, ld_coords, ld_color, ld_alu_out, ld_result;
        output reg [4:0] alu_op;

	localparam S_LOAD_N = 5'd0,
               	   S_LOAD_N_WAIT = 5'd1,
                   S_LOAD_COEF = 5'd2,
               	   S_LOAD_COEF_WAIT = 5'd3,
                   S_LOAD_COLOR = 5'd4,
                   S_LOAD_COLOR_WAIT = 5'd5,
                   S_CYCLE_0 = 5'd6,
                   S_CYCLE_1 = 5'd7,
                   S_CYCLE_A = 5'd8,
                   S_CYCLE_2 = 5'd9,
                   S_CYCLE_3 = 5'd10;
           
        

endmodule

