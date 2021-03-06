// Part 2 skeleton

module pacman
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
		defparam VGA.BACKGROUND_IMAGE = "black.mif";*/
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
    
    // Instansiate datapath
	// datapath d0(...);

    // Instansiate FSM control
    // control c0(...);
    
endmodule

//module datapath(clock, reset_n,);

//module control(clock, reset_n, );

module memory(clock, reset_n, x, y, color, n_yellow_beans, n_red_beans, n_mobs, op, offsetxs_in, offsetys_in, new_colors_in);

     input clock, reset_n;
     input [93:0] offsetxs_in;
     input [93:0] offsetys_in;
     input [140:0] new_colors_in;

     // offeset values: 00 = no change; 01=+1, 10=-1
     wire [1:0] offsetx [0:46];
     wire [1:0] offsety [0:46];

     // new colors
     wire [2:0] newcolors [0:46];
     
     // memory operation types
     input [2:0] op;

     // number of each objects (15 max)
     input [3:0] n_yellow_beans;
     input [3:0] n_red_beans;
     input [3:0] n_mobs;


     // There are 15 mobs + 15 red beans + 15 yellow beans + player coords
     output reg [7:0] x [0:46];
     output reg [7:0] y [0:46];
     output reg [2:0] color [0:46];
     
     integer i;
     integer j;
     genvar k;
     
     generate 
         for (k=0; k<=46; k=k+1)
         begin
            assign offsetx[k] = offsetxs_in[2*k+1:2*k];
            assign offsety[k] = offsetys_in[2*k+1:2*k];
            assign newcolors[k] = new_colors_in[3*k+2:3*k];
         end
     endgenerate 

     
     initial
     begin
        for (i=0; i<=46; i=i+1)
        begin
           x[i] <= 0;
           y[i] <= 0;
           color[i] <= 0;
        end
     end

     always @(posedge clock)
     begin
        if(!reset_n)
        begin
           for (i=0; i<=46; i=i+1)
       	   begin
           	x[i] <= 0;
           	y[i] <= 0;
           	color[i] <= 0;
           end
        end
        else
        begin
           case (op)
               
               3'b001: begin

                          // Updates coord using offset
                          for (i=0;i<=46; i=i+1)
                          begin
                             if(offsetx[i] ==2'b00)
                             begin
                                x[i] = x[i] + 0;
                             end
                             else if(offsetx[i] == 2'b01)
                             begin
                                x[i] = x[i] +8'b0000_00001;
                             end
                             else if(offsetx[i] == 2'b10)
                             begin
                                x[i] = x[i] - 8'b0000_00001;
                             end

                             if(offsety[i] ==2'b00)
                             begin
                                y[i] = y[i] + 0;
                             end
                             else if(offsety[i] == 2'b01)
                             begin
                                y[i] = y[i] +8'b0000_00001;
                             end
                             else if(offsety[i] == 2'b10)
                             begin
                                y[i] = y[i] - 8'b0000_00001;
                             end
                             color[i] = newcolors[i];
                          end
                       end
               3'b010: begin
                          // initializes object points on the screen

                          // Initializes the positions of the 15 mobs
                          for (i=0; i<=14; i=i+1)
                          begin
                              x[i] = i*3;
                              y[i] = 42;
                              color[i] = 3'b111; // white
                          end
                          
                          // Initializes the positions of the 15 red beans
                          for (i=15; i<=29; i=i+1)
                          begin
                          end
                             x[i] = i*3;
                             y[i] = 42;
                             color[i] = 3'b100; // Red
                          // initializes the positions of the 15 yellow beans
                          for (i=30; i<=44; i=i+1)
                          begin
                             x[i] = i*3;
                             y[i] = 42;
                             color[i] = 3'b110; // Yellow
                          end
                          
                          // Initializes the position of the player
                          x[45] = 8'b0100_1111; // 79
                          y[45] = 8'b0110_1110; // 110
                          color[45] = 3'b001;  // Blue
              
                       end
           endcase

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
cycle. 
**/
module RateDivider (clock, q, Clear_b, period, pulse);  
    input [0:0] clock;
    input [0:0] Clear_b;
    input [25:0] period;
    output reg pulse;

    // declares q
    output reg [26:0] q; 
    
    // declares d, not needed
    //wire [27:0] d; 
    initial
    begin
       pulse = 0;
       q = 0;
    end
    
    // triggered every time clock rises
    always@(posedge clock)   
    begin
        pulse = 1'b0;
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
            q <= q + 1'b1;  
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
module fibonacci_lfsr_5bit(
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

