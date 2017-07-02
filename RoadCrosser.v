// Part 2 skeleton

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

// requires major modifications to change to memory for RoadCrosser game from Pacman
module memory(clock, reset_n, x, y, color, playerX, playerY, playerColor, score, lives, n_car1_out, n_car2_out, n_car3_out, n_car1_in, n_car2_in, n_car3_in, car1_x_in, car2_x_in, car3_x_in, car1_y_in,
 car2_y_in, car3_y_in, car1_color_in, car2_color_in, car3_color_in, player_x_in, player_y_in, player_color_in, lives_in, score_in, load_car1, load_car2, load_car3, load_num_cars, load_player, load_lives,
 load_score, reset_score);

     input clock, reset_n;
     
     // memory operation types
     //input [3:0] op;
     
     // new op code inputs
     input load_car1, load_car2, load_car3, load_num_cars, load_player, load_lives, load_score, reset_score;

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
     input [119:0]car3_y_in;
    
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
     
     output reg [7:0] score;
     output reg [3:0] lives; // max 15 lives

     integer i;
     integer j;
     genvar k;
     
     // Initializes all registers
     initial
     begin
        for (i=0; i<=44; i=i+1)
        begin
           for (j=8*i; j<=8*i+7; j=j+1)
           begin
              x[j] <= 1'b0;
              y[j] <= 1'b0;
           end
           
           for (j=3*i; j<=3*i+2; j=j+1)
           begin
              color[j] <= 1'b0;
           end
        end
        playerX <= 8'b0000_0000;
        playerY <= 8'b0000_0000;
        playerColor <= 3'b000;
        score <= 8'b0000_0000;
        lives <= 4'b0001;
     end

     always @(posedge clock)
     begin
        if(!reset_n)
        begin
            
            // Score is not reset here. It is only reset using opcode 3'b110.
           for (i=0; i<=44; i=i+1)
           begin
               for (j=8*i; j<=8*i+7; j=j+1)
               begin
                    x[j] <= 1'b0;
                    y[j] <= 1'b0;
               end
           
               for (j=3*i; j<=3*i+2; j=j+1)
               begin
                    color[j] <= 1'b0;
               end
           end
            playerX <= 8'b0000_0000;
            playerY <= 8'b0000_0000;
            playerColor <= 3'b000;
   	    lives <= 4'b0001;
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
                                  x[j] <= car1_x_in[j-8*i];
           			  y[j] <= car1_y_in[j];
                               end
           		       for (j=3*i; j<=3*i+2; j=j+1)
                               begin
                                  color[j] <= car1_color_in[j];
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
                                  x[j] <= car2_x_in[j-8*i];
           			  y[j] <= car2_y_in[j-120];
                               end
           		       for (j=3*i; j<=3*i+2; j=j+1)
                               begin
                                  color[j] <= car2_color_in[j-45];
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
                                  x[j] <= car3_x_in[j-8*i];
                                  y[j] <= car3_y_in[j-240];
                               end
                               
           		       for (j=3*i; j<=3*i+2; j=j+1)
                               begin
                                  color[j] <= car3_color_in[j-90];
                               end          
           			//x[8*i+7:8*i] <= car3_x_in;
           			//y[8*i+7:8*i] <= car3_y_in[8*(i-30)+7:8*(i-30)];
           			//color[3*i+2:3*i] <= car3_color_in[3*(i-44)+2:3*(i-44)];
       			  end
               end
                
               if(load_player)
               begin
                           // Updates player data
                           playerX <= player_x_in;
                           playerY <= player_y_in;
                           playerColor <= player_color_in;
               end
               
               if(load_lives)
               begin 
                         lives <= lives_in;
               end

               if(reset_score)
               begin
                         // resets score
                         score <= 8'b0000_0000;
               end
               
               if(load_num_cars)
               begin
                           // Updates car numbers for each type
                           n_car1_out <= n_car1_in;
                           n_car2_out <= n_car2_in;
                           n_car3_out <= n_car3_in;
               end
               if(load_score)
               begin
                           score <= score_in;
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
