/**
 * @param xArray is a 120 bit output that represents the x coordinates of a 3x5
 *        matrix of pixels, in row by row order, each coordinate is 8 bits
 * @param yArray is a 120 bit output that represents the y coordinates of a 3x5
 *        matrix of pixels, in row by row order, each coordinate is 8 bits
 * @param colorArray is a 45 bit output that represents the rgb values of a 3x5
 *        matrix of pixels, in row by row order, each pixel is 3 bits
 * @param in is a 4 bit binary input that represents values 0 to F
 */
 module numbersArrayOutput(xArray, yArray, colorArray, in);
   input [3:0]in;
   output reg [119:0]xArray;
   output reg [119:0]yArray;
   output reg [44:0]colorArray;
   initial
   begin
     if(in == 4'b0000)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111111000111111000111111000111111111111;
     end
     else if (in == 4'b0001)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111000000111000000111000000111000111111111;
     end
     else if (in == 4'b0010)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111000000111111111111111000000111111111;
     end
     else if (in == 4'b0011)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111000000111111111111000000111111111111;
     end
     else if (in == 4'b0100)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111000111111000111111111111000000111000000111;
     end
     else if (in == 4'b0101)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111111000000111111111000000111111111111;
     end
     else if (in == 4'b0110)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111111000000111111111111000111111111111;
     end
     else if (in == 4'b0111)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111000000111000000111000000111000000111;
     end
     else if (in == 4'b1000)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111111000111111111111111000111111111111;
     end
     else if (in == 4'b1001)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111111000111111111111000000111000000111;
     end
     else if (in == 4'b1010)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111111000111111111111111000111111000111;
     end
     else if (in == 4'b1011)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111000000111000000111111111111000111111111111;
     end
     else if (in == 4'b1100)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111111000000111000000111000000111111111;
     end
     else if (in == 4'b1101)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b000000111000000111111111111111000111111111111;
     end
     else if (in == 4'b1110)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111111000000111111111111000000111111111;
     end
     else if (in == 4'b1111)
     begin
       xArray = 120'b000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010000000000000000100000010;
       yArray = 120'b000000000000000000000000000000010000000100000001000000100000001000000010000000110000001100000011000001000000010000000100;
       colorArray = 45'b111111111111000000111111111111000000111000000;
     end
   end
 endmodule