module fibonacci_lfsr_5bit(
  input clk,
  input rst_n,

  output reg [89:0] data
);

reg [89:0] data_next;
integer i;
initial
begin
   data = 90'b1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111;
end

always @* begin
  data_next[89] = data[89]^data[1];
  data_next[88] = data[88]^data[0];
  for (i=87; i>=0; i=i-1)
  begin
     data_next[i]=data[i]^data_next[i+2];
  end
  //data_next[2] = data[2]^data_next[4];
  //data_next[1] = data[1]^data_next[3];
  //data_next[0] = data[0]^data_next[2];
end

always @(posedge clk or negedge rst_n)
  if(!rst_n)
    data <= 90'b1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111_1111111111;
  else
    data <= data_next;

endmodule

