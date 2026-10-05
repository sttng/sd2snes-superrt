// Behavioural stand-ins for the Quartus IP used by the core (simulation only)
`timescale 1ns / 1ps
module pll(input areset, input inclk0, output reg c0 = 0, output reg c1 = 0, output locked);
  // c0: 96 MHz system clock, c1: 76.8 MHz SuperRT engine clock (8 MHz * 12 / * 48 / 5);
  // c1 gets a small offset so the two domains do not run in lock step
  always #5.208 c0 = ~c0;
  initial begin #1.3; forever #6.510 c1 = ~c1; end
  assign locked = 1'b1;
endmodule

module snescmd_buf(input [8:0] address_a, input [8:0] address_b, input clock,
                   input [7:0] data_a, input [7:0] data_b, input wren_a, input wren_b,
                   output reg [7:0] q_a, output reg [7:0] q_b);
  reg [7:0] mem [0:511];
  integer i; initial for(i = 0; i < 512; i = i + 1) mem[i] = 0;
  always @(posedge clock) begin
    if(wren_a) mem[address_a] <= data_a;
    if(wren_b) mem[address_b] <= data_b;
    q_a <= mem[address_a];
    q_b <= mem[address_b];
  end
endmodule


module msu_databuf(input clock, input [7:0] data, input [13:0] rdaddress, input [13:0] wraddress,
                   input wren, output reg [7:0] q);
  reg [7:0] mem [0:16383];
  always @(posedge clock) begin
    if(wren) mem[wraddress] <= data;
    q <= mem[rdaddress];
  end
endmodule
module dac_buf(input clock, input [7:0] data, input [8:0] rdaddress, input [10:0] wraddress,
               input wren, output reg [31:0] q);
  reg [7:0] mem [0:2047];
  always @(posedge clock) begin
    if(wren) mem[wraddress] <= data;
    q <= {mem[{rdaddress, 2'b11}], mem[{rdaddress, 2'b10}], mem[{rdaddress, 2'b01}], mem[{rdaddress, 2'b00}]};
  end
endmodule
