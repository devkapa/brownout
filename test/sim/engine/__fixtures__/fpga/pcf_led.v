// Ports named freely and placed by a PCF: a 4-bit bus on package pins, and an enable-gated counter.
module top(input clk, input en, output [3:0] leds);
  reg [3:0] c = 4'd5;
  always @(posedge clk) if (en) c <= c + 1;
  assign leds = c;
endmodule
