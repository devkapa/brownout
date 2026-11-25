// Buses whose indices are not 0 up: sw[3:1] starts at 1 and led[0:2] counts up, so a PCF names their bits by those
// indices, and led[0] (on pin 9) is the MSB Yosys stores last.
module top(input [3:1] sw, output [0:2] led);
  assign led = ~sw;
endmodule
