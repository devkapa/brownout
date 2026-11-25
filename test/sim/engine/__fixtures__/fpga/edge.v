// A register set by a rising edge on gpio_2: an input already high when the FPGA is configured has made no edge.
module top(input gpio_2, output gpio_3);
  reg q = 0;
  always @(posedge gpio_2) q <= 1'b1;
  assign gpio_3 = q;
endmodule
