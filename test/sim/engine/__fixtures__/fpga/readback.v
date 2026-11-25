// An open-drain pin, as I2C uses: gpio_6 pulls low while gpio_9 is high and is released (pulled up) otherwise;
// gpio_3 shows the level the pin's own input reads, and a register clocked by gpio_2 powers up at 0 although its
// clock comes through a global buffer from a pulled-up pin.
module top(inout gpio_6, input gpio_9, output gpio_3, input gpio_2, output gpio_4);
  wire din;
  SB_IO #(.PIN_TYPE(6'b1010_01), .PULLUP(1'b1)) sda (.PACKAGE_PIN(gpio_6), .OUTPUT_ENABLE(gpio_9), .D_OUT_0(1'b0), .D_IN_0(din));
  assign gpio_3 = din;
  wire btn, clk;
  SB_IO #(.PIN_TYPE(6'b0000_01), .PULLUP(1'b1)) b (.PACKAGE_PIN(gpio_2), .D_IN_0(btn));
  SB_GB gb (.USER_SIGNAL_TO_GLOBAL_BUFFER(btn), .GLOBAL_BUFFER_OUTPUT(clk));
  reg q = 0;
  always @(posedge clk) q <= ~q;
  assign gpio_4 = q;
endmodule
