// A push button on gpio_2 (to ground, with the pin's pull-up) lights an LED on gpio_4 while pressed, and an SB_IO
// with an output enable drives gpio_6 only while gpio_9 is high (high impedance otherwise).
module top(input gpio_2, output gpio_4, inout gpio_6, input gpio_9);
  wire pressed_n;
  SB_IO #(.PIN_TYPE(6'b0000_01), .PULLUP(1'b1)) button (.PACKAGE_PIN(gpio_2), .D_IN_0(pressed_n));
  assign gpio_4 = ~pressed_n;
  SB_IO #(.PIN_TYPE(6'b1010_01)) tri_pin (.PACKAGE_PIN(gpio_6), .OUTPUT_ENABLE(gpio_9), .D_OUT_0(1'b1));
endmodule
