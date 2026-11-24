// A plain output on an RGB pad (placed by name on pin 41): the UP5K's RGB pads are open-drain, so a 1 releases it.
module top(input gpio_2, output led_red);
  assign led_red = gpio_2;
endmodule
