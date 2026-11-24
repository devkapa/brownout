// An inout port the design always drives, from an input: gpio_3 is an output carrying gpio_2.
module top(input gpio_2, inout gpio_3);
  assign gpio_3 = gpio_2;
endmodule
