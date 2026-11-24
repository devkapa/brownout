// A tristate net on two inout ports: gpio_4 carries gpio_3 while gpio_2 is high and is released otherwise; gpio_6
// always drives what gpio_4's pad reads.
module top(input gpio_2, input gpio_3, inout gpio_4, inout gpio_6);
  assign gpio_4 = gpio_2 ? gpio_3 : 1'bz;
  assign gpio_6 = gpio_4;
endmodule
