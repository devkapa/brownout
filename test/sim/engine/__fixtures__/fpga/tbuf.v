// An inout port written as a tristate assign (Yosys leaves a $_TBUF_ on it): gpio_6 carries gpio_2 while gpio_9 is
// high and is released otherwise, and gpio_3 reads the pin back.
module top(inout gpio_6, input gpio_9, input gpio_2, output gpio_3);
  assign gpio_6 = gpio_9 ? gpio_2 : 1'bz;
  assign gpio_3 = gpio_6;
endmodule
