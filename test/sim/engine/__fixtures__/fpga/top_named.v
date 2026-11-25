// A submodule beside the top, and a top not named top: the netlist lists Yosys's cell library models too.
module inv(input a, output y);
  assign y = ~a;
endmodule
module blink(input gpio_2, output gpio_3);
  inv u (.a(gpio_2), .y(gpio_3));
endmodule
