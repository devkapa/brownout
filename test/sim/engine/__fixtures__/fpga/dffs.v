// A flip-flop with its asynchronous set tied high: its output is 1 from configuration on.
module top(input gpio_2, output gpio_3);
  SB_DFFS ff (.C(gpio_2), .D(1'b0), .S(1'b1), .Q(gpio_3));
endmodule
