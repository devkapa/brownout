// Flip-flops that clock themselves once gpio_2 rises: q1 on the rising and q2 on the falling edge of q1 ^ q2 ^
// gpio_2, so each toggle makes the next edge in the same instant. It settles at power-up and oscillates after.
module top(input gpio_2, output gpio_3);
  reg q1 = 0, q2 = 0;
  wire c = q1 ^ q2 ^ gpio_2;
  always @(posedge c) q1 <= ~q1;
  always @(negedge c) q2 <= ~q2;
  assign gpio_3 = q1;
endmodule
