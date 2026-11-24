// The board's 12 MHz oscillator on gpio_20 (the OSC jumper), a ripple divider of toggle flip-flops each clocked by
// the previous stage's output, an asynchronous reset from gpio_2 (active high), and a negative-edge register.
module top(input gpio_20, input gpio_2, output gpio_3, output gpio_4, output gpio_6);
  reg [3:0] t = 0;
  always @(posedge gpio_20 or posedge gpio_2) if (gpio_2) t[0] <= 0; else t[0] <= ~t[0];
  genvar i;
  generate for (i = 1; i < 4; i = i + 1) begin : stage
    always @(posedge t[i-1] or posedge gpio_2) if (gpio_2) t[i] <= 0; else t[i] <= ~t[i];
  end endgenerate
  reg n = 0;
  always @(negedge gpio_20) n <= t[0];
  assign gpio_3 = t[3];
  assign gpio_4 = t[0];
  assign gpio_6 = n;
endmodule
