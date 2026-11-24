// A 70-stage ripple counter on the board clock: every stage toggles in the same instant on its first edges.
module top(input gpio_20, output gpio_3);
  reg [69:0] t = 0;
  always @(posedge gpio_20) t[0] <= ~t[0];
  genvar i;
  generate for (i = 1; i < 70; i = i + 1) begin : s
    always @(posedge t[i-1]) t[i] <= ~t[i];
  end endgenerate
  assign gpio_3 = t[69];
endmodule
