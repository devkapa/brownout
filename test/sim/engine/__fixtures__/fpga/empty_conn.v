// An SB_IO whose input is left unconnected (Yosys writes "D_IN_0": []), beside a LUT reading a constant 0: gpio_4 is
// !gpio_3, which the unconnected input must not disturb.
module top(inout gpio_2, input gpio_3, output gpio_4);
  SB_IO #(.PIN_TYPE(6'b0110_01)) io (.PACKAGE_PIN(gpio_2), .D_OUT_0(gpio_3), .D_IN_0());
  SB_LUT4 #(.LUT_INIT(16'h0002)) l (.I0(gpio_3), .I1(1'b0), .I2(1'b0), .I3(1'b0), .O(gpio_4));
endmodule
