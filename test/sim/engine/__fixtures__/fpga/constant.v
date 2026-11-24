// Outputs tied to constants: an LED held on, one held off, and the flash deselected.
module top(output gpio_3, output gpio_4, output spi_ssn);
  assign gpio_3 = 1'b1;
  assign gpio_4 = 1'b0;
  assign spi_ssn = 1'b1;
endmodule
