// The internal 48 MHz oscillator divided to 12 MHz clocks a 24-bit counter; its top bits drive the on-board RGB LED
// through the current-sink driver, and gpio_3 shows bit 3 XOR the level on gpio_2.
module top(input gpio_2, output gpio_3, output led_red, output led_green, output led_blue);
  wire clk;
  SB_HFOSC #(.CLKHF_DIV("0b10")) osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(clk));
  reg [23:0] count = 0;
  always @(posedge clk) count <= count + 1;
  assign gpio_3 = count[3] ^ gpio_2;
  SB_RGBA_DRV #(.CURRENT_MODE("0b1"), .RGB0_CURRENT("0b000001"), .RGB1_CURRENT("0b000001"), .RGB2_CURRENT("0b000001"))
    rgb (.CURREN(1'b1), .RGBLEDEN(1'b1), .RGB0PWM(count[23]), .RGB1PWM(count[22]), .RGB2PWM(count[21]),
         .RGB0(led_green), .RGB1(led_blue), .RGB2(led_red));
endmodule
