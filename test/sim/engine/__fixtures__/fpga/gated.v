// Enables from logic: the 48 MHz oscillator runs only while gpio_2 is high, and the RGB driver lights only while
// gpio_4 is high (green at 1 mA; blue has no current set, so it stays dark).
module top(input gpio_2, input gpio_4, output gpio_3, output led_green, output led_blue);
  wire clk;
  SB_HFOSC #(.CLKHF_DIV("0b11")) osc (.CLKHFPU(1'b1), .CLKHFEN(gpio_2), .CLKHF(clk));
  reg [3:0] n = 0;
  always @(posedge clk) n <= n + 1;
  assign gpio_3 = n[0];
  SB_RGBA_DRV #(.CURRENT_MODE("0b1"), .RGB0_CURRENT("0b000001")) rgb (
    .CURREN(1'b1), .RGBLEDEN(gpio_4), .RGB0PWM(1'b1), .RGB1PWM(1'b1), .RGB2PWM(1'b0),
    .RGB0(led_green), .RGB1(led_blue));
endmodule
