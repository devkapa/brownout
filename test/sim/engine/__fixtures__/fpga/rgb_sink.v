// The RGB driver's current settings: full-current mode, red (RGB2) at 8 mA while gpio_2 is high, green (RGB0) at
// 24 mA always, blue unused.
module top(input gpio_2, output led_red, output led_green);
  SB_RGBA_DRV #(.CURRENT_MODE("0b0"), .RGB0_CURRENT("0b111111"), .RGB2_CURRENT("0b000011")) rgb (
    .CURREN(1'b1), .RGBLEDEN(1'b1), .RGB0PWM(1'b1), .RGB1PWM(1'b0), .RGB2PWM(gpio_2),
    .RGB0(led_green), .RGB2(led_red));
endmodule
