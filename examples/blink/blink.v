// Blink for the AGM TCX board (AG10KL144), native Supra flow.
// 50 MHz oscillator on pin 23, LEDs D0..D3 on pins 141..144, active low.
module blink(
  input  clk,          // 50 MHz
  input  resetn,       // button S, active low (pin 25 as a guess - see .ve)
  output reg [3:0] led
);
  reg [25:0] cnt;
  always @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      cnt <= 26'd0;
      led <= 4'b0000;
    end else if (cnt == 26'd24_999_999) begin
      cnt <= 26'd0;
      led <= ~led;          // 1 Hz rotation of the pattern
    end else begin
      cnt <= cnt + 26'd1;
    end
  end
endmodule
