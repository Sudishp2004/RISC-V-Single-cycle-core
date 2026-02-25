`timescale 1ns/1ps
module tb;

reg clk, reset;
top uut(clk, reset);

always #5 clk = ~clk;

initial begin
    clk = 0;
    reset = 1;
    #20 reset = 0;
    #500 $stop;
end

endmodule
