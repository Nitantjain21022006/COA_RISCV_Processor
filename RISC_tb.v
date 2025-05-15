`timescale 100ns/1ps
`include "processor.v"
module testbench();
    reg [7:0] InpExtWorld1, InpExtWorld2, InpExtWorld3, InpExtWorld4;
    wire [7:0] OutExtWorld1, OutExtWorld2, OutExtWorld3, OutExtWorld4;
    reg clk, Reset;
    RISCprocessor RISCprocessorInst(clk,Reset,InpExtWorld1,InpExtWorld2,InpExtWorld3,InpExtWorld4,OutExtWorld1,OutExtWorld2,OutExtWorld3,OutExtWorld4);
    always begin
        #10 clk = ~clk;
    end
    initial begin
        clk <= 1'b0;
        InpExtWorld1 <= 8'b0010_0100;
        $dumpfile("Wavedump.vcd");
        $dumpvars(0, testbench);
        for (integer i = 0; i < 256; i = i + 1) begin
            $dumpvars(1,testbench.RISCprocessorInst.SRAM_Inst.datamem[i]);
        end
        Reset <= 1'b1;
        #50;
        Reset <= 1'b0;
    end
endmodule