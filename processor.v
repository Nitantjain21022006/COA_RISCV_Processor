`timescale 100ns/1ps
`include "subcomponents.v"
module RISCprocessor(clk, Reset, InpExtWorld1, InpExtWorld2, InpExtWorld3, InpExtWorld4, OutExtWorld1, OutExtWorld2, OutExtWorld3, OutExtWorld4);
    input clk, Reset;
    input [7:0] InpExtWorld1, InpExtWorld2, InpExtWorld3, InpExtWorld4;
    output wire [7:0] OutExtWorld1, OutExtWorld2, OutExtWorld3, OutExtWorld4;
    wire [7:0] RegDataout1, RegDataout2, RegDatain,Imm,SRAMAddress, SRAM_Dataout,StackAddress, StackDatain, StackDataout,PCAddress, PC, PC_D2,InAddress, InDataout,OutAddress, Out,  Datain,Operand1, Operand2, ALU_OUT;
    wire [4:0] OPCODE;
    wire [3:0] AddressR1, AddressR2, AddressW;
    wire [24:0] Instruction;
    wire T0, T1, T2, T3, T4;
    wire PCUpdate, ALUSave, ZflagSave, CflagSave;
    wire SRAMRead, SRAMWrite, StackRead, StackWrite, INportRead, OUTportWrite, RegFileRead, RegFileWrite;
    wire Zflag, Cflag;
    
    TimingGen TimingGenInst(clk, Reset, T0, T1, T2, T3, T4);
    ControlLogic ControlLogicInst(clk, Reset, T1, T2, T3, T4, Zflag, Cflag, OPCODE, PCUpdate, SRAMRead, SRAMWrite, StackRead, StackWrite, ALUSave, ZflagSave, CflagSave, INportRead, OUTportWrite, RegFileRead, RegFileWrite);
    
    Mux32to1_8bit Mux32to1_8bit_PCInst(PC, PC, PC, ALU_OUT, ALU_OUT, PC, PC, PC, PC, PC, PC, PC, PC, PC, Imm, Imm, Imm, Imm, Imm, PC, PC, PC, PC, PC, PC, PC, PC, PC, PC, PC, PC, PC, OPCODE, PCAddress);
    Program_Counter ProgramCounterInst(clk, Reset, T0, PCUpdate, PCAddress, PC, PC_D2);
    InstMEM InstMEMInst(clk, Reset, PC, T0, Instruction, OPCODE, AddressW, AddressR1, AddressR2, Imm);
    
    Mux32to1_8bit Mux32to1_8bit_RegInst(RegDatain, RegDatain, RegDatain, RegDatain, RegDatain, ALU_OUT, ALU_OUT, ALU_OUT, RegDatain, SRAM_Dataout, RegDatain, InDataout, StackDataout, RegDatain, RegDatain, RegDatain, RegDatain, RegDatain, RegDatain, RegDatain, SRAM_Dataout, ALU_OUT, ALU_OUT, ALU_OUT, ALU_OUT, ALU_OUT, ALU_OUT, ALU_OUT, ALU_OUT, ALU_OUT, ALU_OUT, RegDatain, OPCODE, RegDatain);
    RegisterFile RegisterFileInst(clk, Reset, RegFileRead, RegFileWrite, RegDatain, AddressR1, AddressR2, AddressW, RegDataout1, RegDataout2);
    
    Mux32to1_8bit Mux32to1_8bit_ALU_OP1Inst(8'b00000000, 8'b00000000, 8'b00000000, PC_D2, PC_D2, RegDataout1, RegDataout1, RegDataout1, 8'b00000000, 8'b00000000, RegDataout1, 8'b00000000, 8'b00000000, RegDataout1, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, RegDataout1, 8'b00000000, 8'b00000000, RegDataout1, RegDataout1, RegDataout1, RegDataout1, RegDataout1, RegDataout1, RegDataout1, RegDataout1, RegDataout1, 8'b00000000, OPCODE, Operand1);
    Mux32to1_8bit Mux32to1_8bit_ALU_OP2Inst(8'b00000000, 8'b00000000, 8'b00000000, {1'b0, Imm[6:0]}, {1'b0, Imm[6:0]}, RegDataout2, RegDataout2, RegDataout2, RegDataout2, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000, Imm, 8'b00000000, Imm, Imm, Imm, Imm, RegDataout2, RegDataout2, RegDataout2, RegDataout2, 8'b00000000, OPCODE, Operand2);
    ALU_top1 ALU_top1Inst(clk, Reset, Imm[7], Operand1, Operand2, OPCODE, ALUSave, ZflagSave, CflagSave, Zflag, Cflag, ALU_OUT);

    Mux32to1_8bit Mux32to1_8bit_SRAMInst(SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, RegDataout1, RegDataout1, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, Imm, Imm, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress, SRAMAddress,OPCODE,  SRAMAddress);
    SRAM SRAM_Inst(clk, Reset, SRAMAddress, SRAMRead, SRAMWrite, ALU_OUT, SRAM_Dataout);
    In_port In_port_Inst(clk, Reset, INportRead, InpExtWorld1, InpExtWorld2, InpExtWorld3, InpExtWorld4, Imm, InDataout);
    OUT_port OUT_port(clk, Reset, Imm, ALU_OUT, OUTportWrite, OutExtWorld1, OutExtWorld2, OutExtWorld3, OutExtWorld4);
    Stack Stack(clk, Reset, StackRead, StackWrite, ALU_OUT, StackDataout);
endmodule