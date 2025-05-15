//Basic Modules required and are used in subcomponents
module Mux32to1_8bit(
    input [7:0] I31, I30, I29, I28, I27, I26, I25, I24, I23, I22, I21, I20, I19, I18, I17, I16,I15, I14, I13, I12, I11, I10, I9, I8,I7, I6, I5, I4, I3, I2, I1, I0,
    input [4:0] S,
    output reg [7:0] Y
);
    always @(*) begin
        case(S)
            5'b00000: Y = I0;
            5'b00001: Y = I1;
            5'b00010: Y = I2;
            5'b00011: Y = I3;
            5'b00100: Y = I4;
            5'b00101: Y = I5;
            5'b00110: Y = I6;
            5'b00111: Y = I7;
            5'b01000: Y = I8;
            5'b01001: Y = I9;
            5'b01010: Y = I10;
            5'b01011: Y = I11;
            5'b01100: Y = I12;
            5'b01101: Y = I13;
            5'b01110: Y = I14;
            5'b01111: Y = I15;
            5'b10000: Y = I16;
            5'b10001: Y = I17;
            5'b10010: Y = I18;
            5'b10011: Y = I19;
            5'b10100: Y = I20;
            5'b10101: Y = I21;
            5'b10110: Y = I22;
            5'b10111: Y = I23;
            5'b11000: Y = I24;
            5'b11001: Y = I25;
            5'b11010: Y = I26;
            5'b11011: Y = I27;
            5'b11100: Y = I28;
            5'b11101: Y = I29;
            5'b11110: Y = I30;
            5'b11111: Y = I31;
            default:  Y = 8'b0000_0000;
        endcase
    end
endmodule
module Mux4to1_8bit_withoutE(S, I3, I2, I1, I0, Y);
    input [1:0] S;
    input [7:0] I3, I2, I1, I0;
    output wire [7:0] Y;
    wire [7:0] I [0:3];

    assign { I[3], I[2], I[2], I[1], I[0] } = { I3, I2, I2, I1, I0 };
    assign Y = I[S];
endmodule
module Mux32to1_1bit(I,S,Y,E);
    input E;
    input [4:0] S;
    input [31:0] I;
    output wire Y;
    assign Y = (E == 1'b1) ? I[S] : 1'b0;
endmodule
module Mux32to1_1bit_withoutE(I, S,Y);
    input [4:0] S;
    input [31:0] I;
    output wire Y;
    assign Y = I[S];
endmodule
module Mux16to1_8bit_withoutE(S, I15, I14, I13, I12, I11, I10, I9, I8, I7, I6, I5, I4, I3, I2, I1, I0, Y);

    input [3:0] S;
    input [7:0] I15, I14, I13, I12, I11, I10, I9, I8, I7, I6, I5, I4, I3, I2, I1, I0;
    output wire [7:0] Y;
    wire [7:0] I [0:15];
    assign { I[15], I[14], I[13], I[12], I[11], I[10], I[9], I[8], I[7], I[6], I[5], I[4], I[3], I[2], I[1], I[0] } = { I15, I14, I13, I12, I11, I10, I9, I8, I7, I6, I5, I4, I3, I2, I1, I0 };
    assign Y = I[S];
endmodule
module Mux8to1_withoutE (S, I, Y);
    input [2:0] S;
    input [7:0] I;
    output wire Y;
    wire [7:0] Dtemp;
    assign Dtemp[0] = ~S[2] & ~S[1] & ~S[0] & I[0];
    assign Dtemp[1] = ~S[2] & ~S[1] & S[0] & I[1];
    assign Dtemp[2] = ~S[2] & S[1] & ~S[0] & I[2];
    assign Dtemp[3] = ~S[2] & S[1] & S[0] & I[3];
    assign Dtemp[4] = S[2] & ~S[1] & ~S[0] & I[4];
    assign Dtemp[5] = S[2] & ~S[1] & S[0] & I[5];
    assign Dtemp[6] = S[2] & S[1] & ~S[0] & I[6];
    assign Dtemp[7] = S[2] & S[1] & S[0] & I[7];
    assign Y = Dtemp[0] | Dtemp[1] | Dtemp[2] | Dtemp[3] | Dtemp[4] | Dtemp[5] | Dtemp[6] | Dtemp[7];
endmodule
module Decoder3to8_withoutE(A, D);
    input [2:0] A;
    output wire [7:0] D;
    assign D[0] = ~A[2]&~A[1]&~A[0];
    assign D[1] = ~A[2]&~A[1]&A[0];
    assign D[2] = ~A[2]&A[1]&~A[0];
    assign D[3] = ~A[2]&A[1]&A[0];
    assign D[4] = A[2]&~A[1]&~A[0];
    assign D[5] = A[2]&~A[1]&A[0];
    assign D[6] = A[2]&A[1]&~A[0];
    assign D[7] = A[2]&A[1]&A[0];
endmodule
module decoder4to16_withE(A, E, D);
    input [3:0]A; 
	input E;      
	output wire [15:0] D;
    assign D[0] = E&~A[3]&~A[2]&~A[1]&~A[0];
    assign D[1] = E&~A[3]&~A[2]&~A[1]&A[0];  
    assign D[2] = E&~A[3]&~A[2]&A[1]&~A[0]; 
    assign D[3] = E&~A[3]&~A[2]&A[1]&A[0];  
    assign D[4] = E&~A[3]&A[2]&~A[1]&~A[0]; 
    assign D[5] = E&~A[3]&A[2]&~A[1]&A[0]; 
    assign D[6] = E&~A[3]&A[2]&A[1]&~A[0]; 
    assign D[7] = E&~A[3]&A[2]&A[1]&A[0];  
    assign D[8] = E&A[3]&~A[2]&~A[1]&~A[0];
    assign D[9] = E&A[3]&~A[2]&~A[1]&A[0]; 
    assign D[10] = E&A[3]&~A[2]&A[1]&~A[0]; 
    assign D[11] = E&A[3]&~A[2]&A[1]&A[0];  
    assign D[12] = E&A[3]&A[2]&~A[1]&~A[0]; 
    assign D[13] = E&A[3]&A[2]&~A[1]&A[0]; 
    assign D[14] = E&A[3]&A[2]&A[1]&~A[0]; 
    assign D[15] = E&A[3]&A[2]&A[1]&A[0]; 
endmodule

module full_adder (A,B,Cin,Sum,Cout);
    input A, B, Cin;
    output Sum, Cout;
    assign Sum = A ^ B ^ Cin;
    assign Cout = (A & B) | (Cin & (A ^ B));
endmodule

module ripple_carry_adder(A, B, Cin, S, Cout, C7);
  input [7:0] A, B;
  input Cin;
  output wire [7:0] S;
  output wire Cout, C7;
  wire temp1, temp2, temp3, temp4, temp5, temp6;
  full_adder inst1(A[0], B[0], Cin, S[0], temp1);
  full_adder inst2(A[1], B[1], temp1, S[1], temp2);
  full_adder inst3(A[2], B[2], temp2, S[2], temp3);
  full_adder inst4(A[3], B[3], temp3, S[3], temp4);
  full_adder inst5(A[4], B[4], temp4, S[4], temp5);
  full_adder inst6(A[5], B[5], temp5, S[5], temp6);
  full_adder inst7(A[6], B[6], temp6, S[6], C7);
  full_adder inst8(A[7], B[7], C7, S[7], Cout);
endmodule

module Adder_Subtractor(A, B, M, S, Cout, Overflow);
  input [7:0] A, B;
  input M;
  output wire [7:0] S;
  output wire Cout, Overflow;
  wire [7:0] tempB;
  wire C7;
  xor inst0(tempB[0], B[0], M);
  xor inst1(tempB[1], B[1], M);
  xor inst2(tempB[2], B[2], M);
  xor inst3(tempB[3], B[3], M);
  xor inst4(tempB[4], B[4], M);
  xor inst5(tempB[5], B[5], M);
  xor inst6(tempB[6], B[6], M);
  xor inst7(tempB[7], B[7], M);
  ripple_carry_adder inst8(A, tempB, M, S, Cout, C7);
  xor inst10(Overflow, Cout, C7);
endmodule

module DFFwithSynReset(clk, D, Rst, Q);
input clk, D, Rst;
output reg Q;
always @(posedge clk)
	begin
			if(Rst == 1'b1)
				Q<=1'b0;
			else
				Q<=D;
	end
endmodule

module decoder3to8(A, D);
    input [2:0] A;
    output wire [7:0] D;
    assign D = 8'b1 << A;
endmodule

module priorityEncoder_4to1 (I, O);
    input [3:0] I;
    output wire [1:0] O;
    assign O = I[3] == 1'b1 ? 2'b11 : I[2] == 1'b1 ? 2'b10 : I[1] == 1'b1 ? 2'b01 : 2'b00;
endmodule

module Comparator_Unsigned_8bit(A, B, AgB, AlB,AeB);
    input [7:0] A, B;
    output wire AgB, AlB, AeB;

    wire [7:0] temp;
    wire Cout, Overflow, Zero;
    Adder_Subtractor inst1(A, B, 1'b1, temp, Cout, Overflow);
    not inst2(AlB, Cout);
    or inst3(Zero, temp[7], temp[6], temp[5], temp[4], temp[3], temp[2], temp[1], temp[0]);
    and inst4(AeB, ~Zero, Cout);
  and inst5(AgB, Zero, Cout);
endmodule

module PriorityEncoder_4to2bit(Datain, Dataout);
input [3:0] Datain;
output reg [1:0] Dataout;
reg temp1, temp2, temp3, temp4;
always @(*)
    begin
        {temp4, temp3, temp2, temp1} = Datain;
        if (temp4 == 1'b1)
            Dataout = 2'b11;
        else if (temp3 == 1'b1)
            Dataout = 2'b10;
        else if (temp2 == 1'b1)
            Dataout = 2'b01;
        else if (temp1 == 1'b1)
            Dataout = 2'b00;
        else
            Dataout = 2'b00;
    end
endmodule

module onebitRegwithLoad(clk, Reset, load, Datain, Dataout);
    input clk, Reset, load;
    input Datain; 
    output reg Dataout;
    always @(posedge clk) begin
        if (Reset == 1'b1)
            Dataout <= 1'b0;
        else if (load == 1'b1)
            Dataout <= Datain;
    end
endmodule

module eightbitRegwithLoad(clk, Reset, load, Datain, Dataout);
    input clk, Reset, load;
    input [7:0] Datain; 
    output reg [7:0] Dataout;

    always @(posedge clk) begin
        if (Reset == 1'b1)
            Dataout <= 8'b0000_0000;
        else if (load == 1'b1)
            Dataout <= Datain;
    end
endmodule
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module RegisterFile(clk, Reset, RegFileRead, RegFileWrite, Datain, AddressR1, AddressR2, AddressW, Dataout1, Dataout2);

    input clk, Reset, RegFileRead, RegFileWrite;
    input [7:0] Datain;
    input [3:0] AddressR1, AddressR2, AddressW;
    output wire [7:0] Dataout1, Dataout2;
    wire [15:0] dums;
    wire [7:0] read_data_1, read_data_2, I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15;

    decoder4to16_withE inst1(AddressW, RegFileWrite, dums);
    eightbitRegwithLoad inst2(clk, Reset, dums[0], Datain, I0);
    eightbitRegwithLoad inst3(clk, Reset, dums[1], Datain, I1);
    eightbitRegwithLoad inst4(clk, Reset, dums[2], Datain, I2);
    eightbitRegwithLoad inst5(clk, Reset, dums[3], Datain, I3);
    eightbitRegwithLoad inst6(clk, Reset, dums[4], Datain, I4);
    eightbitRegwithLoad inst7(clk, Reset, dums[5], Datain, I5);
    eightbitRegwithLoad inst8(clk, Reset, dums[6], Datain, I6);
    eightbitRegwithLoad inst9(clk, Reset, dums[7], Datain, I7);
    eightbitRegwithLoad inst10(clk, Reset, dums[8], Datain, I8);
    eightbitRegwithLoad inst11(clk, Reset, dums[9], Datain, I9);
    eightbitRegwithLoad inst12(clk, Reset, dums[10], Datain, I10);
    eightbitRegwithLoad inst13(clk, Reset, dums[11], Datain, I11);
    eightbitRegwithLoad inst14(clk, Reset, dums[12], Datain, I12);
    eightbitRegwithLoad inst15(clk, Reset, dums[13], Datain, I13);
    eightbitRegwithLoad inst16(clk, Reset, dums[14], Datain, I14);
    eightbitRegwithLoad inst17(clk, Reset, dums[15], Datain, I15);
    Mux16to1_8bit_withoutE inst18(AddressR1, I15, I14, I13, I12, I11, I10, I9, I8, I7, I6, I5, I4, I3, I2, I1, I0, read_data_1);
    Mux16to1_8bit_withoutE inst19(AddressR2, I15, I14, I13, I12, I11, I10, I9, I8, I7, I6, I5, I4, I3, I2, I1, I0, read_data_2);
    eightbitRegwithLoad inst20(clk, Reset, RegFileRead, read_data_1, Dataout1);
    eightbitRegwithLoad inst21(clk, Reset, RegFileRead, read_data_2, Dataout2);
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module InstMEM(clk, Reset, Address, InstRead, Dataout, OPCODE, AddressW, AddressR1, AddressR2, Imm);
    input clk, Reset, InstRead;
    input [7:0] Address;
    output reg [3:0] AddressW, AddressR1, AddressR2;
    output reg [4:0] OPCODE;
    output reg [7:0] Imm;
    output reg [24:0] Dataout;
    reg [24:0] mem [0:255];
    integer i;
    initial begin
        for (i = 0; i < 256; i++) begin
            mem[i] = 25'b0;
        end
        $readmemb("memfile.txt", mem);
        for (i = 0; i < 256; i++) begin
            if (mem[i])
                $display("line %d: %b", i,mem[i]);
        end
    end

    always @(posedge clk) begin
        if (InstRead) begin
            if (!Dataout & !Reset) $finish;
            Dataout <= mem[Address];
            OPCODE <= mem[Address][24:20];
            AddressW <= mem[Address][19:16];
            AddressR1 <= mem[Address][15:12];
            AddressR2 <= mem[Address][11:8];
            Imm <= mem[Address][7:0];
        end
    end
endmodule
/////////////////////////////////////////////////////////////////////////////
module SRAM(clk, Reset, Address, SRAMRead, SRAMWrite, Datain, Dataout);
    input clk, Reset, SRAMRead, SRAMWrite;
    input [7:0] Address, Datain;
    output reg [7:0] Dataout;
    reg [7:0] datamem [0:255];
    integer i;
    always @(posedge clk) begin
        if (Reset) begin
            for (i = 0; i < 256; i = i + 1)
                datamem[i] <= 8'b00000000; 
        end 
        else if (SRAMWrite) begin
            datamem[Address] <= Datain; 
        end
        else if(SRAMRead)begin
            Dataout <= datamem[Address];
        end
    end
endmodule
////////////////////////////////////////////////////////////////////////
module Stack(clk, Reset, StackRead, StackWrite, Datain, Dataout);
    input clk, Reset, StackRead, StackWrite;
    input [7:0] Datain;
    output wire [7:0] Dataout;
    wire [7:0] SP_minus1, SP_plus1, mux1_out, mux2_out;
    assign SP_plus1 = (Reset == 1'b1 || SP_minus1 == 8'b0000_0000) ? 8'b0000_0000 : SP_minus1 - 8'b0000_0001;
    Mux4to1_8bit_withoutE inst1({ StackRead, StackWrite }, SP_minus1, SP_plus1, SP_minus1 + 8'b0000_0001, SP_minus1, mux1_out);
    eightbitRegwithLoad inst2(clk, Reset, StackRead | StackWrite, mux1_out, SP_minus1);
    Mux4to1_8bit_withoutE inst3({ StackRead, StackWrite }, 8'b0, SP_plus1, SP_minus1, 8'b0, mux2_out);
    SRAM inst4(clk, Reset, mux2_out, StackRead, StackWrite, Datain, Dataout);
endmodule
module and_8bit (
    input  [7:0] a,  
    input  [7:0] b,  
    output [7:0] y   
);
    assign y = a & b; 
endmodule

module or_8bit (
    input  [7:0] a,
    input  [7:0] b,
    output [7:0] y
);
    assign y = a | b; 
endmodule

module xor_8bit (
    input  [7:0] a,
    input  [7:0] b,
    output [7:0] y
);
    assign y = a ^ b; 
endmodule
module left_shift_8bit(A, B, Y);
    input [7:0] A;      
    input [7:0] B;      
    output wire [7:0] Y;
    assign Y = A << B[2:0]; 
endmodule

module right_shift_8bit(A, B, Y);
    input [7:0] A;
    input[7:0]B;
    output wire [7:0] Y;
    assign Y = A >> B[2:0];
endmodule

module ALU_top1(clk,Reset,Imm7,Operand1, Operand2,OPCODE,ALUSave,ZflagSave, CflagSave, Zflag,Cflag,ALU_OUT);
    input clk, Reset, CflagSave, ZflagSave, ALUSave;
    wire Mux_Out_AS, Cout_1, Overflow, GoingtoZflagReg, MUX_GOING_TO_CARRY_FLAG_FF;
    input[7:0] Operand1, Operand2;
    input Imm7;
    wire [7:0] Sum_AS_1, ANDAnswer, ORAnswer, ExORAnswer, LShiftAnswer, RShiftAnswer, ALU_OUT_THR_FF;
    input[4:0] OPCODE;
    output wire Cflag, Zflag;
    output wire [7:0] ALU_OUT;
    wire [7:0] ALUOut_D1;

    Mux32to1_1bit inst1({3'b0, Imm7, Imm7, 2'b0, 1'b1, 24'b0},OPCODE,Mux_Out_AS,1'b1);
    Adder_Subtractor inst2(Operand1, Operand2, Mux_Out_AS, Sum_AS_1, Cout_1, Overflow);
    and_8bit inst3(Operand1, Operand2,ANDAnswer);
    or_8bit inst4( Operand1, Operand2,ORAnswer);
    xor_8bit inst5( Operand1, Operand2,ExORAnswer);

    left_shift_8bit inst6(Operand1, Operand2, LShiftAnswer);
    right_shift_8bit inst7(Operand1,Operand2, RShiftAnswer);

    Mux32to1_8bit inst8(8'b00000000, 8'b00000000, 8'b00000000,Sum_AS_1, Sum_AS_1,LShiftAnswer, RShiftAnswer,Sum_AS_1, Sum_AS_1, 8'b00000000,Sum_AS_1, 8'b00000000, 8'b00000000,Sum_AS_1, 8'b00000000, 8'b00000000, 8'b00000000, 8'b00000000,8'b00000000, Sum_AS_1,8'b00000000, Sum_AS_1, Sum_AS_1, Sum_AS_1,ExORAnswer, ORAnswer, ANDAnswer, Sum_AS_1,ExORAnswer, ORAnswer, ANDAnswer, Sum_AS_1, OPCODE, ALU_OUT_THR_FF);
    eightbitRegwithLoad inst10(clk,  Reset, 1'b1, ALU_OUT, ALUOut_D1);
    eightbitRegwithLoad inst9(clk, Reset, ALUSave,  ALU_OUT_THR_FF, ALU_OUT);
   
    assign GoingtoZflagReg = ~| ALU_OUT_THR_FF;

    onebitRegwithLoad inst12(clk, Reset,ZflagSave,  GoingtoZflagReg, Zflag);

    Mux32to1_1bit_withoutE inst13({1'b0,1'b0,1'b0,1'b0,1'b0,Operand1[7],Operand1[0],Cout_1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,Cout_1,1'b0,1'b0,1'b0,Cout_1,1'b0,1'b0,1'b0,1'b0},OPCODE,MUX_GOING_TO_CARRY_FLAG_FF);

    onebitRegwithLoad inst14(clk,Reset, CflagSave,  MUX_GOING_TO_CARRY_FLAG_FF, Cflag);
endmodule

module TimingGen(clk,Reset,T0, T1, T2, T3, T4);
    input clk;
    input Reset;
    output reg T0, T1, T2, T3, T4;
    reg [2:0] temp;

    always @(posedge clk or posedge Reset) begin
        if (Reset)
            temp <= 3'b000;
        else
            temp <= temp + 1;
    end

    always @(*) begin
        T0 = (temp == 3'b000);
        T1 = (temp == 3'b001);
        T2 = (temp == 3'b010);
        T3 = (temp == 3'b011);
        T4 = (temp == 3'b100);
    end
endmodule

module Program_Counter(clk, Reset, Enable_PC, PCUpdate, New_Address, PC, PC_D2);
    input clk, Reset, Enable_PC, PCUpdate;
    input [7:0] New_Address;
    output reg [7:0] PC, PC_D2;
    reg [7:0] PC_D1;
    reg Reset_Just_Done;
    always @(posedge clk) begin
        PC_D2 <= PC_D1;
        PC_D1 <= PC;
        if (Reset) begin
            PC <= 8'b0;
            Reset_Just_Done <= 1'b1;
        end
        else if (Reset_Just_Done) begin
            Reset_Just_Done <= 1'b0;
        end
        else if (PCUpdate) begin
            PC <= New_Address;
        end
        else if (Enable_PC) begin
            PC <= PC + 8'b0000_0001;
        end
    end
endmodule

module In_port(clk,Reset,INportRead,InpExtWorld1,InpExtWorld2,InpExtWorld3,InpExtWorld4,Address,Dataout);
  input clk, Reset, INportRead;
  input[7:0] InpExtWorld1, InpExtWorld2, InpExtWorld3, InpExtWorld4, Address;
  output wire [7:0] Dataout;
  wire [7:0] ext_1_wire, ext_2_wire, ext_3_wire, ext_4_wire, selected_wire;
  wire comp_wire_1, comp_wire_2, comp_wire_3, comp_wire_4, dummy1, dummy2, dummy3, dummy4, dummy5, dummy6, dummy7, dummy8;
  wire[1:0] selectlinemaijaanivali;

  Comparator_Unsigned_8bit inst1(Address,8'b11110001,dummy1, dummy2, comp_wire_1);
  Comparator_Unsigned_8bit inst2(Address,8'b11110010,dummy3, dummy4, comp_wire_2);
  Comparator_Unsigned_8bit inst3(Address,8'b11110011,dummy5, dummy6, comp_wire_3);
  Comparator_Unsigned_8bit inst4(Address,8'b11110100,dummy7, dummy8, comp_wire_4);

  eightbitRegwithLoad inst5(clk,  Reset, 1'b1, InpExtWorld1, ext_1_wire);
  eightbitRegwithLoad inst6(clk,  Reset, 1'b1, InpExtWorld2, ext_2_wire);
  eightbitRegwithLoad inst7(clk,  Reset, 1'b1, InpExtWorld3, ext_3_wire);
  eightbitRegwithLoad inst8(clk,  Reset, 1'b1, InpExtWorld4, ext_4_wire);

  PriorityEncoder_4to2bit inst9({comp_wire_4, comp_wire_3, comp_wire_2, comp_wire_1}, selectlinemaijaanivali);

  Mux4to1_8bit_withoutE inst10(selectlinemaijaanivali,ext_4_wire, ext_3_wire, ext_2_wire, ext_1_wire,  selected_wire );

  eightbitRegwithLoad inst11(clk, Reset, INportRead,selected_wire, Dataout);
  
endmodule

module OUT_port(clk, Reset, Address, Datain, OUTportWrite, OutExtWorld1, OutExtWorld2, OutExtWorld3, OutExtWorld4);

    input clk, Reset, OUTportWrite;
    input [7:0] Address, Datain;
    output wire [7:0] OutExtWorld1, OutExtWorld2, OutExtWorld3, OutExtWorld4;
    wire comp1, compp2, comp3, comp4, temp;
    Comparator_Unsigned_8bit inst1(Address, 8'b1111_1000, temp, temp, comp1);
    Comparator_Unsigned_8bit inst2(Address, 8'b1111_1001, temp, temp, comp2);
    Comparator_Unsigned_8bit inst3(Address, 8'b1111_1010, temp, temp, comp3);
    Comparator_Unsigned_8bit inst4(Address, 8'b1111_1011, temp, temp, comp4);
    eightbitRegwithLoad inst5(clk, Reset, OUTportWrite & comp1, Datain, OutExtWorld1);
    eightbitRegwithLoad inst6(clk, Reset, OUTportWrite & comp2, Datain, OutExtWorld2);
    eightbitRegwithLoad inst7(clk, Reset, OUTportWrite & comp3, Datain, OutExtWorld3);
    eightbitRegwithLoad inst8(clk, Reset, OUTportWrite & comp4, Datain, OutExtWorld4);
endmodule

module ControlLogic(clk, Reset, T1, T2, T3, T4, Zflag, Cflag, OPCODE, PCUpdate, SRAMRead, SRAMWrite, StackRead, StackWrite, ALUSave, ZflagSave, CflagSave, INportRead, OUTportWrite, RegFileRead, RegFileWrite);
    input clk, Reset, T1, T2, T3, T4, Zflag, Cflag;
    input [4:0] OPCODE;
    output wire PCUpdate, SRAMRead, SRAMWrite, StackRead, StackWrite, ALUSave, ZflagSave, CflagSave, INportRead, OUTportWrite, RegFileRead, RegFileWrite;

    Mux32to1_1bit inst1(32'h00801000, OPCODE,SRAMWrite, T3);
    Mux32to1_1bit inst3(32'h1fa417fe,OPCODE, ALUSave,T2);
    Mux32to1_1bit inst2(32'h00400800,OPCODE, SRAMRead, T3);
    Mux32to1_1bit inst4(32'h070001fe, OPCODE, ZflagSave, T2);
    Mux32to1_1bit inst5(32'h07e413fe,OPCODE, RegFileRead, T1);
    Mux32to1_1bit inst6(32'h07000110, OPCODE,CflagSave, T1);
    Mux32to1_1bit inst7(32'h07580ffe,OPCODE, RegFileWrite, T4);
    Mux32to1_1bit inst8(32'h00040000,OPCODE, StackWrite, T3);
    Mux32to1_1bit inst9(32'h00200000, OPCODE,OUTportWrite, T3);
    Mux32to1_1bit inst10(32'h00080000,OPCODE, StackRead, T3);
    Mux32to1_1bit inst11(32'h00100000, OPCODE,INportRead, T3);
    Mux32to1_1bit inst12({3'b0,~Zflag,Zflag,9'b0,~Cflag,Cflag,~Zflag,Zflag,1'b1,13'b0},OPCODE,PCUpdate,T4);
endmodule
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

