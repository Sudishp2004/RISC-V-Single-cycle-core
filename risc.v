`timescale 1ns/1ps

// ===============================
// Program Counter
// ===============================
module Program_Counter(input clk, reset,
                       input [31:0] PC_in,
                       output reg [31:0] PC_out);
always @(posedge clk or posedge reset)
begin
    if (reset) PC_out <= 0;
    else PC_out <= PC_in;
end
endmodule

// ===============================
// PC + 4
// ===============================
module PCplus4(input [31:0] PC, output [31:0] PC4);
assign PC4 = PC + 4;
endmodule

// ===============================
// Instruction Memory (WORD indexed)
// ===============================
module Instruction_Mem(input [31:0] addr,
                       output [31:0] instr);

reg [31:0] mem[0:63];

assign instr = mem[addr[7:2]];

initial begin
    mem[0]  = 32'h00000013;
    mem[1]  = 32'b0000000_11001_10000_000_01101_0110011;
    mem[2]  = 32'b0100000_00011_01000_000_00101_0110011;
    mem[3]  = 32'b0000000_00011_00011_111_00001_0110011;
    mem[4]  = 32'b0000000_00101_00011_110_00100_0110011;
    mem[5]  = 32'b000000000011_10101_000_10110_0010011;
    mem[6]  = 32'b000000000001_01000_110_01001_0010011;
    mem[7]  = 32'b000000001111_00101_010_01000_0000011;
    mem[8]  = 32'b000000000011_00011_010_01001_0000011;
    mem[9]  = 32'b0000000_01111_00101_010_01100_0100011;
    mem[10] = 32'b0000000_01110_00110_010_01010_0100011;
    mem[11] = 32'h00948663;
end
endmodule

// ===============================
// Register File
// ===============================
module Reg_File(input clk, reset, RegWrite,
                input [4:0] rs1, rs2, rd,
                input [31:0] wd,
                output [31:0] rd1, rd2);

reg [31:0] R[0:31];
integer i;

always @(posedge clk or posedge reset)
begin
    if (reset)
        for (i=0;i<32;i=i+1) R[i] <= 0;
    else if (RegWrite && rd!=0)
        R[rd] <= wd;
end

assign rd1 = R[rs1];
assign rd2 = R[rs2];
endmodule

// ===============================
// Immediate Generator
// ===============================
module ImmGen(input [6:0] op,
              input [31:0] instr,
              output reg [31:0] imm);

always @(*)
begin
    case(op)
        7'b0000011: imm = {{20{instr[31]}}, instr[31:20]};
        7'b0100011: imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};
        7'b1100011: imm = {{19{instr[31]}}, instr[31], instr[30:25], instr[11:8], 1'b0};
        default: imm = 0;
    endcase
end
endmodule

// ===============================
// Control Unit
// ===============================
module Control_Unit(input [6:0] op,
    output reg Branch, MemRead, MemtoReg,
    output reg MemWrite, ALUSrc, RegWrite,
    output reg [1:0] ALUOp);

always @(*)
begin
    case(op)
        7'b0110011: begin ALUSrc=0; MemtoReg=0; RegWrite=1; MemRead=0; MemWrite=0; Branch=0; ALUOp=2'b10; end
        7'b0000011: begin ALUSrc=1; MemtoReg=1; RegWrite=1; MemRead=1; MemWrite=0; Branch=0; ALUOp=2'b00; end
        7'b0100011: begin ALUSrc=1; MemtoReg=0; RegWrite=0; MemRead=0; MemWrite=1; Branch=0; ALUOp=2'b00; end
        7'b1100011: begin ALUSrc=0; MemtoReg=0; RegWrite=0; MemRead=0; MemWrite=0; Branch=1; ALUOp=2'b01; end
        default: begin ALUSrc=0; MemtoReg=0; RegWrite=0; MemRead=0; MemWrite=0; Branch=0; ALUOp=2'b00; end
    endcase
end
endmodule

// ===============================
// ALU Control
// ===============================
module ALU_Control(input [1:0] ALUOp,
                   input fun7,
                   input [2:0] fun3,
                   output reg [3:0] ctl);

always @(*)
begin
    case({ALUOp,fun7,fun3})
        6'b10_0_000: ctl = 4'b0010;
        6'b10_1_000: ctl = 4'b0110;
        6'b10_0_111: ctl = 4'b0000;
        6'b10_0_110: ctl = 4'b0001;
        6'b01_0_000: ctl = 4'b0110;
        default: ctl = 4'b0010;
    endcase
end
endmodule

// ===============================
// ALU
// ===============================
module ALU(input [31:0] A,B,
           input [3:0] ctl,
           output reg [31:0] Y,
           output Zero);

always @(*)
begin
    case(ctl)
        4'b0000: Y = A & B;
        4'b0001: Y = A | B;
        4'b0010: Y = A + B;
        4'b0110: Y = A - B;
        default: Y = 0;
    endcase
end
assign Zero = (Y==0);
endmodule

// ===============================
// Data Memory
// ===============================
module Data_Memory(input clk,
                   input MemWrite, MemRead,
                   input [31:0] addr, wd,
                   output [31:0] rd);

reg [31:0] mem[0:63];
always @(posedge clk)
    if (MemWrite) mem[addr[7:2]] <= wd;

assign rd = MemRead ? mem[addr[7:2]] : 0;
endmodule

// ===============================
// 2:1 MUX
// ===============================
module Mux2(input sel,
            input [31:0] A,B,
            output [31:0] Y);
assign Y = sel ? B : A;
endmodule

// ===============================
// Adder
// ===============================
module Adder(input [31:0] A,B,
             output [31:0] Y);
assign Y = A + B;
endmodule

// ===============================
// TOP MODULE
// ===============================
module top(input clk, reset);

wire [31:0] PC, PC4, Instr, Rd1, Rd2, Imm, ALUin2, ALUout, MemOut, WB;
wire [31:0] PCbranch, PCnext;
wire Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, Zero;
wire [1:0] ALUOp;
wire [3:0] ALUCtl;

Program_Counter pc(clk, reset, PCnext, PC);
PCplus4 pc4(PC, PC4);
Instruction_Mem im(PC, Instr);
Reg_File rf(clk, reset, RegWrite, Instr[19:15], Instr[24:20], Instr[11:7], WB, Rd1, Rd2);
ImmGen ig(Instr[6:0], Instr, Imm);
Control_Unit cu(Instr[6:0], Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, ALUOp);
ALU_Control ac(ALUOp, Instr[30], Instr[14:12], ALUCtl);
Mux2 mux1(ALUSrc, Rd2, Imm, ALUin2);
ALU alu(Rd1, ALUin2, ALUCtl, ALUout, Zero);
Adder bradd(PC, Imm, PCbranch);
Mux2 pcmux(Branch & Zero, PC4, PCbranch, PCnext);
Data_Memory dm(clk, MemWrite, MemRead, ALUout, Rd2, MemOut);
Mux2 wb(MemtoReg, ALUout, MemOut, WB);

endmodule
