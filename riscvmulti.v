`timescale 1ns/1ps

//`include "../instr_data_mem.v"
//`include "../datapath.v"
//`include "../controller.v"


module riscvmulti(
    input        clk,
    input        reset,
    output       MemWrite,
    output [31:0] Adr,
    output [31:0] WriteData,
    input  [31:0] ReadData
);

    // Internal control/status signals
    wire        RegWrite;
    wire [1:0]  ResultSrc;
    wire [1:0]  ImmSrc;
    wire [2:0]  ALUControl;
    wire        PCWrite;
    wire        IRWrite;
    wire [1:0]  ALUSrcA;
    wire [1:0]  ALUSrcB;
    wire        AdrSrc;
    wire        Zero;
    wire [6:0]  op;
    wire [2:0]  funct3;
    wire        funct7b5;

    // Controller FSM
    controller top_fsm(
        .clk(clk),
        .reset(reset),
        .op(op),
        .funct3(funct3),
        .funct7b5(funct7b5),
        .Zero(Zero),
        .ImmSrc(ImmSrc),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .ResultSrc(ResultSrc),
        .AdrSrc(AdrSrc),
        .ALUControl(ALUControl),
        .IRWrite(IRWrite),
        .PCWrite(PCWrite),
        .RegWrite(RegWrite),
        .MemWrite(MemWrite)
    );

    // Datapath
    datapath dp(
        .clk(clk),
        .reset(reset),
        .ImmSrc(ImmSrc),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .ResultSrc(ResultSrc),
        .AdrSrc(AdrSrc),
        .IRWrite(IRWrite),
        .PCWrite(PCWrite),
        .RegWrite(RegWrite),
        .MemWrite(MemWrite),
        .ALUControl(ALUControl),
        .op(op),
        .funct3(funct3),
        .funct7b5(funct7b5),
        .Zero(Zero),
        .Adr(Adr),
        .ReadData(ReadData),
        .WriteData(WriteData)
    );

endmodule


module datapath(clk, reset, ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, IRWrite, PCWrite, RegWrite, MemWrite, ALUControl, op, funct3, funct7b5, Zero, Adr, ReadData, WriteData); //start1

input clk, reset;
input PCWrite, AdrSrc, MemWrite, IRWrite, RegWrite;
input [1:0] ImmSrc, ALUSrcA, ALUSrcB, ResultSrc;
input [2:0] ALUControl;

output  funct7b5, Zero;
output [2:0] funct3;
output [6:0] op;
output [31:0] Adr, ReadData, WriteData;
reg [31:0] Result, SrcA, SrcB;
wire [31:0] instr, pcNext, pc, oldpc, rd1, rd2, a, data, ImmExt, ALUResult, ALUout;
wire [31:0] pc_plus_4; 

ff_reset_en read_instr(.clk(clk), .reset(reset), .en(IRWrite), .D(ReadData), .Q(instr));
extend extend1(.instr(instr[31:7]), .immsrc(ImmSrc), .immext(ImmExt));
regfile reg1(.clk(clk), .we3(RegWrite), .a1(instr[19:15]), .a2(instr[24:20]), .a3(instr[11:7]), .wd3(Result), .rd1(rd1), .rd2(rd2));
ff_reset_nen rd1_alu(.clk(clk), .reset(reset), .D(rd1), .Q(a));
alu true_alu(.A(SrcA), .B(SrcB), .ALUControl(ALUControl), .ALUOut(ALUResult), .Z(Zero));
ff_reset_nen result_out(.clk(clk), .reset(reset), .D(ALUResult), .Q(ALUout));
ff_reset_en pcWrite(.clk(clk), .reset(reset), .en(PCWrite), .D(pcNext), .Q(pc));
ff_reset_nen Data(.clk(clk), .reset(reset), .D(ReadData), .Q(data));
ff_reset_nen rd2_write(.clk(clk), .reset(reset), .D(rd2), .Q(WriteData));
ff_reset_en OldPC(.clk(clk), .reset(reset), .en(IRWrite), .D(pc), .Q(oldpc));

assign Adr = AdrSrc ? ALUout : pc;
assign op = instr[6:0];
assign funct3 = instr[14:12];
assign funct7b5 = instr[30];

assign pc_plus_4 = pc + 32'd4;

always @(data, ResultSrc, ALUout, pc_plus_4)
	case(ResultSrc)
		2'b00: Result = ALUout;
		2'b01: Result = data;
		2'b10: Result = pc_plus_4; 
		default: Result = ALUout;
	endcase

always @(pc, oldpc, a, ALUSrcA)
	case(ALUSrcA)
		2'b0: SrcA = pc;
		2'b01: SrcA = oldpc;
		2'b10: SrcA = a;
		default: SrcA = a;
	endcase

always @(ImmExt, WriteData, ALUSrcB)
	case(ALUSrcB)
		2'b0: SrcB = WriteData;
		2'b01: SrcB = ImmExt;
		2'b10: SrcB = 32'd4;
		default: SrcB = 32'd4;
	endcase

assign pcNext = Result;

endmodule


module controller(clk, reset, op, funct3, funct7b5, Zero, ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUControl, IRWrite, PCWrite, RegWrite, MemWrite);

input wire clk, reset, funct7b5, Zero;
input wire [2:0] funct3;
input wire [6:0] op;

output PCWrite;
output reg AdrSrc, MemWrite, IRWrite, RegWrite;
output reg [1:0] ImmSrc;
output wire [1:0] ALUSrcA;
output wire [1:0] ALUSrcB; 
output wire [1:0] ResultSrc; 
output reg [2:0] ALUControl;

reg PCUpdate, Branch;
reg [1:0] ALUOp;
reg [3:0] state, nextstate;
reg [12:0] controls;


localparam [3:0]
	fetch = 4'b0,
	decode = 4'b0001,
	memadr = 4'b0010,
	memread = 4'b0011,
	memwb = 4'b0100,
	memwrite = 4'b0101,
	executer = 4'b0110,
	aluwb = 4'b0111,
	executei = 4'b1000,
	jal = 4'b1001,
	beq = 4'b1010;


always @(posedge clk or posedge reset)
begin
	if (reset)
		state = fetch;
	else 
		state = nextstate;
end

assign PCWrite = (Zero & Branch) | PCUpdate; 

// ============================================================================
// DIRECT STATE DECODE for critical path signals
// ResultSrc controls the path to pcWrite - must be fast
// ============================================================================
// ResultSrc[1] = 1 only in fetch (0000) -> selects ALUResult
// ResultSrc[0] = 1 only in memwb (0100) -> selects data
assign ResultSrc[1] = ~state[3] & ~state[2] & ~state[1] & ~state[0];
assign ResultSrc[0] = ~state[3] & state[2] & ~state[1] & ~state[0];

// ALUSrcB[1] = 1 in fetch (0000), jal (1001) -> selects constant 4
// ALUSrcB[0] = 1 in decode (0001), memadr (0010), executei (1000) -> selects ImmExt
assign ALUSrcB[1] = ~state[2] & ~state[1] & ~(state[3] ^ state[0]);
assign ALUSrcB[0] = ~state[2] & ((~state[3] & (state[1] ^ state[0])) | (state[3] & ~state[1] & ~state[0]));

// ALUSrcA[1] = 1 in memadr (0010), executer (0110), executei (1000), beq (1010) -> selects 'a' register
// ALUSrcA[0] = 1 in decode (0001), jal (1001) -> selects oldpc
assign ALUSrcA[1] = ~state[0] & ((~state[3] & state[1]) | (state[3] & ~state[2]));
assign ALUSrcA[0] = ~state[2] & ~state[1] & state[0]; 


always @(*)
begin
	if (reset)
	begin
		nextstate = fetch;
		PCUpdate <= 1'b0;
		Branch <= 1'b0;
		AdrSrc <= 1'b0;
		MemWrite <= 1'b0;
		IRWrite <= 1'b0;
		RegWrite <= 1'b0;
		ImmSrc = 2'b0;
		ALUOp <= 2'b0;
	end
	else
	case(state)
		fetch: begin
				nextstate = decode;
				PCUpdate <= 1'b1;
				Branch <= 1'b0;
				AdrSrc <= 1'b0;
				MemWrite <= 1'b0;
				IRWrite <= 1'b1;
				RegWrite <= 1'b0;
				ALUOp <= 2'b0;
		      end
		decode: begin
				PCUpdate <= 1'b0;
				Branch <= 1'b0;
				AdrSrc <= 1'bx;
				MemWrite <= 1'b0;
				IRWrite <= 1'b0;
				RegWrite <= 1'b0;
				ALUOp <= 2'b0;
				case(op)
					7'b0000011: begin //lw
							nextstate = memadr;
							ImmSrc = 2'b0;
						    end
					7'b0100011: begin //sw
							nextstate = memadr;
							ImmSrc = 2'b01;
						    end
					7'b0110011: begin //r-type 
							nextstate = executer;
							ImmSrc = 2'bxx;	
						    end
					7'b0010011: begin //i-type
							nextstate = executei;
							ImmSrc = 2'b0;
						    end
					7'b1101111: begin //jal
							nextstate = jal;
							ImmSrc = 2'b11;
						    end
					7'b1100011: begin //beq
							nextstate =  beq;
							ImmSrc = 2'b10;
						    end
					default: begin
							nextstate = memadr;
							ImmSrc = 2'b00;
						 end
				endcase 
			 end
		memadr: begin
				PCUpdate <= 1'b0;
                                Branch <= 1'b0;
                                AdrSrc <= 1'bx;
                                MemWrite <= 1'b0;
                                IRWrite <= 1'b0;
                                RegWrite <= 1'b0;
                                ALUOp <= 2'b0;
				case (op)
					7'b0000011: nextstate = memread;  //lw
					7'b0100011: nextstate = memwrite; //sw
				endcase
			 end
		memread: begin
				nextstate = memwb;
				PCUpdate <= 1'b0;
                                Branch <= 1'b0;
                                AdrSrc <= 1'b1;
                                MemWrite <= 1'b0;
                                IRWrite <= 1'b0;
                                RegWrite <= 1'b0;
                                ALUOp <= 2'bxx;
			 end
		memwb: begin
				nextstate = fetch;
				PCUpdate <= 1'b0;
                                Branch <= 1'b0;
                                AdrSrc <= 1'bx;
                                MemWrite <= 1'b0;
                                IRWrite <= 1'b0;
                                RegWrite <= 1'b1;
                                ALUOp <= 2'bxx;
			 end
		memwrite: begin
				nextstate = fetch;
				PCUpdate <= 1'b0;
                                Branch <= 1'b0;
                                AdrSrc <= 1'b1;
                                MemWrite <= 1'b1;
                                IRWrite <= 1'b0;
                                RegWrite <= 1'b0;
                                ALUOp <= 2'bxx;
			 end
		executer: begin
				nextstate = aluwb;
				PCUpdate <= 1'b0;
                                Branch <= 1'b0;
                                AdrSrc <= 1'bx;
                                MemWrite <= 1'b0;
                                IRWrite <= 1'b0;
                                RegWrite <= 1'b0;
                                ALUOp <= 2'b10;
			 end
		aluwb: begin
				nextstate = fetch;
				PCUpdate <= 1'b0;
                                Branch <= 1'b0;
                                AdrSrc <= 1'bx;
                                MemWrite <= 1'b0;
                                IRWrite <= 1'b0;
                                RegWrite <= 1'b1;
                                ALUOp <= 2'bxx;
			 end
		executei: begin
				nextstate = aluwb;
				PCUpdate <= 1'b0;
                                Branch <= 1'b0;
                                AdrSrc <= 1'bx;
                                MemWrite <= 1'b0;
                                IRWrite <= 1'b0;
                                RegWrite <= 1'b0;
                                ALUOp <= 2'b10;
			 end
		jal: begin
				nextstate = aluwb;
				PCUpdate <= 1'b1;
                                Branch <= 1'b0;
                                AdrSrc <= 1'bx;
                                MemWrite <= 1'b0;
                                IRWrite <= 1'b0;
                                RegWrite <= 1'b0;
                                ALUOp <= 2'b0;
			 end
		beq: begin
				nextstate = fetch;
				PCUpdate <= 1'b0;
                                Branch <= 1'b1;
                                AdrSrc <= 1'bx;
                                MemWrite <= 1'b0;
                                IRWrite <= 1'b0;
                                RegWrite <= 1'b0;
                                ALUOp <= 2'b01;
			 end
	endcase
end


always @(*)
begin
	case(ALUOp)
		2'b0: ALUControl <= 3'b0;
		2'b01: ALUControl <= 3'b001;
		default: case(funct3)
				3'b0: if (funct7b5 &  op[5])
						ALUControl <= 3'b001;
				      else
						ALUControl <= 3'b0;
				3'b010: ALUControl <= 3'b101;
				3'b110: ALUControl <= 3'b011;
				3'b111: ALUControl <= 3'b010;
				default: ALUControl <= 3'bxxx;
			  endcase
	endcase
end

endmodule



module ff_reset_en(clk, reset, en, D, Q);

input wire clk, reset, en;
input wire [31:0] D;
output reg [31:0] Q;


always @(posedge clk)
begin
	if (reset)
	begin
		Q <= 32'b0;
	end else 
	if (en)
	begin
		Q <= D;
	end
end
endmodule


module ff_reset_nen(clk, reset, D, Q);

input wire clk, reset;
input wire [31:0] D;
output reg [31:0] Q;


always @(posedge clk)
begin
	if(reset)
	begin
		Q <= 32'b0;
	end else
	begin
		Q <= D;
	end
end
endmodule


module alu (A, B, ALUControl, ALUOut, Z);

input wire [2:0] ALUControl;
input wire [31:0] A, B;
output wire Z;
output reg [31:0] ALUOut;

reg [31:0] temp_slt;
reg Cout;
reg [32:0] temp_result;
wire V;

always @(*)
	case(ALUControl)
		3'b0: {Cout, ALUOut} = {1'b0,A} + {1'b0,B} + {32'b0, ALUControl[0]};
		3'b001: begin
				temp_result = {1'b0,A} + {1'b0,~B} + {32'b0, 1'b1};
				{Cout, ALUOut} = temp_result;
			end
		3'b010: ALUOut = A & B;
		3'b011: ALUOut = A | B;
		3'b101: begin
			Cout = 1'b0;
			ALUOut[31:1] = 31'b0;
			if (A[31] ^ B[31])
			begin
				temp_slt = A - B;
				ALUOut[0] = A[31];
			end else
			begin
				temp_slt = A - B;
				ALUOut [0] = V ^ temp_slt[31];
			end
			end
		default: ALUOut = 32'b0;
	endcase

assign Z = (A == B);

assign V = (ALUControl == 3'b101) & (~ALUControl[1] & (A[31] ^ temp_slt[31]) & ~(A[31] ^ B[31] ^ ALUControl[0])) | (ALUControl != 3'b101) & (~ALUControl[1]) & ((A[31] ^ ALUOut[31] & ~(ALUControl[0] ^ A[31] ^ B[31])));

endmodule
