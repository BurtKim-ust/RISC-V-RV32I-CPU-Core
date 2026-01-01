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