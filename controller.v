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