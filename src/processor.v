

module processor(



    // Control signals



    clock,                          // I: The master clock



    reset,                          // I: A reset signal







    // Imem



    address_imem,                   // O: The address of the data to get from imem



    q_imem,                         // I: The data from imem







    // Dmem



    address_dmem,                   // O: The address of the data to get or put from/to dmem



    data,                           // O: The data to write to dmem



    wren,                           // O: Write enable for dmem



    q_dmem,                         // I: The data from dmem



    // Regfile



    ctrl_writeEnable,               // O: Write enable for regfile



    ctrl_writeReg,                  // O: Register to write to in regfile



    ctrl_readRegA,                  // O: Register to read from port A of regfile



    ctrl_readRegB,                  // O: Register to read from port B of regfile



    data_writeReg,                  // O: Data to write to for regfile



    data_readRegA,                  // I: Data from port A of regfile



    data_readRegB                   // I: Data from port B of regfile



);	 

	 

	 // Control signals

    input clock, reset;



    // Imem

    output [11:0] address_imem;

    input [31:0] q_imem;



    // Dmem

    output [11:0] address_dmem;

    output [31:0] data;

    output wren;

    input [31:0] q_dmem;



    // Regfile

    output ctrl_writeEnable;

    output [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;

    output [31:0] data_writeReg;

    input [31:0] data_readRegA, data_readRegB;



    /* YOUR CODE STARTS HERE */

	 

	 //// FETCH STAGE

	 

	 // PC Counter 

	 wire neg_stall;

	 wire multdiv_stall; 

	 wire neg_multdiv_stall; 

	 assign neg_multdiv_stall = ~multdiv_stall;

	 wire pc_mux_select_bit; 

	 wire [31:0] imem_in; 

	 wire [31:0] pc_in;

	 wire [31:0] imem_in_plus_four; 

	 wire pc_adder_out;

	 

	 wire neg_stall_neg_multdiv_stall; 

	 assign neg_stall_neg_multdiv_stall = neg_multdiv_stall & neg_stall;

	 

	 register PC_Counter (.out(imem_in), .in(pc_in), .clk(clock), .en(neg_stall_neg_multdiv_stall), .reset(reset)); 

    

	 // add the current PC by 1 -> next instruction if no branch taken 

	 Adder_32_bit pc_adder (.out(imem_in_plus_four), .c_out(pc_adder_out), .data_A(imem_in), .data_B(32'd1), .c_in(1'b0)); 

    

	 // instruction memory - use the last 12 bits from PC ??? 

	 wire [31:0] instruction; 

	 

	 assign address_imem = imem_in[11:0];

	 assign instruction = q_imem; 

	 

	 // FD latch

	 wire [31:0] next_PC_D; 

	 wire [31:0] instruction_D;

	 wire [31:0] instruction_stall_checked;

	 register FD_latch (.out(instruction_D), .in(instruction_stall_checked), .clk(clock), .en(neg_stall_neg_multdiv_stall), .reset(reset));

	 register FD_next_PC_latch (.out(next_PC_D), .in(imem_in_plus_four), .clk(clock), .en(neg_stall_neg_multdiv_stall), .reset(reset)); 

	 

	 

	 

	 //// DECODE STAGE

	 

	 wire [4:0] opcode_D, rd_D, rs_D, rt_D; 

	 assign opcode_D = instruction_D[31:27]; 

	 assign rd_D = instruction_D[26:22]; 

	 assign ctrl_readRegA = instruction_D[21:17]; 

	 assign rt_D = instruction_D[16:12];

	 assign rs_D = instruction_D[21:17];

	 // register file  

	 wire rd_as_rt; 

	 // put in rd instead of rt for sw, branches and jumps

	 

	 wire [4:0] select_rdst_out; 

	 Mux_5_input select_rdst (.out(select_rdst_out), .a(rt_D), .b(rd_D), .in(rd_as_rt)); 

	 

	 wire bex; 

	 assign bex = (opcode_D[4] & ~opcode_D[3] & opcode_D[2] & opcode_D[1] & ~opcode_D[0]); // bex 10110 

	 

	 Mux_5_input select_bex (.out(ctrl_readRegB), .a(select_rdst_out), .b(5'd30), .in(bex)); 

	 

	 assign rd_as_rt = (~opcode_D[4] & ~opcode_D[3] & opcode_D[2] & opcode_D[1] & opcode_D[0]) // sw 00111

	                  |(~opcode_D[4] & ~opcode_D[3] & ~opcode_D[2] & opcode_D[1] & ~opcode_D[0]) // bne 00010 

							|(~opcode_D[4] & ~opcode_D[3] & opcode_D[2] & ~opcode_D[1] & ~opcode_D[0]) // jr 00100

							|(~opcode_D[4] & ~opcode_D[3] & opcode_D[2] & opcode_D[1] & ~opcode_D[0]); // blt 00110 

	 

	 

	 // DX latch

	 wire [31:0] instruction_D_stall_checked; 

	 wire [31:0] next_PC_X; 

	 wire [31:0] instruction_X; 

	 wire [31:0] ReadReg_DataA_X, ReadReg_DataB_X; 

	 register DX_latch (.out(instruction_X), .in(instruction_D_stall_checked), .clk(clock), .en(neg_multdiv_stall), .reset(reset));

	 register DX_next_PC_latch (.out(next_PC_X), .in(next_PC_D), .clk(clock), .en(neg_multdiv_stall), .reset(reset)); 

	 register DX_Data_A (.out(ReadReg_DataA_X), .in(data_readRegA), .clk(clock), .en(neg_multdiv_stall), .reset(reset));

	 register DX_Data_B (.out(ReadReg_DataB_X), .in(data_readRegB), .clk(clock), .en(neg_multdiv_stall), .reset(reset));

	 

	 //// EXECUTE STAGE 

	 wire signed [16:0] immediate_unextended; 

	 wire signed [31:0] immediate_extended; 

	 wire [4:0] aluOP, shamt; 

	 wire [31:0] alu_output; 

	 wire [31:0] alu_output_ALU;

	 wire alu_in_B; // imm -> is B or immeidate going into the ALU? 

	 wire [4:0] opcode_X; 

	 

	 assign opcode_X = instruction_X[31:27]; 

	 

	 wire alu_isNotEqual, alu_isLessThan, alu_overflow;

	 assign shamt = instruction_X[11:7];

	 

	 wire isAddi_LW_SW;

	 assign isAddi_LW_SW = (~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & ~opcode_X[1] & opcode_X[0]) // addi 00101 

								 |(~opcode_X[4] & opcode_X[3] & ~opcode_X[2] & ~opcode_X[1] & ~opcode_X[0]) // lw 01000

								 |(~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & opcode_X[0]); // sw 00111

	 

	 wire [4:0] mux_aluOP_addi_out; 

	 Mux_5_input mux_aluOP_addi (.out(mux_aluOP_addi_out), .a(instruction_X[6:2]), .b(5'b00000), .in(isAddi_LW_SW)); 

	 

	 wire isBranch; 

	 assign isBranch = (~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & opcode_X[1] & ~opcode_X[0]) // bne 00010 

							|(~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & ~opcode_X[0]); // blt 00110

	 Mux_5_input mux_aluOP_branch (.out(aluOP), .a(mux_aluOP_addi_out), .b(5'b00001), .in(isBranch)); 

	 //assign aluOP = instruction_X[6:2]; 

	 assign immediate_unextended = instruction_X[16:0]; 

	 

	 assign immediate_extended = {{15{immediate_unextended[16]}}, immediate_unextended}; 

	 

	 wire [31:0] alu_input_B; 

	 wire [31:0] alu_input_A; 

	 

	 wire alu_overflow_ALU;

	 

	 alu alu_X (

		.data_operandA(alu_input_A), 

		.data_operandB(alu_input_B), 

		.ctrl_ALUopcode(aluOP), 

		.ctrl_shiftamt(shamt), 

		.data_result(alu_output_ALU), 

		.isNotEqual(alu_isNotEqual), 

		.isLessThan(alu_isLessThan), 

		.overflow(alu_overflow_ALU));

	 

	 assign alu_in_B = (~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & ~opcode_X[1] & ~opcode_X[0]) // alus without addi 

							|(~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & opcode_X[1] & ~opcode_X[0]) // bne 00010 

							|(~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & ~opcode_X[0]); // blt 00110 

	 

	 wire [31:0] bypass_ReadReg_DataB_X; 

	 wire BNotZero; 

	 assign BNotZero = (bypass_ReadReg_DataB_X[31] 

							 |bypass_ReadReg_DataB_X[30]

							 |bypass_ReadReg_DataB_X[29] 

							 |bypass_ReadReg_DataB_X[28]

						    |bypass_ReadReg_DataB_X[27]

							 |bypass_ReadReg_DataB_X[26] 

							 |bypass_ReadReg_DataB_X[25]

							 |bypass_ReadReg_DataB_X[24]

							 |bypass_ReadReg_DataB_X[23] 

							 |bypass_ReadReg_DataB_X[22]

							 |bypass_ReadReg_DataB_X[21]

							 |bypass_ReadReg_DataB_X[20] 

							 |bypass_ReadReg_DataB_X[19]

							 |bypass_ReadReg_DataB_X[18]

							 |bypass_ReadReg_DataB_X[17] 

							 |bypass_ReadReg_DataB_X[16]

							 |bypass_ReadReg_DataB_X[15]

							 |bypass_ReadReg_DataB_X[14] 

							 |bypass_ReadReg_DataB_X[13]

							 |bypass_ReadReg_DataB_X[12]

							 |bypass_ReadReg_DataB_X[11] 

							 |bypass_ReadReg_DataB_X[10]

							 |bypass_ReadReg_DataB_X[9]

							 |bypass_ReadReg_DataB_X[8] 

							 |bypass_ReadReg_DataB_X[7]

							 |bypass_ReadReg_DataB_X[6]

							 |bypass_ReadReg_DataB_X[5] 

							 |bypass_ReadReg_DataB_X[4]

							 |bypass_ReadReg_DataB_X[3]

							 |bypass_ReadReg_DataB_X[2] 

							 |bypass_ReadReg_DataB_X[1]

							 |bypass_ReadReg_DataB_X[0]);

	 wire bex_check;

	 wire bypass_rstatus_check;

	 

	 assign bex_check = BNotZero | bypass_rstatus_check;

	 

	 Mux_32_input mux_select_SX_or_B (.out(alu_input_B), .a(immediate_extended), .b(bypass_ReadReg_DataB_X), .in(alu_in_B)); 

	 

	 //wire [31:0] shifter_out; 

	 // shifter_left_32_bit pc_shifter (.out(shifter_out), .in(immediate_extended), .shamt(5'd2));

	 

	 wire [31:0] pc_adder_N_out;

	 wire pc_adder_N_c_out;

	 Adder_32_bit pc_adder_N (.out(pc_adder_N_out), .c_out(pc_adder_N_c_out), .data_A(next_PC_X), .data_B(immediate_extended), .c_in(1'b0));

	 

	 wire BNEandNotEqual; 

	 

	 assign BNEandNotEqual = (~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & opcode_X[1] & ~opcode_X[0]) // bne 00010 

									&(alu_isNotEqual); 

									

	 assign BLTandnotLessThan = (~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & ~opcode_X[0]) // blt 00110 

									&(~alu_isLessThan & alu_isNotEqual); 

	 

	 wire branch_taken; 

	 assign branch_taken = (BNEandNotEqual | BLTandnotLessThan); 

	 

	 wire [31:0] take_branch_out; 

	 Mux_32_input take_branch (.out(take_branch_out), .a(imem_in_plus_four), .b(pc_adder_N_out), .in(branch_taken)); 

	 

	 wire jump; 

	 assign jump = (~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & ~opcode_X[1] & opcode_X[0])// j 00001 

					  |(~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & opcode_X[1] & opcode_X[0]) // jal 00011 

					  |(opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & ~opcode_X[0] & bex_check); // bex 10110 

	 

	 wire [26:0] target; 

	 assign target = instruction_X[26:0]; 

	 

	 wire [31:0] target_extended; 

	 assign target_extended = {5'b0, target};

	 

	 wire [31:0] jump_mux_out; 

	 Mux_32_input jump_mux (.out(jump_mux_out), .a(take_branch_out), .b(target_extended), .in(jump)); 

	 

	 wire jump_return; 

	 assign jump_return = (~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & ~opcode_X[1] & ~opcode_X[0]); // jr 00100 

	 

	 Mux_32_input jump_return_mux (.out(pc_in), .a(jump_mux_out), .b(bypass_ReadReg_DataB_X), .in(jump_return)); 

	 

	 wire jal; 

	 assign jal = (~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & opcode_X[1] & opcode_X[0]); 

	 

	 wire [31:0] jal_store_mux_out; 

	 Mux_32_input jal_store_mux (.out(jal_store_mux_out), .a(alu_output), .b(next_PC_X), .in(jal)); 

	 

	 wire setx; 

	 assign setx = (opcode_X[4] & ~opcode_X[3] & opcode_X[2] & ~opcode_X[1] & opcode_X[0]); // setx 10101 

	 

	 wire [31:0] setx_mux_output; 

	 Mux_32_input setx_mux (.out(setx_mux_output), .a(jal_store_mux_out), .b(target_extended), .in(setx)); 

	 

	 // XM Latch

	 wire [31:0] instruction_M; 

	 wire [31:0] alu_output_M; 

	 wire [31:0] ReadReg_DataB_M; 

	 wire [31:0] instruction_X_stall_checked;

	 wire overflow_exception_M; 

	 register XM_latch (.out(instruction_M), .in(instruction_X_stall_checked), .clk(clock), .en(1'b1), .reset(reset));

	 register XM_alu_output (.out(alu_output_M), .in(setx_mux_output), .clk(clock), .en(1'b1), .reset(reset));

	 register XM_Data_B (.out(ReadReg_DataB_M), .in(bypass_ReadReg_DataB_X), .clk(clock), .en(1'b1), .reset(reset));

	 

	 // alu_overflow needs to be muxed to allow for mult, div, etc. 

	 dffe_ref XM_overflow_exception (.q(overflow_exception_M), .d(alu_overflow), .clk(clock), .en(1'b1), .clr(reset));

	 

	 //// MEMORY STAGE 

	 wire DMwe; 

	 wire [31:0] memory_out; 

	 

	 wire [4:0] opcode_M; 

	 assign opcode_M = instruction_M[31:27]; 

	 

	 // MX bypass

	 wire DX_bypass_INSN;

	 assign DX_bypass_INSN = (~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & ~opcode_X[1] & ~opcode_X[0]) // alu ops

						  |(~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & ~opcode_X[1] & opcode_X[0]) // addi 00101

						  |(~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & opcode_X[0]) // sw 00111

						  |(~opcode_X[4] & opcode_X[3] & ~opcode_X[2] & ~opcode_X[1] & ~opcode_X[0]) // lw 01000

						  |(~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & opcode_X[1] & ~opcode_X[0]) // bne 00010

						  |(~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & ~opcode_X[0]); // blt 00110

						  

	 wire XM_bypass_INSN;

	 assign XM_bypass_INSN = (~opcode_M[4] & ~opcode_M[3] & ~opcode_M[2] & ~opcode_M[1] & ~opcode_M[0]) // alu ops

								 |(~opcode_M[4] & ~opcode_M[3] & opcode_M[2] & ~opcode_M[1] & opcode_M[0]) // addi 

								 |(~opcode_M[4] & opcode_M[3] & ~opcode_M[2] & ~opcode_M[1] & ~opcode_M[0]); // lw

								 

	 

	 wire [4:0] rd_M; 

	 assign rd_M = instruction_M[26:22]; 

	 

	 wire [4:0] rs_X; 

	 assign rs_X = instruction_X[21:17]; 

	 

	 wire [4:0] rd_X; 

	 assign rd_X = instruction_X[26:22]; 

	 

	 wire [4:0] rt_X; 

	 assign rt_X = instruction_X[16:12];

	 

	 

	 wire [4:0] rd_W; 

	 wire MX_out_to_A; 

	 assign MX_out_to_A = ((DX_bypass_INSN & XM_bypass_INSN 

								                 &(((rd_M[4] == rs_X[4])

													  & (rd_M[3] == rs_X[3])

													  & (rd_M[2] == rs_X[2])

													  & (rd_M[1] == rs_X[1])

													  & (rd_M[0] == rs_X[0])) & ~(~rd_M[4] & ~rd_M[3] & ~rd_M[2] & ~rd_M[1] & ~rd_M[0])))); 

	 

	 wire MX_out_to_B; 

	 wire [4:0] jr_rd;

	 assign jr_rd = instruction_X[26:22];

	 wire bbjs;

	 assign bbjs = ((~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & opcode_X[1] & ~opcode_X[0]) // bne 00010

									   |(~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & ~opcode_X[0]) // blt

										|(~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & ~opcode_X[1] & ~opcode_X[0]) // jr

										|(~opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & opcode_X[0])); // sw

										

	 assign MX_out_to_B = ( ((DX_bypass_INSN & ~bbjs) & XM_bypass_INSN 

													  &(((rd_M[4] == rt_X[4])

													  & (rd_M[3] == rt_X[3])

													  & (rd_M[2] == rt_X[2])

													  & (rd_M[1] == rt_X[1])

													  & (rd_M[0] == rt_X[0])) & ~(~rd_M[4] & ~rd_M[3] & ~rd_M[2] & ~rd_M[1] & ~rd_M[0])))

								 |( (opcode_M[4] & ~opcode_M[3] & opcode_M[2] & ~opcode_M[1] & opcode_M[0]) 

									 &((opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & ~opcode_X[0]))) // setx followed by bex

								 |( bbjs & XM_bypass_INSN

									&( ((rd_X[4] == rd_M[4])

									  &(rd_X[3] == rd_M[3])

									  &(rd_X[2] == rd_M[2])

									  &(rd_X[1] == rd_M[1])

									  &(rd_X[0] == rd_M[0]) & ~(~rd_M[4] & ~rd_M[3] & ~rd_M[2] & ~rd_M[1] & ~rd_M[0])))));

									

	 

	 

	 

	 assign address_dmem = alu_output_M[11:0]; 

	 assign memory_out = q_dmem; 

	 assign wren = DMwe; 

	 

	 

	 assign DMwe = (~opcode_M[4] & ~opcode_M[3] & opcode_M[2] & opcode_M[1] & opcode_M[0]); 

	 

	 // MW latch 

	 wire [31:0] memory_out_W;

	 wire [31:0] instruction_W; 

	 wire [31:0] alu_output_W; 

	 register MW_latch (.out(instruction_W), .in(instruction_M), .clk(clock), .en(1'b1), .reset(reset));

	 register MW_memory_output (.out(memory_out_W), .in(memory_out), .clk(clock), .en(1'b1), .reset(reset));

	 register MW_alu_output (.out(alu_output_W), .in(alu_output_M), .clk(clock), .en(1'b1), .reset(reset));

	 // alu_overflow needs to be muxed to allow for mult, div, etc. 

	 wire overflow_exception_W; 

	 dffe_ref MW_overflow_exception (.q(overflow_exception_W), .d(overflow_exception_M), .clk(clock), .en(1'b1), .clr(reset)); 

	 

	 //// WRITE STAGE

	 wire load; 

	 wire [4:0] opcode_W;

	 assign opcode_W = instruction_W[31:27]; 

	 

	 assign load = (~opcode_W[4] & opcode_W[3] & ~opcode_W[2] & ~opcode_W[1] & ~opcode_W[0]); // lw 01000

	 

	 wire [31:0] Dmem_or_ALU_out; 

	 Mux_32_input mux_select_Dmem_or_ALU (.out(Dmem_or_ALU_out), .a(alu_output_W), .b(memory_out_W), .in(load));

	 

	 wire [4:0] aluOP_W; 

	 assign aluOP_W = instruction_W[6:2]; 

	 

	 

	 

	 wire MW_bypass_INSN;

	 assign MW_bypass_INSN = (~opcode_W[4] & ~opcode_W[3] & ~opcode_W[2] & ~opcode_W[1] & ~opcode_W[0]) 

									|(~opcode_W[4] & ~opcode_W[3] & opcode_W[2] & ~opcode_W[1] & opcode_W[0]) 

									|(~opcode_W[4] & opcode_W[3] & ~opcode_W[2] & ~opcode_W[1] & ~opcode_W[0]);

									

									

	 // WX bypass

	 

	 assign rd_W = instruction_W[26:22];

	 

	 wire WX_out_to_A;

	 assign WX_out_to_A = ( (DX_bypass_INSN & MW_bypass_INSN 

													  &(((rd_W[4] == rs_X[4])

													  & (rd_W[3] == rs_X[3])

													  & (rd_W[2] == rs_X[2])

													  & (rd_W[1] == rs_X[1])

													  & (rd_W[0] == rs_X[0])) & ~(~rd_W[4] & ~rd_W[3] & ~rd_W[2] & ~rd_W[1] & ~rd_W[0]))));

	 

	 wire WX_out_to_B; 

	 assign WX_out_to_B = ( ((DX_bypass_INSN & ~bbjs) & MW_bypass_INSN 

													  &(((rd_W[4] == rt_X[4])

													  & (rd_W[3] == rt_X[3])

													  & (rd_W[2] == rt_X[2])

													  & (rd_W[1] == rt_X[1])

													  & (rd_W[0] == rt_X[0])) & ~(~rd_W[4] & ~rd_W[3] & ~rd_W[2] & ~rd_W[1] & ~rd_W[0])))

								 |( (opcode_W[4] & ~opcode_W[3] & opcode_W[2] & ~opcode_W[1] & opcode_W[0]) 

									 &((opcode_X[4] & ~opcode_X[3] & opcode_X[2] & opcode_X[1] & ~opcode_X[0]))) // setx followed by bex

								 |( bbjs & MW_bypass_INSN

									&(( (rd_X[4] == rd_W[4])

									  &(rd_X[3] == rd_W[3])

									  &(rd_X[2] == rd_W[2])

									  &(rd_X[1] == rd_W[1])

									  &(rd_X[0] == rd_W[0]) & ~(~rd_W[4] & ~rd_W[3] & ~rd_W[2] & ~rd_W[1] & ~rd_W[0]))))); 

									

	 //WM bypass

	 wire WM_write_to_mem; 

	 assign WM_write_to_mem = ( MW_bypass_INSN & (~opcode_M[4] & ~opcode_M[3] & opcode_M[2] & opcode_M[1] & opcode_M[0])

									  &( ((rd_M[4] == rd_W[4])

										 &(rd_M[3] == rd_W[3])

										 &(rd_M[2] == rd_W[2])

										 &(rd_M[1] == rd_W[1])

										 &(rd_M[0] == rd_W[0]) & ~(~rd_W[4] & ~rd_W[3] & ~rd_W[2] & ~rd_W[1] & ~rd_W[0]))));

	 

	 Mux_32_input WM_bypass_mux (.out(data), .a(ReadReg_DataB_M), .b(data_writeReg), .in(WM_write_to_mem));  

	 

	 

	 wire bypass_mux_A_select_0;

	 wire bypass_mux_A_select_1; 

	 assign bypass_mux_A_select_0 = MX_out_to_A; 

	 assign bypass_mux_A_select_1 = WX_out_to_A;

	 genvar i4;

	 generate 

		for (i4=0; i4<32; i4=i4+1) begin: bypass_mux_A

			Mux_4_input u (.out(alu_input_A[i4]), .a(ReadReg_DataA_X[i4]), .b(alu_output_M[i4]), .c(data_writeReg[i4]), .d(alu_output_M[i4]), .in0(bypass_mux_A_select_0), .in1(bypass_mux_A_select_1)); 

		end 

	 endgenerate 

	 wire bypass_mux_B_select_0;

	 wire bypass_mux_B_select_1;

	 assign bypass_mux_B_select_0 = MX_out_to_B; 

	 assign bypass_mux_B_select_1 = WX_out_to_B;

	 genvar i5;

	 generate 

		for (i5=0; i5<32; i5=i5+1) begin: bypass_mux_5

			Mux_4_input u (.out(bypass_ReadReg_DataB_X[i5]), .a(ReadReg_DataB_X[i5]), .b(alu_output_M[i5]), .c(data_writeReg[i5]), .d(alu_output_M[i5]), .in0(bypass_mux_B_select_0), .in1(bypass_mux_B_select_1)); 

		end 

	 endgenerate 

	 

	 wire insn_in_M_overflow;

	 assign insn_in_M_overflow = overflow_exception_M & ((~opcode_M[4] & ~opcode_M[3] & ~opcode_M[2] & ~opcode_M[1] & ~opcode_M[0])

																			|

																		  (~opcode_M[4] & ~opcode_M[3] & opcode_M[2] & ~opcode_M[1] & opcode_M[0]));

	 

	 wire insn_in_W_overflow;

	 assign insn_in_W_overflow = overflow_exception_W & ((~opcode_W[4] & ~opcode_W[3] & ~opcode_W[2] & ~opcode_W[1] & ~opcode_W[0])

																			|

																		  (~opcode_W[4] & ~opcode_W[3] & opcode_W[2] & ~opcode_W[1] & opcode_W[0]));

										

	 assign bypass_rstatus_check = (insn_in_M_overflow | (insn_in_W_overflow & ~(opcode_M[4] & ~opcode_M[3] & opcode_M[2] & ~opcode_M[1] & opcode_M[0]))); 

	 

	 

	 

	 // control - RegWriteEnable 

	 assign ctrl_writeEnable = (~opcode_W[0] & ~opcode_W[1] & ~opcode_W[2] & ~opcode_W[4]) // lw and alus 0d000

									| (opcode_W[0] & ~opcode_W[1] & opcode_W[2] & ~opcode_W[3] & ~opcode_W[4]) // addi 00101

									| (~opcode_W[4] & ~opcode_W[3] & ~opcode_W[2] & opcode_W[1] & opcode_W[0]) // jal 00011 

									| (opcode_W[4] & ~opcode_W[3] & opcode_W[2] & ~opcode_W[1] & opcode_W[0]); // setx 10101 

									

	 wire [4:0] wr_jal_out; 

	 wire jal_W; 

	 assign jal_W = (~opcode_W[4] & ~opcode_W[3] & ~opcode_W[2] & opcode_W[1] & opcode_W[0]); 

	 

	 Mux_5_input wr_jal (.out(wr_jal_out), .a(instruction_W[26:22]), .b(5'd31), .in(jal_W));

	 



	 wire setx_W; 

	 assign setx_W = (opcode_W[4] & ~opcode_W[3] & opcode_W[2] & ~opcode_W[1] & opcode_W[0]); 

	 

	 wire rstatus_W; 

	 wire inv_rstatus_W;

	 assign inv_rstatus_W = ~rstatus_W; 

	 

	 wire add_overflow; 

	 assign add_overflow = (~opcode_W[4] & ~opcode_W[3] & ~opcode_W[2] & ~opcode_W[1] & ~opcode_W[0]) // add 00000

								 &(~aluOP_W[4] & ~aluOP_W[3] & ~aluOP_W[2] & ~aluOP_W[1] & ~aluOP_W[0]) // aluop 00000

								 &overflow_exception_W; 

								 

	 wire addi_overflow; 

	 assign addi_overflow = (~opcode_W[4] & ~opcode_W[3] & opcode_W[2] & ~opcode_W[1] & opcode_W[0]) // addi 00101 

								  &overflow_exception_W; 

								  

	 wire sub_overflow; 

	 assign sub_overflow = (~opcode_W[4] & ~opcode_W[3] & ~opcode_W[2] & ~opcode_W[1] & ~opcode_W[0]) // sub 00000 

								 &(~aluOP_W[4] & ~aluOP_W[3] & ~aluOP_W[2] & ~aluOP_W[1] & aluOP_W[0]) // aluop 00001 

								 &overflow_exception_W; 

	

	 wire mult_overflow; 

	 assign mult_overflow = (~opcode_W[4] & ~opcode_W[3] & ~opcode_W[2] & ~opcode_W[1] & ~opcode_W[0]) // mult 00000 

								  &(~aluOP_W[4] & ~aluOP_W[3] & aluOP_W[2] & aluOP_W[1] & ~aluOP_W[0]) // aluop 00110

								  &overflow_exception_W; 

								  

	 wire div_overflow; 

	 assign div_overflow = (~opcode_W[4] & ~opcode_W[3] & ~opcode_W[2] & ~opcode_W[1] & ~opcode_W[0]) // div 00000 

								  &(~aluOP_W[4] & ~aluOP_W[3] & aluOP_W[2] & aluOP_W[1] & aluOP_W[0]) // aluop 00111

								  &overflow_exception_W;

								  

	 assign rstatus_W = (setx_W | add_overflow | addi_overflow | sub_overflow | mult_overflow | div_overflow); 

	 

	 Mux_5_input wr_rstatus (.out(ctrl_writeReg), .a(wr_jal_out), .b(5'd30), .in(rstatus_W)); 

	 

	 wire [2:0] rstatus_encode_out; 

	 encoder_8_to_3 rstatus_encode (.out(rstatus_encode_out), .in0(add_overflow), .in1(addi_overflow), .in2(sub_overflow), 

											  .in3(mult_overflow), .in4(div_overflow), .in5(setx_W), .in6(inv_rstatus_W), .in7(1'b0)); 

											  

	 wire [31:0] add_overflow_status_val; 

	 assign add_overflow_status_val = 32'd1; 

	 wire [31:0] addi_overflow_status_val;

	 assign addi_overflow_status_val = 32'd2; 

	 wire [31:0] sub_overflow_status_val; 

	 assign sub_overflow_status_val = 32'd3; 

	 wire [31:0] mult_overflow_status_val; 

	 assign mult_overflow_status_val = 32'd4; 

	 wire [31:0] div_overflow_status_val;

	 assign div_overflow_status_val = 32'd5; 

	 

	 genvar i2; 

	 generate 

		for (i2=0; i2<32; i2=i2+1) begin: mux_8_to_1

			Mux_8_input u (.out(data_writeReg[i2]), 

								.in0(add_overflow_status_val[i2]), 

								.in1(addi_overflow_status_val[i2]), 

								.in2(sub_overflow_status_val[i2]),

								.in3(mult_overflow_status_val[i2]), 

								.in4(div_overflow_status_val[i2]), 

								.in5(alu_output_W[i2]),

								.in6(Dmem_or_ALU_out[i2]),

								.in7(1'b0),

								.en2(rstatus_encode_out[2]), 

								.en1(rstatus_encode_out[1]),

								.en0(rstatus_encode_out[0]));

	 end

	 endgenerate

	 

	 //// stall 



	 wire stall_check; 

	 assign stall_check = (~opcode_X[4] & opcode_X[3] & ~opcode_X[2] & ~opcode_X[1] & ~opcode_X[0]) // lw 01000

								&( ( ( (rs_D[4] == rd_X[4])

										&(rs_D[3] == rd_X[3])

										&(rs_D[2] == rd_X[2])

										&(rs_D[1] == rd_X[1])

										&(rs_D[0] == rd_X[0]) ) 

								 | ( ( ((rt_D[4] == rd_X[4])

										&(rt_D[3] == rd_X[3])

										&(rt_D[2] == rd_X[2])

										&(rt_D[1] == rd_X[1])) & ~(~opcode_D[4] & ~opcode_D[3] & opcode_D[2] & opcode_D[1] & opcode_D[0]) )

										) ) );

										

	 assign neg_stall = ~stall_check;

	 

	 

	 //// flush 

	 wire flush_check; 

	 

	 assign flush_check = (branch_taken | jump | jump_return); 

	 

	 Mux_32_input DX_stall_mux (.out(instruction_D_stall_checked), .a(instruction_D), .b(32'b0), .in(stall_check | flush_check)); 

	 

	 Mux_32_input FD_stall_mux (.out(instruction_stall_checked), .a(instruction), .b(32'b0), .in(flush_check)); 

	 

	 wire [31:0] alu_output_MultDiv;

	 wire data_rdy; 

	 wire alu_overflow_MultDiv; 

	 wire is_mult;

	 wire is_div;

	 wire multdiv_in_process;

	 assign is_mult =  (~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & ~opcode_X[1] & ~opcode_X[0])

							&(~aluOP[4] & ~aluOP[3] & aluOP[2] & aluOP[1] & ~aluOP[0]);

	 assign is_div = (~opcode_X[4] & ~opcode_X[3] & ~opcode_X[2] & ~opcode_X[1] & ~opcode_X[0])

						 &(~aluOP[4] & ~aluOP[3] & aluOP[2] & aluOP[1] & aluOP[0]);

	 //// multdiv 

	 multdiv MultDiv (.data_operandA(alu_input_A), 

							.data_operandB(alu_input_B), 

							.ctrl_MULT(is_mult & ~multdiv_in_process), 

							.ctrl_DIV(is_div & ~multdiv_in_process), 

							.clock(clock), 

							.data_result(alu_output_MultDiv), 

							.data_exception(alu_overflow_MultDiv), 

							.data_resultRDY(data_rdy)); 

	 wire is_multdiv; 

	 assign is_multdiv = ( is_mult | is_div ); // mul 00110 or div 00111

	 

	 wire multdiv_ongoing_reset;

	 assign multdiv_ongoing_reset = (reset | data_rdy);

	 

	 dffe_ref multdiv_ongoing (.q(multdiv_in_process), .d(is_multdiv ^ data_rdy), .clk(clock), .en(1'b1), .clr(reset)); 

	 

	 wire multdiv_or_alu_result_select;

	 assign multdiv_or_alu_result_select = (is_multdiv & data_rdy);

	 

	 Mux_32_input multdiv_or_alu_result (.out(alu_output), .a(alu_output_ALU), .b(alu_output_MultDiv), .in(multdiv_or_alu_result_select)); 

	 

	 Mux_2_input multdiv_or_alu_overflow (.out(alu_overflow), .a(alu_overflow_ALU), .b(alu_overflow_MultDiv), .in(multdiv_or_alu_result_select)); 

	 

	 assign multdiv_stall = (is_multdiv) & (~data_rdy); 

	 

	 // insert noop to XM latch 

	 

	 Mux_32_input instruction_X_stall_check_mux (.out(instruction_X_stall_checked), .a(instruction_X), .b(32'b0), .in(multdiv_stall));

	 

endmodule



module Adder_16_bit (out, c_out, data_A, data_B, c_in);  



input[15:0] data_A, data_B; 

input c_in;

output c_out; 

output[15:0] out; 



wire[7:0] s0, s1; 

wire c0, c1, c2; 



CLA_8_bit cla0 (out[7:0], c0, data_A[7:0], data_B[7:0], c_in); 

CLA_8_bit cla1 (s0[7:0], c1, data_A[15:8], data_B[15:8], 1'b0); 

CLA_8_bit cla2 (s1[7:0], c2, data_A[15:8], data_B[15:8], 1'b1); 



Mux_2_input mux0 (c_out, c1, c2, c0); 



genvar i;

generate 

	for (i=8; i<16; i=i+1) begin: mux1

		Mux_2_input u (out[i], s0[i-8], s1[i-8], c0);

end

endgenerate 



endmodule 



module Adder_32_bit (out, c_out, data_A, data_B, c_in); 



input[31:0] data_A, data_B; 

input c_in; 

output[31:0] out;

output c_out;



wire[15:0] s0, s1; 

wire c0, c1, c2; 



Adder_16_bit csa0 (out[15:0], c0, data_A[15:0], data_B[15:0], c_in); 

Adder_16_bit csa1 (s0[15:0], c1, data_A[31:16], data_B[31:16], 1'b0); 

Adder_16_bit csa2 (s1[15:0], c2, data_A[31:16], data_B[31:16], 1'b1); 



Mux_2_input mux0 (c_out, c1, c2, c0); 



genvar i;

generate 

	for (i=16; i<32; i=i+1) begin: mux1

		Mux_2_input u (out[i], s0[i-16], s1[i-16], c0);

end

endgenerate 



/*



wire c0, c1, c2; 



CLA_8_bit CLA0 (out[7:0], c0, data_A[7:0], data_B[7:0], c_in); 

CLA_8_bit CLA1 (out[15:8], c1, data_A[15:8], data_B[15:8], c0); 

CLA_8_bit CLA2 (out[23:16], c2, data_A[23:16], data_B[23:16], c1); 

CLA_8_bit CLA3 (out[31:24], c_out, data_A[31:24], data_B[31:24], c2); 



*/

endmodule 



module CLA_8_bit(out, c_out, data_A, data_B, c_in); 



input[7:0] data_A, data_B; 

input c_in; 



output[7:0] out; 

output c_out; 



wire p0, p1, p2, p3, p4, p5, p6, p7, g0, g1, g2, g3, g4, g5, g6, g7, c1, c2, c3, c4, c5, c6; 

wire temp0, temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10, temp11, temp12, temp13, temp14, temp15, temp16, temp17, temp18, temp19, temp20, temp21, temp22, temp23, temp24, temp25, temp26, temp27, temp28, temp29, temp30, temp31, temp32, temp33, temp34, temp35, temp36; 



or or0 (p0, data_A[0], data_B[0]); 

or or1 (p1, data_A[1], data_B[1]);

or or2 (p2, data_A[2], data_B[2]); 

or or3 (p3, data_A[3], data_B[3]); 

or or4 (p4, data_A[4], data_B[4]); 

or or5 (p5, data_A[5], data_B[5]); 

or or6 (p6, data_A[6], data_B[6]); 

or or7 (p7, data_A[7], data_B[7]); 



and and0 (g0, data_A[0], data_B[0]); 

and and1 (g1, data_A[1], data_B[1]);

and and2 (g2, data_A[2], data_B[2]); 

and and3 (g3, data_A[3], data_B[3]); 

and and4 (g4, data_A[4], data_B[4]); 

and and5 (g5, data_A[5], data_B[5]); 

and and6 (g6, data_A[6], data_B[6]); 

and and7 (g7, data_A[7], data_B[7]);



and and8 (temp1, p0, c_in); // p0 and c0 

or or8 (c1, g0, temp1); // (p0 and c0) or g0 



and and9 (temp2, p1, p0, c_in); 

and and10 (temp3, g0, p1); 

or or9 (c2, temp2, temp3, g1); 



and and11 (temp4, p2, p1, p0, c_in); 

and and12 (temp5, p2, p1, g0); 

and and13 (temp6, g1, p2); 

or or10 (c3, temp4, temp5, temp6, g2); 



and and14 (temp7, p3, p2, p1, p0, c_in); 

and and15 (temp8, p3, p2, p1, g0); 

and and16 (temp9, p3, p2, g1); 

and and17 (temp10, p3, g2); 

or or11 (c4, temp7, temp8, temp9, temp10, g3);



and and18 (temp11, p4, p3, p2, p1, p0, c_in); 

and and19 (temp12, p4, p3, p2, p1, g0);

and and20 (temp13, p4, p3, p2, g1); 

and and21 (temp14, p4, p3, g2); 

and and22 (temp15, p4, g3); 

or or12 (c5, temp11, temp12, temp13, temp14, temp15, g4); 



and and23 (temp16, p5, p4, p3, p2, p1, p0, c_in); 

and and24 (temp17, p5, p4, p3, p2, p1, g0); 

and and25 (temp18, p5, p4, p3, p2, g1); 

and and26 (temp19, p5, p4, p3, g2); 

and and27 (temp20, p5, p4, g3); 

and and28 (temp21, p5, g4); 

or or13 (c6, temp16, temp17, temp18, temp19, temp20, temp21, g5); 



and and29 (temp22, p6, p5, p4, p3, p2, p1, p0, c_in); 

and and30 (temp23, p6, p5, p4, p3, p2, p1, g0); 

and and31 (temp24, p6, p5, p4, p3, p2, g1); 

and and32 (temp25, p6, p5, p4, p3, g2); 

and and33 (temp26, p6, p5, p4, g3); 

and and34 (temp27, p6, p5, g4); 

and and35 (temp28, p6, g5); 

or or14 (c7, temp22, temp23, temp24, temp25, temp26, temp27, temp28, g6); 



and and36 (temp29, p7, p6, p5, p4, p3, p2, p1, p0, c_in); 

and and37 (temp30, p7, p6, p5, p4, p3, p2, p1, g0); 

and and38 (temp31, p7, p6, p5, p4, p3, p2, g1); 

and and39 (temp32, p7, p6, p5, p4, p3, g2); 

and and40 (temp33, p7, p6, p5, p4, g3); 

and and41 (temp34, p7, p6, p5, g4); 

and and42 (temp35, p7, p6, g5); 

and and43 (temp36, p7, g6);

or or15 (c_out, temp29, temp30, temp31, temp32, temp33, temp34, temp35, temp36, g7); 



xor xor0 (out[0], data_A[0], data_B[0], c_in); 

xor xor1 (out[1], data_A[1], data_B[1], c1); 

xor xor2 (out[2], data_A[2], data_B[2], c2); 

xor xor3 (out[3], data_A[3], data_B[3], c3); 

xor xor4 (out[4], data_A[4], data_B[4], c4); 

xor xor5 (out[5], data_A[5], data_B[5], c5); 

xor xor6 (out[6], data_A[6], data_B[6], c6); 

xor xor7 (out[7], data_A[7], data_B[7], c7); 



endmodule 



module Mux_2_input (out, a, b, in); 



input a, b, in; 

output out; 



wire n_in, temp0, temp1; 



not not0 (n_in, in); 

and and0 (temp0, n_in, a); 

and and1 (temp1, in, b); 

or or0 (out, temp0, temp1); 



endmodule 



module Mux_32_input(out, a, b, in); 



input[31:0] a, b;

input in; 

output[31:0] out; 



genvar i; 

	generate 

		for (i=0; i<32; i=i+1) begin: out_mux

			Mux_2_input u (out[i], a[i], b[i], in);

	end

	endgenerate

	

endmodule



module Mux_4_input (out, a, b, c, d, in0, in1); 



input a, b, c, d, in0, in1;

output out;



wire neg_in0, neg_in1;



not not_a (neg_in0, in0); 

not not_b (neg_in1, in1); 



wire and1_out, and2_out, and3_out, and4_out; 



and and1 (and1_out, a, neg_in0, neg_in1); 

and and2 (and2_out, b, in0, neg_in1);

and and3 (and3_out, c, neg_in0, in1); 

and and4 (and4_out, d, in0, in1);



or or1 (out, and1_out, and2_out, and3_out, and4_out); 



endmodule 



module Mux_5_input(out, a, b, in); 



input[4:0] a, b;

input in; 

output[4:0] out; 



genvar i; 

	generate 

		for (i=0; i<5; i=i+1) begin: out_mux

			Mux_2_input u (out[i], a[i], b[i], in);

	end

	endgenerate

	

endmodule



module Mux_8_input(out, in0, in1, in2, in3, in4, in5, in6, in7, en2, en1, en0); 



input en2, en1, en0; 

input in0, in1, in2, in3, in4, in5, in6, in7;

output out; 



wire temp0, temp1, temp2, temp3, temp4, temp5; 



Mux_2_input mux0 (temp0, in0, in1, en0); 

Mux_2_input mux1 (temp1, in2, in3, en0); 

Mux_2_input mux2 (temp2, in4, in5, en0); 

Mux_2_input mux3 (temp3, in6, in7, en0); 



Mux_2_input mux4 (temp4, temp0, temp1, en1); 

Mux_2_input mux5 (temp5, temp2, temp3, en1); 



Mux_2_input mux6 (out, temp4, temp5, en2); 



endmodule 



module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan, overflow);



   input [31:0] data_operandA, data_operandB;

   input [4:0] ctrl_ALUopcode, ctrl_shiftamt;



   output [31:0] data_result;

   output isNotEqual, isLessThan, overflow;



   // YOUR CODE HERE //

	

	// add or substract function 

	wire sub, add_c_out, sub_c_out; 

	wire[31:0] add_out, sub_out, negated_data_operandB; 

	genvar i0; 

	generate 

		for (i0=0; i0<32; i0=i0+1) begin: negate_B 

			not temp_not (negated_data_operandB[i0], data_operandB[i0]); 

	end 

	endgenerate 

	

	Adder_32_bit adder (add_out, add_c_out, data_operandA, data_operandB, 1'b0); 

	Adder_32_bit subtractor (sub_out, sub_c_out, data_operandA, negated_data_operandB, 1'b1); 

	

	// and or bitwise function

	wire[31:0] and_out;

	wire[31:0] or_out; 

	

	genvar i1;

	generate 

		for (i1=0; i1<32; i1=i1+1) begin: and_bitwise

			and temp_and (and_out[i1], data_operandA[i1], data_operandB[i1]);

	end

	endgenerate

	

	genvar i2; 

	generate 

		for (i2=0; i2<32; i2=i2+1) begin: or_bitwise 

			or temp_or (or_out[i2], data_operandA[i2], data_operandB[i2]); 

	end 

	endgenerate 

	

	// logical left shift and arithmetic right shift 

	wire[31:0] sll, sra; 

	shifter_left_32_bit sl (sll, data_operandA, ctrl_shiftamt); 

	shifter_right_32_bit sr (sra, data_operandA, ctrl_shiftamt); 

	

	// isNotEqual? 

	/*or or_equal (isNotEqual, sub_out[0], sub_out[1], sub_out[2], sub_out[3], sub_out[4], sub_out[5], 

					 sub_out[6], sub_out[7], sub_out[8], sub_out[9], sub_out[10], sub_out[11], 

					 sub_out[12], sub_out[13], sub_out[14], sub_out[15], sub_out[16], sub_out[17], 

					 sub_out[18], sub_out[19], sub_out[20], sub_out[21], sub_out[22], sub_out[23], 

					 sub_out[24], sub_out[25], sub_out[26], sub_out[27], sub_out[28], sub_out[29], 

					 sub_out[30], sub_out[31]); 

	*/			 

	

	wire[31:0] bitCheck; 

	genvar j;

	generate 

		for (j=0; j<32; j=j+1) begin: xor_equal 

			xor u (bitCheck[j], data_operandA[j], data_operandB[j]);

	end

	endgenerate 

	

	or or_equal (isNotEqual, bitCheck[0], bitCheck[1], bitCheck[2], bitCheck[3], bitCheck[4], bitCheck[5], 

					 bitCheck[6], bitCheck[7], bitCheck[8], bitCheck[9], bitCheck[10], bitCheck[11], 

					 bitCheck[12], bitCheck[13], bitCheck[14], bitCheck[15], bitCheck[16], bitCheck[17], 

					 bitCheck[18], bitCheck[19], bitCheck[20], bitCheck[21], bitCheck[22], bitCheck[23], 

					 bitCheck[24], bitCheck[25], bitCheck[26], bitCheck[27], bitCheck[28], bitCheck[29], 

					 bitCheck[30], bitCheck[31]);

	

	

	// overflow? 

	wire overflow_0, overflow_1, overflow_2, overflow_3; 

	wire isSub;

	wire isAdd;

	wire MSB_A, MSB_B, n_MSB_A, n_MSB_B; 

	assign MSB_A = data_operandA[31];

	assign MSB_B = data_operandB[31]; 

	assign isSub = ctrl_ALUopcode[0]; 

	not sign_flip (isAdd, isSub); 

	not flip_MSB_A (n_MSB_A, MSB_A); 

	not flip_MSB_B (n_MSB_B, MSB_B); 

	and and_over_0 (overflow_0, n_MSB_A, n_MSB_B, isAdd, add_out[31]); 

	and and_over_1 (overflow_1, n_MSB_A, MSB_B, isSub, sub_out[31]);

	wire n_MSB_sub;

	wire n_MSB_add;

	not flip_MSB_sub (n_MSB_sub, sub_out[31]);

	not flip_MSB_add (n_MSB_add, add_out[31]); 

	and and_over_2 (overflow_2, MSB_A, n_MSB_B, isSub, n_MSB_sub); 

	and and_over_3 (overflow_3, MSB_A, MSB_B, isAdd, n_MSB_add); 

	or overflow_calc (overflow, overflow_0, overflow_1, overflow_2, overflow_3); 

	

	// isLessThan? 

	// if overflow, then compare the MSBs 

	wire lessthan_overflow; 

	and and_over (lessthan_overflow, overflow, MSB_A, n_MSB_B); 

	or or_lessthan (isLessThan, sub_out[31], lessthan_overflow); 

	

	// select the one to output 

	genvar i3; 

	generate 

		for (i3=0; i3<32; i3=i3+1) begin: out_mux

			Mux_8_input u (data_result[i3], add_out[i3], sub_out[i3], and_out[i3], or_out[i3], sll[i3], sra[i3], 1'b0, 1'b0, ctrl_ALUopcode[2], ctrl_ALUopcode[1], ctrl_ALUopcode[0]); 

	end

	endgenerate

	

endmodule



module counter_4_bit (q3, q2, q1, q0, en, clk, clr); 



input clk, en, clr; 

output q3, q2, q1, q0; 



dffe_ref d0 (q0, ~q0, clk, en, clr);

dffe_ref d1 (q1, ~q1, q0, en, clr);

dffe_ref d2 (q2, ~q2, q1, en, clr);

dffe_ref d3 (q3, ~q3, q2, en, clr);



endmodule 



module counter_5_bit (q4, q3, q2, q1, q0, en, clk, clr); 



input clk, en, clr; 

output q4, q3, q2, q1, q0; 



dffe_ref d0 (q0, ~q0, clk, en, clr);

dffe_ref d1 (q1, ~q1, ~q0, en, clr);

dffe_ref d2 (q2, ~q2, ~q1, en, clr);

dffe_ref d3 (q3, ~q3, ~q2, en, clr);

dffe_ref d4 (q4, ~q4, ~q3, en, clr); 



endmodule 



module counter_6_bit (q5, q4, q3, q2, q1, q0, en, clk, clr); 



input clk, en, clr; 

output q5, q4, q3, q2, q1, q0; 



dffe_ref d0 (q0, ~q0, clk, en, clr);

dffe_ref d1 (q1, ~q1, ~q0, en, clr);

dffe_ref d2 (q2, ~q2, ~q1, en, clr);

dffe_ref d3 (q3, ~q3, ~q2, en, clr);

dffe_ref d4 (q4, ~q4, ~q3, en, clr); 

dffe_ref d5 (q5, ~q5, ~q4, en, clr); 



endmodule 



module decoder (out, in); 



input[4:0] in;

output[31:0] out; 



wire in0;

wire in1;

wire in2;

wire in3;

wire in4;



assign in0 = in[0];

assign in1 = in[1];

assign in2 = in[2];

assign in3 = in[3]; 

assign in4 = in[4]; 



wire inv_in0;

wire inv_in1;

wire inv_in2;

wire inv_in3;

wire inv_in4;



not not0 (inv_in0, in0);

not not1 (inv_in1, in1);

not not2 (inv_in2, in2);

not not3 (inv_in3, in3); 

not not4 (inv_in4, in4); 



and and31 (out[31], in4, in3, in2, in1, in0); 

and and30 (out[30], in4, in3, in2, in1, inv_in0); 

and and29 (out[29], in4, in3, in2, inv_in1, in0);

and and28 (out[28], in4, in3, in2, inv_in1, inv_in0); 

and and27 (out[27], in4, in3, inv_in2, in1, in0); 

and and26 (out[26], in4, in3, inv_in2, in1, inv_in0);

and and25 (out[25], in4, in3, inv_in2, inv_in1, in0); 

and and24 (out[24], in4, in3, inv_in2, inv_in1, inv_in0); 

and and23 (out[23], in4, inv_in3, in2, in1, in0); 

and and22 (out[22], in4, inv_in3, in2, in1, inv_in0);

and and21 (out[21], in4, inv_in3, in2, inv_in1, in0); 

and and20 (out[20], in4, inv_in3, in2, inv_in1, inv_in0); 

and and19 (out[19], in4, inv_in3, inv_in2, in1, in0);

and and18 (out[18], in4, inv_in3, inv_in2, in1, inv_in0); 

and and17 (out[17], in4, inv_in3, inv_in2, inv_in1, in0); 

and and16 (out[16], in4, inv_in3, inv_in2, inv_in1, inv_in0); 

and and15 (out[15], inv_in4, in3, in2, in1, in0); 

and and14 (out[14], inv_in4, in3, in2, in1, inv_in0); 

and and13 (out[13], inv_in4, in3, in2, inv_in1, in0);

and and12 (out[12], inv_in4, in3, in2, inv_in1, inv_in0); 

and and11 (out[11], inv_in4, in3, inv_in2, in1, in0); 

and and10 (out[10], inv_in4, in3, inv_in2, in1, inv_in0);

and and09 (out[9], inv_in4, in3, inv_in2, inv_in1, in0); 

and and08 (out[8], inv_in4, in3, inv_in2, inv_in1, inv_in0); 

and and07 (out[7], inv_in4, inv_in3, in2, in1, in0); 

and and06 (out[6], inv_in4, inv_in3, in2, in1, inv_in0);

and and05 (out[5], inv_in4, inv_in3, in2, inv_in1, in0); 

and and04 (out[4], inv_in4, inv_in3, in2, inv_in1, inv_in0); 

and and03 (out[3], inv_in4, inv_in3, inv_in2, in1, in0);

and and02 (out[2], inv_in4, inv_in3, inv_in2, in1, inv_in0); 

and and01 (out[1], inv_in4, inv_in3, inv_in2, inv_in1, in0); 

and and00 (out[0], inv_in4, inv_in3, inv_in2, inv_in1, inv_in0); 



endmodule 



module dffe_ref(q, d, clk, en, clr);

   

   //Inputs

   input d, clk, en, clr;

   

   //Internal wire

   wire clr;



   //Output

   output q;

   

   //Register

   reg q;



   //Intialize q to 0

   initial

   begin

       q = 1'b0;

   end



   //Set value of q on positive edge of the clock or clear

   always @(posedge clk or posedge clr) begin

       //If clear is high, set q to 0

       if (clr) begin

           q <= 1'b0;

       //If enable is high, set q to the value of d

       end else if (en) begin

           q <= d;

       end

   end

endmodule



module dflipflop(d, clk, clrn, prn, ena, q);

    input d, clk, ena, clrn, prn;

    wire clr;

    wire pr;



    output q;

    reg q;



    assign clr = ~clrn;

    assign pr = ~prn;



    initial

    begin

        q = 1'b0;

    end



    always @(posedge clk) begin

        if (q == 1'bx) begin

            q <= 1'b0;

        end else if (clr) begin

            q <= 1'b0;

        end else if (ena) begin

            q <= d;

        end

    end

endmodule



module divider (data_operandA, data_operandB, clock, data_result, data_exception, data_resultRDY, ctrl_DIV); 



input [31:0] data_operandA, data_operandB; 

input clock, ctrl_DIV; 



output [31:0] data_result; 

output data_exception, data_resultRDY; 

// output [63:0] product;



// output q5, q4, q3, q2, q1, q0;

wire[63:0] product;

wire[31:0] divisor;

wire[31:0] left_reg_in, right_reg_in; 



wire q5, q4, q3, q2, q1, q0; 



counter_6_bit cntr (q5, q4, q3, q2, q1, q0, 1'b1, clock, ctrl_DIV);



register div (divisor, data_operandB, clock, ctrl_DIV, 1'b0); 



register reg_63_32 (product[63:32], left_reg_in, clock, 1'b1, ctrl_DIV); 



register reg_31_0 (product[31:0], right_reg_in, clock, 1'b1, 1'b0); 



wire[31:0] adder_out; 

wire overflow_MSB;

wire[31:0] neg_div; 

assign neg_div = ~divisor;

Adder_32_bit adder (adder_out, overflow_MSB, product[63:32], neg_div, 1'b1);



wire sub_MSB; 

assign sub_MSB = adder_out[31]; 



wire[31:0] result; 

Mux_32_input restore_mux (result, adder_out, product[63:32], sub_MSB); // if subtract result is negative, restore the value 



wire[63:0] combiner; 

assign combiner[63:32] = result; 

assign combiner[31:0] = product[31:0]; 

wire[63:0] shifted_comb; 

assign shifted_comb = combiner <<< 1; 



wire[31:0] right_res;

assign left_reg_in = shifted_comb[63:32];

assign right_res[0] = ~sub_MSB;

assign right_res[31:1] = shifted_comb[31:1];



Mux_32_input mult_in (right_reg_in, right_res, data_operandA, ctrl_DIV);



assign data_result = product[31:0]; 

assign data_resultRDY = (q5 & ~q4 & ~q3 & ~q2 & ~q1 & q0);



wire all_zero;

assign all_zero = (~data_operandB[0] & 

                   ~data_operandB[1] &

						 ~data_operandB[2] & 

						 ~data_operandB[3] & 

						 ~data_operandB[4] &

						 ~data_operandB[5] &

						 ~data_operandB[6] &

						 ~data_operandB[7] &

						 ~data_operandB[8] &

						 ~data_operandB[9] &

						 ~data_operandB[10] &

						 ~data_operandB[11] & 

                   ~data_operandB[12] &

						 ~data_operandB[13] & 

						 ~data_operandB[14] & 

						 ~data_operandB[15] &

						 ~data_operandB[16] &

						 ~data_operandB[17] &

						 ~data_operandB[18] &

						 ~data_operandB[19] &

						 ~data_operandB[20] &

						 ~data_operandB[21] &

						 ~data_operandB[22] &

						 ~data_operandB[23] & 

                   ~data_operandB[24] &

						 ~data_operandB[25] & 

						 ~data_operandB[26] & 

						 ~data_operandB[27] &

						 ~data_operandB[28] &

						 ~data_operandB[29] &

						 ~data_operandB[30] &

						 ~data_operandB[31]);

						 

dffe_ref excep (data_exception, all_zero, clock, ctrl_DIV, 1'b0); 



endmodule 



module encoder_8_to_3 (out, in0, in1, in2, in3, in4, in5, in6, in7); 



input in0, in1, in2, in3, in4, in5, in6, in7; 

output [2:0] out; 



assign out[0] = (in1 | in3 | in5 | in7); 

assign out[1] = (in2 | in3 | in6 | in7); 

assign out[2] = (in4 | in5 | in6 | in7); 



endmodule 



module multdiv(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_resultRDY);

    input [31:0] data_operandA, data_operandB;

    input ctrl_MULT, ctrl_DIV, clock;



    output [31:0] data_result;

    output data_exception, data_resultRDY;

	 

	 wire[31:0] mult_res, div_res, real_div_res, neg_div_res; 

	 wire mult_excep, div_excep, mult_RDY, div_RDY;

	 // Your code here

	 wire is_DIV; 

	 wire same_sign;

	 wire dividend_isPos, divisor_isPos, c1_out, c2_out, c3_out; 

	 assign dividend_isPos = ~data_operandA[31];

	 assign divisor_isPos = ~data_operandB[31]; 

	 xor x_gate (same_sign, dividend_isPos, ~divisor_isPos);

	 wire same_sign_stored;

	 dffe_ref isDiv (is_DIV, ctrl_DIV, clock, (ctrl_DIV | ctrl_MULT), 1'b0);

	 dffe_ref div_sign (.q(same_sign_stored), .d(same_sign), .clk(clock), .en(ctrl_Div), .clr(1'b0));

	 wire [31:0] input_DIV_1, input_DIV_2, adder1_in_1, adder2_in_1; 

	 

	 Mux_32_input m1(adder1_in_1, ~data_operandA, data_operandA, dividend_isPos);

	 Mux_32_input m2(adder2_in_1, ~data_operandB, data_operandB, divisor_isPos);

	 

	 Adder_32_bit adder1 (input_DIV_1, c1_out, adder1_in_1, 32'b0, ~dividend_isPos); 

	 Adder_32_bit adder2 (input_DIV_2, c2_out, adder2_in_1, 32'b0, ~divisor_isPos); 

	 

	 multiplier multi (data_operandA, data_operandB, clock, mult_res, mult_excep, mult_RDY, ctrl_MULT); 

	 divider div (input_DIV_1, input_DIV_2, clock, div_res, div_excep, div_RDY, ctrl_DIV); 

	 

	 Adder_32_bit adder3 (neg_div_res, c3_out, ~div_res, 32'b0, 1'b1);

	 

	 Mux_32_input m_div (.out(real_div_res), .a(neg_div_res), .b(div_res), .in(same_sign_stored)); 

	 

	 Mux_2_input mux_1 (data_exception, mult_excep, div_excep, is_DIV); 

	 Mux_2_input mux_2 (data_resultRDY, mult_RDY, div_RDY, is_DIV); 

	 Mux_32_input mux_3 (data_result, mult_res, real_div_res, is_DIV); 

	 

endmodule



module multiplier (data_operandA, data_operandB, clock, data_result, data_exception, data_resultRDY, ctrl_MULT); 

// multiplicand for tests



input [31:0] data_operandA, data_operandB; 

input clock, ctrl_MULT; 



output [31:0] data_result; 

output data_exception, data_resultRDY; 



// modified Booth's - repeat 16 times 



wire [63:0] product; 

wire last_bit; 

wire [31:0] multiplicand; 



wire q4, q3, q2, q1, q0; 



counter_5_bit cntr (q4, q3, q2, q1, q0, 1'b1, clock, ctrl_MULT); // clr = clear 



register cand (multiplicand[31:0], data_operandA, clock, ctrl_MULT, 1'b0); 



wire [31:0] multiplier_in, left_reg_in; 

wire[31:0] shifted_final_res; 



register reg_63_32 (product[63:32], left_reg_in, clock, 1'b1, ctrl_MULT); 



register reg_31_0 (product[31:0], multiplier_in, clock, 1'b1, 1'b0); 



dffe_ref d0 (last_bit, product[1], clock, 1'b1, ctrl_MULT); 



// shift the multiplicand to the desired 



wire isShift, isSub, isChange; 



assign isShift = ((product[1] & ~product[0] & ~last_bit) | (~product[1] & product[0] & last_bit)); 

assign isSub = ((product[1] & ~product[0] & ~last_bit) | (product[1] & product[0] & ~last_bit) | (product[1] & ~product[0] & last_bit));

assign isChange = ~((~product[1] & ~product[0] & ~last_bit) | (product[1] & product[0] & last_bit)); 



// multiplicand shift result 



wire[31:0] shifted_cand; 

assign shifted_cand = multiplicand[31:0] << 1; 



wire[31:0] shift_checked; 

Mux_32_input shift_mux (shift_checked, multiplicand, shifted_cand, isShift); 



wire[31:0] neg_cand; 

assign neg_cand = ~shift_checked; 



wire[31:0] sub_checked; 

Mux_32_input sub_mux (sub_checked, shift_checked, neg_cand, isSub); 



wire overflow_MSB; 

wire[31:0] adder_out; 

Adder_32_bit adder (adder_out, overflow_MSB, product[63:32], sub_checked, isSub); 



// overflow/exception check for add/sub 

wire overflow_addsub_temp, overflow_addsub; 

assign overflow_addsub_temp = (isChange) & 

											((isSub & 

											(

												(adder_out[31] & ~product[63] & (shift_checked[31])) 

												|

												((~adder_out[31]) & (product[63]) & ~shift_checked[31])

											)) 

											|

											((~isSub) & 

											(((shift_checked[31]) & (product[63]) & (~adder_out[31]))

												|

												(~shift_checked[31] & ~product[63] & adder_out[31]))));

												

dffe_ref addsub_dff (overflow_addsub, 1'b1, clock, overflow_addsub_temp, ctrl_MULT); 



// overflow/exception check for left shift 

wire overflow_shift_temp, overflow_shift; 

assign overflow_shift_temp = (isShift) & ((~shifted_cand[31] & multiplicand[31]) | (shifted_cand[31] & ~multiplicand[31])); 



dffe_ref shift_dff (overflow_shift, 1'b1, clock, overflow_shift_temp, ctrl_MULT);



wire[31:0] final_res; 

Mux_32_input change_mux (final_res, product[63:32], adder_out, isChange); 



wire signed [63:0] before_shift; 

assign before_shift[31:0] = product[31:0]; 

assign before_shift[63:32] = final_res[31:0]; 

wire signed [63:0] after_shift; 



//shifter_right_32_bit sh_right (after_shift, before_shift, 5'b00010);

assign after_shift = before_shift >>> 2; 



assign left_reg_in = after_shift[63:32]; 

Mux_32_input mult_in (multiplier_in, after_shift[31:0], data_operandB, ctrl_MULT); 



assign data_result = product[31:0]; 

assign data_resultRDY = (q4 & ~q3 & ~q2 & ~q1 & ~q0); 



wire overflow;

wire same_sign, cand_isZero, plier_isZero, result_isZero;

dffe_ref input_sign (same_sign, ((data_operandA[31] & data_operandB[31]) | ((~data_operandA[31]) & (~data_operandB[31]))), clock, ctrl_MULT, 1'b0); 

dffe_ref plier_zero (plier_isZero, ~(data_operandB[0] | 

												data_operandB[1] | 

												data_operandB[2] | 

												data_operandB[3] | 

												data_operandB[4] | 

												data_operandB[5] | 

												data_operandB[6] | 

												data_operandB[7] | 

												data_operandB[8] | 

												data_operandB[9] | 

												data_operandB[10] | 

												data_operandB[11] | 

												data_operandB[12] | 

												data_operandB[13] | 

												data_operandB[14] | 

												data_operandB[15] | 

												data_operandB[16] | 

												data_operandB[17] | 

												data_operandB[18] | 

												data_operandB[19] | 

												data_operandB[20] | 

												data_operandB[21] | 

												data_operandB[22] | 

												data_operandB[23] | 

												data_operandB[24] | 

												data_operandB[25] | 

												data_operandB[26] | 

												data_operandB[27] | 

												data_operandB[28] | 

												data_operandB[29] | 

												data_operandB[30] | 

												data_operandB[31])

							, clock, ctrl_MULT, 1'b0);

							

 

assign cand_isZero =	~(multiplicand[0] |

							 multiplicand[1] |

							 multiplicand[2] |

							 multiplicand[3] |

							 multiplicand[4] | 

							 multiplicand[5] | 

							 multiplicand[6] | 

							 multiplicand[7] | 

							 multiplicand[8] | 

							 multiplicand[9] | 

							 multiplicand[10] | 

							 multiplicand[11] | 

							 multiplicand[12] | 

							 multiplicand[13] | 

							 multiplicand[14] | 

							 multiplicand[15] | 

							 multiplicand[16] | 

							 multiplicand[17] | 

							 multiplicand[18] | 

							 multiplicand[19] | 

							 multiplicand[20] | 

							 multiplicand[21] | 

							 multiplicand[22] | 

							 multiplicand[23] | 

							 multiplicand[24] | 

							 multiplicand[25] | 

							 multiplicand[26] | 

							 multiplicand[27] | 

							 multiplicand[28] | 

							 multiplicand[29] | 

							 multiplicand[30] | 

							 multiplicand[31]);

							 

assign result_isZero=~(product[0] |

							 product[1] | 

							 product[2] | 

							 product[3] | 

							 product[4] | 

							 product[5] | 

							 product[6] | 

							 product[7] | 

							 product[8] | 

							 product[9] | 

							 product[10] | 

							 product[11] | 

							 product[12] | 

							 product[13] | 

							 product[14] | 

							 product[15] | 

							 product[16] | 

							 product[17] | 

							 product[18] | 

							 product[19] | 

							 product[20] | 

							 product[21] | 

							 product[22] | 

							 product[23] | 

							 product[24] | 

							 product[25] | 

							 product[26] | 

							 product[27] | 

							 product[28] | 

							 product[29] | 

							 product[30] | 

							 product[31]);

							 

assign data_exception = overflow_addsub | overflow_shift | 

								(((same_sign & product[31]) | ((~same_sign) & (~product[31]))) & (~cand_isZero) & (~plier_isZero)) | (same_sign & (multiplicand[31]) & result_isZero);



endmodule 



module one_bit_adder (s, c_out, a, b, c_in); 



input a, b, c_in; 

output s, c_out; 



xor xor1 (t0, a, b); 

xor xor2 (s, t0, c_in); 



and and1 (t1, t0, c_in); 

and and2 (t2, a, b); 



or or1 (c_out, t1, t2); 



endmodule 



module regfile (

    clock,

    ctrl_writeEnable,

    ctrl_reset, ctrl_writeReg,

    ctrl_readRegA, ctrl_readRegB, data_writeReg,

    data_readRegA, data_readRegB,
	 
	 write1, write2, write3, write4, write5, write6, write7, write8, write9, write10, write11, write12,
	 
	 read16

);



   input clock, ctrl_writeEnable, ctrl_reset;

   input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;

   input [31:0] data_writeReg;

	

	wire[31:0] ctrl_readRegA_decoded, ctrl_readRegB_decoded, ctrl_writeReg_decoded; 

	

	decoder ctrl_write_decoder (ctrl_writeReg_decoded, ctrl_writeReg); 

	decoder ctrl_readA_decoder (ctrl_readRegA_decoded, ctrl_readRegA); 

	decoder ctrl_readB_decoder (ctrl_readRegB_decoded, ctrl_readRegB); 

	

   output [31:0] data_readRegA, data_readRegB; 

	input [31:0] write1, write2, write3, write4, write5, write6, write7, write8, write9, write10, write11, write12;

	wire[31:0] read0, read1, read2, read3, read4, read5, read6, read7, read8, read9, read10, read11, read12, read13, read14, read15, read17, read18, read19, read20, read21, read22, read23, read24, read25, read26, read27, read28, read29, read30, read31; 

	output [31:0] read16;

	wire en0, en1, en2, en3, en4, en5, en6, en7, en8, en9, en10, en11, en12, en13, en14, en15, en16, en17, en18, en19, en20, en21, en22, en23, en24, en25, en26, en27, en28, en29, en30, en31;

	

	and and0 (en0, ctrl_writeEnable, ctrl_writeReg_decoded[0]); 

	and and1 (en1, ctrl_writeEnable, ctrl_writeReg_decoded[1]); 

	and and2 (en2, ctrl_writeEnable, ctrl_writeReg_decoded[2]); 

	and and3 (en3, ctrl_writeEnable, ctrl_writeReg_decoded[3]); 

	and and4 (en4, ctrl_writeEnable, ctrl_writeReg_decoded[4]); 

	and and5 (en5, ctrl_writeEnable, ctrl_writeReg_decoded[5]); 

	and and6 (en6, ctrl_writeEnable, ctrl_writeReg_decoded[6]); 

	and and7 (en7, ctrl_writeEnable, ctrl_writeReg_decoded[7]); 

	and and8 (en8, ctrl_writeEnable, ctrl_writeReg_decoded[8]); 

	and and9 (en9, ctrl_writeEnable, ctrl_writeReg_decoded[9]); 

	and and10 (en10, ctrl_writeEnable, ctrl_writeReg_decoded[10]); 

	and and11 (en11, ctrl_writeEnable, ctrl_writeReg_decoded[11]); 

	and and12 (en12, ctrl_writeEnable, ctrl_writeReg_decoded[12]); 

	and and13 (en13, ctrl_writeEnable, ctrl_writeReg_decoded[13]); 

	and and14 (en14, ctrl_writeEnable, ctrl_writeReg_decoded[14]); 

	and and15 (en15, ctrl_writeEnable, ctrl_writeReg_decoded[15]); 

	and and16 (en16, ctrl_writeEnable, ctrl_writeReg_decoded[16]); 

	and and17 (en17, ctrl_writeEnable, ctrl_writeReg_decoded[17]); 

	and and18 (en18, ctrl_writeEnable, ctrl_writeReg_decoded[18]); 

	and and19 (en19, ctrl_writeEnable, ctrl_writeReg_decoded[19]); 

	and and20 (en20, ctrl_writeEnable, ctrl_writeReg_decoded[20]); 

	and and21 (en21, ctrl_writeEnable, ctrl_writeReg_decoded[21]); 

	and and22 (en22, ctrl_writeEnable, ctrl_writeReg_decoded[22]); 

	and and23 (en23, ctrl_writeEnable, ctrl_writeReg_decoded[23]); 

	and and24 (en24, ctrl_writeEnable, ctrl_writeReg_decoded[24]); 

	and and25 (en25, ctrl_writeEnable, ctrl_writeReg_decoded[25]); 

	and and26 (en26, ctrl_writeEnable, ctrl_writeReg_decoded[26]); 

	and and27 (en27, ctrl_writeEnable, ctrl_writeReg_decoded[27]); 

	and and28 (en28, ctrl_writeEnable, ctrl_writeReg_decoded[28]); 

	and and29 (en29, ctrl_writeEnable, ctrl_writeReg_decoded[29]); 

	and and30 (en30, ctrl_writeEnable, ctrl_writeReg_decoded[30]); 

	and and31 (en31, ctrl_writeEnable, ctrl_writeReg_decoded[31]); 

	

	

	register reg0 (read0, data_writeReg, clock, 1'b0, ctrl_reset); 

	register reg1 (read1, write1, clock, 1'b1, ctrl_reset); 

	register reg2 (read2, write2, clock, 1'b1, ctrl_reset); 

	register reg3 (read3, write3, clock, 1'b1, ctrl_reset); 

	register reg4 (read4, write4, clock, 1'b1, ctrl_reset); 

	register reg5 (read5, write5, clock, 1'b1, ctrl_reset); 

	register reg6 (read6 , write6, clock, 1'b1, ctrl_reset); 

	register reg7 (read7, write7, clock, 1'b1, ctrl_reset); 

	register reg8 (read8 , write8, clock, 1'b1, ctrl_reset); 

	register reg9 (read9 , write9, clock, 1'b1, ctrl_reset); 

	register reg10 (read10, write10, clock, 1'b1, ctrl_reset); 

	register reg11 (read11, write11, clock, 1'b1, ctrl_reset); 

	register reg12 (read12, write12, clock, 1'b1, ctrl_reset); 

	register reg13 (read13, data_writeReg, clock, en13, ctrl_reset); 

	register reg14 (read14, data_writeReg, clock, en14, ctrl_reset); 

	register reg15 (read15, data_writeReg, clock, en15, ctrl_reset); 

	register reg16 (read16, data_writeReg, clock, en16, ctrl_reset); 

	register reg17 (read17, data_writeReg, clock, en17, ctrl_reset); 

	register reg18 (read18, data_writeReg, clock, en18, ctrl_reset); 

	register reg19 (read19, data_writeReg, clock, en19, ctrl_reset); 

	register reg20 (read20, data_writeReg, clock, en20, ctrl_reset); 

	register reg21 (read21, data_writeReg, clock, en21, ctrl_reset); 

	register reg22 (read22, data_writeReg, clock, en22, ctrl_reset); 

	register reg23 (read23, data_writeReg, clock, en23, ctrl_reset); 

	register reg24 (read24, data_writeReg, clock, en24, ctrl_reset); 

	register reg25 (read25, data_writeReg, clock, en25, ctrl_reset); 

	register reg26 (read26, data_writeReg, clock, en26, ctrl_reset); 

	register reg27 (read27, data_writeReg, clock, en27, ctrl_reset); 

	register reg28 (read28, data_writeReg, clock, en28, ctrl_reset); 

	register reg29 (read29, data_writeReg, clock, en29, ctrl_reset); 

	register reg30 (read30, data_writeReg, clock, en30, ctrl_reset); 

	register reg31 (read31, data_writeReg, clock, en31, ctrl_reset); 

	

	triStateBuffer triBuffA0 (data_readRegA, read0, ctrl_readRegA_decoded[0]); 

	triStateBuffer triBuffA1 (data_readRegA, read1, ctrl_readRegA_decoded[1]); 

	triStateBuffer triBuffA2 (data_readRegA, read2, ctrl_readRegA_decoded[2]); 

	triStateBuffer triBuffA3 (data_readRegA, read3, ctrl_readRegA_decoded[3]); 

	triStateBuffer triBuffA4 (data_readRegA, read4, ctrl_readRegA_decoded[4]); 

	triStateBuffer triBuffA5 (data_readRegA, read5, ctrl_readRegA_decoded[5]); 

	triStateBuffer triBuffA6 (data_readRegA, read6, ctrl_readRegA_decoded[6]); 

	triStateBuffer triBuffA7 (data_readRegA, read7, ctrl_readRegA_decoded[7]); 

	triStateBuffer triBuffA8 (data_readRegA, read8, ctrl_readRegA_decoded[8]); 

	triStateBuffer triBuffA9 (data_readRegA, read9, ctrl_readRegA_decoded[9]); 

	triStateBuffer triBuffA10 (data_readRegA, read10, ctrl_readRegA_decoded[10]); 

	triStateBuffer triBuffA11 (data_readRegA, read11, ctrl_readRegA_decoded[11]); 

	triStateBuffer triBuffA12 (data_readRegA, read12, ctrl_readRegA_decoded[12]); 

	triStateBuffer triBuffA13 (data_readRegA, read13, ctrl_readRegA_decoded[13]); 

	triStateBuffer triBuffA14 (data_readRegA, read14, ctrl_readRegA_decoded[14]); 

	triStateBuffer triBuffA15 (data_readRegA, read15, ctrl_readRegA_decoded[15]); 

	triStateBuffer triBuffA16 (data_readRegA, read16, ctrl_readRegA_decoded[16]); 

	triStateBuffer triBuffA17 (data_readRegA, read17, ctrl_readRegA_decoded[17]); 

	triStateBuffer triBuffA18 (data_readRegA, read18, ctrl_readRegA_decoded[18]); 

	triStateBuffer triBuffA19 (data_readRegA, read19, ctrl_readRegA_decoded[19]); 

	triStateBuffer triBuffA20 (data_readRegA, read20, ctrl_readRegA_decoded[20]); 

	triStateBuffer triBuffA21 (data_readRegA, read21, ctrl_readRegA_decoded[21]); 

	triStateBuffer triBuffA22 (data_readRegA, read22, ctrl_readRegA_decoded[22]); 

	triStateBuffer triBuffA23 (data_readRegA, read23, ctrl_readRegA_decoded[23]); 

	triStateBuffer triBuffA24 (data_readRegA, read24, ctrl_readRegA_decoded[24]); 

	triStateBuffer triBuffA25 (data_readRegA, read25, ctrl_readRegA_decoded[25]); 

	triStateBuffer triBuffA26 (data_readRegA, read26, ctrl_readRegA_decoded[26]); 

	triStateBuffer triBuffA27 (data_readRegA, read27, ctrl_readRegA_decoded[27]); 

	triStateBuffer triBuffA28 (data_readRegA, read28, ctrl_readRegA_decoded[28]); 

	triStateBuffer triBuffA29 (data_readRegA, read29, ctrl_readRegA_decoded[29]); 

	triStateBuffer triBuffA30 (data_readRegA, read30, ctrl_readRegA_decoded[30]); 

	triStateBuffer triBuffA31 (data_readRegA, read31, ctrl_readRegA_decoded[31]); 

	

	triStateBuffer triBuffB0 (data_readRegB, read0, ctrl_readRegB_decoded[0]); 

	triStateBuffer triBuffB1 (data_readRegB, read1, ctrl_readRegB_decoded[1]); 

	triStateBuffer triBuffB2 (data_readRegB, read2, ctrl_readRegB_decoded[2]); 

	triStateBuffer triBuffB3 (data_readRegB, read3, ctrl_readRegB_decoded[3]); 

	triStateBuffer triBuffB4 (data_readRegB, read4, ctrl_readRegB_decoded[4]); 

	triStateBuffer triBuffB5 (data_readRegB, read5, ctrl_readRegB_decoded[5]); 

	triStateBuffer triBuffB6 (data_readRegB, read6, ctrl_readRegB_decoded[6]); 

	triStateBuffer triBuffB7 (data_readRegB, read7, ctrl_readRegB_decoded[7]); 

	triStateBuffer triBuffB8 (data_readRegB, read8, ctrl_readRegB_decoded[8]); 

	triStateBuffer triBuffB9 (data_readRegB, read9, ctrl_readRegB_decoded[9]); 

	triStateBuffer triBuffB10 (data_readRegB, read10, ctrl_readRegB_decoded[10]); 

	triStateBuffer triBuffB11 (data_readRegB, read11, ctrl_readRegB_decoded[11]); 

	triStateBuffer triBuffB12 (data_readRegB, read12, ctrl_readRegB_decoded[12]); 

	triStateBuffer triBuffB13 (data_readRegB, read13, ctrl_readRegB_decoded[13]); 

	triStateBuffer triBuffB14 (data_readRegB, read14, ctrl_readRegB_decoded[14]); 

	triStateBuffer triBuffB15 (data_readRegB, read15, ctrl_readRegB_decoded[15]); 

	triStateBuffer triBuffB16 (data_readRegB, read16, ctrl_readRegB_decoded[16]); 

	triStateBuffer triBuffB17 (data_readRegB, read17, ctrl_readRegB_decoded[17]); 

	triStateBuffer triBuffB18 (data_readRegB, read18, ctrl_readRegB_decoded[18]); 

	triStateBuffer triBuffB19 (data_readRegB, read19, ctrl_readRegB_decoded[19]); 

	triStateBuffer triBuffB20 (data_readRegB, read20, ctrl_readRegB_decoded[20]); 

	triStateBuffer triBuffB21 (data_readRegB, read21, ctrl_readRegB_decoded[21]); 

	triStateBuffer triBuffB22 (data_readRegB, read22, ctrl_readRegB_decoded[22]); 

	triStateBuffer triBuffB23 (data_readRegB, read23, ctrl_readRegB_decoded[23]); 

	triStateBuffer triBuffB24 (data_readRegB, read24, ctrl_readRegB_decoded[24]); 

	triStateBuffer triBuffB25 (data_readRegB, read25, ctrl_readRegB_decoded[25]); 

	triStateBuffer triBuffB26 (data_readRegB, read26, ctrl_readRegB_decoded[26]); 

	triStateBuffer triBuffB27 (data_readRegB, read27, ctrl_readRegB_decoded[27]); 

	triStateBuffer triBuffB28 (data_readRegB, read28, ctrl_readRegB_decoded[28]); 

	triStateBuffer triBuffB29 (data_readRegB, read29, ctrl_readRegB_decoded[29]); 

	triStateBuffer triBuffB30 (data_readRegB, read30, ctrl_readRegB_decoded[30]); 

	triStateBuffer triBuffB31 (data_readRegB, read31, ctrl_readRegB_decoded[31]); 

	

	

endmodule



module register(out, in, clk, en, reset);



input clk, en, reset; 

input[31:0] in; 

output[31:0] out; 



dffe_ref df31 (out[31], in[31], clk, en, reset);

dffe_ref df30 (out[30], in[30], clk, en, reset);

dffe_ref df29 (out[29], in[29], clk, en, reset);

dffe_ref df28 (out[28], in[28], clk, en, reset);

dffe_ref df27 (out[27], in[27], clk, en, reset);

dffe_ref df26 (out[26], in[26], clk, en, reset);

dffe_ref df25 (out[25], in[25], clk, en, reset);

dffe_ref df24 (out[24], in[24], clk, en, reset);

dffe_ref df23 (out[23], in[23], clk, en, reset);

dffe_ref df22 (out[22], in[22], clk, en, reset);

dffe_ref df21 (out[21], in[21], clk, en, reset);

dffe_ref df20 (out[20], in[20], clk, en, reset);

dffe_ref df19 (out[19], in[19], clk, en, reset);

dffe_ref df18 (out[18], in[18], clk, en, reset);

dffe_ref df17 (out[17], in[17], clk, en, reset);

dffe_ref df16 (out[16], in[16], clk, en, reset);

dffe_ref df15 (out[15], in[15], clk, en, reset);

dffe_ref df14 (out[14], in[14], clk, en, reset);

dffe_ref df13 (out[13], in[13], clk, en, reset);

dffe_ref df12 (out[12], in[12], clk, en, reset);

dffe_ref df11 (out[11], in[11], clk, en, reset);

dffe_ref df10 (out[10], in[10], clk, en, reset);

dffe_ref df09 (out[9], in[9], clk, en, reset);

dffe_ref df08 (out[8], in[8], clk, en, reset);

dffe_ref df07 (out[7], in[7], clk, en, reset);

dffe_ref df06 (out[6], in[6], clk, en, reset);

dffe_ref df05 (out[5], in[5], clk, en, reset);

dffe_ref df04 (out[4], in[4], clk, en, reset);

dffe_ref df03 (out[3], in[3], clk, en, reset);

dffe_ref df02 (out[2], in[2], clk, en, reset);

dffe_ref df01 (out[1], in[1], clk, en, reset);

dffe_ref df00 (out[0], in[0], clk, en, reset);





endmodule 



module shifter_left_16_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<16; i0=i0+1) begin: l0

		assign out[i0] = 1'b0; 

end

endgenerate 



genvar i1;

generate 

	for (i1=16; i1<32; i1=i1+1) begin: l1 

		assign out[i1] = in[i1-16];

end

endgenerate 



endmodule 



module shifter_left_1_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<1; i0=i0+1) begin: l0

		assign out[i0] = 1'b0; 

end

endgenerate 



genvar i1;

generate 

	for (i1=1; i1<32; i1=i1+1) begin: l1 

		assign out[i1] = in[i1-1]; 

end

endgenerate 



endmodule 



module shifter_left_2_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<2; i0=i0+1) begin: l0

		assign out[i0] = 1'b0;

end

endgenerate 



genvar i1;

generate 

	for (i1=2; i1<32; i1=i1+1) begin: l1 

		assign out[i1] = in[i1-2]; 

end

endgenerate 



endmodule 



module shifter_left_32_bit(out, in, shamt); 

input[4:0] shamt; 

input[31:0] in;

output[31:0] out; 



wire[31:0] t0, t1, t2, t3, t4, t5, t6, t7, t8; 



shifter_left_16_bit sl_16 (t0, in);



genvar i0; 

generate 

	for (i0=0; i0<32; i0=i0+1) begin: mux0

		Mux_2_input u (t1[i0], in[i0], t0[i0], shamt[4]);

end

endgenerate



shifter_left_8_bit sl_8 (t2, t1); 



genvar i1; 

generate 

	for (i1=0; i1<32; i1=i1+1) begin: mux1

		Mux_2_input u (t3[i1], t1[i1], t2[i1], shamt[3]);

end

endgenerate



shifter_left_4_bit sl_4 (t4, t3); 



genvar i2;

generate 

	for (i2=0; i2<32; i2=i2+1) begin: mux2

		Mux_2_input u (t5[i2], t3[i2], t4[i2], shamt[2]);

end

endgenerate



shifter_left_2_bit sl_2 (t6, t5); 



genvar i3;

generate 

	for (i3=0; i3<32; i3=i3+1) begin: mux3

		Mux_2_input u (t7[i3], t5[i3], t6[i3], shamt[1]);

end

endgenerate



shifter_left_1_bit sl_1 (t8, t7); 



genvar i4;

generate 

	for (i4=0; i4<32; i4=i4+1) begin: mux4

		Mux_2_input u (out[i4], t7[i4], t8[i4], shamt[0]);

end

endgenerate





/*



wire [31:0] s0, s1, s2, s3;



Mux_2_input mux0_0 (s0[0], in[0], 0, shamt[0]); 

genvar i0;

generate 

	for (i0=1; i0<32; i0=i0+1) begin: mux0

		Mux_2_input u (s0[i0], in[i0], in[i0-1], shamt[0]);

end

endgenerate 



Mux_2_input mux1_0 (s1[0], s0[0], 0, shamt[1]); 

Mux_2_input mux1_1 (s1[1], s0[1], 0, shamt[1]); 

genvar i1;

generate 

	for (i1=2; i1<32; i1=i1+1) begin: mux1

		Mux_2_input u (s1[i1], s0[i1], s0[i1-2], shamt[1]); 

end

endgenerate 



Mux_2_input mux2_0 (s2[0], s1[0], 0, shamt[2]); 

Mux_2_input mux2_1 (s2[1], s1[1], 0, shamt[2]); 

Mux_2_input mux2_2 (s2[2], s1[2], 0, shamt[2]); 

Mux_2_input mux2_3 (s2[3], s1[3], 0, shamt[2]); 



genvar i2;

generate 

	for (i2=4; i2<32; i2=i2+1) begin: mux2

		Mux_2_input u (s2[i2], s1[i2], s1[i2-4], shamt[2]); 

end

endgenerate 



genvar i3;

generate 

	for (i3=0; i3<8; i3=i3+1) begin: mux3

		Mux_2_input u (s3[i3], s2[i3], 0, shamt[3]);

end

endgenerate



genvar i4; 

generate 

	for (i4=8; i4<32; i4=i4+1) begin: mux4

		Mux_2_input u (s3[i4], s2[i4], s2[i4-8], shamt[3]);

end

endgenerate 



genvar i5; 

generate 

	for (i5=0; i5<16; i5=i5+1) begin: mux5

		Mux_2_input u (out[i5], s3[i5], 0, shamt[4]);

end

endgenerate



genvar i6; 

generate 

	for (i6=16; i6<32; i6=i6+1) begin: mux6

		Mux_2_input u (out[i6], s3[i6], s3[i6-16], shamt[4]);

end

endgenerate 

*/



endmodule 



module shifter_left_4_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<4; i0=i0+1) begin: l0

		assign out[i0] = 1'b0;

end

endgenerate 



genvar i1;

generate 

	for (i1=4; i1<32; i1=i1+1) begin: l1 

		assign out[i1] = in[i1-4]; 

end

endgenerate 



endmodule 



module shifter_left_8_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<8; i0=i0+1) begin: l0

		assign out[i0] = 1'b0;

end

endgenerate 



genvar i1;

generate 

	for (i1=8; i1<32; i1=i1+1) begin: l1 

		assign out[i1] = in[i1-8]; 

end

endgenerate 



endmodule 



module shifter_right_16_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<16; i0=i0+1) begin: l0

		assign out[i0] = in[i0+16]; 

end

endgenerate 



genvar i1;

generate 

	for (i1=16; i1<32; i1=i1+1) begin: l1 

		assign out[i1] = in[31]; 

end

endgenerate 



endmodule 



module shifter_right_1_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<31; i0=i0+1) begin: l0

		assign out[i0] = in[i0+1]; 

end

endgenerate 



genvar i1;

generate 

	for (i1=31; i1<32; i1=i1+1) begin: l1 

		assign out[i1] = in[31]; 

end

endgenerate 



endmodule 



module shifter_right_2_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<30; i0=i0+1) begin: l0

		assign out[i0] = in[i0+2]; 

end

endgenerate 



genvar i1;

generate 

	for (i1=30; i1<32; i1=i1+1) begin: l1

		assign out[i1] = in[31]; 

end

endgenerate 



endmodule 



module shifter_right_32_bit(out, in, shamt); 



input[31:0] in;

input[4:0] shamt; 

output[31:0] out; 





wire[31:0] t0, t1, t2, t3, t4, t5, t6, t7, t8; 



shifter_right_16_bit sr_16 (t0, in);



genvar i0; 

generate 

	for (i0=0; i0<32; i0=i0+1) begin: mux0

		Mux_2_input u (t1[i0], in[i0], t0[i0], shamt[4]);

end

endgenerate



shifter_right_8_bit sr_8 (t2, t1); 



genvar i1; 

generate 

	for (i1=0; i1<32; i1=i1+1) begin: mux1

		Mux_2_input u (t3[i1], t1[i1], t2[i1], shamt[3]);

end

endgenerate



shifter_right_4_bit sr_4 (t4, t3); 



genvar i2;

generate 

	for (i2=0; i2<32; i2=i2+1) begin: mux2

		Mux_2_input u (t5[i2], t3[i2], t4[i2], shamt[2]);

end

endgenerate



shifter_right_2_bit sr_2 (t6, t5); 



genvar i3;

generate 

	for (i3=0; i3<32; i3=i3+1) begin: mux3

		Mux_2_input u (t7[i3], t5[i3], t6[i3], shamt[1]);

end

endgenerate



shifter_right_1_bit sr_1 (t8, t7); 



genvar i4;

generate 

	for (i4=0; i4<32; i4=i4+1) begin: mux4

		Mux_2_input u (out[i4], t7[i4], t8[i4], shamt[0]);

end

endgenerate





/*



wire[31:0] s0, s1, s2, s3; 



Mux_2_input mux0_0 (s0[31], in[31], in[31], shamt[0]); 

genvar i0;

generate 

	for (i0=30; i0>=0; i0=i0-1) begin: mux0

		Mux_2_input u (s0[i0], in[i0], in[i0+1], shamt[0]);

end

endgenerate 



Mux_2_input mux1_0 (s1[31], s0[31], in[31], shamt[1]); 

Mux_2_input mux1_1 (s1[30], s0[30], in[31], shamt[1]); 

genvar i1;

generate 

	for (i1=29; i1>=0; i1=i1-1) begin: mux1

		Mux_2_input u (s1[i1], s0[i1], s0[i1+2], shamt[1]); 

end

endgenerate 



Mux_2_input mux2_0 (s2[31], s1[31], in[31], shamt[2]); 

Mux_2_input mux2_1 (s2[30], s1[30], in[31], shamt[2]); 

Mux_2_input mux2_2 (s2[29], s1[29], in[31], shamt[2]); 

Mux_2_input mux2_3 (s2[28], s1[28], in[31], shamt[2]); 



genvar i2;

generate 

	for (i2=27; i2>=0; i2=i2-1) begin: mux2

		Mux_2_input u (s2[i2], s1[i2], s1[i2+4], shamt[2]); 

end

endgenerate 



genvar i3;

generate 

	for (i3=31; i3>=24; i3=i3-1) begin: mux3

		Mux_2_input u (s3[i3], s2[i3], in[31], shamt[3]);

end

endgenerate



genvar i4; 

generate 

	for (i4=23; i4>=0; i4=i4-1) begin: mux4

		Mux_2_input u (s3[i4], s2[i4], s2[i4+8], shamt[3]);

end

endgenerate 



genvar i5; 

generate 

	for (i5=31; i5>=16; i5=i5-1) begin:  mux5

		Mux_2_input u (out[i5], s3[i5], in[31], shamt[4]);

end

endgenerate



genvar i6; 

generate 

	for (i6=15; i6>=0; i6=i6-1) begin: mux6

		Mux_2_input u (out[i6], s3[i6], s3[i6+16], shamt[4]);

end

endgenerate



*/

endmodule 



module shifter_right_4_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<28; i0=i0+1) begin: l0

		assign out[i0] = in[i0+4]; 

end

endgenerate 



genvar i1;

generate 

	for (i1=28; i1<32; i1=i1+1) begin: l1 

		assign out[i1] = in[31]; 

end

endgenerate 



endmodule 



module shifter_right_8_bit(out, in); 



input[31:0] in;

output[31:0] out; 



genvar i0;

generate 

	for (i0=0; i0<24; i0=i0+1) begin: l0

		assign out[i0] = in[i0+8]; 

end

endgenerate 



genvar i1;

generate 

	for (i1=24; i1<32; i1=i1+1) begin: l1 

		assign out[i1] = in[31];

end

endgenerate 



endmodule 



module triStateBuffer (out, in, en); 



input[31:0] in;

input en; 

output[31:0] out; 



assign out = en ? in : 32'bz; 



endmodule 
