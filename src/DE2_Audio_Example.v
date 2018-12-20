
module DE2_Audio_Example (
	// Inputs
	CLOCK_50,
	CLOCK_27,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT,

	I2C_SCLK,
	c, csharp, d, dsharp, e, f, fsharp, g, gsharp, a, asharp, b, oct_up, oct_down
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				CLOCK_50;
input				CLOCK_27;

input				AUD_ADCDAT;
input 			c, csharp, d, dsharp, e, f, fsharp, g, gsharp, a, asharp, b;
input				oct_up, oct_down;

// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

inout				I2C_SDAT;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;

output				I2C_SCLK;

/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;

	///// processor /////
	
	/** IMEM **/
    // Figure out how to generate a Quartus syncram component and commit the generated verilog file.
    // Make sure you configure it correctly!
    wire [11:0] address_imem;
    wire [31:0] q_imem;
    imem my_imem(
        .address    (address_imem),            // address of data
        .clock      (~CLOCK_50),                  // you may need to invert the clock
		  .data		  (32'b0),
		  .wren		  (1'b0),
        .q          (q_imem)                   // the raw instruction
    );
	
	 wire [11:0] address_dmem;
    wire [31:0] data;
    wire wren;
    wire [31:0] q_dmem;

    dmem my_dmem(
        .rdaddress    (address_dmem),       // address of data
		  .wraddress		(address_dmem),
        .clock      (~CLOCK_50),                  // may need to invert the clock
        .data	    (data),    // data you want to write
        .wren	    (wren),      // write enable
        .q          (q_dmem)    // data from dmem
    );

    /** REGFILE **/
    // Instantiate your regfile
	 
    wire ctrl_writeEnable;
    wire [4:0] ctrl_readRegA, ctrl_readRegB;
	 wire [4:0] ctrl_writeReg;
    wire [31:0] data_writeReg;
    wire [31:0] data_readRegA, data_readRegB;

	processor my_processor(CLOCK_50, reset, address_imem, q_imem, address_dmem, data, wren, q_dmem, 
		ctrl_writeEnable, ctrl_writeReg, ctrl_readRegA, ctrl_readRegB, data_writeReg, data_readRegA, data_readRegB);
		
	
	wire [31:0] out;
	
	regfile my_regfile(
        ~CLOCK_50,
        ctrl_writeEnable,
        reset,
        ctrl_writeReg,
        ctrl_readRegA,
        ctrl_readRegB,
        data_writeReg,
        data_readRegA,
        data_readRegB,
		  c_out, csharp_out, d_out, dsharp_out, e_out, f_out, fsharp_out, g_out, gsharp_out, a_out, asharp_out, b_out,
		  out 
    );

// Internal Registers

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/
 
 /*****************************************************************************
 *                         LAB 6 SOUNDS GO HERE                               *
 *****************************************************************************/
 
 
 /*****************************************************************************
 *                         LAB 6 SOUNDS END HERE                              *
 *****************************************************************************/


assign read_audio_in			= audio_in_available & audio_out_allowed;

/*
reg [18:0] delay_cnt;
wire [18:0] delay;
reg snd;

always @(posedge CLOCK_50)
	if (delay_cnt == delay) begin
		delay_cnt <= 0;
		snd <= !snd;
	end else delay_cnt <= delay_cnt + 1;
	

assign delay = 19'd223114;
*/
reg sound_c, sound_csharp, sound_d, sound_dsharp, sound_e, sound_f, sound_fsharp, sound_g, sound_gsharp, sound_a, sound_asharp, sound_b;
//reg signed [2:-8] cordic_in;
integer 		delay_c = 191109;
integer		delay_csharp = 180388;
integer		delay_d = 170265;
integer		delay_dsharp = 160705;
integer		delay_e = 151685;
integer		delay_f = 143172;
integer		delay_fsharp = 135139;
integer		delay_g = 127551;
integer		delay_gsharp = 120395;
integer		delay_a = 113636;
integer		delay_asharp = 107259;
integer		delay_b = 101239;
integer 		count_c, count_csharp, count_d, count_dsharp, count_e, count_f, count_fsharp, count_g, count_gsharp, count_a, count_asharp, count_b = 0;

initial
	begin
		//cordic_in = 11'b100_00011001;
		sound_c = 1'b1;
		sound_csharp = 1'b1;
		sound_d = 1'b1;
		sound_dsharp = 1'b1;
		sound_e = 1'b1;
		sound_f = 1'b1; 
		sound_fsharp = 1'b1; 
		sound_g = 1'b1;
		sound_gsharp = 1'b1;
		sound_a = 1'b1;
		sound_asharp = 1'b1;
		sound_b = 1'b1;
	end
	
always @(negedge oct_up)
	begin
			delay_c = delay_c/2;
			delay_csharp = delay_csharp/2;
			delay_d = delay_d/2;
			delay_dsharp = delay_dsharp/2;
			delay_e = delay_e/2;
			delay_f = delay_f/2;
			delay_fsharp = delay_fsharp/2;
			delay_g = delay_g/2;
			delay_gsharp = delay_gsharp/2;
			delay_a = delay_a/2;
			delay_asharp = delay_asharp/2;
			delay_b = delay_b/2;
	end

always @(posedge CLOCK_50)
	begin
		count_c = count_c + 1;
		count_csharp = count_csharp + 1;
		count_d = count_d + 1;
		count_dsharp = count_dsharp + 1;
		count_e = count_e + 1;
		count_f = count_f + 1;
		count_fsharp = count_fsharp + 1;
		count_g = count_g + 1;
		count_gsharp = count_gsharp + 1;
		count_a = count_a + 1;
		count_asharp = count_asharp + 1;
		count_b = count_b + 1;
		if (count_c >= delay_c) begin
			//cordic_in = cordic_in + 11'b000_00011001;
			sound_c <= !sound_c;
			count_c <= 0;
		end
		if (count_csharp >= delay_csharp) begin
			sound_csharp <= !sound_csharp;
			count_csharp <= 0;
		end
		if (count_d >= delay_d) begin
			sound_d <= !sound_d;
			count_d <= 0;
		end
		if (count_dsharp >= delay_dsharp) begin
			sound_dsharp <= !sound_dsharp;
			count_dsharp <= 0;
		end
		if (count_e >= delay_e) begin
			sound_e <= !sound_e;
			count_e <= 0;
		end
		if (count_f >= delay_f) begin
			sound_f <= !sound_f;
			count_f <= 0;
		end
		if (count_fsharp >= delay_fsharp) begin
			sound_fsharp <= !sound_fsharp;
			count_fsharp <= 0;
		end
		if (count_g >= delay_g) begin
			sound_g <= !sound_g;
			count_g <= 0;
		end
		if (count_gsharp >= delay_gsharp) begin
			sound_gsharp <= !sound_gsharp;
			count_gsharp <= 0;
		end
		if (count_a >= delay_a) begin
			sound_a <= !sound_a;
			count_a <= 0;
		end
		if (count_asharp >= delay_asharp) begin
			sound_asharp <= !sound_asharp;
			count_asharp <= 0;
		end
		if (count_b >= delay_b) begin
			sound_b <= !sound_b;
			count_b <= 0;
		end
	end

//wire signed [1:-8] cordic_out_sine, cordic_out_cosine;
//cordic my_cordic(.clk(CLOCK_50), .areset(1'b0), .en(1'b1), .a(cordic_in), .s(cordic_out_sine), .c(cordic_out_cosine));
//assign sound = cordic_out_sine * 32'd10000000;

wire [31:0] c_out = (!c) ? 0 : sound_c ? 32'd100000000 : -32'd100000000;
wire [31:0] csharp_out = (!csharp) ? 0 : sound_csharp ? 32'd100000000 : -32'd100000000;
wire [31:0] d_out = (!d) ? 0 : sound_d ? 32'd100000000 : -32'd100000000;
wire [31:0] dsharp_out = (!dsharp) ? 0 : sound_dsharp ? 32'd100000000 : -32'd100000000;
wire [31:0] e_out = (!e) ? 0 : sound_e ? 32'd100000000 : -32'd100000000;
wire [31:0] f_out = (!f) ? 0 : sound_f ? 32'd100000000 : -32'd100000000;
wire [31:0] fsharp_out = (!fsharp) ? 0 : sound_fsharp ? 32'd100000000 : -32'd100000000;
wire [31:0] g_out = (!g) ? 0 : sound_g ? 32'd100000000 : -32'd100000000;
wire [31:0] gsharp_out = (!gsharp) ? 0 : sound_gsharp ? 32'd100000000 : -32'd100000000;
wire [31:0] a_out = (!a) ? 0 : sound_a ? 32'd100000000 : -32'd100000000;
wire [31:0] asharp_out = (!asharp) ? 0 : sound_asharp ? 32'd100000000 : -32'd100000000;
wire [31:0] b_out = (!b) ? 0 : sound_b ? 32'd100000000 : -32'd100000000;


assign left_channel_audio_out	= out;//c_out + csharp_out + d_out + dsharp_out + e_out + f_out + fsharp_out + g_out + gsharp_out + a_out + asharp_out + b_out;
assign right_channel_audio_out= out;//c_out + csharp_out + d_out + dsharp_out + e_out + f_out + fsharp_out + g_out + gsharp_out + a_out + asharp_out + b_out;
assign write_audio_out = audio_out_allowed;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(1'b0),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT),

);

/*
avconf #(.USE_MIC_INPUT(1)) avc (
	.I2C_SCLK					(I2C_SCLK),
	.I2C_SDAT					(I2C_SDAT),
	.CLOCK_50					(CLOCK_50),
	.reset						(~KEY[0]),
	.key1							(KEY[1]),
	.key2							(KEY[2])
);*/

endmodule

