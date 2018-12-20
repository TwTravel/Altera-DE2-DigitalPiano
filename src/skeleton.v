`timescale 1 ns / 100 ps
module skeleton(resetn, 
	VGA_CLK,   														//	VGA Clock
	VGA_HS,															//	VGA H_SYNC
	VGA_VS,															//	VGA V_SYNC
	VGA_BLANK,														//	VGA BLANK
	VGA_SYNC,														//	VGA SYNC
	VGA_R,   														//	VGA Red[9:0]
	VGA_G,	 														//	VGA Green[9:0]
	VGA_B,															//	VGA Blue[9:0]
	CLOCK_50,// 50 MHz clock
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
	 c, csharp, d, dsharp, e, f, fsharp, g, gsharp, a, asharp, b, oct_up, oct_down,
	 // SD
	 cs, mosi, miso, sclk, sd_dat_1, sd_dat_2,
	 // modes
	 mode_write_enter, mode_write_exit, mode_read, mode_playalong, init_mode_read,
	 // debugging
	 reset, status,
	 recv_data, CLOCK_IO, wr, rd, mode_write, csharp_in, wb_drain_complete1, wb_drain_complete2);  													
		
	////////////////////////	VGA	////////////////////////////
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK;				//	VGA BLANK
	output			VGA_SYNC;				//	VGA SYNC
	output	[7:0]	VGA_R;   				//	VGA Red[9:0]
	output	[7:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[7:0]	VGA_B;   				//	VGA Blue[9:0]
	input 			c, csharp, d, dsharp, e, f, fsharp, g, gsharp, a, asharp, b;
	input				CLOCK_50;
	input 			resetn;
		
	// Modes ///
	output reg mode_write = 1'b0;
	input mode_write_enter, mode_write_exit, mode_playalong, mode_read, init_mode_read;
	
	always @(posedge CLOCK_IO) begin
		if (~mode_write_enter) begin
			mode_write <= 1'b1;
		end
		else if (~mode_write_exit) begin
			mode_write <= 1'b0;
		end
	end
	
	//// Audio ////
	input				CLOCK_27;
	input				AUD_ADCDAT;
	input 			oct_up, oct_down;

	inout				AUD_BCLK;
	inout				AUD_ADCLRCK;
	inout				AUD_DACLRCK;

	inout				I2C_SDAT;
	output			AUD_XCK;
	output			AUD_DACDAT;

	output			I2C_SCLK;
	
	//// SD ////
	input miso;
	output cs, mosi, sclk;
	output sd_dat_1, sd_dat_2;
	assign sd_dat_1 = 1'b1;
	assign sd_dat_2 = 1'b1;
	
	/* debugging */
	output [4:0] status;
	/* debugging */
	
	/// SD Card ports ///
	output reg rd = 1'b0;
	output reg wr = 1'b0;
	
	input reset;
	wire ready;
	
	wire [7:0] dout;
	wire byte_available, ready_for_next_byte;
	
	reg [7:0] din = 8'b0;
	
	reg [31:0] wr_address, rd_address = 32'b0;
	wire [31:0] address = (wr) ? wr_address : rd_address;
	
	// Audio
	reg [7:0] note_data = 8'b0;
	reg [3:0] note_next = 4'b1;
	
	reg c_read, csharp_read, d_read, dsharp_read, e_read, f_read, fsharp_read, g_read, gsharp_read, a_read, asharp_read, b_read = 1'b0;
	wire c_in = (mode_read) ? c_read : ~c;
	output wire csharp_in = (mode_read) ? csharp_read : ~csharp;
	wire d_in = (mode_read) ? d_read : ~d;
   wire dsharp_in = (mode_read) ? dsharp_read : ~dsharp;
   wire e_in = (mode_read) ? e_read : ~e;
   wire f_in = (mode_read) ? f_read : ~f;
	wire fsharp_in = (mode_read) ? fsharp_read : ~fsharp;
	wire g_in = (mode_read) ? g_read : ~g;
	wire gsharp_in = (mode_read) ? gsharp_read : ~gsharp;
	wire a_in = (mode_read) ? a_read : ~a;
	wire asharp_in = (mode_read) ? asharp_read : ~asharp;
   wire b_in = (mode_read) ? b_read : ~b;	
	
	// SD Read Write
	wire [7:0] wb_out, wb_out1, wb_out2;
	reg [8:0] ADDR_WB_SD, ADDR_WB1, ADDR_WB2 = 8'b0;
	
	reg buffer_select1, buffer_select2 = 1'b0;
	reg wb_ready1, wb_ready2 = 1'b0;
	output reg wb_drain_complete1, wb_drain_complete2 = 1'b0;
	reg [7:0] sd_note_data = 8'b0;
	reg RB_write_enable = 1'b0;
	reg RB_read_enable = 1'b1;
	wire [7:0] buffer_data;
	reg [15:0] ADDR_RB_SD_read = 16'b0; 
	reg [15:0] ADDR_RB_SD_write = 16'b0; 
	wire [15:0] ADDR_RB_SD;
	
	// Clocks
	integer count = 0;
	integer count_read = 0;
	wire CLOCK_10;
   output reg CLOCK_IO = 1'b0;
	reg CLOCK_READ = 1'b0;
	
	always @(posedge CLOCK_50) begin
		count_read = count_read + 1;
		if (count_read >= 8000000) begin
			count_read = 0;
			CLOCK_READ = ~CLOCK_READ;
		end
	end
	
	always @(posedge CLOCK_50) begin
		count = count + 1;
		if (count >= 2000000) begin
			count = 0;
			CLOCK_IO = ~CLOCK_IO;
		end
	end
	pll my_pll(CLOCK_50, CLOCK_10);
	
	// VGA
	wire note_pa_out;
	
	Reset_Delay			r0	(.iCLK(CLOCK_50),.oRESET(DLY_RST)	);
	VGA_Audio_PLL 		p1	(.areset(~DLY_RST),.inclk0(CLOCK_50),.c0(VGA_CTRL_CLK),.c1(AUD_CTRL_CLK),.c2(VGA_CLK)	);
	vga_controller vga_ins(.iRST_n(DLY_RST),
								 .iVGA_CLK(VGA_CLK),
								 .oBLANK_n(VGA_BLANK),
								 .oHS(VGA_HS),
								 .oVS(VGA_VS),
								 .b_data(VGA_B),
								 .g_data(VGA_G),
								 .r_data(VGA_R),
								 .c(c_in), .csharp(csharp_in), .d(d_in), .dsharp(dsharp_in), .e(e_in), .f(f_in), .fsharp(fsharp_in), .g(g_in), .gsharp(gsharp_in), .a(a_in), .asharp(asharp_in), .b(b_in),
								 .note_next_in(note_next), .isPlayAlong(mode_playalong), .note_pa_out(note_pa_out)); 
	
	// Audio
	DE2_Audio_Example my_audio(CLOCK_50, CLOCK_27, AUD_ADCDAT, AUD_BCLK, AUD_ADCLRCK, AUD_DACLRCK, I2C_SDAT, AUD_XCK, AUD_DACDAT, I2C_SCLK, 
		c_in, csharp_in, d_in, dsharp_in, e_in, f_in, fsharp_in, g_in, gsharp_in, a_in, asharp_in, b_in, oct_up, oct_down);
		
	// SD Card
	wire [7:0] recv_data_dummy;
	output [7:0] recv_data;
	assign recv_data = buffer_data;
	
	sd_controller my_sd_controller(cs, // Connect to SD_DAT[3].
	mosi, // Connect to SD_CMD.
	miso, // Connect to SD_DAT[0].
   sclk, // Connect to SD_SCK.
                // For SPI mode, SD_DAT[2] and SD_DAT[1] should be held HIGH. 
                // SD_RESET should be held LOW.
	rd, dout, 
   byte_available, // A new byte has been presented on [dout].
	wr, din, // Data input for WRITE operation.
   ready_for_next_byte, // A new byte should be presented on [din].

   reset, // Resets controller on assertion.
   ready, // HIGH if the SD card is ready for a read or write operation.
   address,   // Memory address for read/write operation. This MUST 
                            // be a multiple of 512 bytes, due to SD sectoring.
   CLOCK_10,  // 25 MHz clock.
   status, // For debug purposes: Current state of controller.
	recv_data_dummy
	);
	
	always @(posedge CLOCK_50) begin
		if (~c) note_data <= 8'd1;
		else if (~csharp) note_data <= 8'd2;
		else if (~d) note_data <= 8'd3;
		else if (~dsharp) note_data <= 8'd4;
		else if (~e) note_data <= 8'd5;
		else if (~f) note_data <= 8'd6;
		else if (~fsharp) note_data <= 8'd7;
		else if (~g) note_data <= 8'd8;
		else if (~gsharp) note_data <= 8'd9;
		else if (~a) note_data <= 8'd10;
		else if (~asharp) note_data <= 8'd11;
		else if (~b) note_data <= 8'd12;
		else note_data <= 8'd0;
	end
	
	/// sd write ///
	wire [8:0] ADDR_WB_after1 = (wb_ready1) ? ADDR_WB_SD : ADDR_WB1;
	wire [8:0] ADDR_WB_after2 = (wb_ready2) ? ADDR_WB_SD : ADDR_WB2;
	
	write_buffer my_write_buffer1(ADDR_WB_after1, CLOCK_50, note_data, buffer_select1, wb_out1);
	write_buffer my_write_buffer2(ADDR_WB_after2, CLOCK_50, note_data, buffer_select2, wb_out2);

	always @(posedge CLOCK_IO) begin
		if (ADDR_WB1 > 510) ADDR_WB1 <= 0;
		else if (buffer_select1) ADDR_WB1 <= ADDR_WB1 + 1;
		if (ADDR_WB2 > 510) ADDR_WB2 <= 0;
		else if (buffer_select2) ADDR_WB2 <= ADDR_WB2 + 1;
	end

	always @(posedge CLOCK_10, negedge resetn) begin
		if (!resetn) begin
			wr <= 1'b0;
			wb_drain_complete1 <= 1'b0;
			wb_drain_complete2 <= 1'b0;
			ADDR_WB_SD <= 9'b0;
			wr_address <= 32'b0;
		end
		else if (wb_ready1) begin
			wr <= 1'b1;
			wb_drain_complete1 <= 1'b0;
			wb_drain_complete2 <= 1'b0;
			
			if (ready_for_next_byte) begin
				ADDR_WB_SD <= ADDR_WB_SD + 1;
				din <= wb_out1;
			end
			
			if (ADDR_WB_SD > 510) begin
				ADDR_WB_SD <= 9'b0;
				wb_drain_complete1 <= 1'b1;
				wr <= 1'b0;
				wr_address <= wr_address + 512;
			end
		
		end
		else if (wb_ready2) begin
			wr <= 1'b1;
			wb_drain_complete1 <= 1'b0;
			wb_drain_complete2 <= 1'b0;
			
			if (ready_for_next_byte) begin
				ADDR_WB_SD <= ADDR_WB_SD + 1;
				din <= wb_out2;
			end
			
			if (ADDR_WB_SD > 510) begin
				ADDR_WB_SD <= 9'b0;
				wb_drain_complete2 <= 1'b1;
				wr <= 1'b0;
				wr_address <= wr_address + 512;
			end
		
		end
	end

	always @(posedge CLOCK_50) begin
		if (!resetn) begin
			wb_ready1 <= 1'b0;
			wb_ready2 <= 1'b0;
			buffer_select1 <= 1'b0;
			buffer_select2 <= 1'b0;
		end
		if (wb_drain_complete1) wb_ready1 <= 1'b0;
		if (wb_drain_complete2) wb_ready2 <= 1'b0;
		if (mode_write && !buffer_select1 && !buffer_select2) begin
			buffer_select1 <= 1'b1;
			buffer_select2 <= 1'b0;
		end
		else if (mode_write && buffer_select1 && (ADDR_WB1 > 510)) begin
			buffer_select1 <= 1'b0;
			buffer_select2 <= 1'b1;
			wb_ready1 <= 1'b1;
			wb_ready2 <= 1'b0;
		end
		else if (mode_write && buffer_select2 && (ADDR_WB2 > 510)) begin
			buffer_select1 <= 1'b1;
			buffer_select2 <= 1'b0;
			wb_ready1 <= 1'b0;
			wb_ready2 <= 1'b1;
		end
		else if (!mode_write && buffer_select1 && !buffer_select2) begin
			buffer_select1 <= 1'b0;
			buffer_select2 <= 1'b0;
			wb_ready1 <= 1'b1;
			wb_ready2 <= 1'b0;
		end
		else if (!mode_write && !buffer_select1 && buffer_select2) begin
			buffer_select1 <= 1'b0;
			buffer_select2 <= 1'b0;
			wb_ready1 <= 1'b0;
			wb_ready2 <= 1'b1;
		end
		
	end
	
	// SD Read //
	always @(posedge CLOCK_50) begin
		rd <= init_mode_read; 
		RB_read_enable <= mode_read;
	end
	
	assign ADDR_RB_SD = (mode_read) ? ADDR_RB_SD_read : ADDR_RB_SD_write; 
	
	// convert data from dmem to note signals 
	always @(posedge CLOCK_50) begin
		if (buffer_data == 1) begin
			c_read <= 1'b1;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 2) begin
			csharp_read <= 1'b1;
			c_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 3) begin
			d_read <= 1'b1;
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 4) begin
			dsharp_read <= 1'b1;
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 5) begin
			e_read <= 1'b1;
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 6) begin 
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b1;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 7) begin
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b1;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 8) begin
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b1;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 9) begin
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b1;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 10) begin
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b1;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
		else if (buffer_data == 11) begin
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b1;
			b_read <= 1'b0;
		end
		else if (buffer_data == 12) begin
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b1;
		end
		else begin 
			c_read <= 1'b0;
			csharp_read <= 1'b0;
			d_read <= 1'b0;
			dsharp_read <= 1'b0;
			e_read <= 1'b0;
			f_read <= 1'b0;
			fsharp_read <= 1'b0;
			g_read <= 1'b0;
			gsharp_read <= 1'b0;
			a_read <= 1'b0;
			asharp_read <= 1'b0;
			b_read <= 1'b0;
		end
	end
	
	// read note out from dmem 
	always @(posedge CLOCK_READ) begin
		if (mode_read) begin
			ADDR_RB_SD_read <= ADDR_RB_SD_read + 1; 
		end
		else begin 
			ADDR_RB_SD_read <= 16'd0;
		end
	end
	// init_mode_read - reads the data from SD card to read_buffer // 
	// mode_read - plays the notes according to the read_buffer // 
	
	always @(posedge CLOCK_10) begin
		if (init_mode_read) begin
			if (byte_available) begin
				RB_write_enable <= 1'b1;
				sd_note_data <= dout;
				if (ADDR_RB_SD_write < 4096) ADDR_RB_SD_write <= ADDR_RB_SD_write + 1;
				if (ADDR_RB_SD_write % 512 > 510) begin
					//if (rd_address < 1024 && ADDR_RB_SD_write < 1024) rd_address <= rd_address + 512;
				end
			end
			
		end
		else begin
			ADDR_RB_SD_write <= 16'd0; 
			rd_address <= 32'd0;
			RB_write_enable <= 1'b0;
		end
	end
	
	read_buffer my_read_buffer (ADDR_RB_SD, CLOCK_50, sd_note_data, mode_read, RB_write_enable, buffer_data); 
	
	// playalong //
		
	always @(posedge CLOCK_50) 
	begin
		if (note_pa_out) begin
			if (note_next == 1) note_next <= 3;
			else if (note_next == 3) note_next <= 5;
			else if (note_next == 5) note_next <= 6;
			else if (note_next == 6) note_next <= 8;
			else if (note_next == 8) note_next <= 10;
			else if (note_next == 10) note_next <= 12;
			else note_next <= 1;
		end
	end
	
endmodule
