module vga_controller(iRST_n,
                      iVGA_CLK,
                      oBLANK_n,
                      oHS,
                      oVS,
                      b_data,
                      g_data,
                      r_data,
							 c, csharp, d, dsharp, e, f, fsharp, g, gsharp, a, asharp, b,
							 note_next_in, isPlayAlong, note_pa_out);

	
input iRST_n;
input iVGA_CLK;
input c, csharp, d, dsharp, e, f, fsharp, g, gsharp, a, asharp, b;
input [3:0] note_next_in;
input isPlayAlong;
output reg oBLANK_n;
output reg oHS;
output reg oVS;
output [7:0] b_data;
output [7:0] g_data;  
output [7:0] r_data;     
output note_pa_out;                   
///////// ////                     
reg [18:0] ADDR;
reg [23:0] bgr_data;
reg [3:0] note_next;

initial
	begin
		note_next = 4'b0;
	end

wire VGA_CLK_n;

wire empty, c_pa, csharp_pa, d_pa, dsharp_pa, e_pa, f_pa, fsharp_pa, g_pa, gsharp_pa, a_pa, asharp_pa, b_pa;
wire [7:0] index, index_playalong, index_improvise, index_original;
wire note_playalong;
assign note_pa_out = note_playalong;
mux_16to1_1bit playalong_mux(empty, c, csharp, d, dsharp, e, f, fsharp, g, gsharp, a, asharp, b, 1'b0, 1'b0, 1'b0, note_next[0], note_next[1], note_next[2], note_next[3], note_playalong);
decoder_12 playalong_decoder(note_next, empty, c_pa, a_pa, asharp_pa, b_pa, csharp_pa, d_pa, dsharp_pa, e_pa, f_pa, fsharp_pa, g_pa, gsharp_pa);

wire [23:0] bgr_data_raw;
wire cBLANK_n,cHS,cVS,rst;
////
assign rst = ~iRST_n;
video_sync_generator LTM_ins (.vga_clk(iVGA_CLK),
                              .reset(rst),
                              .blank_n(cBLANK_n),
                              .HS(cHS),
                              .VS(cVS));
////
////Addresss generator
always@(posedge iVGA_CLK,negedge iRST_n)
begin
  if (!iRST_n)
     ADDR<=19'd0;
  else if (cHS==1'b0 && cVS==1'b0)
     ADDR<=19'd0;
  else if (cBLANK_n==1'b1)
     ADDR<=ADDR+1;
end
//////////////////////////
//////INDEX addr.
assign VGA_CLK_n = ~iVGA_CLK;

img_data	img_data_inst (
	.address ( ADDR ),
	.clock ( VGA_CLK_n ),
	.q (index_original )
	);

wire isC, isCSharp, isD, isDSharp, isE, isF, isFSharp, isG, isGSharp, isA, isASharp, isB;

integer x = 44;
integer y = 288;

assign isC = (((ADDR % 640) <= x) || (((ADDR % 640) >= x) && ((ADDR % 640) <= 2*x) && (ADDR/640 >= y)));
assign isCSharp = ((ADDR % 640) >= x) && ((ADDR % 640) <= 110) && (ADDR/640 <= y);
assign isD = ((((ADDR % 640) >= 2*x) && ((ADDR % 640) <= 4*x) && (ADDR/640 >= y)) || ((ADDR%640 <= 154) && (ADDR%640 >= 110)));
assign isDSharp = ((ADDR % 640) >= 154) && ((ADDR % 640) <= 5*x) && (ADDR/640 <= y);
assign isE = ((((ADDR % 640) >= 4*x) && ((ADDR % 640) <= 6*x) && (ADDR/640 >= y)) || ((ADDR%640 <= 6*x) && (ADDR%640 >= 5*x)));
assign isF = ((((ADDR % 640) >= 6*x) && ((ADDR % 640) <= 8*x) && (ADDR/640 >= y)) || ((ADDR%640 <= 7*x) && (ADDR%640 >= 6*x)));
assign isFSharp = ((ADDR % 640) >= 7*x) && ((ADDR % 640) <= 374) && (ADDR/640 <= y);
assign isG = ((((ADDR % 640) >= 8*x) && ((ADDR % 640) <= 10*x) && (ADDR/640 >= y)) || ((ADDR%640 <= 418) && (ADDR%640 >= 374)));
assign isGSharp = ((ADDR % 640) >= 418) && ((ADDR % 640) <= 11*x) && (ADDR/640 <= y);
assign isA = ((((ADDR % 640) >= 10*x) && ((ADDR % 640) <= 550) && (ADDR/640 >= y)) || ((ADDR%640 <= 12*x) && (ADDR%640 >= 11*x)));
assign isASharp = ((ADDR % 640) >= 12*x) && ((ADDR % 640) <= 594) && (ADDR/640 <= y);
assign isB = ((((ADDR % 640) >= 550) && (ADDR/640 >= y)) || (ADDR%640 >= 594));

always @(note_next_in) 
begin
	note_next <= note_next_in;
end

assign index_improvise = ((c & isC) | (csharp & isCSharp) | (d & isD) | (dsharp & isDSharp) | (e & isE) | (f & isF) | (fsharp & isFSharp) | (g & isG) | (gsharp & isGSharp) | (a & isA) | (asharp & isASharp) | (b & isB)) ? 8'h02 : index_original;
assign index_playalong = ((c_pa & isC) | (csharp_pa & isCSharp) | (d_pa & isD) | (dsharp_pa & isDSharp) | (e_pa & isE) | (f_pa & isF) | (fsharp_pa & isFSharp) | (g_pa & isG) | (gsharp_pa & isGSharp) | (a_pa & isA) | (asharp_pa & isASharp) | (b_pa & isB)) ? 8'h03 : index_original;
assign index = (isPlayAlong) ? index_playalong : index_improvise;

/////////////////////////
//////Add switch-input logic here
	
//////Color table output

img_index	img_index_inst (
	.address ( index ),
	.clock ( iVGA_CLK ),
	.q (bgr_data_raw)
	);

//////
//////latch valid data at falling edge;
always@(posedge VGA_CLK_n) bgr_data <= bgr_data_raw;
assign b_data = bgr_data[23:16];
assign g_data = bgr_data[15:8];
assign r_data = bgr_data[7:0]; 
///////////////////
//////Delay the iHD, iVD,iDEN for one clock cycle;
always@(negedge iVGA_CLK)
begin
  oHS<=cHS;
  oVS<=cVS;
  oBLANK_n<=cBLANK_n;
end

endmodule
 	















