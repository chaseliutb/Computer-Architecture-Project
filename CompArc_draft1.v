///////////////////////////////////////////
// LRU IMPLEMENTATION
///////////////////////////////////////////

module D_ff_LRU(input d, output reg q );
always@(d)
begin
 q=d; 
end
endmodule

module decoder3to8_LRU(input [2:0] in, output reg [7:0] out);
  always@(in)
    case(in)
      3'b000:  out = 8'b00000001;
      3'b001:  out = 8'b00000010;
      3'b010:  out = 8'b00000100;
      3'b011:  out = 8'b00001000;
      3'b100:  out = 8'b00010000;
      3'b101:  out = 8'b00100000;
      3'b110:  out = 8'b01000000;
      3'b111:  out = 8'b10000000;
    endcase
endmodule

module mux2to1_3bit_LRU(input [2:0] in1, input [2:0] in2, input sel, output reg [2:0] out);
  always@(in1,in2,sel)
    case(sel)
      1'b0:  out = in1;
      1'b1:  out = in2;
    endcase
endmodule

module mux8to1_3bit_LRU(input [2:0] in0, input [2:0] in1, input [2:0] in2, input [2:0] in3, input [2:0] in4,
                    input [2:0] in5, input [2:0] in6, input [2:0] in7,
                    input [2:0] sel, output reg [2:0] out);
  always@(in1,in2,in3,in0,in4,in5,in6,in7,sel)
    begin
      case(sel)
        3'b000 : out = in0;
        3'b001 : out = in1;
        3'b010 : out = in2;
        3'b011 : out = in3;
        3'b100 : out = in4;
        3'b101 : out = in5;
        3'b110 : out = in6;
        3'b111 : out = in7;
      endcase
    end
endmodule

module counter_LRU(input clk, input reset,input writeEnable, input dec, input load, output [2:0] val);
  reg[2:0] temp;
  reg[1:0] three;

  D_ff_LRU d0 (temp[0],val[0]);
  D_ff_LRU d1 (temp[1],val[1]);
  D_ff_LRU d2 (temp[2],val[2]);

  always@(clk)
  begin
    if(reset)
      temp=3'b000;
    else if(load&writeEnable)
      temp=3'b111;
    else if (dec&writeEnable)
      temp=temp-3'b001;
  end
endmodule

module priority_encoder_LRU (input [7:0] sel, output reg [2:0] code);  
  
 
  always @(sel)  
  begin  
        if (sel[0]) code = 3'b000;  
   else if (sel[1]) code = 3'b001;  
   else if (sel[2]) code = 3'b010;  
   else if (sel[3]) code = 3'b011;  
   else if (sel[4]) code = 3'b100;  
   else if (sel[5]) code = 3'b101;  
   else if (sel[6]) code = 3'b110;  
   else if (sel[7]) code = 3'b111;  
   else code = 3'b000;  
  end  
endmodule 

module comparator_LRU(input[2:0] in1,input [2:0] in2, output reg dec, output reg ifzero);
  always @ (in1,in2)
  begin
  if(in1>in2)
    dec=1'b1;
  else
    dec=1'b0;
  if(in1==3'b000)
   ifzero=1'b1;
   else
   ifzero=1'b0;
end

endmodule

module LRU_Counter(input clk, input reset, input hit,input writeEnable, input [2:0] lineIndex, output [2:0] outIndex);
  
  wire[2:0] dec_in;
  wire[7:0] dec_out,comp_out,ifzero;
  wire[2:0] out0,out1,out2,out3,out4,out5,out6,out7;
  wire[2:0] comp_in;
  
  mux2to1_3bit_LRU m1(outIndex,lineIndex, hit, dec_in);
  
  decoder3to8_LRU d1(dec_in, dec_out);
  
  counter_LRU c0(clk,reset, writeEnable, comp_out[0]&(~dec_out[0]),dec_out[0],out0);
  counter_LRU c1(clk,reset, writeEnable, comp_out[1]&(~dec_out[1]),dec_out[1],out1);
  counter_LRU c2(clk,reset, writeEnable, comp_out[2]&(~dec_out[2]),dec_out[2],out2);
  counter_LRU c3(clk,reset, writeEnable, comp_out[3]&(~dec_out[3]),dec_out[3],out3);
  counter_LRU c4(clk,reset, writeEnable, comp_out[4]&(~dec_out[4]),dec_out[4],out4);
  counter_LRU c5(clk,reset, writeEnable, comp_out[5]&(~dec_out[5]),dec_out[5],out5);
  counter_LRU c6(clk,reset, writeEnable, comp_out[6]&(~dec_out[6]),dec_out[6],out6);
  counter_LRU c7(clk,reset, writeEnable, comp_out[7]&(~dec_out[7]),dec_out[7],out7);
  
  mux8to1_3bit_LRU m2(out0,out1,out2,out3,out4,out5,out6,out7,dec_in,comp_in);
  
  comparator_LRU cc0(out0,comp_in,comp_out[0],ifzero[0]);
  comparator_LRU cc1(out1,comp_in,comp_out[1],ifzero[1]);
  comparator_LRU cc2(out2,comp_in,comp_out[2],ifzero[2]);
  comparator_LRU cc3(out3,comp_in,comp_out[3],ifzero[3]);
  comparator_LRU cc4(out4,comp_in,comp_out[4],ifzero[4]);
  comparator_LRU cc5(out5,comp_in,comp_out[5],ifzero[5]);
  comparator_LRU cc6(out6,comp_in,comp_out[6],ifzero[6]);
  comparator_LRU cc7(out7,comp_in,comp_out[7],ifzero[7]);
  
  priority_encoder_LRU pe(ifzero, outIndex);

endmodule

  
//////////////////////////////////////////  
//CACHE IMPLEMENTATION 
//////////////////////////////////////////  
  
module D_FF(input clk, input reset, input write, input d, output reg q);
  always @(negedge clk) 
  if(reset) q=0;
  else
   if(write) q=d;
   endmodule
   
module mux2to1_1b(input in1,input in2, input sel,output reg tagOut);
  always @(in1,in2,sel)
  begin
    case(sel)
      1'b0:tagOut=in1;
      1'b1:tagOut=in2;
    endcase
  end
endmodule

module mux2to1_3b(input [2:0] in1, input [2:0] in2, input sel,output reg [2:0] tagOut);
  always @(in1,in2,sel)
  begin
    case(sel)
      1'b0:tagOut=in1;
      1'b1:tagOut=in2;
    endcase
  end
endmodule

module mux2to1_128b(input [127:0] in1, input [127:0] in2, input sel,output reg [127:0] tagOut);
  always @(in1,in2,sel)
  begin
    case(sel)
      1'b0:tagOut=in1;
      1'b1:tagOut=in2;
    endcase
  end
endmodule

module mux4to1_1b(input in1,input in2, input in3,input in4,input[1:0] sel,output reg tagOut);
  always @(in1,in2,in3,in4,sel)
  begin
    case(sel)
      2'b00:tagOut=in1;
      2'b01:tagOut=in2;
      2'b10:tagOut=in3;
      2'b11:tagOut=in4;
    endcase
  end
endmodule

module mux4to1_3b(input[2:0] in1, in2, in3, in4,input[1:0] sel,output reg [2:0] tagOut);
  always @(in1,in2,in3,in4,sel)
  begin
    case(sel)
      2'b00:tagOut=in1;
      2'b01:tagOut=in2;
      2'b10:tagOut=in3;
      2'b11:tagOut=in4;
    endcase
  end
endmodule


module mux4to1_16b(input[15:0] in1, in2, in3, in4,input[1:0] sel,output reg[15:0] tagOut);
  always @(in1,in2,in3,in4,sel)
  begin
    case(sel)
      2'b00:tagOut=in1;
      2'b01:tagOut=in2;
      2'b10:tagOut=in3;
      2'b11:tagOut=in4;
    endcase
  end
endmodule

module mux4to1_26b(input [25:0] in1,input [25:0] in2, input [25:0] in3, input [25:0] in4, input[1:0] sel,output reg[25:0] tagOut);
  always @(in1,in2,in3,in4,sel)
  begin
    case(sel)
      2'b00:tagOut=in1;
      2'b01:tagOut=in2;
      2'b10:tagOut=in3;
      2'b11:tagOut=in4;
    endcase
  end
endmodule

module mux4to1_128b(input [127:0] in1,input [127:0] in2,input [127:0]  in3,input [127:0] in4,input[1:0] sel,output reg[127:0] tagOut);
  always @(in1,in2,in3,in4,sel)
  begin
    case(sel)
      2'b00:tagOut=in1;
      2'b01:tagOut=in2;
      2'b10:tagOut=in3;
      2'b11:tagOut=in4;
    endcase
  end
endmodule

module mux8to1_1b(input in1, input in2, input in3,input in4, input in5, input in6, input in7, input in8 ,input[2:0] sel,output reg tagOut);
  always @(in1,in2,in3,in4,in5,in6,in7,in8,sel)
  begin
    case(sel)
      3'b000:tagOut=in1;
      3'b001:tagOut=in2;
      3'b010:tagOut=in3;
      3'b011:tagOut=in4;
      3'b100:tagOut=in5;
      3'b101:tagOut=in6;
      3'b110:tagOut=in7;
      3'b111:tagOut=in8;  
    endcase
  end
endmodule

module mux8to1_128b(input [127:0] in1,  in2,  in3, in4,  in5,  in6,  in7,  in8 ,input[2:0] sel,output reg[127:0] tagOut);
  always @(in1,in2,in3,in4,in5,in6,in7,in8,sel)
  begin
    case(sel)
      3'b000:tagOut=in1;
      3'b001:tagOut=in2;
      3'b010:tagOut=in3;
      3'b011:tagOut=in4;
      3'b100:tagOut=in5;
      3'b101:tagOut=in6;
      3'b110:tagOut=in7;
      3'b111:tagOut=in8;  
    endcase
  end
endmodule

module mux16to1_8b(input [7:0] in1, in2, in3, in4, in5, in6, in7, in8, in9, in10, in11, in12, in13, in14, in15, in16, input[3:0] sel, output reg[7:0] tagOut);
  always @(in1,in2,in3,in4,in5,in6,in7,in8,sel)
  begin
    case(sel)
      4'b0000:tagOut=in1;
      4'b0001:tagOut=in2;
      4'b0010:tagOut=in3;
      4'b0011:tagOut=in4;
      4'b0100:tagOut=in5;
      4'b0101:tagOut=in6;
      4'b0110:tagOut=in7;
      4'b0111:tagOut=in8;  
      4'b1000:tagOut=in9;
      4'b1001:tagOut=in10;
      4'b1010:tagOut=in11;
      4'b1011:tagOut=in12;
      4'b1100:tagOut=in13;
      4'b1101:tagOut=in14;
      4'b1110:tagOut=in15;
      4'b1111:tagOut=in16; 
    endcase
  end
endmodule

module comparator4bit(input [3:0] in1, input [3:0] in2,output reg compOut);
  always @(in1,in2)
  begin      
    if(in1==in2)
      compOut=1'b1;
    else
        compOut=1'b0;
   end
endmodule
  
module comparator26bit(input [25:0] in1, input [25:0] in2,output reg compOut);
  always @(in1,in2)
  begin      
    if(in1==in2)
      compOut=1'b1;
    else
        compOut=1'b0;
    end
  endmodule

module decoder2to4(input [1:0] in,output reg [3:0] decOut);
  always @(in)
  begin
    case(in)
      2'b00:decOut=4'b0001;
      2'b01:decOut=4'b0010;
      2'b10:decOut=4'b0100;
      2'b11:decOut=4'b1000;  
      
    endcase
  end
endmodule

module decoder3to8(input [2:0] way,output reg [7:0] wayOut);
  always @(way)
  begin
    case(way)
      3'b000:wayOut=8'b00000001;
      3'b001:wayOut=8'b00000010;
      3'b010:wayOut=8'b00000100;
      3'b011:wayOut=8'b00001000;
      3'b100:wayOut=8'b00010000;
      3'b101:wayOut=8'b00100000;
      3'b110:wayOut=8'b01000000;
      3'b111:wayOut=8'b10000000; 
      
    endcase
  end
endmodule

module decoder4to16(input [3:0] way,output reg [15:0] wayOut);
  always @(way)
  begin
    case(way)
      4'b0000:wayOut=16'h0001;
      4'b0001:wayOut=16'h0002;
      4'b0010:wayOut=16'h0004;
      4'b0011:wayOut=16'h0008;
      4'b0100:wayOut=16'h0010;
      4'b0101:wayOut=16'h0020;
      4'b0110:wayOut=16'h0040;
      4'b0111:wayOut=16'h0080; 
      4'b1000:wayOut=16'h0100;
      4'b1001:wayOut=16'h0200;
      4'b1010:wayOut=16'h0400;
      4'b1011:wayOut=16'h0800;
      4'b1100:wayOut=16'h1000;
      4'b1101:wayOut=16'h2000;
      4'b1110:wayOut=16'h4000;
      4'b1111:wayOut=16'h8000; 
    endcase
  end
endmodule

module encoder8to3(input [7:0] encoder_in, output reg[2:0]binary_out);
	always @ (encoder_in)
	begin
    	case (encoder_in) 
  		8'b00000001 : binary_out = 3'b000;
		8'b00000010 : binary_out = 3'b001;
		8'b00000100 : binary_out = 3'b010;
		8'b00001000 : binary_out = 3'b011;
		8'b00010000 : binary_out = 3'b100;
		8'b00100000 : binary_out = 3'b101;
		8'b01000000 : binary_out = 3'b110;
		8'b10000000 : binary_out = 3'b111;
   endcase
  end

endmodule


module dataByte(input clk, input reset, input writeEnable, input [7:0] in,output [7:0] out);    
    D_FF d0(clk,reset, writeEnable, in[0], out[0]);
    D_FF d1(clk,reset, writeEnable, in[1], out[1]);
    D_FF d2(clk,reset, writeEnable, in[2], out[2]);
    D_FF d3(clk,reset, writeEnable, in[3], out[3]);
    D_FF d4(clk,reset, writeEnable, in[4], out[4]);
    D_FF d5(clk,reset, writeEnable, in[5], out[5]);
    D_FF d6(clk,reset, writeEnable, in[6], out[6]);
    D_FF d7(clk,reset, writeEnable, in[7], out[7]);
 endmodule

module dataBlock(input clk, input reset, input[15:0] write, input[7:0] in0, input[7:0] in1, input[7:0] in2, input[7:0] in3,
input[7:0] in4, input[7:0] in5, input[7:0] in6, input[7:0] in7,  input[7:0] in8, input[7:0] in9, input[7:0] in10, input[7:0] in11,
input[7:0] in12, input[7:0] in13, input[7:0] in14, input[7:0] in15, output[7:0] out0, output[7:0] out1, output[7:0] out2, output[7:0] out3, 
output[7:0] out4, output[7:0] out5, output[7:0] out6, output[7:0] out7, output[7:0] out8, output[7:0] out9, output[7:0] out10, output[7:0] out11, 
output[7:0] out12, output[7:0] out13, output[7:0] out14, output[7:0] out15);

dataByte db0(clk,reset,write[0], in0, out0);
dataByte db1(clk,reset,write[1], in1, out1);
dataByte db2(clk,reset,write[2], in2, out2);
dataByte db3(clk,reset,write[3], in3, out3);
dataByte db4(clk,reset,write[4], in4, out4);
dataByte db5(clk,reset,write[5], in5, out5);
dataByte db6(clk,reset,write[6], in6, out6);
dataByte db7(clk,reset,write[7], in7, out7);
dataByte db8(clk,reset,write[8], in8, out8);
dataByte db9(clk,reset,write[9], in9, out9);
dataByte db10(clk,reset,write[10], in10, out10);
dataByte db11(clk,reset,write[11], in11, out11);
dataByte db12(clk,reset,write[12], in12, out12);
dataByte db13(clk,reset,write[13], in13, out13);
dataByte db14(clk,reset,write[14], in14, out14);
dataByte db15(clk,reset,write[15], in15, out15);
endmodule

module dataArray(input clk, input reset, input[63:0] writeEnable, input[127:0] in_dataBlock, 
                 output[127:0] out_dataBlock0, out_dataBlock1, out_dataBlock2, out_dataBlock3);

 dataBlock db0( clk,  reset,  writeEnable[15:0],  
 in_dataBlock[7:0],  in_dataBlock[15:8] ,  in_dataBlock[23:16],  in_dataBlock[31:24],
 in_dataBlock[39:32],  in_dataBlock[47:40],  in_dataBlock[55:48],  in_dataBlock[63:56], in_dataBlock[71:64], in_dataBlock[79:72], 
 in_dataBlock[87:80], in_dataBlock[95:88], in_dataBlock[103:96], in_dataBlock[111:104], in_dataBlock[119:112], in_dataBlock[127:120],
 out_dataBlock0[7:0],  out_dataBlock0[15:8] ,  out_dataBlock0[23:16],  out_dataBlock0[31:24], 
 out_dataBlock0[39:32], out_dataBlock0[47:40],  out_dataBlock0[55:48],  out_dataBlock0[63:56], out_dataBlock0[71:64], out_dataBlock0[79:72], 
 out_dataBlock0[87:80], out_dataBlock0[95:88], out_dataBlock0[103:96], out_dataBlock0[111:104], out_dataBlock0[119:112], out_dataBlock0[127:120]); 
 
 dataBlock db1( clk,  reset,  writeEnable[31:16],  
 in_dataBlock[7:0],  in_dataBlock[15:8] ,  in_dataBlock[23:16],  in_dataBlock[31:24],
 in_dataBlock[39:32],  in_dataBlock[47:40],  in_dataBlock[55:48],  in_dataBlock[63:56], in_dataBlock[71:64], in_dataBlock[79:72], 
 in_dataBlock[87:80], in_dataBlock[95:88], in_dataBlock[103:96], in_dataBlock[111:104], in_dataBlock[119:112], in_dataBlock[127:120],
 out_dataBlock1[7:0],  out_dataBlock1[15:8] ,  out_dataBlock1[23:16],  out_dataBlock1[31:24], 
 out_dataBlock1[39:32], out_dataBlock1[47:40],  out_dataBlock1[55:48],  out_dataBlock1[63:56], out_dataBlock1[71:64], out_dataBlock1[79:72], 
 out_dataBlock1[87:80], out_dataBlock1[95:88], out_dataBlock1[103:96], out_dataBlock1[111:104], out_dataBlock1[119:112], out_dataBlock1[127:120]); 
 
 dataBlock db2( clk,  reset,  writeEnable[47:32],  
 in_dataBlock[7:0],  in_dataBlock[15:8] ,  in_dataBlock[23:16],  in_dataBlock[31:24],
 in_dataBlock[39:32],  in_dataBlock[47:40],  in_dataBlock[55:48],  in_dataBlock[63:56], in_dataBlock[71:64], in_dataBlock[79:72], 
 in_dataBlock[87:80], in_dataBlock[95:88], in_dataBlock[103:96], in_dataBlock[111:104], in_dataBlock[119:112], in_dataBlock[127:120],
 out_dataBlock2[7:0],  out_dataBlock2[15:8] ,  out_dataBlock2[23:16],  out_dataBlock2[31:24], 
 out_dataBlock2[39:32], out_dataBlock2[47:40],  out_dataBlock2[55:48],  out_dataBlock2[63:56], out_dataBlock2[71:64], out_dataBlock2[79:72], 
 out_dataBlock2[87:80], out_dataBlock2[95:88], out_dataBlock2[103:96], out_dataBlock2[111:104], out_dataBlock2[119:112], out_dataBlock2[127:120]);  
 
  dataBlock db3( clk,  reset,  writeEnable[63:48],  
 in_dataBlock[7:0],  in_dataBlock[15:8] ,  in_dataBlock[23:16],  in_dataBlock[31:24],
 in_dataBlock[39:32],  in_dataBlock[47:40],  in_dataBlock[55:48],  in_dataBlock[63:56], in_dataBlock[71:64], in_dataBlock[79:72], 
 in_dataBlock[87:80], in_dataBlock[95:88], in_dataBlock[103:96], in_dataBlock[111:104], in_dataBlock[119:112], in_dataBlock[127:120],
 out_dataBlock3[7:0],  out_dataBlock3[15:8] ,  out_dataBlock3[23:16],  out_dataBlock3[31:24], 
 out_dataBlock3[39:32], out_dataBlock3[47:40],  out_dataBlock3[55:48],  out_dataBlock3[63:56], out_dataBlock3[71:64], out_dataBlock3[79:72], 
 out_dataBlock3[87:80], out_dataBlock3[95:88], out_dataBlock3[103:96], out_dataBlock3[111:104], out_dataBlock3[119:112], out_dataBlock3[127:120]); 
 
endmodule

module tagBlock(input clk, input reset, input write, input [21:0] tag ,output [21:0] tagData);
    D_FF d8(clk,reset, write, tag[0], tagData[0]);
    D_FF d9(clk,reset, write, tag[1], tagData[1]);
    D_FF d10(clk,reset, write, tag[2], tagData[2]);
    D_FF d11(clk,reset, write, tag[3], tagData[3]);
    D_FF d12(clk,reset, write, tag[4], tagData[4]);
    D_FF d13(clk,reset, write, tag[5], tagData[5]);
    D_FF d14(clk,reset, write, tag[6], tagData[6]);
    D_FF d15(clk,reset, write, tag[7], tagData[7]);
    D_FF d16(clk,reset, write, tag[8], tagData[8]);
    D_FF d17(clk,reset, write, tag[9], tagData[9]);
    D_FF d18(clk,reset, write, tag[10], tagData[10]);
    D_FF d19(clk,reset, write, tag[11], tagData[11]);
    D_FF d20(clk,reset, write, tag[12], tagData[12]);
    D_FF d21(clk,reset, write, tag[13], tagData[13]);
    D_FF d22(clk,reset, write, tag[14], tagData[14]);
    D_FF d23(clk,reset, write, tag[15], tagData[15]);
    D_FF d24(clk,reset, write, tag[16], tagData[16]);
    D_FF d25(clk,reset, write, tag[17], tagData[17]);
    D_FF d26(clk,reset, write, tag[18], tagData[18]);
    D_FF d27(clk,reset, write, tag[19], tagData[19]);
    D_FF d28(clk,reset, write, tag[20], tagData[20]);
    D_FF d29(clk,reset, write, tag[21], tagData[21]);
 endmodule


module tagArray(input clk, input reset, input [3:0] we, input [21:0] tagin, 
output [21:0] tagOut0, output [21:0] tagOut1, output [21:0] tagOut2, output [21:0] tagOut3);
   
    tagBlock tb0(clk,reset, we[0], tagin, tagOut0);
    tagBlock tb1(clk,reset, we[1], tagin, tagOut1);
    tagBlock tb2(clk,reset, we[2], tagin, tagOut2);
    tagBlock tb3(clk,reset, we[3], tagin, tagOut3);
    
  endmodule
         
module vaildArray(input clk, input reset, input [3:0] we, input validBit,
    output validOut0, output validOut1, output validOut2, output validOut3); 
    
    D_FF d30(clk,reset,we[0],validBit,validOut0);
    D_FF d31(clk,reset,we[1],validBit,validOut1);
    D_FF d32(clk,reset,we[2],validBit,validOut2);
    D_FF d33(clk,reset,we[3],validBit,validOut3);
    
  endmodule

module haltTagBlock(input clk, input reset, input write, input [3:0] tag,output [3:0] halttagData);
    
    D_FF d34(clk,reset, write, tag[0], halttagData[0]);
    D_FF d35(clk,reset, write, tag[1], halttagData[1]);
    D_FF d36(clk,reset, write, tag[2], halttagData[2]);
    D_FF d37(clk,reset, write, tag[3], halttagData[3]);
endmodule

module haltTagArray(input clk, input reset, input [3:0] we, input [3:0] htagin,
 output [3:0] halttagOut0, output [3:0] halttagOut1, output [3:0] halttagOut2, output [3:0] halttagOut3);
   
    haltTagBlock htb0(clk,reset, we[0], htagin, halttagOut0);
    haltTagBlock htb1(clk,reset, we[1], htagin, halttagOut1);
    haltTagBlock htb2(clk,reset, we[2], htagin, halttagOut2);
    haltTagBlock htb3(clk,reset, we[3], htagin, halttagOut3); 
endmodule


module dirtyArray(input clk, input reset, input [3:0] we, input dirtyBit,
    output dirtyOut0, output dirtyOut1, output dirtyOut2, output dirtyOut3); 

    D_FF d38(clk,reset,we[0],dirtyBit,dirtyOut0);
    D_FF d39(clk,reset,we[1],dirtyBit,dirtyOut1);
    D_FF d40(clk,reset,we[2],dirtyBit,dirtyOut2);
    D_FF d41(clk,reset,we[3],dirtyBit,dirtyOut3);
endmodule

/////////////////////////////////////////
// DIRTY BIT BLOCK
/////////////////////////////////////////
module get_setDirtyBit(input clk, input reset, input [2:0] WayNo, input [7:0] wayOut, 
                       input [1:0] index, input [3:0] indexDecoded, input DB_data, input DB_signal, 
                       output dirtyBit);
  
  wire [7:0] dirtyOut;
  wire [3:0] dirtyEnable0,dirtyEnable1,dirtyEnable2,dirtyEnable3,dirtyEnable4,dirtyEnable5,dirtyEnable6,dirtyEnable7;
  wire [3:0] dirtyOut0,dirtyOut1,dirtyOut2,dirtyOut3,dirtyOut4,dirtyOut5,dirtyOut6,dirtyOut7; 
    
  assign dirtyEnable0 = {wayOut[0]&indexDecoded[3]&DB_signal, wayOut[0]&indexDecoded[2]&DB_signal, wayOut[0]&indexDecoded[1]&DB_signal, wayOut[0]&indexDecoded[0]&DB_signal};
  assign dirtyEnable1 = {wayOut[1]&indexDecoded[3]&DB_signal, wayOut[1]&indexDecoded[2]&DB_signal, wayOut[1]&indexDecoded[1]&DB_signal, wayOut[1]&indexDecoded[0]&DB_signal};
  assign dirtyEnable2 = {wayOut[2]&indexDecoded[3]&DB_signal, wayOut[2]&indexDecoded[2]&DB_signal, wayOut[2]&indexDecoded[1]&DB_signal, wayOut[2]&indexDecoded[0]&DB_signal};
  assign dirtyEnable3 = {wayOut[3]&indexDecoded[3]&DB_signal, wayOut[3]&indexDecoded[2]&DB_signal, wayOut[3]&indexDecoded[1]&DB_signal, wayOut[3]&indexDecoded[0]&DB_signal};
  assign dirtyEnable4 = {wayOut[4]&indexDecoded[3]&DB_signal, wayOut[4]&indexDecoded[2]&DB_signal, wayOut[4]&indexDecoded[1]&DB_signal, wayOut[4]&indexDecoded[0]&DB_signal};
  assign dirtyEnable5 = {wayOut[5]&indexDecoded[3]&DB_signal, wayOut[5]&indexDecoded[2]&DB_signal, wayOut[5]&indexDecoded[1]&DB_signal, wayOut[5]&indexDecoded[0]&DB_signal};
  assign dirtyEnable6 = {wayOut[6]&indexDecoded[3]&DB_signal, wayOut[6]&indexDecoded[2]&DB_signal, wayOut[6]&indexDecoded[1]&DB_signal, wayOut[6]&indexDecoded[0]&DB_signal};
  assign dirtyEnable7 = {wayOut[7]&indexDecoded[3]&DB_signal, wayOut[7]&indexDecoded[2]&DB_signal, wayOut[7]&indexDecoded[1]&DB_signal, wayOut[7]&indexDecoded[0]&DB_signal};
  
  dirtyArray DA0(clk, reset, dirtyEnable0, DB_data, dirtyOut0[0],dirtyOut0[1],dirtyOut0[2],dirtyOut0[3]);
  dirtyArray DA1(clk, reset, dirtyEnable1, DB_data, dirtyOut1[0],dirtyOut1[1],dirtyOut1[2],dirtyOut1[3]);
  dirtyArray DA2(clk, reset, dirtyEnable2, DB_data, dirtyOut2[0],dirtyOut2[1],dirtyOut2[2],dirtyOut2[3]);
  dirtyArray DA3(clk, reset, dirtyEnable3, DB_data, dirtyOut3[0],dirtyOut3[1],dirtyOut3[2],dirtyOut3[3]);
  dirtyArray DA4(clk, reset, dirtyEnable4, DB_data, dirtyOut4[0],dirtyOut4[1],dirtyOut4[2],dirtyOut4[3]);
  dirtyArray DA5(clk, reset, dirtyEnable5, DB_data, dirtyOut5[0],dirtyOut5[1],dirtyOut5[2],dirtyOut5[3]);
  dirtyArray DA6(clk, reset, dirtyEnable6, DB_data, dirtyOut6[0],dirtyOut6[1],dirtyOut6[2],dirtyOut6[3]);
  dirtyArray DA7(clk, reset, dirtyEnable7, DB_data, dirtyOut7[0],dirtyOut7[1],dirtyOut7[2],dirtyOut7[3]);
  
  mux4to1_1b setSelectM0(dirtyOut0[0],dirtyOut0[1],dirtyOut0[2],dirtyOut0[3],index,dirtyOut[0]);
  mux4to1_1b setSelectM1(dirtyOut1[0],dirtyOut1[1],dirtyOut1[2],dirtyOut1[3],index,dirtyOut[1]);
  mux4to1_1b setSelectM2(dirtyOut2[0],dirtyOut2[1],dirtyOut2[2],dirtyOut2[3],index,dirtyOut[2]);
  mux4to1_1b setSelectM3(dirtyOut3[0],dirtyOut3[1],dirtyOut3[2],dirtyOut3[3],index,dirtyOut[3]);
  mux4to1_1b setSelectM4(dirtyOut4[0],dirtyOut4[1],dirtyOut4[2],dirtyOut4[3],index,dirtyOut[4]);
  mux4to1_1b setSelectM5(dirtyOut5[0],dirtyOut5[1],dirtyOut5[2],dirtyOut5[3],index,dirtyOut[5]);
  mux4to1_1b setSelectM6(dirtyOut6[0],dirtyOut6[1],dirtyOut6[2],dirtyOut6[3],index,dirtyOut[6]);
  mux4to1_1b setSelectM7(dirtyOut7[0],dirtyOut7[1],dirtyOut7[2],dirtyOut7[3],index,dirtyOut[7]);  
  
  mux8to1_1b waySelectM0(dirtyOut[0],dirtyOut[1], dirtyOut[2], dirtyOut[3], dirtyOut[4], dirtyOut[5], dirtyOut[6], dirtyOut[7], WayNo, dirtyBit);

endmodule

///////////////////////////////////////////
// CACHE DATA 
///////////////////////////////////////////
module get_setData(input clk, input reset, input [2:0] WayNo, input [7:0] wayOut, input [1:0] index, 
                   input [3:0] indexDecoded, input [127:0] blockData,  
                   input [15:0] writeEnable, input[3:0] offset, 
                   output [127:0] outBlockData, output [7:0] outByteData);
  
  wire [63:0] writeEnable0, writeEnable1, writeEnable2, writeEnable3, writeEnable4, writeEnable5, writeEnable6, writeEnable7;
  wire [511:0] outBlockData0, outBlockData1, outBlockData2, outBlockData3, outBlockData4, outBlockData5, outBlockData6, outBlockData7;
  wire [127:0] blockData0, blockData1, blockData2, blockData3, blockData4, blockData5, blockData6, blockData7;
  
  assign writeEnable0 = {({16{wayOut[0]}} & {16{indexDecoded[0]}} & {writeEnable}),({16{wayOut[0]}} & {16{indexDecoded[1]}} & {writeEnable}),({16{wayOut[0]}} & {16{indexDecoded[2]}} & {writeEnable}),({16{wayOut[0]}} & {16{indexDecoded[3]}} & {writeEnable})};  
  assign writeEnable1 = {({16{wayOut[1]}} & {16{indexDecoded[0]}} & {writeEnable}),({16{wayOut[1]}} & {16{indexDecoded[1]}} & {writeEnable}),({16{wayOut[1]}} & {16{indexDecoded[2]}} & {writeEnable}),({16{wayOut[1]}} & {16{indexDecoded[3]}} & {writeEnable})};
  assign writeEnable2 = {({16{wayOut[2]}} & {16{indexDecoded[0]}} & {writeEnable}),({16{wayOut[2]}} & {16{indexDecoded[1]}} & {writeEnable}),({16{wayOut[2]}} & {16{indexDecoded[2]}} & {writeEnable}),({16{wayOut[2]}} & {16{indexDecoded[3]}} & {writeEnable})};
  assign writeEnable3 = {({16{wayOut[3]}} & {16{indexDecoded[0]}} & {writeEnable}),({16{wayOut[3]}} & {16{indexDecoded[1]}} & {writeEnable}),({16{wayOut[3]}} & {16{indexDecoded[2]}} & {writeEnable}),({16{wayOut[3]}} & {16{indexDecoded[3]}} & {writeEnable})};
  assign writeEnable4 = {({16{wayOut[4]}} & {16{indexDecoded[0]}} & {writeEnable}),({16{wayOut[4]}} & {16{indexDecoded[1]}} & {writeEnable}),({16{wayOut[4]}} & {16{indexDecoded[2]}} & {writeEnable}),({16{wayOut[4]}} & {16{indexDecoded[3]}} & {writeEnable})};
  assign writeEnable5 = {({16{wayOut[5]}} & {16{indexDecoded[0]}} & {writeEnable}),({16{wayOut[5]}} & {16{indexDecoded[1]}} & {writeEnable}),({16{wayOut[5]}} & {16{indexDecoded[2]}} & {writeEnable}),({16{wayOut[5]}} & {16{indexDecoded[3]}} & {writeEnable})};
  assign writeEnable6 = {({16{wayOut[6]}} & {16{indexDecoded[0]}} & {writeEnable}),({16{wayOut[6]}} & {16{indexDecoded[1]}} & {writeEnable}),({16{wayOut[6]}} & {16{indexDecoded[2]}} & {writeEnable}),({16{wayOut[6]}} & {16{indexDecoded[3]}} & {writeEnable})};
  assign writeEnable7 = {({16{wayOut[7]}} & {16{indexDecoded[0]}} & {writeEnable}),({16{wayOut[7]}} & {16{indexDecoded[1]}} & {writeEnable}),({16{wayOut[7]}} & {16{indexDecoded[2]}} & {writeEnable}),({16{wayOut[7]}} & {16{indexDecoded[3]}} & {writeEnable})};       
  
  dataArray DA0(clk, reset, writeEnable0, blockData, outBlockData0[127:0], outBlockData0[255:128], outBlockData0[383:256], outBlockData0[511:384]);
  dataArray DA1(clk, reset, writeEnable1, blockData, outBlockData1[127:0], outBlockData1[255:128], outBlockData1[383:256], outBlockData1[511:384]);
  dataArray DA2(clk, reset, writeEnable2, blockData, outBlockData2[127:0], outBlockData2[255:128], outBlockData2[383:256], outBlockData2[511:384]);
  dataArray DA3(clk, reset, writeEnable3, blockData, outBlockData3[127:0], outBlockData3[255:128], outBlockData3[383:256], outBlockData3[511:384]);
  dataArray DA4(clk, reset, writeEnable4, blockData, outBlockData4[127:0], outBlockData4[255:128], outBlockData4[383:256], outBlockData4[511:384]);
  dataArray DA5(clk, reset, writeEnable5, blockData, outBlockData5[127:0], outBlockData5[255:128], outBlockData5[383:256], outBlockData5[511:384]);
  dataArray DA6(clk, reset, writeEnable6, blockData, outBlockData6[127:0], outBlockData6[255:128], outBlockData6[383:256], outBlockData6[511:384]);
  dataArray DA7(clk, reset, writeEnable7, blockData, outBlockData7[127:0], outBlockData7[255:128], outBlockData7[383:256], outBlockData7[511:384]);

  mux4to1_128b lineSelectM0(outBlockData0[511:384], outBlockData0[383:256], outBlockData0[255:128], outBlockData0[127:0], index, blockData0);
  mux4to1_128b lineSelectM1(outBlockData1[511:384], outBlockData1[383:256], outBlockData1[255:128], outBlockData1[127:0], index, blockData1);
  mux4to1_128b lineSelectM2(outBlockData2[511:384], outBlockData2[383:256], outBlockData2[255:128], outBlockData2[127:0], index, blockData2);
  mux4to1_128b lineSelectM3(outBlockData3[511:384], outBlockData3[383:256], outBlockData3[255:128], outBlockData3[127:0], index, blockData3);
  mux4to1_128b lineSelectM4(outBlockData4[511:384], outBlockData4[383:256], outBlockData4[255:128], outBlockData4[127:0], index, blockData4);
  mux4to1_128b lineSelectM5(outBlockData5[511:384], outBlockData5[383:256], outBlockData5[255:128], outBlockData5[127:0], index, blockData5);
  mux4to1_128b lineSelectM6(outBlockData6[511:384], outBlockData6[383:256], outBlockData6[255:128], outBlockData6[127:0], index, blockData6);
  mux4to1_128b lineSelectM7(outBlockData7[511:384], outBlockData7[383:256], outBlockData7[255:128], outBlockData7[127:0], index, blockData7);

    
  mux8to1_128b waySelectM(blockData0, blockData1, blockData2, blockData3, blockData4, blockData5, blockData6, blockData7, WayNo, outBlockData);
  
  mux16to1_8b offsetSelectM(outBlockData[7:0], outBlockData[15:8], outBlockData[23:16], outBlockData[31:24], outBlockData[39:32], outBlockData[47:40], 
                         outBlockData[55:48], outBlockData[63:56], outBlockData[71:64], outBlockData[79:72], outBlockData[87:80], outBlockData[95:88], 
                         outBlockData[103:96], outBlockData[111:104], outBlockData[119:112], outBlockData[127:120], offset, outByteData);

endmodule

module signalGenerator(input hit, input write, output [1:0] signal, output DB_signal, output DB_dataSignal, output blockByteEnable);
  assign signal = {write & hit, (~hit)};
  assign DB_signal = write | (~hit);
  assign DB_dataSignal = write;
  assign blockByteEnable = (~hit);
endmodule
 
 
 module priority_encoder (input [7:0] sel, output reg [2:0] code);  
   
  always @(sel)  
  begin  
        if (sel[0]) code = 3'b000;  
   else if (sel[1]) code = 3'b001;  
   else if (sel[2]) code = 3'b010;  
   else if (sel[3]) code = 3'b011;  
   else if (sel[4]) code = 3'b100;  
   else if (sel[5]) code = 3'b101;  
   else if (sel[6]) code = 3'b110;  
   else if (sel[7]) code = 3'b111;  
   else code = 3'b000;  
  end  
endmodule 

//////////////////////////////////////
// DEFINITION OF A WAY
//////////////////////////////////////
module Way(input clk, input reset, input[31:0] inputAddress, input haltWriteEnable, input way_write, input valid_write, 
           input[3:0] decin,  
           output way_hit, output vamuxout,output way_halt_hit);
	
	wire[3:0] decout,htao0, htao1, htao2, htao3;
	wire[21:0] tagout0, tagout1, tagout2, tagout3;
	wire[25:0] tagmuxout;
	wire cmp0, cmp1, cmp2, cmp3, tagcompout, haltmuxout;
	wire vaout0,vaout1,vaout2,vout3;

	haltTagArray hta0(clk, reset, ({4{haltWriteEnable}}&decin) , inputAddress[9:6], htao0, htao1, htao2, htao3);  
	comparator4bit cp0(htao0, inputAddress[9:6], cmp0);
	comparator4bit cp1(htao1, inputAddress[9:6], cmp1);
	comparator4bit cp2(htao2, inputAddress[9:6], cmp2);
	comparator4bit cp3(htao3, inputAddress[9:6], cmp3);

  or o1(way_halt_hit,cmp0,cmp1,cmp2,cmp3);

	tagArray ta0(clk, reset, (decin& {4{way_write}}),inputAddress[31:10], tagout0, tagout1, tagout2, tagout3);
	mux4to1_26b m0({tagout0,htao0}, {tagout1,htao1}, {tagout2,htao2}, {tagout3,htao3}, inputAddress[5:4], tagmuxout);
	comparator26bit cp4(tagmuxout, inputAddress[31:6],tagcompout);
	mux2to1_1b m1(1'b0, tagcompout, way_halt_hit, haltmuxout);

	vaildArray va0(clk, reset, (decin&{4{way_write}}), 1'b1, vaout0,vaout1,vaout2,vout3);
	mux4to1_1b valid_bit_mux(vaout0,vaout1,vaout2,vout3,inputAddress[5:4],vamuxout);
	and a4(way_hit, vamuxout,haltmuxout);
  
endmodule

////////////////////////////////////////
// TOP CACHE MODULE
////////////////////////////////////////
module cacheModule(input clk, input reset, input[31:0] inputAddress, input MemWrite, 
                    input [127:0] blockWriteData, input [7:0] byteWriteData, input lru_write,
                    output [127:0] blockToMemData, output [7:0] byteToCPUData, output haltHit, output cacheHit, output dirtyBit,
                    output [26:0]cacheTag, output [3:0]cacheOffset, output [1:0] cacheIndex);
  
  wire [1:0] Signal;
  wire [3:0] indexDecoder;
  wire [7:0] wayHit, way_halt_hit,vaout;
  wire [2:0] finalWayNo, addrWayNo, LRU_WayNo0, LRU_WayNo1, LRU_WayNo2, LRU_WayNo3, LRU_WayNo, placement_way_no, write_way_out;
  wire DB_signal, DB_data, replace, blockByteEnable;
  wire [15:0] offsetDecoded, writeDataEnable;
  wire [7:0] wayNoDecoded;
  wire [127:0] writeDataValue;
  
  
  assign cacheTag = inputAddress[31:6];
  assign cacheOffset = inputAddress[3:0];
  assign cacheIndex = inputAddress[5:4];  
  
  decoder2to4 indexDec(inputAddress[5:4], indexDecoder);
  
  Way way0(clk, reset, inputAddress, ~cacheHit& wayNoDecoded[0], ~cacheHit& wayNoDecoded[0] ,1'b1,indexDecoder, wayHit[0], vaout[0],way_halt_hit[0]);
  Way way1(clk, reset, inputAddress, ~cacheHit& wayNoDecoded[1], ~cacheHit& wayNoDecoded[1] ,1'b1,indexDecoder, wayHit[1], vaout[1],way_halt_hit[1]);
  Way way2(clk, reset, inputAddress, ~cacheHit& wayNoDecoded[2], ~cacheHit& wayNoDecoded[2] ,1'b1,indexDecoder, wayHit[2], vaout[2],way_halt_hit[2]);
  Way way3(clk, reset, inputAddress, ~cacheHit& wayNoDecoded[3], ~cacheHit& wayNoDecoded[3] ,1'b1,indexDecoder, wayHit[3], vaout[3],way_halt_hit[3]);
  Way way4(clk, reset, inputAddress, ~cacheHit& wayNoDecoded[4], ~cacheHit& wayNoDecoded[4] ,1'b1,indexDecoder, wayHit[4], vaout[4],way_halt_hit[4]);
  Way way5(clk, reset, inputAddress, ~cacheHit& wayNoDecoded[5], ~cacheHit& wayNoDecoded[5] ,1'b1,indexDecoder, wayHit[5], vaout[5],way_halt_hit[5]);
  Way way6(clk, reset, inputAddress, ~cacheHit& wayNoDecoded[6], ~cacheHit& wayNoDecoded[6] ,1'b1,indexDecoder, wayHit[6], vaout[6],way_halt_hit[6]);
  Way way7(clk, reset, inputAddress, ~cacheHit& wayNoDecoded[7], ~cacheHit& wayNoDecoded[7] ,1'b1,indexDecoder, wayHit[7], vaout[7],way_halt_hit[7]);

  //PLACEMENT CIRCUIT
  and a1 (replace,vaout[0],vaout[1],vaout[2],vaout[3],vaout[4],vaout[5],vaout[6],vaout[7]);
  
  priority_encoder pe(~vaout, placement_way_no); 
  LRU_Counter lru0(clk, reset,  cacheHit, lru_write&indexDecoder[0], addrWayNo, LRU_WayNo0);
  LRU_Counter lru1(clk, reset,  cacheHit, lru_write&indexDecoder[1], addrWayNo, LRU_WayNo1);
  LRU_Counter lru2(clk, reset,  cacheHit, lru_write&indexDecoder[2], addrWayNo, LRU_WayNo2);
  LRU_Counter lru3(clk, reset,  cacheHit, lru_write&indexDecoder[3], addrWayNo, LRU_WayNo3);
  
  mux4to1_3b lru_mux(LRU_WayNo0, LRU_WayNo1, LRU_WayNo2, LRU_WayNo3, inputAddress[5:4], LRU_WayNo);
    
  mux2to1_3b placement_replacement(placement_way_no, LRU_WayNo, replace,write_way_out);
  mux2to1_3b way_no(write_way_out,addrWayNo, cacheHit,finalWayNo);
  
  decoder3to8 wayCalc(finalWayNo, wayNoDecoded);
  
  //HALTING CIRCUIT
  or halt_hit_miss( haltHit, way_halt_hit[0], way_halt_hit[1], way_halt_hit[2], way_halt_hit[3], way_halt_hit[4], way_halt_hit[5], way_halt_hit[6], way_halt_hit[7]); 
  or cache_hit_miss(tempCacheHit, wayHit[0], wayHit[1], wayHit[2], wayHit[3], wayHit[4], wayHit[5], wayHit[6], wayHit[7]); 
  
  mux2to1_1b hitMux(1'b0, tempCacheHit, haltHit, cacheHit);
  
  encoder8to3 Enc_way(wayHit, addrWayNo);
  mux2to1_128b blockByteSelect({16{byteWriteData}}, blockWriteData, blockByteEnable, writeDataValue);
  get_setData getsetCacheData(clk, reset, finalWayNo, wayNoDecoded, inputAddress[5:4], indexDecoder, writeDataValue, writeDataEnable, inputAddress[3:0], blockToMemData, byteToCPUData); 

  signalGenerator sigGen(cacheHit, MemWrite, Signal, DB_signal, DB_data, blockByteEnable);
  
  get_setDirtyBit setDB(clk, reset, finalWayNo, wayNoDecoded, inputAddress[5:4], indexDecoder, DB_data, DB_signal, dirtyBit);
  
  decoder4to16 dec(inputAddress[3:0], offsetDecoded);
  mux4to1_16b enableSelectM(16'b0, 16'b1111111111111111, offsetDecoded, 16'b0, Signal, writeDataEnable); 

endmodule

////////////////////////////////////////////
// TEST BENCH
////////////////////////////////////////////
module cache_TestBench; 
reg clk, reset, MemWrite;
reg[31:0] inputAddress;
reg [127:0] blockWriteData;
reg [7:0] byteWriteData;
reg lru_write;
wire [127:0] blockToMemData;
wire[7:0] byteToCPUData;
wire haltHit,cacheHit, dirtyBit;
wire [26:0]cacheTag;
wire [1:0]cacheIndex;
wire [3:0]cacheOffset;


cacheModule uut(clk, reset, inputAddress, MemWrite, blockWriteData,byteWriteData,lru_write, 
                  blockToMemData, byteToCPUData, haltHit, cacheHit, dirtyBit, cacheTag, cacheOffset, cacheIndex);
  

   
always #5 clk=~clk;   
  initial  
  begin 
    
    clk = 0; reset = 1;
     //Compulsory Write Miss 1
    #30 reset = 0; MemWrite=1; inputAddress=32'b00000000000000000000010001010001;
     blockWriteData=128'hff000000000000000000000000000000;
     byteWriteData=8'h0f;
    
    #20 lru_write = 1;
    #10 lru_write = 0; 
    //Compulsory Write Miss 2
     MemWrite=1; inputAddress=32'b00000000000000000000100001010001;
     blockWriteData=128'hff000000000000000000000000000000;
     byteWriteData=8'h0e; 
     //Compulsory Write Miss 3
    #20 lru_write = 1;
    #10 lru_write = 0;
     MemWrite=1; inputAddress=32'b00000000000000000000110001010001;
     blockWriteData=128'hff000000000000000000000000000000;
     byteWriteData=8'h0d;
     //Compulsory Write Miss 4     
    #20 lru_write = 1;
    #10 lru_write = 0;
     MemWrite=1; inputAddress=32'b00000000000000000001000001010001;
     blockWriteData=128'hff000000000000000000000000000000;
     byteWriteData=8'h0c;  
     //Compulsory Write Miss 5
    #20 lru_write = 1;
    #10 lru_write = 0;
     MemWrite=1; inputAddress=32'b00000000000000000001010001010001;
     blockWriteData=128'hff000000000000000000000000000000;
     byteWriteData=8'h0b;  
     //Compulsory Write Miss 6
    #20 lru_write = 1;
    #10 lru_write = 0;
     MemWrite=1; inputAddress=32'b00000000000000000001100001010001;
     blockWriteData=128'hff000000000000000000000000000000;
     byteWriteData=8'h0a; 
     //Compulsory Write Miss 7
    #20 lru_write = 1;
    #10 lru_write = 0;
     MemWrite=1; inputAddress=32'b00000000000000000001110001010001;
     blockWriteData=128'hff000000000000000000000000000000;
     byteWriteData=8'h09;  
     //Compulsory Write Miss 8
    #20 lru_write = 1;
    #10 lru_write = 0;
     MemWrite=1; inputAddress=32'b00000000000000000010000001010001;
     blockWriteData=128'hff000000000000000000000000000000;
     byteWriteData=8'h08; 
     //Read Hit
    #20 lru_write = 1;
    #10 lru_write = 0;
     MemWrite=0; inputAddress=32'b00000000000000000001000001010001;
     blockWriteData=128'h00000000000000000000000000000000;
     byteWriteData=8'h00;  
     //Conflict Write Miss
    lru_write = 1;
    #10 lru_write = 0;
     MemWrite=1; inputAddress=32'b00000000000000000110000001010001;
     blockWriteData=128'h00ff0000000000000000000000000000;
     byteWriteData=8'h80; 
     
     //Write Hit
    #20 lru_write = 1;
    #10 lru_write = 0;
     MemWrite=1; inputAddress=32'b00000000000000000000100001010001;
     blockWriteData=128'h0000ff00000000000000000000000000;
     byteWriteData=8'h8e;  
     //Conflict Read Miss
    #10 lru_write = 1;
    #10 lru_write = 0;
     MemWrite=0; inputAddress=32'b00000000000000001110000001010001;
     blockWriteData=128'h00000000000000000000000000000f00;
     byteWriteData=8'h00; 
     //Read Miss Halt Miss Set 10
     
     #10 lru_write = 1;    
     #10 lru_write = 0;
     MemWrite=0; inputAddress=32'b00000000000000000000010010100001;
     blockWriteData=128'h00000000000000000000000000abf200;
     byteWriteData=8'h00;
     
     //Read Miss Halt Miss Set 11
     #10 lru_write = 1;    
     #10 lru_write = 0;
     MemWrite=0; inputAddress=32'b00000000000000000000000011100001;
     blockWriteData=128'h00000000000000000000000000abf300;
     byteWriteData=8'h00;
     
     //Read Miss Halt Miss Set 00
     #10 lru_write = 1;    
     #10 lru_write = 0;
     MemWrite=0; inputAddress=32'b00000000000000000000010110000001;
     blockWriteData=128'h00000000000000000000000000abf400;
     byteWriteData=8'h00;    
     
     //Read Miss Halt hit Set 00
     #10 lru_write = 1;    
     #10 lru_write = 0;
     MemWrite=0; inputAddress=32'b00000000000000000000110011110001;
     blockWriteData=128'h00000000000000000000000000abf500;
     byteWriteData=8'h00;
     
     lru_write = 1;    
     #10 lru_write = 0;
        
    #50 $finish; 
  end 
endmodule 
