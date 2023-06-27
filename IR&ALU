`timescale 1ns / 1ps
 //////////////////////////////////////////////////////////////////////////////////
 // Company: 
 // Engineer: @princesuman
 // 
 // Create Date: 09.06.2023 06:59:19
 // Design Name: 
 // Module Name: Microprocessor_Design
 // Project Name: Microprocessor_Design
 // Target Devices: 
 // Tool Versions: 
 // Description: 
 // 
 // Dependencies: 
 // 
 // Revision:
 // Revision 0.01 - File Created
 // Additional Comments:
 // 
 //////////////////////////////////////////////////////////////////////////////////
 
 //defining the name to subparts of IR
 `define oper_type IR[31:27]
 `define rdst      IR[26:22]
 `define rsrc1     IR[21:17]
 `define addmode   IR[16]
 `define rsrc2     IR[15:11]
 `define isrc      IR[15:0]
 
 
 //defining the codes to arithmetic operations >> opcode>>
 `define movsgpr  5'b00000
 `define mov      5'b00001
 `define add      5'b00010
 `define sub      5'b00011
 `define mul      5'b00100
  
  //defining the codes to logical operations >> opcode>>
  `define rand   5'b00101
  `define ror    5'b00110
  `define rnot   5'b00111
  `define rnand  5'b01000
  `define rnor   5'b01001
  `define rxor   5'b01010
  `define rxnor  5'b01011
 
 module processor();
 
     reg [31:0]IR;             //main IR 
     reg [15:0] GPR[31:0];    // 32 registers of 16 bit 
     reg [15:0] SGPR;          
     reg [31:0] mul_temp;
     
         always @ (*)
         begin
         case(`oper_type)
            `movsgpr:begin 
             GPR[`rdst]=SGPR;
             end
             //////////
             `mov:begin 
             GPR[`rdst]=GPR[`rsrc1];
              end
              /////////
             `add:begin
             if(`addmode)
              GPR[`rdst]=GPR[`rsrc1]+`isrc;
             else
              GPR[`rdst]=GPR[`rsrc1]+GPR[`rsrc2];
              
              end
              //////////
             `sub:begin
             if(`addmode)
              GPR[`rdst]=GPR[`rsrc1]-`isrc;
             else
              GPR[`rdst]=GPR[`rsrc1]-GPR[`rsrc2]; 
              end
              //////////
              `mul:begin 
                  if(`addmode)
                  begin
                  mul_temp=GPR[`rsrc1]*`isrc;
                  GPR[`rdst]=mul_temp[15:0];
                 SGPR=mul_temp[31:16];
                 end
                   
                  else
                  begin
                  mul_temp=GPR[`rsrc1]*GPR[`rsrc2];
                  
                 GPR[`rdst]=mul_temp[15:0];
                 SGPR=mul_temp[31:16];
                 end
               end
               ////////
               `rand:begin
               if(`addmode)
              GPR[`rdst]=GPR[`rsrc1]&`isrc;
             else
              GPR[`rdst]=GPR[`rsrc1]& GPR[`rsrc2];
              end
              /////
              `ror:begin
               if(`addmode)
              GPR[`rdst]=GPR[`rsrc1]|`isrc;
             else
              GPR[`rdst]=GPR[`rsrc1]|GPR[`rsrc2];
              end
              ///
              `rnot:begin
               if(`addmode)
              GPR[`rdst]=~(`isrc);
             else
              GPR[`rdst]=~(GPR[`rsrc1]);
              end
              ////
               `rnand:begin
              if(`addmode)
                GPR[`rdst]= ~(GPR[`rsrc1]&`isrc);
              else
                GPR[`rdst]=~(GPR[`rsrc1]&GPR[`rsrc2]);
              end
              ////
              `rnor:begin
              if(`addmode)
                GPR[`rdst]= ~(GPR[`rsrc1]|`isrc);
              else
                GPR[`rdst]=~(GPR[`rsrc1]|GPR[`rsrc2]);
              end
              ////
              
              `rxnor:begin
              if(`addmode)
                GPR[`rdst]= ~(GPR[`rsrc1]^`isrc);
              else
                GPR[`rdst]=~(GPR[`rsrc1]^GPR[`rsrc2]);
              end
              ////
              `rxor:begin
              if(`addmode)
                GPR[`rdst]= GPR[`rsrc1]^`isrc;
              else
                GPR[`rdst]=GPR[`rsrc1]^GPR[`rsrc2];
              end
              ////
              
              ////
              
               
             endcase //end of case
             //////
             end //end of always block 
             
 //logic for setting flags
 
 reg zero =0,carry=0,sign=0,overflow=0,parity=0;  //parity to be added later
 reg [16:0]add_temp;
  always @(*) 
  begin
  ///////////////
  //carry
  if(`oper_type==`add)
  begin
  if(`addmode)
  begin
  add_temp=GPR[`rsrc1]+`isrc;
  carry=add_temp[16];
  end 
  else
  begin
  add_temp=GPR[`rsrc1]+GPR[`rsrc2];
  carry=add_temp[16];
  end 
  end
  else
  carry=1'b0;  // carry=0; difference?
  ///////////////
  //sign
  if(`oper_type==`mul)
  sign=SGPR[15];
  else
  sign=GPR[`rdst][15];
  ////////////////
  //zero
  if(`oper_type==`mul)
  zero=~((|GPR[`rdst])&(|SGPR[15:0]));
  else
  zero=~(|GPR[`rdst]);
  
  /////////////////
  //overflow
  if(`oper_type==`add)
  begin
  if(`addmode)
overflow=((GPR[`rdst][15])&(~GPR[`rsrc1][15])&(~IR[15]))|~((GPR[`rdst][15])&(~GPR[`rsrc1][15])&(~IR[15])) ;
  else
overflow=((GPR[`rdst][15])&(~GPR[`rsrc1][15])&(~GPR[`rsrc2][15]))|~((GPR[`rdst][15])&(~GPR[`rsrc1][15])&(~GPR[`rsrc2][15])) ;
 
  end
  else if(`oper_type==`sub)
  begin
  if(`addmode)
  overflow=((GPR[`rdst][15])&(~GPR[`rsrc1][15])&(IR[15]))|((~GPR[`rdst][15])&(GPR[`rsrc1][15])&(~IR[15])) ;
  else
  overflow=((GPR[`rdst][15])&(~GPR[`rsrc1][15])&(GPR[`rsrc2][15]))|((~GPR[`rdst][15])&(GPR[`rsrc1][15])&(~GPR[`rsrc2][15])) ;
  end
  
  //////////////////
  
  
  
  
  end
     
 endmodule
