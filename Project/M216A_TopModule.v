`timescale 1ns / 100ps

//Do NOT Modify This
module P1_Reg_8_bit (DataIn, DataOut, rst, clk);

    input [7: 0] DataIn;
    output [7: 0] DataOut;
    input rst;
    input clk;
    reg [7:0] DataReg;
    
    always @(posedge clk)
        if(rst)
            DataReg  <= 8'b0;
        else
            DataReg <= DataIn;
    assign DataOut = DataReg ;          
endmodule

module P1_Reg_5_bit (DataIn, DataOut, rst, clk);

    input [4: 0] DataIn;
    output [4: 0] DataOut;
    input rst;
    input clk;
    reg [4:0] DataReg;
    
    always @(posedge clk)
        if(rst)
            DataReg  <= 5'b0;
        else
            DataReg <= DataIn;
    assign DataOut = DataReg ;          
endmodule

module P1_Reg_4_bit (DataIn, DataOut, rst, clk);

    input [3: 0] DataIn;
    output [3: 0] DataOut;
    input rst;
    input clk;
    reg [3:0] DataReg;
    
    always @(posedge clk)
        if(rst)
            DataReg  <= 4'b0;
        else
            DataReg <= DataIn;
    assign DataOut = DataReg ;          
endmodule

module comp_leq_8_bit (A, B, LT);

    input [7: 0] A;
    input [7: 0] B;
    output LT;
    
    assign LT = (A < B);
endmodule

//Do NOT Modify This
module M216A_TopModule(
    clk_i,
    width_i,
    height_i,
    index_x_o,
    index_y_o,
    strike_o,
    Occupied_Width,
    rst_i);
  input clk_i;
  input [4:0]width_i;
  input [4:0]height_i;
  output [7:0]index_x_o, index_y_o;
  output [7:0]Occupied_Width[12:0];  // Connect it to a 13 element Register array
  output [3:0]strike_o;
  input rst_i;

wire [4:0] width_i, height_i;
wire clk_i, rst_i;
reg [7:0]Occupied_Width_internal[12:0];
reg fail;

//Add your code below 
//Make sure to Register the outputs using the Register modules given above
reg [7:0] index_x_reg, index_y_reg;
reg [3:0] strike_reg;
reg [7:0] rolling_sum_table [12:0];
initial begin
    rolling_sum_table[0] = 8'd0;
    rolling_sum_table[1] = 8'd12;
    rolling_sum_table[2] = 8'd16;
    rolling_sum_table[3] = 8'd27;
    rolling_sum_table[4] = 8'd32;
    rolling_sum_table[5] = 8'd42;
    rolling_sum_table[6] = 8'd48;
    rolling_sum_table[7] = 8'd57;
    rolling_sum_table[8] = 8'd64;
    rolling_sum_table[9] = 8'd72;
    rolling_sum_table[10] = 8'd80;
    rolling_sum_table[11] = 8'd96;
    rolling_sum_table[12] = 8'd112;

    Occupied_Width_internal[0] = 8'd0;
    Occupied_Width_internal[1] = 8'd0;
    Occupied_Width_internal[2] = 8'd0;
    Occupied_Width_internal[3] = 8'd0;
    Occupied_Width_internal[4] = 8'd0;
    Occupied_Width_internal[5] = 8'd0;
    Occupied_Width_internal[6] = 8'd0;
    Occupied_Width_internal[7] = 8'd0;
    Occupied_Width_internal[8] = 8'd0;
    Occupied_Width_internal[9] = 8'd0;
    Occupied_Width_internal[10] = 8'd0;
    Occupied_Width_internal[11] = 8'd0;
    Occupied_Width_internal[12] = 8'd0;
end
initial begin
    fail = 1'b0;
end


reg [3:0] allocated_strip;

// Main FSM states
// 0: input
// 1: check
// 2: update
// 3: output
reg [1:0] state;
initial begin
    state = 2'b11;
    allocated_strip = 4'd0;
end
always @(posedge clk_i) begin
    if (rst_i) begin
        state <= 2'b11;
    end else begin
        case (state)
            2'b00: begin
                state <= 2'b01;
            end
            2'b01: begin
                state <= 2'b10;
            end
            2'b10: begin
                state <= 2'b11;
            end
            2'b11: begin
                state <= 2'b00;
            end
        endcase
    end
end


assign index_x_o = index_x_reg;
assign index_y_o = index_y_reg;
assign strike_o = strike_reg;

reg[7:0] curr_width, old_width;
reg comp_res;
reg carry;

always @(posedge clk_i)
begin
    if(rst_i)
    begin
        index_x_reg <= 8'b0;
        index_y_reg <= 8'b0;
        strike_reg <= 4'b0;
        // also zero out Occupied width array
        Occupied_Width_internal[0] <= 8'd0;
        Occupied_Width_internal[1] <= 8'd0;
        Occupied_Width_internal[2] <= 8'd0;
        Occupied_Width_internal[3] <= 8'd0;
        Occupied_Width_internal[4] <= 8'd0;
        Occupied_Width_internal[5] <= 8'd0;
        Occupied_Width_internal[6] <= 8'd0;
        Occupied_Width_internal[7] <= 8'd0;
        Occupied_Width_internal[8] <= 8'd0;
        Occupied_Width_internal[9] <= 8'd0;
        Occupied_Width_internal[10] <= 8'd0;
        Occupied_Width_internal[11] <= 8'd0;
        Occupied_Width_internal[12] <= 8'd0;
        curr_width <= 8'd0;

    end
    else
    begin
        case (state)
            2'b00: begin // CHECK
                case(height_i)
                    // if 16,15,14,13 (check if two top bits are identical)
                    5'd16, 5'd15, 5'd14, 5'd13: begin
                        if (Occupied_Width_internal[12] < Occupied_Width_internal[11]) begin
                            allocated_strip <= (Occupied_Width_internal[12] < Occupied_Width_internal[10]) ? 4'd12 : 4'd10;
                        end else begin
                            allocated_strip <= (Occupied_Width_internal[11] < Occupied_Width_internal[10]) ? 4'd11 : 4'd10;
                        end
                    end
                    4'd12:
                        allocated_strip <= 4'd0;
                    4'd11: begin
                        if (Occupied_Width_internal[0] < Occupied_Width_internal[2]) begin
                            allocated_strip <= 4'd0;
                        end else begin
                            allocated_strip <= 4'd2;
                        end
                    end
                    4'd10: begin
                        if (Occupied_Width_internal[2] < Occupied_Width_internal[4]) begin
                            allocated_strip <= 4'd2;
                        end else begin
                            allocated_strip <= 4'd4;
                        end
                    end
                    4'd9:
                        if (Occupied_Width_internal[4] < Occupied_Width_internal[6]) begin
                            allocated_strip <= 4'd4;
                        end else begin
                            allocated_strip <= 4'd6;
                    end
                    4'd8: begin
                        if (Occupied_Width_internal[6] < Occupied_Width_internal[9]) begin
                            allocated_strip <= (Occupied_Width_internal[6] < Occupied_Width_internal[8]) ? 4'd6 : 4'd8;
                        end else begin
                            allocated_strip <= (Occupied_Width_internal[9] < Occupied_Width_internal[8]) ? 4'd9 : 4'd8;
                        end
                    end
                    4'd7: begin
                        if (Occupied_Width_internal[9] < Occupied_Width_internal[8]) begin
                            allocated_strip <= (Occupied_Width_internal[9] < Occupied_Width_internal[7]) ? 4'd9 : 4'd7;
                        end else begin
                            allocated_strip <= (Occupied_Width_internal[8] < Occupied_Width_internal[7]) ? 4'd8 : 4'd7;
                        end
                    end
                    4'd6: begin
                        if (Occupied_Width_internal[7] < Occupied_Width_internal[5]) begin
                            allocated_strip <= 4'd7;
                        end else begin
                            allocated_strip <= 4'd5;
                        end
                    end
                    4'd5: begin
                        if (Occupied_Width_internal[5] < Occupied_Width_internal[3]) begin
                            allocated_strip <= 4'd5;
                        end else begin
                            allocated_strip <= 4'd3;
                        end
                    end
                    4'd4: begin
                        if (Occupied_Width_internal[3] < Occupied_Width_internal[1]) begin
                            allocated_strip <= 4'd3;
                        end else begin
                            allocated_strip <= 4'd1;
                        end
                    end
                    default:
                        allocated_strip <= 4'd0;
                endcase
            end
            2'b01: begin // UPDATE1
                old_width <= Occupied_Width_internal[allocated_strip];
                {carry, curr_width[3:0]} <= Occupied_Width_internal[allocated_strip][3:0] + width_i[3:0];
            end
            2'b10: begin // UPDATE2 -- HOLDOUT HERE
                curr_width[7:4] <= old_width[7:4] + (width_i[4] + carry);
                // curr_width <= old_width + width_i;
            end
            2'b11: begin // OUTPUT + UPDATE3
                // if we failed
                if (curr_width > 8'd128) begin
                    index_x_reg <= 8'd128;
                    index_y_reg <= 8'd128;
                    strike_reg <= strike_o + 1'b1;
                // if we succeeded
                end else begin
                    Occupied_Width_internal[allocated_strip] <= curr_width;
                    index_x_reg <= old_width;
                    index_y_reg <= rolling_sum_table[allocated_strip];
                end
            end
        endcase
    end
end



// generate for all 
genvar i;
generate
    for (i = 0; i < 13; i = i + 1) begin: gen_loop
        assign Occupied_Width[i] = Occupied_Width_internal[i];
    end
endgenerate

endmodule