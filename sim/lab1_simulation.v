module lab1_simulation (SW,LED, clk, outclk,selected);
    input logic [3:0]SW;
    output logic [7:0] LED;
    input logic clk;
    output logic outclk;
    output logic [2:0] selected;

    clkdivider clkdivider (.freq_select(SW[3:1]),.rst(~SW[0]),.clk(clk),.outclk(outclk),.selected(selected));
    LED_flash LED_flash(.clk(clk),.LED_count(LED[7:0]));

endmodule

module clkdivider(freq_select, rst, clk, outclk, selected);
    input [2:0] freq_select;
    input rst;
    input clk; //input is 50MHz
    output logic outclk; //output frequency

    output logic [2:0] selected; //for simulation to show what is being 
    assign selected = freq_select;



    //DIVISION FACTOR = SYSTEM CLK FREQ/(2*TARGET FREQUENCY)
    //values for simulation testing
    parameter LOW_DO_CYCLES = 16'd2;
    parameter RE_CYCLES = 16'd4;
    parameter MI_CYCLES = 16'd8;
    parameter FA_CYCLES = 16'd10;
    parameter SO_CYCLES = 16'd12;
    parameter LA_CYCLES = 16'd14;
    parameter SI_CYCLES = 16'd16;
    parameter HIGH_DO_CYCLES = 16'd18;

    //register to hold current counter value
    //max count is 47801, so we need 2^16 (65536) bits minimum for the counter to count all the cycles for the different target frequencies
    logic [15:0] counter;  //initialize to 0 so the first cycle is consistent with the others (same amount of clk cycles for a transition)
    //target counter holds the cycles needed to be counted for the desired frequency as known from the current input from the switches
    logic [15:0] target_count;

    //combination logic used to always check for changes for the input to change the frequency output
    always @(*)  begin
      case(freq_select)
        3'b000: target_count = LOW_DO_CYCLES;
        3'b001: target_count = RE_CYCLES;
        3'b010: target_count = MI_CYCLES;
        3'b011: target_count = FA_CYCLES;
        3'b100: target_count = SO_CYCLES;
        3'b101: target_count = LA_CYCLES;
        3'b110: target_count = SI_CYCLES;
        3'b111: target_count = HIGH_DO_CYCLES;
        default: target_count = 16'd0;
      endcase
    end

    //always trigger when the system clk rises to 1 to count the number of clk cycles that pass
    always @(posedge clk or posedge rst) begin 
      if (rst) begin //asynchronous active-high reset to reset values if need to start at a known starting point
        counter <= 0; //begin counting from the start again
        outclk <= 0;
      end
      else begin
        if (counter == target_count ) begin //when counter == target_count, the number of desired cycles have been counted
          counter <= 1; //reset the counter
          outclk <= ~outclk; //toggles the output clk between 0 and 1, outputting the desired frequency of clk
        end
        else begin
          counter <= counter + 1; //increment counter
        end
      end
    end
endmodule

module LED_flash(clk, LED_count); //module for LEDs 0-7 to flash for 1 second
    input clk; //input clk is 50MHz
    output logic [7:0] LED_count = 8'b00000100; //begin with LED2 on


    //count the number of system clk cycles that have passed
    //26 bits because the amount we need to count is 50,000,000, which is stored by a minimum of 2^26, 26 bits
    logic [25:0] counter = 0; //initialize to 0 so the first cycle is consistent with the others (same amount of clk cycles for a transition)
    
    //DIVISION FACTOR = SYSTEM CLK FREQ/(TARGET FREQUENCY)
    //50MHz/1 = 50M (do not divide by 2 because we are not toggling on and off for an equal amount of time unlike before. 
    //We just want to turn the LED on and then instantly off
    logic [25:0] target_count = 26'd2; //number of system clk cycles that have to be counted before LED is shifted
    //adjusted to a small number (2) for simulation demonstration purposes
   
   
    logic left_shift = 0; //checks if we are shifting left or right.

    always @(posedge clk) begin
      if(counter == target_count) begin
        counter = 1;
        if(LED_count[7] == 1 || LED_count[0] == 1) begin
          left_shift = ~left_shift;
        end
        LED_count = left_shift ? LED_count << 1 : LED_count >> 1; //if left_shift is 1, move left. If left_shift is 0, move left
      end
      else begin
        counter = counter + 1;
      end
    end
endmodule