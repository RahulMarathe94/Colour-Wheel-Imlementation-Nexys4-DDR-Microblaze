/*============================================================================================
*File name: pwm_hw.v : This .v file contains the Module for Pulse Width Detection via Hardware.

* Author: Rahul Marathe (rmarathe@pdx.edu)
* Version 1.0: 12th April '18
* Version 2.0: 14th April '18
* Version 3.0: 16th April '18

Description:  
   - The module's(pwm_hw) algorithm works by detecting the Transitions (Low to High transition) and falling (High to Low Transition) by a Simple Edge detection algorithm.
   - It instantiates another sub-module(HW_PWM) which has the detection algorithm.
   - At every rising edge of the Clock is assigned to Low_Count and at every falling transition the 
     count value is assigned to the High_Count.
     If there is no transition, the count keeps on incrementing
   - HW_PWM sub-module is instantiated three times(each for red, green and blue detection)
================================================================================================*/
/*=================================================================================================
Declaration and instantiation for the PWM detection
This module instantiates another sub-module which has the detection for the Red, Green and Blue colors.
The outputs are connected to GPIO inputs to send the count value to the software for detetction which 
which have been wrapped around in the top level module.
==================================================================================================*/

module pwm_hw(
input clk,      			// 5 Mhz Clock
input reset,  			    // Active Low Reset
input red,                  // Red Pulse Input
input green,                // Green Pulse Input
input blue,                 // Blue Pulse Input
output [31:0] Red_High_HW,  // Red High Value Output
output [31:0] Red_Low_HW,   // Red Low Value Output
output [31:0] Green_High_HW,// Green High Value Output 
output [31:0] Green_Low_HW, // Green Low Value Output 
output [31:0] Blue_High_HW, // Blue High Value Output 
output [31:0] Blue_Low_HW   // Blue Low Value Output
             );

HW_PWM Red(.clk(clk),.reset(reset),.Colour_input(red),.High_Count(Red_High_HW),.Low_Count(Red_Low_HW));          // Instantiation for Red PWM Detection

HW_PWM Green(.clk(clk),.reset(reset),.Colour_input(green),.High_Count(Green_High_HW), .Low_Count(Green_Low_HW)); // Instantiation for Green PWM Detection 

HW_PWM Blue(.clk(clk),.reset(reset),.Colour_input(blue),.High_Count(Blue_High_HW), .Low_Count(Blue_Low_HW));     // Instantiation for Blue PWM Detection

endmodule

/*=================================================================================================
Declaration and instantiation for the PWM detection Logic
This module has the algorithm for detection for the Red, Green and Blue colors.
The outputs are connected to the Top Module(pwm_hw) to send the count value to the software for detetction which 
which have been wrapped around in the top level module.
==================================================================================================*/
module HW_PWM(
input clk,                    // 5 Mhz Clock
input reset,                  // Active Low Reset
input Colour_input,           // Red/Green/Blue Pulse Input
output reg [31:0] High_Count, // Red/Green/Blue High Value Output
output reg [31:0] Low_Count   // Red/Green/Blue Low Value Output
            );
reg PrevPwm=0;                // Temporary Reg to store Previous PWM for comparison
reg [31:0]Count=0;            // Temporary Count Value

always@(posedge clk)
begin
if (!reset)                   // If Active Low Reset is Asserted
    begin
	Count<=32'b0;
	High_Count<=32'b0;
	Low_Count<=32'b0;         // Clear all the Counters
	PrevPwm<=1'b0;
	end
else if(Colour_input==1)      // If Pulse is High
   begin
	if(PrevPwm!=Colour_input) // Check if Pulse is Equal to Previous Value
		begin
 		Low_Count<=Count;     // If not Equal Assign Temporary Count to Final Low Value
		PrevPwm<=1'b1;        // Update Previous PWM to 1  
		Count<=32'b0;         // Reset Count
		end
	else
		begin                 // If Colour Pulse is Equal to Previous Value
		Count<=Count+1'b1;    // Increment Count 
		end
  end
else if(Colour_input==0)      // If Pulse is Low
	begin
	if(PrevPwm!=Colour_input) // Check if Pulse is Equal to Previous Value
		begin
		High_Count<=Count;    // If not Equal Assign Temporary Count to Final Low Value
		PrevPwm<=Colour_input;// Update Previous PWM to Current Pulse  
		Count<=32'b0;         // Reset Count
		end
else
     begin                    // If Colour Pulse is Equal to Previous Value
     Count<=Count+1'b1;       // Increment Count   
     end     
end
end			
endmodule			
