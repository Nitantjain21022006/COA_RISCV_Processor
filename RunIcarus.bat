cd\
cd C:\Users\admin\Desktop\Group10_CSE302
del processor.vvp
del Wavedump.vcd


C:\iverilog\bin\iverilog -o processor.vvp RISC_tb.v
PAUSE
C:\iverilog\bin\vvp processor.vvp
dir
PAUSE
C:\iverilog\gtkwave\bin\gtkwave -f Wavedump.vcd
PAUSE
