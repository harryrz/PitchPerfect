# set the working dir, where all compiled Verilog goes
vlib work
# compile all Verilog modules in mux.v to working dir
# could also have multiple Verilog files
vlog project.v
# load simulation using mux as the top level simulation module
vsim controlpath
#log all signals and add some signals to waveform window
log {/*}
# add wave {/*} would add all items in top level simulation module
add wave {/*}


force CLOCK_50 0 0ns, 1 {5ns} -repeat 10ns
force resetn 0
run 10ns

force resetn 1
force ps2_key_pressed 1
force ps2_key_data 8'b00101100
run 10ns

force ps2_key_pressed 0
run 30ns

force ps2_key_pressed 1
force ps2_key_data 8'b00101001
run 10ns

force ps2_key_pressed 0
run 30ns

force ps2_key_pressed 1
force ps2_key_data 8'h32
run 10ns

force ps2_key_pressed 0
run 30ns

force ps2_key_pressed 1
force ps2_key_data 8'h1c
run 10ns

force ps2_key_pressed 0
run 30ns



