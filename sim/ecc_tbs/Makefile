# ==============================================================================
# Makefile for ECC Encoder/Decoder Testbench (ecc_tbs) using Vivado XSIM
# ==============================================================================

# Project directories
PROJ_ROOT   = ../..
RTL_DIR     = $(PROJ_ROOT)/rtl/ecc
TB_DIR      = .

# Source files (SystemVerilog)
RTL_SOURCES = \
	$(RTL_DIR)/ecc_enc_core.sv \
	$(RTL_DIR)/ecc_dec_core.sv

TB_SOURCES = \
	$(TB_DIR)/ecc_tb_channel.sv \
	$(TB_DIR)/ecc_tb_checker.sv \
	$(TB_DIR)/ecc_tb_top.sv

# Simulation parameters
TOP_MODULE = ecc_tb_top
SNAPSHOT   = $(TOP_MODULE)_snapshot
VCD_FILE   = $(TOP_MODULE).vcd

# Tool commands (assumes Vivado is in PATH)
XVLOG = xvlog
XELAB = xelab
XSIM  = xsim

# Common options
XVLOG_OPTS = --sv                    # No --Wall, use --sv only
XELAB_OPTS = --debug typical         # Enable waveform dumping
XSIM_OPTS  = --runall                # Run simulation until $finish

# ------------------------------------------------------------------------------
# Targets
# ------------------------------------------------------------------------------

.PHONY: all compile elaborate run run_vcd view clean help

all: compile elaborate run

# Step 1: Compile all source files into the 'work' library
compile: $(RTL_SOURCES) $(TB_SOURCES)
	$(XVLOG) $(XVLOG_OPTS) --work work $^

# Step 2: Elaborate the top module and create simulation snapshot
elaborate:
	$(XELAB) $(XELAB_OPTS) work.$(TOP_MODULE) -s $(SNAPSHOT)

# Step 3: Run the simulation (no VCD dump)
run: elaborate
	$(XSIM) $(SNAPSHOT) $(XSIM_OPTS)

# Step 4: Run simulation with VCD waveform dump
run_vcd: elaborate
	$(XSIM) $(SNAPSHOT) $(XSIM_OPTS) --wdb $(TOP_MODULE).wdb --vcd $(VCD_FILE)

# Step 5: View the waveform (requires GTKWave)
view: $(VCD_FILE)
	gtkwave $(VCD_FILE)

# Clean up generated files
clean:
	rm -rf xsim.dir work *.log *.pb *.jou *.wdb $(VCD_FILE) webtalk* .Xil

# Help
help:
	@echo "=========================================================================="
	@echo "ECC Testbench Makefile for Vivado XSIM"
	@echo "=========================================================================="
	@echo "Targets:"
	@echo "  all        - Compile, elaborate, and run (default)"
	@echo "  compile    - Compile all RTL and TB sources (xvlog)"
	@echo "  elaborate  - Elaborate the design (xelab)"
	@echo "  run        - Run simulation without VCD dump"
	@echo "  run_vcd    - Run simulation and generate VCD file"
	@echo "  view       - Open VCD file with GTKWave"
	@echo "  clean      - Remove all generated files"
	@echo "  help       - Show this help message"
	@echo "=========================================================================="
	@echo "Requirements:"
	@echo "  - Vivado installed and accessible in PATH"
	@echo "  - GTKWave (optional, for waveform viewing)"
	@echo "=========================================================================="
	@echo "Usage:"
	@echo "  make all          # Compile, elaborate, run simulation"
	@echo "  make run_vcd      # Generate VCD file for waveform analysis"
	@echo "  make view         # View waveform in GTKWave"
	@echo "=========================================================================="