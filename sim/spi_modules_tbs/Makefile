# =============================================================================
# Makefile — SPI Module Testbenches (Vivado xsim)
# sim/spi_module_tbs/
# =============================================================================

# ---------------------------------------------------------------------------
# Tool & flags
# ---------------------------------------------------------------------------
XVLOG   := xvlog
XELAB   := xelab
XSIM    := xsim

XVLOG_FLAGS := --sv --nolog
XELAB_FLAGS := --nolog --debug typical
XSIM_FLAGS  := --nolog --runall

# ---------------------------------------------------------------------------
# Paths  (relative to this Makefile's location: sim/spi_module_tbs/)
# ---------------------------------------------------------------------------
RTL_DIR := ../../rtl/spi_modules
TB_DIR  := .

# Include directories — passed to xvlog with -i so `include directives resolve
INCLUDE_DIRS := \
    ../../rtl/spi_modules \
    ../../rtl

INCLUDE_FLAGS := $(foreach d, $(INCLUDE_DIRS), -i $(d))

# RTL sources (compile order: core first, then wrappers)
RTL_SRCS := \
    $(RTL_DIR)/spi_master_phy_core.sv \
    $(RTL_DIR)/spi_master_phy.sv      \
    $(RTL_DIR)/spi_slave_phy.sv

# Testbench sources
TB_MASTER_CORE := $(TB_DIR)/spi_master_phy_core_tb.sv
TB_MASTER_PHY  := $(TB_DIR)/spi_master_phy_tb.sv
TB_SLAVE_PHY   := $(TB_DIR)/spi_slave_phy_tb.sv

# Top-level module names (must match the module name inside each TB file)
TOP_MASTER_CORE := spi_master_phy_core_tb
TOP_MASTER_PHY  := spi_master_phy_tb
TOP_SLAVE_PHY   := spi_slave_phy_tb

# Work library directory
WORK_DIR := xsim_work

# ---------------------------------------------------------------------------
# Phony targets
# ---------------------------------------------------------------------------
.PHONY: all compile \
        sim_master_core sim_master_phy sim_slave_phy \
        clean help

# ---------------------------------------------------------------------------
# Default: compile everything then run all three testbenches
# ---------------------------------------------------------------------------
all: sim_slave_phy sim_master_core sim_master_phy

# ---------------------------------------------------------------------------
# Step 1 — Compile all RTL + all TBs into the work library
# ---------------------------------------------------------------------------
compile:
	@echo "=== Compiling RTL sources ==="
	$(XVLOG) $(XVLOG_FLAGS) $(INCLUDE_FLAGS) $(RTL_SRCS)
	@echo "=== Compiling testbench sources ==="
	$(XVLOG) $(XVLOG_FLAGS) $(INCLUDE_FLAGS) $(TB_MASTER_CORE) $(TB_MASTER_PHY) $(TB_SLAVE_PHY)

# ---------------------------------------------------------------------------
# Step 2 — Elaborate + simulate each top-level
# ---------------------------------------------------------------------------
sim_master_core: compile
	@echo "=== Elaborating $(TOP_MASTER_CORE) ==="
	$(XELAB) $(XELAB_FLAGS) $(TOP_MASTER_CORE) -s snap_$(TOP_MASTER_CORE)
	@echo "=== Simulating $(TOP_MASTER_CORE) ==="
	$(XSIM) snap_$(TOP_MASTER_CORE) $(XSIM_FLAGS)

sim_master_phy: compile
	@echo "=== Elaborating $(TOP_MASTER_PHY) ==="
	$(XELAB) $(XELAB_FLAGS) $(TOP_MASTER_PHY) -s snap_$(TOP_MASTER_PHY)
	@echo "=== Simulating $(TOP_MASTER_PHY) ==="
	$(XSIM) snap_$(TOP_MASTER_PHY) $(XSIM_FLAGS)

sim_slave_phy: compile
	@echo "=== Elaborating $(TOP_SLAVE_PHY) ==="
	$(XELAB) $(XELAB_FLAGS) $(TOP_SLAVE_PHY) -s snap_$(TOP_SLAVE_PHY)
	@echo "=== Simulating $(TOP_SLAVE_PHY) ==="
	$(XSIM) snap_$(TOP_SLAVE_PHY) $(XSIM_FLAGS)

# ---------------------------------------------------------------------------
# Waveform targets — elaborate with full debug, open in GUI
# ---------------------------------------------------------------------------
wave_master_core: compile
	$(XELAB) $(XELAB_FLAGS) --debug all $(TOP_MASTER_CORE) -s snap_wave_$(TOP_MASTER_CORE)
	$(XSIM) snap_wave_$(TOP_MASTER_CORE) --gui

wave_master_phy: compile
	$(XELAB) $(XELAB_FLAGS) --debug all $(TOP_MASTER_PHY) -s snap_wave_$(TOP_MASTER_PHY)
	$(XSIM) snap_wave_$(TOP_MASTER_PHY) --gui

wave_slave_phy: compile
	$(XELAB) $(XELAB_FLAGS) --debug all $(TOP_SLAVE_PHY) -s snap_wave_$(TOP_SLAVE_PHY)
	$(XSIM) snap_wave_$(TOP_SLAVE_PHY) --gui

# ---------------------------------------------------------------------------
# Clean
# ---------------------------------------------------------------------------
clean:
	@echo "=== Cleaning simulation artefacts ==="
	rm -rf xsim_work xsim.dir *.pb *.wdb *.log *.jou *.str .Xil *.vcd

# ---------------------------------------------------------------------------
# Help
# ---------------------------------------------------------------------------
help:
	@echo ""
	@echo "  make                  — compile & run all three testbenches"
	@echo "  make sim_master_core  — compile & run spi_master_phy_core_tb"
	@echo "  make sim_master_phy   — compile & run spi_master_phy_tb"
	@echo "  make sim_slave_phy    — compile & run spi_slave_phy_tb"
	@echo "  make wave_master_core — open spi_master_phy_core_tb in Vivado GUI"
	@echo "  make wave_master_phy  — open spi_master_phy_tb in Vivado GUI"
	@echo "  make wave_slave_phy   — open spi_slave_phy_tb in Vivado GUI"
	@echo "  make clean            — remove all generated files"
	@echo ""