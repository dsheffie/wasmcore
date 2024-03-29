UNAME_S = $(shell uname -s)

OBJ = top.o verilated.o verilated_vcd_c.o helper.o saveState.o 

SV_SRC = core_l1d_l1i.sv core.sv exec.sv decode_riscv.sv shiftregbit.sv shift_right.sv mul.sv find_first_set.sv divider.sv l1d.sv l1i.sv machine.vh rob.vh uop.vh ram1r1w.sv ram2r1w.sv popcount.sv count_leading_zeros.sv fair_sched.sv ppa32.sv ppa64.sv csa.sv rf6r3w.sv reg_ram1rw.sv perfect_l1d.sv l2.sv mwidth_add.sv addsub.sv

CXX = em++
MAKE = make
VERILATOR_SRC = /home/dsheffie/local/share/verilator/include/verilated.cpp
VERILATOR_VCD = /home/dsheffie/local/share/verilator/include/verilated_vcd_c.cpp
VERILATOR_INC = /home/dsheffie/local/share/verilator/include
VERILATOR_DPI_INC = /home/dsheffie/local/share/verilator/include/vltstd/
VERILATOR = /home/dsheffie/local/bin/verilator
EXTRA_LD = 



OPT = -O3 -g -std=c++14 -fomit-frame-pointer
CXXFLAGS = -std=c++11 -g  $(OPT) -I$(VERILATOR_INC) -I$(VERILATOR_DPI_INC) #-DLINUX_SYSCALL_EMULATION=1
LIBS =  $(EXTRA_LD) -lpthread

DEP = $(OBJ:.o=.d)

EXE = rv64_core.html

.PHONY : all clean

all: $(EXE)

$(EXE) : $(OBJ) obj_dir/Vcore_l1d_l1i__ALL.a
	$(CXX) $(CXXFLAGS) $(OBJ) obj_dir/*.o $(LIBS) -o $(EXE) --preload-file bbl.bin0.bin -s TOTAL_MEMORY=384MB

top.o: top.cc obj_dir/Vcore_l1d_l1i__ALL.a
	$(CXX) -MMD $(CXXFLAGS) -Iobj_dir -c $< 

verilated.o: $(VERILATOR_SRC)
	$(CXX) -MMD $(CXXFLAGS) -c $< 

verilated_vcd_c.o: $(VERILATOR_VCD)
	$(CXX) -MMD $(CXXFLAGS) -c $< 

%.o: %.cc
	$(CXX) -MMD $(CXXFLAGS) -c $< 

obj_dir/Vcore_l1d_l1i__ALL.a : $(SV_SRC)
	$(VERILATOR) -cc core_l1d_l1i.sv
	$(MAKE) OPT_FAST="-O3" CXX="em++" -C obj_dir -f Vcore_l1d_l1i.mk

-include $(DEP)

clean:
	rm -rf $(EXE) $(OBJ) $(DEP) obj_dir
