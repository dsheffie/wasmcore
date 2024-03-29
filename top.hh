#ifndef __tophh__
#define __tophh__

#include <cstdint>
#include <cstdlib>
#include <cstdint>
#include <vector>
#include <cmath>
#include <tuple>
#include <map>

#include <sys/time.h>

#include <sys/mman.h>
#include <unistd.h>
#include <fstream>
#include <sys/stat.h>
#include <fcntl.h>
#include <fenv.h>
#include <verilated.h>
#include "Vcore_l1d_l1i.h"
#include "helper.hh"
#include "interpret.hh"
#include "globals.hh"
#include "saveState.hh"

#include "Vcore_l1d_l1i__Dpi.h"
#include "svdpi.h"

template <typename T>
static inline T round_to_alignment(T x, T m) {
  return ((x+m-1) / m) * m;
}

static inline uint32_t to_uint32(float f) {
  return *reinterpret_cast<uint32_t*>(&f);
}

static inline uint64_t to_uint64(double d) {
  return *reinterpret_cast<uint64_t*>(&d);
}


static inline
void reset_core(std::unique_ptr<Vcore_l1d_l1i> &tb, uint64_t &cycle,
		uint32_t init_pc) {
  for(; (cycle < 4) && !Verilated::gotFinish(); ++cycle) {
    tb->mem_rsp_valid = 0;
    tb->monitor_ack = 0;
    tb->reset = 1;
    tb->extern_irq = 0;
    tb->clk = 1;
    tb->eval();
    tb->clk = 0;
    tb->eval();
    ++cycle;
  }
  //deassert reset
  tb->reset = 0;
  tb->clk = 1;
  tb->eval();
  tb->clk = 0;
  tb->eval();

  tb->resume_pc = init_pc;
  while(!tb->ready_for_resume) {
    ++cycle;  
    tb->clk = 1;
    tb->eval();
    tb->clk = 0;
    tb->eval();
  }
  
  ++cycle;
  tb->resume = 1;

  tb->clk = 1;
  tb->eval();
  tb->clk = 0;
  tb->eval();
  
  ++cycle;  
  tb->resume = 0;
  tb->clk = 1;
  tb->eval();
  tb->clk = 0;
  tb->eval();
}




#endif
