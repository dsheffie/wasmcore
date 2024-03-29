#include "top.hh"
#include "temu_code.hh"

#define BRANCH_DEBUG 1
#define CACHE_STATS 1

bool globals::syscall_emu = true;
uint32_t globals::tohost_addr = 0;
uint32_t globals::fromhost_addr = 0;
bool globals::log = false;
std::map<std::string, uint32_t> globals::symtab;


char **globals::sysArgv = nullptr;
int globals::sysArgc = 0;


static uint64_t cycle = 0;
static bool trace_retirement = false;

static uint64_t mem_reqs = 0;
static state_t *s = nullptr;
static uint64_t insns_retired = 0, insns_allocated = 0;
static uint64_t cycles_in_faulted = 0, fetch_stalls = 0;

static uint64_t last_retire_cycle = 0, last_retire_pc  = 0;
static bool pending_fault = false;
static uint64_t fault_start_cycle = 0;
static bool verbose_ic_translate = false;

bool done = false;
void terminate() {
  done = true;
}

void csr_putchar(char c) {
  if(c==0) std::cout << "\n";
  else std::cout << c;
}

uint8_t *gptr(long long pa) {
  int pid = pa >> 12;
  //printf("page %d getting accessed\n", pid);
  if(s->mtbl[pid] == nullptr) {
    //printf("creating page %d\n", pid);
    s->mtbl[pid] = new uint8_t[4096];
  }
  assert(s->mtbl[pid] != nullptr);
  uint8_t *ptr = s->mtbl[pid] + (pa & 4095);
  //if(pa != 0) printf("base %p, ptr = %p\n", s->mtbl[pid], ptr);
  return ptr;
}

long long translate(long long va, long long root, bool iside, bool store) {
  uint64_t a = 0, u = 0;
  int mask_bits = -1;
  a = root + (((va >> 30) & 511)*8);
  u = *reinterpret_cast<int64_t*>(gptr(a));
  if((u & 1) == 0) {
    return (~0ULL);
  }
  if((u>>1)&7) {
    mask_bits = 30;
    goto translation_complete;
  }

  //2nd level walk
  root = ((u >> 10) & ((static_cast<uint64_t>(1UL)<<44)-1)) * 4096;
  a = root + (((va >> 21) & 511)*8);
  u = *reinterpret_cast<int64_t*>(gptr(a));
  if((u & 1) == 0) {
    if(verbose_ic_translate)
      printf("failed translation for %llx at level 2\n", va);
    return (~0ULL);
  }
  if((u>>1)&7) {
    mask_bits = 21;
    goto translation_complete;
  }
  
  //3rd level walk
  root = ((u >> 10) & ((static_cast<uint64_t>(1UL)<<44)-1)) * 4096;  
  a = root + (((va >> 12) & 511)*8);
  u = *reinterpret_cast<int64_t*>(gptr(a));
  if((u & 1) == 0) {
    if(verbose_ic_translate)
      printf("failed translation for %llx at level 1\n", va);
    return (~0ULL);
  }
  assert((u>>1)&7);
  mask_bits = 12;

 translation_complete:
  int64_t m = ((1L << mask_bits) - 1);

  /* accessed bit */
  bool accessed = ((u >> 6) & 1);
  bool dirty = ((u >> 7) & 1);
  if(!accessed) {
    u |= 1 << 6;
    *reinterpret_cast<int64_t*>(gptr(a)) = u;
  }

  if(store and not(dirty)) {
    u |= 1<<7;
    *reinterpret_cast<int64_t*>(gptr(a)) = u;    
  }
  
  u = ((u >> 10) & ((static_cast<uint64_t>(1UL)<<44)-1)) * 4096;
  uint64_t pa = (u&(~m)) | (va & m);
  //exit(-1);
  return pa;
}

long long ic_translate(long long va, long long root) {
  uint64_t pa = 0;
  pa = translate(va,root,true, false);
  return pa;
}

long long dc_translate(long long va, long long root, int mark_dirty) {
  return translate(va,root, false, mark_dirty);
}

uint64_t csr_time = 0;

long long csr_gettime() {
  return csr_time;
}


long long read_dword(long long addr) {
  int64_t pa = addr;
  pa &= ((static_cast<uint64_t>(1)<<32)-1);
  long long x = *reinterpret_cast<long long*>(gptr(pa));
  return x;
}

int read_word(long long addr) {
  int64_t pa = addr;
  pa &= ((static_cast<uint64_t>(1)<<32)-1);
  return *reinterpret_cast<int*>(gptr(pa));
}

void write_byte(long long addr, char data, long long root) {
  int64_t pa = addr;
  //printf("%s:%lx:%lx\n", __PRETTY_FUNCTION__, addr, root);  
  if(root) {
    pa = translate(addr, root, false, true);
    //printf("translate %lx to %lx\n", addr, pa);    
    assert(pa != -1);
  }  
  uint8_t d = *reinterpret_cast<uint8_t*>(&data);
  *reinterpret_cast<uint8_t*>(gptr(pa)) = d;  
}

void write_half(long long addr, short data, long long root) {
  int64_t pa = addr;
  //printf("%s:%lx:%lx\n", __PRETTY_FUNCTION__, addr, root);  
  if(root) {
    pa = translate(addr, root, false, true);
    ///printf("translate %lx to %lx\n", addr, pa);    
    assert(pa != -1);
  }  
  uint16_t d = *reinterpret_cast<uint16_t*>(&data);
  *reinterpret_cast<uint16_t*>(gptr(pa)) = d;  

}

void write_word(long long addr, int data, long long root, int id) {
  int64_t pa = addr;
  //printf("%s:%lx:%lx\n", __PRETTY_FUNCTION__, addr, root);  
  if(root) {
    pa = translate(addr, root, false, true);
    //printf("translate %lx to %lx\n", addr, pa);    
    assert(pa != -1);
  }  
  uint32_t d = *reinterpret_cast<uint32_t*>(&data);
  *reinterpret_cast<uint32_t*>(gptr(pa)) = d;
}

void write_dword(long long addr, long long data, long long root, int id) {
  int64_t pa = addr;

  if(root) {
    pa = translate(addr, root, false, true);
    //printf("translate %lx to %lx\n", addr, pa);    
    assert(pa != -1);
  }
  uint64_t d = *reinterpret_cast<uint64_t*>(&data);
  *reinterpret_cast<uint64_t*>(gptr(pa)) = d;
}


void initState(state_t *s) {
  memset(s, 0, sizeof(state_t));
  s->misa = 0x8000000000141101L;
  s->priv = priv_machine;
  s->mstatus = ((uint64_t)2 << MSTATUS_UXL_SHIFT) |((uint64_t)2 << MSTATUS_SXL_SHIFT);
  memset(s->mtbl, 0, sizeof(s->mtbl));
}
static uint64_t record_insns_retired = 0;


int main(int argc, char **argv) {
  std::string rv32_binary = "bbl.bin0.bin";
  uint64_t heartbeat = 1ULL<<24;
  uint64_t max_cycle = 0, max_icnt = 0, mem_lat = 2;
  uint64_t last_store_addr = 0, last_load_addr = 0, last_addr = 0;
  int misses_inflight = 0;
  std::map<uint64_t, uint64_t> pushout_histo;
  int64_t mem_reply_cycle = -1L;
  
  mem_lat = 4;
  max_cycle = 1ULL<<34;
  max_icnt = 1ULL<<50;

  
  uint32_t max_insns_per_cycle = 4;
  uint32_t max_insns_per_cycle_hist_sz = 2*max_insns_per_cycle;

  std::map<uint32_t, uint64_t> mispredicts;

  uint64_t hist = 0, spec_hist = 0;
  
  uint64_t inflight[32] = {0};
  uint32_t max_inflight = 0;


  const std::unique_ptr<VerilatedContext> contextp{new VerilatedContext};
  contextp->commandArgs(argc, argv);  
  s = new state_t;
  initState(s);
  
  std::unique_ptr<Vcore_l1d_l1i> tb(new Vcore_l1d_l1i);
  uint64_t last_retire = 0, last_check = 0, last_restart = 0;
  uint64_t last_retired_pc = 0, last_retired_fp_pc = 0;
  uint64_t mismatches = 0, n_stores = 0, n_loads = 0;
  uint64_t n_branches = 0, n_mispredicts = 0, n_checks = 0, n_flush_cycles = 0;
  bool got_mem_req = false, got_mem_rsp = false, got_monitor = false, incorrect = false;

  
  globals::syscall_emu = false;
  tb->syscall_emu = 0;
  
  loadState(*s, rv32_binary.c_str());
  
  for(int i = 0; i < 32; i++) {
    assert(s->gpr[i] == 0);
  }
    
  reset_core(tb, cycle, s->pc);
  
  double t0 = timestamp();
  while(!Verilated::gotFinish() && not(done)) {
    contextp->timeInc(1);  // 1 timeprecision periodd passes...    

    tb->clk = 1;
    tb->eval();
    assert(!tb->got_monitor);
    

    if(tb->retire_reg_valid) {
      s->gpr[tb->retire_reg_ptr] = tb->retire_reg_data;
    }

    
#ifdef BRANCH_DEBUG
    if(tb->branch_pc_valid) {
      ++n_branches;
    }
    if(tb->branch_fault) {
      mispredicts[tb->branch_pc]++; 
    }
    if(tb->branch_fault) {
      ++n_mispredicts;
    }
#endif
    if(tb->in_flush_mode) {
      ++n_flush_cycles;
    }
    
    if(tb->alloc_two_valid) {
      insns_allocated+=2;
    }
    else if(tb->alloc_valid) {
      insns_allocated++;
    }

    if(tb->iq_one_valid) {
      fetch_stalls++;
    }
    else if(tb->iq_none_valid) {
      fetch_stalls+=2;
    }
    
    if(tb->in_branch_recovery) {
      cycles_in_faulted++;
    }
    
    if(tb->retire_valid) {
      ++insns_retired;
      if(last_retire > 1) {
	pushout_histo[tb->retire_pc] += last_retire;
      }
      last_retire = 0;

      last_retired_pc = tb->retire_pc;


      if(((insns_retired % (1<<20)) == 0)) {
	++csr_time;
      }
      
      if(((insns_retired % heartbeat) == 0) or trace_retirement ) {

	std::cout << "port a "
		  << " cycle " << cycle
		  << " "
		  << std::hex
		  << tb->retire_pc
		  << std::dec
		  << " "
		  << std::fixed
		  << ", " << static_cast<double>(insns_retired) / cycle << " IPC "
		  << ", insns_retired "
		  << insns_retired
		  << ", mem pki "
		  << ((static_cast<double>(mem_reqs)/insns_retired)*100.0)
		  << ", mispredict pki "
		  << (static_cast<double>(n_mispredicts) / insns_retired) * 1000.0
		  << std::defaultfloat
		  << std::hex
		  << " "
		  << tb->retire_reg_data
		  << std::dec	  
		  <<" \n";
      }
      if(tb->retire_two_valid) {
	++insns_retired;
	last_retired_pc = tb->retire_pc;	
	if(((insns_retired % heartbeat) == 0) or trace_retirement ) {
	  std::cout << "port b "
		    << " cycle " << cycle
		    << " "
		    << std::hex
		    << tb->retire_two_pc
		    << std::dec
		    << " "
		    << std::fixed
		    << ", " << static_cast<double>(insns_retired) / cycle << " IPC "	    
		    << ", insns_retired "
		    << insns_retired
		    << ", mem pki "
		    << ((static_cast<double>(mem_reqs)/insns_retired)*100.0)
		    << ", mispredict pki "
		    << (static_cast<double>(n_mispredicts) / insns_retired) * 1000.0
		    << std::defaultfloat
		    << std::hex
		    << " "
		    << tb->retire_reg_two_data
		    << std::dec
		    <<" \n";
	}
      }

      
      if(tb->got_bad_addr) {
	std::cout << "fatal - unaligned address\n";
	break;
      }
    }
    
    if(tb->retire_reg_two_valid) {
      s->gpr[tb->retire_reg_two_ptr] = tb->retire_reg_two_data;
    }
    
    ++last_retire;
    if(last_retire > (1U<<16) && not(tb->in_flush_mode)) {
      std::cout << "in flush mode = " << static_cast<int>(tb->in_flush_mode) << "\n";
      std::cerr << "no retire in " << last_retire << " cycles, last retired "
    		<< std::hex
    		<< last_retired_pc + 0
    		<< std::dec
    		<< "\n";
      break;
    }
    if(tb->got_break) {
      std::cout << "got break, epc = " << std::hex << tb->epc << std::dec << "\n";      
      break;
    }

    
    if(tb->got_ud) {
      std::cerr << "GOT UD for "
		<< std::hex
		<< tb->epc
		<< std::dec
		<< "\n";
      break;
    }
    else if(tb->got_bad_addr) {
      std::cerr << "GOT VA for "
		<< std::hex
		<< tb->epc
		<< std::dec
		<< "\n";
      break;
    }
    inflight[tb->inflight & 31]++;
    max_inflight = std::max(max_inflight, static_cast<uint32_t>(tb->inflight));

    //negedge
    tb->mem_rsp_valid = 0;

    if(tb->mem_req_valid && (mem_reply_cycle == -1)) {
      ++mem_reqs;
      mem_reply_cycle = cycle + (tb->mem_req_opcode == 4 ? 1 : 2)*mem_lat;
      
    }
    
    if(mem_reply_cycle ==cycle) {
      last_retire = 0;
      mem_reply_cycle = -1;
      assert(tb->mem_req_valid);

      
      if(tb->mem_req_opcode == 4) {/*load word */
	for(int i = 0; i < 4; i++) {
	  uint64_t ea = (tb->mem_req_addr + 4*i) & ((1ULL<<32)-1);
	  tb->mem_rsp_load_data[i] = read_word(ea);
	}
	last_load_addr = tb->mem_req_addr;
	assert((tb->mem_req_addr & 0xf) == 0);
	++n_loads;
      }
      else if(tb->mem_req_opcode == 7) { /* store word */
	for(int i = 0; i < 4; i++) {
	  uint64_t ea = (tb->mem_req_addr + 4*i) & ((1ULL<<32)-1);
	  write_word(ea, tb->mem_req_store_data[i], 0, 0);
	}
	last_store_addr = tb->mem_req_addr;
	++n_stores;
      }
      last_addr = tb->mem_req_addr;
      tb->mem_rsp_valid = 1;
    }

    
    tb->clk = 0;
    tb->eval();
    if(got_mem_req) {
      got_mem_req = false;
    }
    if(got_mem_rsp) {
      tb->mem_rsp_valid = 0;
      got_mem_rsp = false;
    }
    
    if(got_monitor) {
      tb->monitor_ack = 0;
      got_monitor = false;
    }
    ++cycle;
  }
  tb->final();
  t0 = timestamp() - t0;

  
  if(!incorrect) {
    std::cout << "total_retire = " << insns_retired << "\n";
    std::cout << "total_cycle  = " << cycle << "\n";
    std::cout << "total ipc    = " << static_cast<double>(insns_retired) / cycle << "\n";
  }
  else {
    std::cout << "instructions retired = " << insns_retired << "\n";
  }
  
  std::cout << "simulation took " << t0 << " seconds, " << (insns_retired/t0)
	    << " insns per second\n";


  delete s;

  exit(EXIT_SUCCESS);
}
