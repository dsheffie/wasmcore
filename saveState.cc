#include <cstdint>
#include <cassert>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include "interpret.hh"
#include "globals.hh"

struct page {
  uint32_t va;
  uint8_t data[4096];
} __attribute__((packed));

static const uint64_t MAGICNUM = 0x64646464beefd00dUL;

struct header {
  uint64_t magic;
  uint64_t pc;
  int64_t gpr[32];
  uint64_t icnt;
  uint32_t num_nz_pages;
  uint64_t tohost_addr;
  uint64_t fromhost_addr;  
  header() {}
} __attribute__((packed));

void dumpState(const state_t &s, const std::string &filename) {}

void loadState(state_t &s, const std::string &filename) {
  header h;
  int fd = ::open(filename.c_str(), O_RDONLY, 0600);
  assert(fd != -1);
  size_t sz = read(fd, &h, sizeof(h));
  assert(sz == sizeof(h));

  s.pc = h.pc;
  memcpy(&s.gpr,&h.gpr,sizeof(s.gpr));
  s.icnt = h.icnt;

  
  for(uint32_t i = 0; i < h.num_nz_pages; i++) {
    page p;
    sz = read(fd, &p, sizeof(p));
    uint64_t page_id = (p.va >> 12);

    if(s.mtbl[page_id] == nullptr) {
      //printf("allocating page %d\n", page_id);
      s.mtbl[page_id] = new uint8_t[4096];
    }
    assert(sz == sizeof(p));
    memcpy(s.mtbl[page_id], p.data, 4096);
  }
  close(fd);
}

