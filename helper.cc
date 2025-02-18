
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <cxxabi.h>

#ifdef __amd64__
#include <x86intrin.h>
#endif

#include "helper.hh"
#include "interpret.hh"


#define	_SIM_FOPEN		(-1)	
#define	_SIM_FREAD		0x0001	
#define	_SIM_FWRITE		0x0002	
#define	_SIM_FAPPEND	        0x0008	
#define	_SIM_FMARK		0x0010
#define	_SIM_FDEFER		0x0020
#define	_SIM_FASYNC		0x0040
#define	_SIM_FSHLOCK	        0x0080
#define	_SIM_FEXLOCK	        0x0100
#define	_SIM_FCREAT		0x0200
#define	_SIM_FTRUNC		0x0400
#define	_SIM_FEXCL		0x0800
#define	_SIM_FNBIO		0x1000
#define	_SIM_FSYNC		0x2000
#define	_SIM_FNONBLOCK	        0x4000
#define	_SIM_FNDELAY	        _SIM_FNONBLOCK
#define	_SIM_FNOCTTY	        0x8000	

#define	O_SIM_RDONLY	0		/* +1 == FREAD */
#define	O_SIM_WRONLY	1		/* +1 == FWRITE */
#define	O_SIM_RDWR	2		/* +1 == FREAD|FWRITE */
#define	O_SIM_APPEND	_SIM_FAPPEND
#define	O_SIM_CREAT	_SIM_FCREAT
#define	O_SIM_TRUNC     _SIM_FTRUNC
#define	O_SIM_EXCL      _SIM_FEXCL
#define O_SIM_SYNC	_SIM_FSYNC
#define	O_SIM_NONBLOCK	_SIM_FNONBLOCK
#define	O_SIM_NOCTTY	_SIM_FNOCTTY
#define	O_SIM_ACCMODE	(O_SIM_RDONLY|O_SIM_WRONLY|O_SIM_RDWR)

static const int32_t simIOFlags[] = 
  {O_SIM_RDONLY,
   O_SIM_WRONLY,
   O_SIM_RDWR,
   O_SIM_APPEND,
   O_SIM_CREAT,
   O_SIM_TRUNC,
   O_SIM_EXCL,
   O_SIM_SYNC,
   O_SIM_NONBLOCK,
   O_SIM_NOCTTY
  };

static const int32_t hostIOFlags[] = 
  {
    O_RDONLY,
    O_WRONLY,
    O_RDWR,
    O_APPEND,
    O_CREAT,
    O_TRUNC,
    O_EXCL,
    O_SYNC,
    O_NONBLOCK,
    O_NOCTTY
  };


uint32_t update_crc(uint32_t crc, uint8_t *buf, size_t len) {
  static const uint32_t POLY = 0x82f63b78;
  uint32_t c = crc;
  for(size_t n=0;n<len;n++) {
    uint8_t b = buf[n];
    c ^= b;
    for(int k = 0; k < 8; k++) {
      c = c & 1 ? (c>>1) ^ POLY : c>>1;
    }
  }
  return c;
}

uint32_t crc32(uint8_t *buf, size_t len) {
  return update_crc(~0x0, buf, len) ^ (~0x0);
}


int32_t remapIOFlags(int32_t flags) {
  int32_t nflags = 0;
  for(size_t i = 0; i < sizeof(simIOFlags)/sizeof(simIOFlags[0]); i++) {
    if(flags & simIOFlags[i])
      nflags |= hostIOFlags[i];
  }
  return nflags;
}

double timestamp() {
  struct timeval t;
  gettimeofday(&t,nullptr);
  return t.tv_sec + 1e-6*t.tv_usec;
}

void dbt_backtrace() {}
