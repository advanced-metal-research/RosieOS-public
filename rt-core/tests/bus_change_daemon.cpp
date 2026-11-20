#define ROSIE_RT_SIMULATION 1
// Unverified fixture version; the probe runs only the simulated backend.
#define ROSIE_RT_VERSION "bus-change-test"
#include "ecrt_detect.hpp"
#include <cassert>
#include <cstdio>
// Unverified trace capacity covers every uint16 ring position.
struct Counts { unsigned configs=0,pdos=0,downloads=0,startup=0,registrations=0,bytes=0; };
Counts counts[65536];
ec_slave_config_t* trace_config(ec_master_t* m,uint16_t a,uint16_t p,uint32_t v,uint32_t c) {
  ++counts[p].configs; return ecrt_master_slave_config(m,a,p,v,c);
}
int trace_pdos(ec_slave_config_t* s,unsigned n,const ec_sync_info_t* syncs) {
  ++counts[s->position].pdos; return ecrt_slave_config_pdos(s,n,syncs);
}
int trace_download(ec_master_t* m,uint16_t p,uint16_t i,uint8_t sub,const uint8_t* b,size_t n,uint32_t* a) {
  ++counts[p].downloads; return ecrt_master_sdo_download(m,p,i,sub,b,n,a);
}
int trace_startup(ec_slave_config_t* s,uint16_t i,uint8_t sub,const uint8_t* b,size_t n) {
  ++counts[s->position].startup; return ecrt_slave_config_sdo(s,i,sub,b,n);
}
int trace_startup16(ec_slave_config_t* s,uint16_t i,uint8_t sub,uint16_t v) {
  ++counts[s->position].startup; return ecrt_slave_config_sdo16(s,i,sub,v);
}
int trace_regs(ec_domain_t* d,const ec_pdo_entry_reg_t* entries) {
  for (auto* r=entries;r->index;++r) ++counts[r->position].registrations;
  return ecrt_domain_reg_pdo_entry_list(d,entries);
}
int trace_reg(ec_slave_config_t* s,uint8_t sync,unsigned pdo,unsigned entry,ec_domain_t* d,unsigned* bit) {
  ++counts[s->position].registrations;
  return ecrt_slave_config_reg_pdo_entry_pos(s,sync,pdo,entry,d,bit);
}
int trace_send(ec_master_t* m) {
  for (const auto& s:m->slaves) for (const auto& e:s->entries)
    if (e.domain && e.domain->queued && e.sync==2) counts[s->position].bytes+=e.info.bit_length/8;
  return ecrt_master_send(m);
}
#define ecrt_master_slave_config trace_config
#define ecrt_slave_config_pdos trace_pdos
#define ecrt_master_sdo_download trace_download
#define ecrt_slave_config_sdo trace_startup
#define ecrt_slave_config_sdo16 trace_startup16
#define ecrt_domain_reg_pdo_entry_list trace_regs
#define ecrt_slave_config_reg_pdo_entry_pos trace_reg
#define ecrt_master_send trace_send
#include <cstdlib>
#include <fstream>
// The publisher owns topology observations under the shared master mutex. Only
// its scanned extra-device fields change here; cyclic PDO fixtures stay intact.
int changed_master_info(ec_master_t* master, ec_master_info_t* info) {
  if (master->active) {
    const char* path = std::getenv("RTCORE_TEST_TOPOLOGY_CHANGE");
    if (path) {
      std::ifstream command(path);
      unsigned change = 0;
      if (command >> change) {
        assert(master->faults.extra_slaves.size() == 3);
        master->faults.extra_slaves[0].vendor_id = change == 1 ? 1 : 4194304;
        master->faults.extra_slaves[2].present = change != 2;
      }
    }
  }
  return ecrt_master(master, info);
}
#define ecrt_master changed_master_info
#define main daemon_main
#include "src/main.cpp"
#undef main
int main(int argc, char** argv) {
  const int code = daemon_main(argc, argv);
  for (unsigned position = 6; position < 9; ++position) {
    const auto& c = counts[position];
    std::printf("TRACE position=%u configs=%u pdos=%u downloads=%u startup=%u registrations=%u output_bytes=%u\n",
                position, c.configs, c.pdos, c.downloads, c.startup, c.registrations, c.bytes);
    assert(c.configs == 0 && c.pdos == 0 && c.downloads == 0 && c.startup == 0 && c.registrations == 0 && c.bytes == 0);
  }
  assert(counts[0].configs > 0 && counts[0].bytes > 0);
  return code;
}
