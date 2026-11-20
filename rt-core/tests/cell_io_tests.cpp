#define ROSIE_RT_SIMULATION 1
#include "cell_io.hpp"
#include "io_wait.hpp"
#include "cycle_bus.hpp"
#include <cassert>
#include <cstdlib>
#include <sstream>
#include <unistd.h>
#include <vector>

using namespace rosie::rtcore;
using namespace rosie::protocol;
// Unverified simulator terminal, two contacts and one 5000000 ns process lease.
constexpr const char* fixture_unverified="1|2|4026585352|0|5632|6656|2|3|28672|1|24576|1;consent|0|1|1,door_display|2|0|0;gas|0|0|5000000|0|1|1";

void TestCellIOFencingExpiryAndReadback() {
  auto config=parse_cell_io(fixture_unverified); CellIO io(config);
  assert(io.arm(1,1)==reason_io_exchange_lost);
  io.observe(1,true,1000000,1); assert(io.fast_satisfied());
  assert(io.arm(1,1)==0); assert(io.intent(0,true,1000000,1,1000000)==0);
  assert(io.output(1000000,true,1,1)==1);
  assert(io.input_valid() && io.input_word()==1 && io.output_word()==1);
  assert(io.state().outputs[0].intent==1 && io.state().outputs[0].commanded==1 && io.state().outputs[0].readback==0);
  io.observe(3,true,2000000,2); assert(io.state().outputs[0].readback==1);
  assert(io.output(5999999,true,1,1)==1); assert(io.output(6000000,true,1,1)==0);
  assert(io.state().outputs[0].expiry_ns==0);
  io.observe(1,true,7000000,7); assert(io.intent(0,true,7000000,7,7000000)==0);
  assert(io.output(7000000,true,1,1)==1);
  io.fence(1,1); assert(io.output(7000000,true,1,1)==0);
  assert(io.intent(0,true,7000000,7,7000000)==reason_io_not_armed);
  assert(io.arm(1,1)==reason_wrong_generation);
  assert(io.arm(1,2)==reason_io_readback_disagreement);
  io.begin_cycle(8000000,8); assert(io.arm(1,2)==reason_io_readback_disagreement);
  io.observe(1,true,8000000,8);
  assert(io.arm(1,2)==0); assert(io.intent(0,true,8000000,8,8000000)==0);
  assert(io.output(8000000,true,1,2)==1);
  io.observe(1,true,10000000,10); assert(io.state().reason!=reason_io_readback_disagreement);
  io.observe(1,true,11000000,11); assert(io.state().reason==reason_io_readback_disagreement);
  assert(io.output(11000000,true,1,2)==0 && io.state().armed==0);
  io.observe(0,false,12000000,12); assert(!io.fast_satisfied());
  assert(!io.input_valid() && io.input_word()==0 && io.output_word()==0);
  assert(io.state().inputs[0].valid==0 && io.state().inputs[0].value==0 && io.state().inputs[0].observed_ns==11000000);
  assert(io.arm(1,2)==reason_io_exchange_lost);
}

void TestCellIOSimulatedExchange() {
  auto config=parse_cell_io(fixture_unverified); CellIO io(config);
  auto* master=ecrt_request_master(0); auto* domain=ecrt_master_create_domain(master);
  auto* terminal=ecrt_master_slave_config(master,0,1,2,4026585352);
  ec_pdo_entry_info_t output{0x7000,1,8},input{0x6000,1,8};
  ec_pdo_info_t rx{0x1600,1,&output},tx{0x1a00,1,&input};
  ec_sync_info_t syncs[]={{2,EC_DIR_OUTPUT,1,&rx,EC_WD_ENABLE},{3,EC_DIR_INPUT,1,&tx,EC_WD_DISABLE},{0xff,EC_DIR_INVALID,0,nullptr,EC_WD_DISABLE}};
  assert(ecrt_slave_config_pdos(terminal,EC_END,syncs)==0);
  terminal->dio=true; terminal->dio_input_index=0x6000; terminal->dio_input_subindex=1;
  terminal->dio_output_index=0x7000; terminal->dio_output_subindex=1; terminal->dio_feedback_mask[0]=2;
  unsigned int in=0,out=0;
  ec_pdo_entry_reg_t regs[]={{0,1,2,4026585352,0x6000,1,&in,nullptr},{0,1,2,4026585352,0x7000,1,&out,nullptr},{}};
  assert(ecrt_domain_reg_pdo_entry_list(domain,regs)==0);
  master->faults.dio_inputs=1; master->faults.dio_readback_lag_cycles=1;
  // Activation consumes receive call 1; injected edges use absolute call counts.
  master->faults.dio_edge_cycle=5; master->faults.dio_edge_inputs=0; master->faults.dio_loss_cycle=6;
  assert(ecrt_master_activate(master)==0); CycleBus bus;
  DomainBinding binding{domain,1,ecrt_domain_data(domain)}; assert(bus.configure(master,1,&binding,1));
  assert(!bus.begin_cycle(1000000)); assert(bus.queue(1).succeeded()); assert(bus.submit(1).succeeded());
  assert(bus.begin_cycle(2000000)); io.observe(bus.axis(0).read_u8(in),true,2000000,2);
  assert(io.arm(1,1)==0); assert(io.intent(0,true,2000000,2,2000000)==0);
  bus.axis(0).write_u8(out,io.output(2000000,true,1,1)); assert(bus.queue(1).succeeded()); assert(bus.submit(1).succeeded());
  assert(bus.begin_cycle(3000000)); io.observe(bus.axis(0).read_u8(in),true,3000000,3);
  assert(io.state().outputs[0].readback==0); assert(bus.queue(1).succeeded()); assert(bus.submit(1).succeeded());
  assert(bus.begin_cycle(4000000)); io.observe(bus.axis(0).read_u8(in),true,4000000,4);
  assert(io.state().outputs[0].readback==1 && !io.fast_satisfied());
  bus.axis(0).write_u8(out,io.output(4000000,true,1,1)); assert(bus.axis(0).read_u8(out)==0);
  assert(bus.queue(1).succeeded()); assert(bus.submit(1).succeeded());
  assert(!bus.begin_cycle(5000000)); io.observe(bus.axis(0).read_u8(in),false,5000000,5);
  assert(io.state().inputs[0].value==0 && io.state().inputs[0].valid==0);
  assert(io.state().outputs[0].valid==0 && io.output(5000000,true,1,1)==0);
  ecrt_release_master(master);
}

void TestCellIOMarkerRefusalsAndWireLayout() {
  auto config=parse_cell_io(fixture_unverified); CellIO io(config);
  io.observe(1,true,10000000,10); assert(io.arm(2,3)==0);
  assert(io.intent(0,true,8000000,8,10000000)==reason_io_marker_late);
  assert(io.state().outputs[0].intent==0 && io.output(10000000,true,2,3)==0);
  assert(io.intent(0,true,9000000,9,10000000)==0);
  assert(io.output(10000000,true,2,3)==1);
  io.observe(3,true,11000000,11);
  auto wire=gradient::ipc::v1::encode_IoStateV1(io.state());
  static_assert(sizeof(gradient::ipc::v1::IoStateV1)==2464);
  static_assert(offsetof(gradient::ipc::v1::IoStateV1,outputs)==928);
  assert(wire[16]==1 && wire[24]==2 && wire[28]==1);
  assert(wire[32]=='c' && wire[72]==1 && wire[76]==1);
  assert(wire[928]=='g' && wire[976]==1 && wire[980]==1 && wire[984]==1 && wire[988]==1);
  gradient::ipc::v1::IoStateV1 decoded{};
  assert(gradient::ipc::v1::decode_IoStateV1(wire.data(),wire.size(),decoded));
  assert(decoded.outputs[0].expiry_ns==15000000 && decoded.outputs[0].observed_ns==11000000);
  io.fence(2,3); io.observe(1,true,12000000,12);
  assert(io.arm(2,3)==reason_wrong_generation);
  assert(io.arm(2,4)==0);
  assert(io.intent(0,true,12000000,12,12000000)==0);
  assert(io.output(12000000,true,2,3)==0); // A delayed grant cannot command ON.
  config.outputs[0].torch=true; CellIO torch(config);
  torch.observe(1,true,12000000,12); assert(torch.arm(2,4)==0);
  assert(torch.intent(0,false,12000000,12,12000000)==reason_io_torch_unqualified);
  assert(torch.intent(0,true,12000000,12,12000000)==reason_io_torch_unqualified);
}

// Unverified example: PLANET IECS-1116-DO identity from its ESI with no input
// module fitted (owner decision 2026-10-12); gas on channel 0, weld on channel
// 11 in the second output byte, 5000000 ns leases.
constexpr const char* planet_unverified="v2|ethercat|1|2271|68|1|16|0|0:6656:12545:1+1:6657:12545:2|;;gas|0|0|5000000|0|-|-,weld|11|0|5000000|0|-|-";

void TestCellIOOutputsWithoutReadback() {
  auto config=parse_cell_io(planet_unverified); CellIO io(config);
  assert(config.schema==2 && config.on_bus() && config.output_pdo_count==2 && config.input_pdo_count==0);
  assert(!config.outputs[0].wired && !config.outputs[1].wired && config.outputs[1].bit==11);
  assert(io.arm(1,1)==reason_io_exchange_lost);
  io.observe(0,true,1000000,1);
  assert(io.arm(1,1)==0); // no OFF observation exists to require
  assert(io.intent(1,true,1000000,1,1000000)==0);
  assert(io.output(1000000,true,1,1)==(1u<<11) && io.output_word()==(1u<<11));
  // Commanded ON and never observed: no disagreement fault, however long.
  for(uint64_t cycle=2;cycle<6;++cycle) {
    io.observe(0,true,cycle*1000000,cycle);
    assert(io.state().reason==0 && io.state().armed==1);
  }
  assert(io.state().outputs[1].commanded==1 && io.state().outputs[1].valid==0 && io.state().outputs[1].readback==0);
  assert(io.output(5999999,true,1,1)==(1u<<11)); assert(io.output(6000000,true,1,1)==0);
  assert(io.intent(0,true,6000000,5,6000000)==0); assert(io.output(6000000,true,1,1)==1);
  io.fence(1,1); assert(io.output(6000000,true,1,1)==0);
  // Without feedback the only post-fence proof is a fresh exchange after the fence.
  assert(io.arm(1,2)==reason_io_readback_disagreement);
  io.observe(0,true,7000000,7); assert(io.arm(1,2)==0);
  assert(io.intent(0,true,7000000,7,7000000)==0);
  io.observe(0,false,8000000,8);
  assert(io.output(8000000,true,1,2)==0 && io.state().reason==reason_io_exchange_lost);
  std::ostringstream describe; append_cell_io_config_json(describe,config);
  assert(describe.str()=="{\"backend\":\"ethercat\",\"output_channels\":16,\"input_channels\":0,\"slave_position\":1,"
      "\"vendor_id\":2271,\"product_code\":68,\"revision_no\":1,\"torch_qualified\":false,\"inputs\":[],\"outputs\":["
      "{\"name\":\"gas\",\"bit\":0,\"class\":\"process\",\"safe_state\":false,\"expiry_ns\":5000000,\"readback\":\"none_unverified\"},"
      "{\"name\":\"weld\",\"bit\":11,\"class\":\"process\",\"safe_state\":false,\"expiry_ns\":5000000,\"readback\":\"none_unverified\"}]}");
}

void TestCellIOSchema2Refusals() {
  auto refused=[](const std::string& text) {
    try { (void)parse_cell_io(text); } catch(const std::invalid_argument&) { return true; }
    return false;
  };
  assert(!refused(planet_unverified));
  assert(refused("v2|ethercat|1|2271|68|1|16|0|0:6656:12545:1|;;gas|0|0|5000000|0|-|-"));               // 16 channels, one byte
  assert(refused("v2|ethercat|1|2271|68|1|16|0|0:6656:12545:1+0:6657:12545:2|;;gas|0|0|5000000|0|-|-")); // shared sync manager
  assert(refused("v2|ethercat|1|2271|68|1|16|0|0:6656:12545:1+1:6657:12545:2|;;gas|16|0|5000000|0|-|-")); // channel 16
  assert(refused("v2|ethercat|1|2271|68|1|16|0|0:6656:12545:1+1:6657:12545:2|;;gas|0|0|5000000|0|0|1"));  // wired, no inputs
  assert(refused("v2|ethercat|1|2271|68|1|16|0|0:6656:12545:1+1:6657:12545:2|;;gas|0|0|5000000|0|-|1"));  // half a readback
  assert(refused("v2|ethercat|1|2271|68|1|16|0|0:6656:12545:1+1:6657:12545:2|;door|0|1|0;gas|0|0|5000000|0|-|-")); // input, no channels
  assert(refused("v2|picontrol|/dev/../x|70|0|14|14;;weld|4|0|5000000|0|-|-"));
  assert(refused("v2|picontrol|/dev/piControl0|70|71|14|14;;weld|4|0|5000000|0|-|-"));   // overlapping words
  assert(refused("v2|picontrol|/dev/piControl0|4095|0|14|14;;weld|4|0|5000000|0|-|-"));  // past the image end
  assert(refused("v2|picontrol|/dev/piControl0|70|0|14|14;;weld|14|0|5000000|0|-|-"));   // O_15 does not exist
  auto pi=parse_cell_io("v2|picontrol|/dev/piControl0|70|0|14|14;;weld|4|0|5000000|0|13|1");
  assert(pi.backend==IoBackend::picontrol && !pi.on_bus() && pi.outputs[0].wired && pi.outputs[0].readback_bit==13);
}

// A regular file stands in for /dev/piControl0: same pread/pwrite offsets.
void TestCellIOPiControlImage() {
  const char* tmp=std::getenv("TMPDIR");
  std::string path=std::string(tmp && *tmp ? tmp : "/tmp")+"/cellio-picontrol-XXXXXX";
  const int fd=mkstemp(path.data()); assert(fd>=0);
  const std::vector<uint8_t> image(picontrol_image_bytes);
  assert(::write(fd,image.data(),image.size())==ssize_t(image.size()));
  auto config=parse_cell_io("v2|picontrol|"+path+"|70|0|14|14;door|2|1|0;weld|4|0|5000000|0|-|-");
  CellIO io(config); CellIOWire wire; PdoView unused(nullptr);
  assert(wire.picontrol.open(config));
  uint16_t word=0;
  assert(!wire.read(config,unused,true,word)); // nothing written by this process yet
  wire.write(config,unused,0); assert(wire.picontrol_written);
  const uint8_t inputs[2]={0x04,0x00}; assert(pwrite(fd,inputs,2,0)==2); // I_3 high
  assert(wire.read(config,unused,false,word) && word==0x0004); // EtherCAT freshness is irrelevant
  io.observe(word,true,1000000,1); assert(io.state().inputs[0].value==1);
  assert(io.arm(1,1)==0 && io.intent(0,true,1000000,1,1000000)==0);
  wire.write(config,unused,io.output(1000000,true,1,1));
  uint8_t outputs[2]={};
  assert(pread(fd,outputs,2,70)==2 && outputs[0]==0x10 && outputs[1]==0); // O_5
  wire.write(config,unused,uint16_t(1u<<9));
  assert(pread(fd,outputs,2,70)==2 && outputs[0]==0 && outputs[1]==0x02);  // O_10, second byte
  wire.picontrol.close();
  assert(pread(fd,outputs,2,70)==2 && outputs[0]==0 && outputs[1]==0);     // clean close writes OFF
  assert(!wire.picontrol.write(1));
  ::close(fd); ::unlink(path.c_str());
}

// Two one-byte output PDOs on separate sync managers and no input PDO, as the
// IECS-1116-DO ESI declares; the scanned layout is what bring-up compares.
void TestCellIOMailboxFreeSimExchange() {
  auto config=parse_cell_io(planet_unverified); CellIO io(config);
  auto* master=ecrt_request_master(0); auto* domain=ecrt_master_create_domain(master);
  auto* terminal=ecrt_master_slave_config(master,0,1,2271,68);
  ec_pdo_entry_info_t low{0x3101,1,8},high{0x3101,2,8};
  ec_pdo_info_t rx0{0x1a00,1,&low},rx1{0x1a01,1,&high};
  ec_sync_info_t syncs[]={{0,EC_DIR_OUTPUT,1,&rx0,EC_WD_ENABLE},{1,EC_DIR_OUTPUT,1,&rx1,EC_WD_ENABLE},{0xff,EC_DIR_INVALID,0,nullptr,EC_WD_DISABLE}};
  assert(ecrt_slave_config_pdos(terminal,EC_END,syncs)==0);
  ec_sync_info_t sync{}; ec_pdo_info_t pdo{}; ec_pdo_entry_info_t entry{};
  assert(ecrt_master_get_sync_manager(master,1,1,&sync)==0 && sync.n_pdos==1);
  assert(ecrt_master_get_pdo(master,1,1,0,&pdo)==0 && pdo.index==0x1a01 && pdo.n_entries==1);
  assert(ecrt_master_get_pdo_entry(master,1,1,0,0,&entry)==0 && entry.index==0x3101 && entry.subindex==2 && entry.bit_length==8);
  terminal->dio=true; terminal->dio_output_index=0x3101; terminal->dio_output_subindex=1;
  terminal->dio_output_high_index=0x3101; terminal->dio_output_high_subindex=2;
  CellIOWire wire;
  ec_pdo_entry_reg_t regs[]={{0,1,2271,68,0x3101,1,&wire.output_offsets[0],nullptr},{0,1,2271,68,0x3101,2,&wire.output_offsets[1],nullptr},{}};
  assert(ecrt_domain_reg_pdo_entry_list(domain,regs)==0);
  assert(ecrt_master_activate(master)==0); CycleBus bus;
  DomainBinding binding{domain,1,ecrt_domain_data(domain)}; assert(bus.configure(master,1,&binding,1));
  assert(!bus.begin_cycle(1000000)); assert(bus.queue(1).succeeded()); assert(bus.submit(1).succeeded());
  uint16_t word=0xffff;
  assert(bus.begin_cycle(2000000)); assert(wire.read(config,bus.axis(0),true,word) && word==0);
  io.observe(word,true,2000000,2); assert(io.arm(1,1)==0);
  assert(io.intent(0,true,2000000,2,2000000)==0 && io.intent(1,true,2000000,2,2000000)==0);
  wire.write(config,bus.axis(0),io.output(2000000,true,1,1));
  assert(bus.queue(1).succeeded()); assert(bus.submit(1).succeeded());
  assert(bus.begin_cycle(3000000)); assert(terminal->dio_physical==((1u<<11)|1u));
  ecrt_release_master(master);
}

void TestSixteenNamedChannelsAndManualFencing() {
  IoConfig config;
  config.output_count=16; config.input_count=16;
  for (unsigned i=0;i<16;++i) {
    config.inputs[i]={"I"+std::to_string(i+1),uint8_t(i),true,false};
    config.outputs[i]={"O"+std::to_string(i+1),uint8_t(i),0,false,false,false,200000000};
  }
  config.output_channels=16; config.input_channels=16;
  config.canonical="test";
  CellIO io(config);
  io.observe(1u<<13,true,1000000,1);
  assert(io.state().inputs[13].value && io.state().inputs[13].valid);
  assert(!io.state().inputs[12].value && io.state().inputs[12].valid);
  assert(io.arm(2,3)==0);
  assert(io.test(13,true,1,11000000,1000000,2,3,true)==0);
  assert(io.manual() && io.output(1000000,true,2,3)==(1u<<13));
  assert(io.test(13,false,3,0,2000000,2,3)==0);
  assert(io.test(13,true,2,12000000,2000000,2,3,true)==reason_invalid_command);
  assert(io.output(2000000,true,2,3)==0);
  assert(io.test(13,true,4,12000000,2000000,2,3,true)==0);
  assert(io.output(12000000,true,2,3)==0);
  io.disarm(); assert(!io.manual());
  assert(io.test(13,true,5,22000000,12000000,2,3)==reason_io_not_armed);
  assert(io.arm(2,4)==0);
  // An OFF arriving before the first ON of a new session still fences it.
  assert(io.test(13,false,2,0,12000000,2,4)==0);
  assert(io.test(13,true,1,22000000,12000000,2,4,true)==reason_invalid_command);
}

void TestInputWaitStability() {
  IoWait wait; using R=IoWait::Result;
  assert(wait.step(0,true,false,true,100,20)==R::pending);
  assert(wait.step(5,true,true,true,100,20)==R::pending);
  assert(wait.step(15,true,false,true,100,20)==R::pending);
  assert(wait.step(20,true,true,true,100,20)==R::pending);
  assert(wait.step(39,true,true,true,100,20)==R::pending);
  assert(wait.step(40,true,true,true,100,20)==R::satisfied);
  wait.reset(); assert(wait.step(0,true,false,true,100,0)==R::pending);
  assert(wait.step(100,true,false,true,100,0)==R::timed_out);
  wait.reset(); assert(wait.step(0,false,true,true,100,0)==R::invalid_input);
  wait.reset(); assert(wait.step(0,true,false,true,100,0)==R::pending);
  assert(wait.step(101,true,true,true,100,0)==R::timed_out); // late edge never resumes
}

void TestManualTorchIsBoundedAndDoesNotQualifyProduction() {
  auto config=parse_cell_io(fixture_unverified);
  config.outputs[0].torch=true; config.outputs[0].wired=false;
  CellIO io(config); io.observe(1,true,1000000,1);
  assert(io.test(0,true,1,5000000,1000000,1,1,true)==0);
  assert(io.manual() && io.output(1000000,true,1,1)==1);
  assert(io.intent(0,true,1000000,1,1000000)==reason_io_torch_unqualified);
  assert(io.test(0,false,3,0,2000000,1,1)==0);
  assert(io.test(0,true,2,6000000,2000000,1,1,true)==reason_invalid_command);
  assert(io.output(2000000,true,1,1)==0);
  assert(io.test(0,true,4,6000000,2000000,1,1,true)==0);
  assert(io.output(2000000,true,1,1)==1);
  io.expire(6000000); assert(io.output(6000000,true,1,1)==0);
  assert(io.state().reason==reason_none);
  assert(io.test(0,true,5,10000000,6000000,1,1)==reason_io_not_armed);
  assert(io.test(0,true,6,10000000,6000000,1,1,true)==0);
  io.fence(1,1); assert(io.output(6000000,true,1,1)==0);
  assert(io.intent(0,true,6000000,1,6000000)==reason_io_torch_unqualified);
}

void TestQualifiedWeldExpiryStopsAllOutputs() {
  auto config=parse_cell_io(fixture_unverified);
  config.torch_qualified=true; config.outputs[0].torch=true;
  CellIO io(config);
  io.observe(1,true,1000000,1); assert(io.arm(1,1)==0);
  assert(io.intent(0,true,1000000,1,1000000)==0);
  assert(io.output(1000000,true,1,1)==1);
  io.observe(3,true,2000000,2);
  io.expire(5999999); assert(io.state().armed);
  io.expire(6000000);
  assert(!io.state().armed && io.state().reason==reason_io_marker_late);
  assert(io.output(6000000,true,1,1)==0);
  assert(io.intent(0,true,6000000,2,6000000)==reason_io_not_armed);
}

// Deterministic native-time check complements the composed process test, whose
// real-clock scheduling can legitimately hit the strict late-marker guard in a VM.
void TestQualifiedWeldScheduleOnTimeAndLate() {
  auto config=parse_cell_io(fixture_unverified);
  config.torch_qualified=true; config.outputs[0].torch=true;
  config.outputs[0].expiry_ns=200000000;
  CellIO io(config);
  uint16_t commanded=0;
  for(uint64_t tick=1;tick<=2001;++tick) {
    const uint64_t now=tick*1000000;
    io.begin_cycle(now,tick);
    io.observe(1|(commanded ? 2 : 0),true,now,tick);
    if(tick==1) assert(io.arm(1,1)==0);
    io.expire(now);
    if((tick-1)%50==0) assert(io.intent(0,tick<2001,now,tick,now)==0);
    commanded=io.output(now,true,1,1);
    assert(commanded==(tick<2001 ? 1 : 0));
    assert(io.state().armed);
  }
  // The same marker after its delivery bound cannot turn a stopped torch on.
  assert(io.intent(0,true,1999000000,1999,2001000000)==reason_io_marker_late);
  assert(io.output(2001000000,true,1,1)==0);
}

void TestControllerIndicators() {
  const std::string spec="v2|picontrol|/dev/piControl0|83|70|14|14;;"
      "green|2|0|200000000|2|-|-|drive_power|1000000000|16|16,"
      "alarm|3|0|200000000|2|-|-|arm_pulse|3000000000|0|0,"
      "gas|0|0|200000000|0|-|-";
  auto config=parse_cell_io(spec); CellIO io(config);
  uint16_t sw[2]={0x50,0x50}; uint64_t cycle=0;
  auto tick=[&](uint64_t now,bool armed,bool available=true,bool alarm_permit=true) {
    io.begin_cycle(now,++cycle); io.observe(0,available,now,cycle);
    io.indicators(now,sw,2,available?3:0,available,armed,alarm_permit);
    return io.output(now,false,1,1); // no application lease, no I/O permission
  };
  assert(tick(1000000,false)==4); // drive power, not logical arm
  assert(tick(2000000000,false)==4); // no process expiry or client heartbeat
  assert(tick(3000000000,true)==12);
  assert(tick(3499999999,true)==12);
  assert(tick(3500000000,true)==8); // 0.5 s off half-cycle
  assert(tick(4000000000,true)==12);
  assert(tick(5999999999,true)==8);
  assert(tick(6000000000,true)==4); // exactly three seconds, never renewed
  assert(tick(6100000000,false)==4); // returns to steady immediately
  assert(tick(6200000000,true)==12); // next successful arm restarts pulse
  io.fence(1,1); // process fencing cannot extinguish the powered indicator
  assert(tick(6300000000,false)==12); // brief arm still gives full alarm pulse
  assert(tick(9200000000,false)==4);
  assert(io.test(0,true,1,9400000000,9300000000,1,2,true)==reason_io_not_configured);
  assert(io.test(1,false,2,9400000000,9300000000,1,2)==reason_io_not_configured);
  assert(io.intent(0,false,9300000000,cycle,9300000000)==reason_io_not_configured);
  assert(io.intent(1,true,9300000000,cycle,9300000000)==reason_io_not_configured);
  assert(tick(9400000000,true)==12);
  assert(tick(9500000000,false,true,false)==4); // fault cancels pulse, power still indicated
  sw[0]=0x40; sw[1]=0x40; assert(tick(9600000000,false)==0);
  sw[1]=0x50; assert(tick(9700000000,false)==4); // any powered drive is energized
  assert(tick(9800000000,true,false)==0); // lost exchange/shutdown clears both
  assert(io.state().outputs[0].commanded==0 && io.state().outputs[1].commanded==0);
  assert(tick(9900000000,false)==4); // recovery never replays alarm
  assert(!io.state().armed && !io.state().outputs[2].commanded);
  std::ostringstream description; append_cell_io_config_json(description,config);
  assert(description.str().find("\"class\":\"indicator\"")!=std::string::npos);
  assert(description.str().find("\"source\":\"drive_power\"")!=std::string::npos);
  for(const auto& invalid:{
      "drive_power|1000000000|0|0", "drive_power|1|16|16",
      "drive_power|1000000000|16|32", "arm_pulse|0|0|0",
      "arm_pulse|3000000000|16|16", "arm_pulse|60000000001|0|0"}) {
    auto bad=spec; auto at=bad.find("drive_power|1000000000|16|16");
    bad.replace(at,std::strlen("drive_power|1000000000|16|16"),invalid);
    bool rejected=false;try { (void)parse_cell_io(bad); } catch(const std::invalid_argument&) { rejected=true; }
    assert(rejected);
  }
}

int main() {
  TestControllerIndicators();
  TestQualifiedWeldScheduleOnTimeAndLate();
  TestQualifiedWeldExpiryStopsAllOutputs();
  TestManualTorchIsBoundedAndDoesNotQualifyProduction();
  TestInputWaitStability();
  TestSixteenNamedChannelsAndManualFencing();
  TestCellIOFencingExpiryAndReadback(); TestCellIOSimulatedExchange(); TestCellIOMarkerRefusalsAndWireLayout();
  TestCellIOOutputsWithoutReadback(); TestCellIOSchema2Refusals(); TestCellIOPiControlImage(); TestCellIOMailboxFreeSimExchange();
}
