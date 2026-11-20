#pragma once
#include "ipc_v1.hpp"
#include "control_protocol.hpp"
#include <algorithm>
#include <array>
#include <charconv>
#include <cstring>
#include <fcntl.h>
#include <ostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace rosie::rtcore {
// Unverified hardware qualification bounds: sixteen channels, a 1 s lease,
// three exchanges for feedback settling and one cycle for marker delivery.
inline constexpr size_t io_channels_unverified = 16;
inline constexpr uint64_t io_expiry_max_ns_unverified = 1000000000;
inline constexpr uint64_t io_readback_cycles_unverified = 3;
inline constexpr uint64_t io_marker_delivery_cycles_unverified = 1;
// Wire and named-channel capacity per direction: two process-image bytes.
inline constexpr uint32_t io_wire_channels = 16;
inline constexpr size_t io_wire_bytes = 2;
// KUNBUS piControl process image size (KB_PI_LEN).
inline constexpr uint32_t picontrol_image_bytes = 4096;

enum class IoBackend : uint8_t { ethercat, picontrol };
struct IoInputConfig { std::string name; uint8_t bit=0; bool active_high=false, fast=false; };
// wired=false is the owner's 2026-10-12 exception: no input module fitted, so
// the output is commanded but never observed (readback none_unverified).
struct IoOutputConfig {
  std::string name; uint8_t bit=0, readback_bit=0;
  bool torch=false, readback_high=false, wired=true; uint64_t expiry_ns=0;
  // Controller-owned indications, never a process or manual-test output.
  // 1: powered/armed lamp, 2: pulse on a successful native arm edge.
  uint8_t indicator=0;
  uint64_t indicator_period_ns=0;
  uint16_t power_mask=0, power_value=0;
};
struct IoPdoConfig { uint16_t pdo=0, index=0; uint8_t sync=0, subindex=0; };
struct IoConfig {
  std::string canonical;
  uint16_t position=0, rx_pdo=0, tx_pdo=0, output_index=0, input_index=0;
  uint32_t vendor=0, product=0, revision=0;
  uint8_t rx_sync=0, tx_sync=0, output_subindex=0, input_subindex=0;
  uint32_t input_count=0, output_count=0;
  std::array<IoInputConfig,io_channels_unverified> inputs{};
  std::array<IoOutputConfig,io_channels_unverified> outputs{};
  // Schema 1 is the combined eight-bit CoE terminal; schema 2 is a mailbox-free
  // EtherCAT terminal (one-byte fixed PDOs) or a RevPi piControl process image.
  uint32_t schema=1;
  IoBackend backend=IoBackend::ethercat;
  uint32_t output_channels=8, input_channels=8, output_pdo_count=0, input_pdo_count=0;
  std::array<IoPdoConfig,io_wire_bytes> output_pdos{}, input_pdos{};
  std::string device;
  uint32_t output_offset=0, input_offset=0;
  bool torch_qualified=false;
  bool enabled() const noexcept { return !canonical.empty(); }
  // Only an EtherCAT terminal is a bus device; piControl I/O is not on the bus.
  bool on_bus() const noexcept { return enabled() && backend==IoBackend::ethercat; }
  uint32_t output_bytes() const noexcept { return (output_channels+7)/8; }
  uint32_t input_bytes() const noexcept { return (input_channels+7)/8; }
};

// Publish configured names and policy separately from live observations, so
// clients cannot mistake the safe state or expiry budget for output readback.
inline void append_cell_io_config_json(std::ostream& out, const IoConfig& io) {
  out << '{';
  if (io.schema >= 2) {
    out << "\"backend\":\"" << (io.backend==IoBackend::picontrol ? "picontrol" : "ethercat") << "\",";
    if (io.backend==IoBackend::picontrol)
      out << "\"device\":\"" << io.device << "\",\"output_offset\":" << io.output_offset
          << ",\"input_offset\":" << io.input_offset << ',';
    out << "\"output_channels\":" << io.output_channels << ",\"input_channels\":" << io.input_channels << ',';
  }
  if (io.backend==IoBackend::ethercat)
    out << "\"slave_position\":" << io.position << ",\"vendor_id\":" << io.vendor
        << ",\"product_code\":" << io.product << ",\"revision_no\":" << io.revision << ',';
  out << "\"torch_qualified\":" << (io.torch_qualified ? "true" : "false") << ",\"inputs\":[";
  for (uint32_t i = 0; i < io.input_count; ++i) {
    const auto& input = io.inputs[i];
    if (i) out << ',';
    out << "{\"name\":\"" << input.name << "\",\"bit\":" << unsigned(input.bit)
        << ",\"class\":\"" << (input.fast ? "fast" : "supervisory")
        << "\",\"polarity\":\"" << (input.active_high ? "active_high" : "active_low") << "\"}";
  }
  out << "],\"outputs\":[";
  for (uint32_t i = 0; i < io.output_count; ++i) {
    const auto& output = io.outputs[i];
    if (i) out << ',';
    out << "{\"name\":\"" << output.name << "\",\"bit\":" << unsigned(output.bit)
        << ",\"class\":\"" << (output.indicator ? "indicator" : output.torch ? "torch" : "process")
        << "\",\"safe_state\":false,\"expiry_ns\":" << output.expiry_ns;
    if(output.indicator)
      out << ",\"indicator\":{\"source\":\"" << (output.indicator==1 ? "drive_power" : "arm_pulse")
          << "\",\"period_ns\":" << output.indicator_period_ns
          << ",\"statusword_mask\":" << output.power_mask << ",\"statusword_value\":" << output.power_value << '}';
    if (output.wired)
      out << ",\"readback_bit\":" << unsigned(output.readback_bit)
          << ",\"readback_polarity\":\"" << (output.readback_high ? "active_high" : "active_low") << "\"}";
    else
      out << ",\"readback\":\"none_unverified\"}";
  }
  out << "]}";
}

// Parse before activation; reject malformed or duplicate channels rather than
// silently changing the compiler's expiry, polarity or physical feedback wiring.
inline IoConfig parse_cell_io(const std::string& text) {
  IoConfig c; if (text.empty()) return c;
  auto split=[](const std::string& s,char sep) {
    std::vector<std::string> out; size_t from=0;
    do { auto end=s.find(sep,from); out.push_back(s.substr(from,end-from)); if(end==std::string::npos) break; from=end+1; } while(true);
    return out;
  };
  auto fail=[] { throw std::invalid_argument("P6-05: invalid cell I/O configuration (torch unqualified)"); };
  auto number=[&](const std::string& s,uint64_t min,uint64_t max) {
    uint64_t n=0; auto r=std::from_chars(s.data(),s.data()+s.size(),n);
    if(r.ec!=std::errc{} || r.ptr!=s.data()+s.size() || n<min || n>max || s!=std::to_string(n)) fail();
    return n;
  };
  auto parts=split(text,';'); if(parts.size()!=3 && parts.size()!=4) fail();
  if(parts.size()==4) { if(parts[3]!="torch-qualified") fail(); c.torch_qualified=true; }
  auto h=split(parts[0],'|');
  if(h[0]=="v2") {
    c.schema=2;
    if(h.size()==10 && h[1]=="ethercat") {
      c.position=number(h[2],0,65535); c.vendor=number(h[3],1,UINT32_MAX); c.product=number(h[4],1,UINT32_MAX);
      c.revision=number(h[5],0,UINT32_MAX);
      c.output_channels=number(h[6],0,io_wire_channels); c.input_channels=number(h[7],0,io_wire_channels);
      unsigned syncs=0;
      auto pdos=[&](const std::string& list,std::array<IoPdoConfig,io_wire_bytes>& out,uint32_t& count,uint32_t channels) {
        if(!list.empty()) for(const auto& p:split(list,'+')) {
          auto f=split(p,':'); if(f.size()!=4 || count>=io_wire_bytes) fail();
          IoPdoConfig pdo{uint16_t(number(f[1],1,65535)),uint16_t(number(f[2],1,65535)),uint8_t(number(f[0],0,15)),uint8_t(number(f[3],0,255))};
          if(syncs&(1u<<pdo.sync)) fail();
          syncs|=1u<<pdo.sync; out[count++]=pdo;
        }
        if(channels>8*count) fail();
      };
      pdos(h[8],c.output_pdos,c.output_pdo_count,c.output_channels);
      pdos(h[9],c.input_pdos,c.input_pdo_count,c.input_channels);
    } else if(h.size()==7 && h[1]=="picontrol") {
      c.backend=IoBackend::picontrol; c.device=h[2];
      if(c.device.size()<2 || c.device[0]!='/' || c.device.find("..")!=std::string::npos) fail();
      for(char x:c.device) if(!((x>='a'&&x<='z')||(x>='A'&&x<='Z')||(x>='0'&&x<='9')||x=='_'||x=='.'||x=='/'||x=='-')) fail();
      c.output_offset=number(h[3],0,picontrol_image_bytes-1); c.input_offset=number(h[4],0,picontrol_image_bytes-1);
      c.output_channels=number(h[5],0,io_wire_channels); c.input_channels=number(h[6],0,io_wire_channels);
      const uint32_t out_end=c.output_offset+c.output_bytes(), in_end=c.input_offset+c.input_bytes();
      if(out_end>picontrol_image_bytes || in_end>picontrol_image_bytes) fail();
      if(c.output_bytes() && c.input_bytes() && c.output_offset<in_end && c.input_offset<out_end) fail();
    } else fail();
  } else {
    if(h.size()!=12) fail();
    c.position=number(h[0],0,65535); c.vendor=number(h[1],1,UINT32_MAX); c.product=number(h[2],1,UINT32_MAX);
    c.revision=number(h[3],0,UINT32_MAX); c.rx_pdo=number(h[4],1,65535); c.tx_pdo=number(h[5],1,65535);
    c.rx_sync=number(h[6],0,15); c.tx_sync=number(h[7],0,15); if(c.rx_sync==c.tx_sync) fail();
    c.output_index=number(h[8],1,65535); c.output_subindex=number(h[9],0,255);
    c.input_index=number(h[10],1,65535); c.input_subindex=number(h[11],0,255);
    c.output_pdos[0]={c.rx_pdo,c.output_index,c.rx_sync,c.output_subindex}; c.output_pdo_count=1;
    c.input_pdos[0]={c.tx_pdo,c.input_index,c.tx_sync,c.input_subindex}; c.input_pdo_count=1;
  }
  std::vector<std::string> names;
  auto name=[&](const std::string& n) {
    if(n.empty() || n.size()>31) fail();
    for(char x:n) if(!((x>='a'&&x<='z')||(x>='A'&&x<='Z')||(x>='0'&&x<='9')||x=='_')) fail();
    for(const auto& prior:names) if(n==prior) fail();
    names.push_back(n); return n;
  };
  auto channel=[&](const std::string& s,uint32_t channels) { if(!channels) fail(); return number(s,0,channels-1); };
  for(unsigned dir=1;dir<=2;++dir) {
    if(parts[dir].empty()) continue;
    auto rows=split(parts[dir],','); if(rows.size()>io_channels_unverified) fail(); uint32_t bits=0, readbacks=0;
    for(const auto& row:rows) {
      auto f=split(row,'|'); if(dir==1 ? f.size()!=4 : (f.size()!=7 && f.size()!=11)) fail();
      auto n=name(f[0]); auto bit=channel(f[1],dir==1?c.input_channels:c.output_channels);
      if(bits&(1u<<bit)) fail();
      bits|=1u<<bit;
      if(dir==1) c.inputs[c.input_count++]={n,uint8_t(bit),bool(number(f[2],0,1)),bool(number(f[3],0,1))};
      else {
        number(f[2],0,0); auto expiry=number(f[3],1,io_expiry_max_ns_unverified);
        auto kind=number(f[4],0,2);
        IoOutputConfig output{n,uint8_t(bit),0,kind==1,false,true,expiry};
        if(kind==2) {
          if(f.size()!=11) fail();
          if(f[7]=="drive_power") output.indicator=1;
          else if(f[7]=="arm_pulse") output.indicator=2;
          else fail();
          output.indicator_period_ns=number(f[8],1,60000000000ull);
          output.power_mask=number(f[9],0,65535); output.power_value=number(f[10],0,65535);
          if(output.indicator==1 && (!output.power_mask || !output.power_value ||
             (output.power_value & ~output.power_mask) || output.indicator_period_ns<2)) fail();
          if(output.indicator==2 && (output.power_mask || output.power_value)) fail();
        } else if(f.size()!=7) fail();
        if(f[5]=="-" && f[6]=="-") output.wired=false;
        else {
          auto rb=channel(f[5],c.input_channels);
          if(readbacks&(1u<<rb)) fail();
          readbacks|=1u<<rb;
          output.readback_bit=uint8_t(rb); output.readback_high=bool(number(f[6],0,1));
        }
        c.outputs[c.output_count++]=output;
      }
    }
  }
  if(c.torch_qualified) {
    unsigned torches=0,fast=0;
    for(unsigned i=0;i<c.input_count;++i) if(c.inputs[i].fast) ++fast;
    for(unsigned i=0;i<c.output_count;++i) if(c.outputs[i].torch) { ++torches; if(!c.outputs[i].wired) fail(); }
    if(torches!=1 || !fast) fail();
  }
  c.canonical=text; return c;
}

// RevPi piControl process image (KUNBUS piControl driver): rt-core is the only
// writer of its output word and rewrites it every cycle, so another writer
// (piTest -w) cannot hold an output against the native lease. piControl has no
// bus watchdog: if rt-core dies without an OFF write (SIGKILL, hang) the module
// keeps the last word. A clean close writes OFF first.
class PiControlPort {
 public:
  PiControlPort() = default;
  PiControlPort(const PiControlPort&) = delete;
  PiControlPort& operator=(const PiControlPort&) = delete;
  ~PiControlPort() { close(); }
  bool open(const IoConfig& c) noexcept {
    close();
    if(c.backend!=IoBackend::picontrol) return false;
    output_offset_=c.output_offset; input_offset_=c.input_offset;
    output_bytes_=c.output_bytes(); input_bytes_=c.input_bytes();
    fd_=::open(c.device.c_str(),O_RDWR|O_CLOEXEC);
    return fd_>=0;
  }
  bool is_open() const noexcept { return fd_>=0; }
  // Little-endian: channel n is bit n%8 of byte n/8 at the configured offset.
  bool write(uint16_t word) noexcept {
    if(fd_<0) return false;
    const uint8_t bytes[io_wire_bytes]={uint8_t(word),uint8_t(word>>8)};
    return !output_bytes_ || ::pwrite(fd_,bytes,output_bytes_,output_offset_)==ssize_t(output_bytes_);
  }
  bool read(uint16_t& word) noexcept {
    word=0;
    if(fd_<0) return false;
    if(!input_bytes_) return true;
    uint8_t bytes[io_wire_bytes]={};
    if(::pread(fd_,bytes,input_bytes_,input_offset_)!=ssize_t(input_bytes_)) return false;
    word=uint16_t(bytes[0]|(input_bytes_>1 ? bytes[1]<<8 : 0));
    return true;
  }
  void close() noexcept {
    if(fd_<0) return;
    (void)write(0);
    ::close(fd_); fd_=-1;
  }
 private:
  int fd_=-1;
  uint32_t output_offset_=0, input_offset_=0, output_bytes_=0, input_bytes_=0;
};

// Process-image binding of the configured I/O: the terminal's PDO byte offsets
// in its EtherCAT domain, or the piControl device. Only the native cycle uses it.
struct CellIOWire {
  std::array<unsigned int,io_wire_bytes> input_offsets{}, output_offsets{};
  PiControlPort picontrol;
  bool picontrol_written=false;
  // A piControl observation is fresh only after this process wrote the output
  // word successfully; the bus terminal's freshness is the exchange's.
  template<class View>
  bool read(const IoConfig& c,const View& view,bool bus_fresh,uint16_t& word) noexcept {
    word=0;
    if(c.backend==IoBackend::picontrol) return picontrol.read(word) && picontrol_written;
    for(uint32_t i=0;i<c.input_pdo_count;++i) word|=uint16_t(view.read_u8(input_offsets[i]))<<(8*i);
    return bus_fresh;
  }
  template<class View>
  void write(const IoConfig& c,const View& view,uint16_t word) noexcept {
    if(c.backend==IoBackend::picontrol) { picontrol_written=picontrol.write(word); return; }
    for(uint32_t i=0;i<c.output_pdo_count;++i) view.write_u8(output_offsets[i],uint8_t(word>>(8*i)));
  }
};

// Single native-cycle owner. Intent, submitted command and independent input
// feedback remain separate; expiry and fencing never depend on a client worker.
class CellIO {
 public:
  explicit CellIO(const IoConfig& config) : config_(config) {
    state_.input_count=config.input_count; state_.output_count=config.output_count;
    for(size_t i=0;i<config.input_count;++i) {
      auto& s=state_.inputs[i]; const auto& c=config.inputs[i];
      std::memcpy(s.name,c.name.data(),c.name.size()); s.bit=c.bit; s.fast=c.fast;
    }
    for(size_t i=0;i<config.output_count;++i) {
      auto& s=state_.outputs[i]; const auto& c=config.outputs[i];
      std::memcpy(s.name,c.name.data(),c.name.size()); s.bit=c.bit; s.torch=c.torch;
    }
  }
  const gradient::ipc::v1::IoStateV1& state() const noexcept { return state_; }
  // Stamp permission changes before control consumption without inventing a
  // fresh input observation; Stop must report the cycle that commanded OFF.
  void begin_cycle(uint64_t now,uint64_t cycle) noexcept { state_.time_ns=now; state_.cycle=cycle; }
  // Only a running, stationary program barrier calls this. Keep the existing
  // intent under native authority while its trajectory clock is paused.
  void refresh_program_wait(uint64_t now) noexcept {
    if(!state_.armed || manual_) return;
    for(uint32_t i=0;i<config_.output_count;++i)
      if(state_.outputs[i].intent && !config_.outputs[i].torch && !config_.outputs[i].indicator)
        state_.outputs[i].expiry_ns=now+config_.outputs[i].expiry_ns;
  }
  bool configured() const noexcept { return config_.enabled(); }
  // Preserve the observed and prepared process words for capture without
  // re-reading process data. Invalid input is zero; submission evidence belongs to the record.
  uint16_t input_word() const noexcept { return input_word_; }
  uint16_t output_word() const noexcept { return output_word_; }
  bool input_valid() const noexcept { return configured() && fresh_; }
  bool fast_satisfied() const noexcept {
    for(size_t i=0;i<config_.input_count;++i) if(config_.inputs[i].fast && (!state_.inputs[i].valid || !state_.inputs[i].value)) return false;
    return fresh_;
  }
  void fence(uint64_t native_generation, uint64_t application_generation) noexcept {
    floor_native_=native_generation; floor_app_=application_generation; state_.armed=0; manual_=false;
    have_fence_=true; fence_cycle_=state_.cycle;
    safe();
  }
  bool manual() const noexcept { return manual_; }
  void disarm() noexcept { state_.armed=0; manual_=false; safe(); }
  void fault(uint32_t reason) noexcept { state_.reason=reason; disarm(); }
  // Losing the refresh of a qualified weld is an execution failure, not an
  // ordinary OFF transition. Check before motion sampling as well as at output.
  void expire(uint64_t now) noexcept {
    for(size_t i=0;i<config_.output_count;++i) {
      if(config_.outputs[i].indicator) continue;
      auto& s=state_.outputs[i];
      if(!s.expiry_ns || now<s.expiry_ns) continue;
      if(s.intent && config_.outputs[i].torch && !manual_) {
        fault(rosie::protocol::reason_io_marker_late); return;
      }
      s.intent=0; s.expiry_ns=0;
    }
  }
  uint32_t arm(uint64_t native_generation,uint64_t application_generation) noexcept {
    using namespace rosie::protocol;
    if(!configured()) return reason_io_not_configured;
    if(!application_generation || native_generation<floor_native_ ||
       (native_generation==floor_native_ && application_generation<=floor_app_)) return reason_wrong_generation;
    if(!fresh_) return reason_io_exchange_lost;
    if(!fast_satisfied()) return reason_io_fast_input_unsatisfied;
    // A pre-fence OFF sample cannot prove that the fenced command reached the terminal.
    if(have_fence_ && last_observed_cycle_<=fence_cycle_) return reason_io_readback_disagreement;
    // An output without wired feedback has no OFF observation to require.
    for(size_t i=0;i<config_.output_count;++i)
      if(!config_.outputs[i].indicator && config_.outputs[i].wired && (!state_.outputs[i].valid || state_.outputs[i].readback)) return reason_io_readback_disagreement;
    state_.reason=reason_none; state_.armed=1; armed_native_=native_generation; armed_app_=application_generation; return reason_none;
  }
  // Samples from an explicitly enabled manual-test session. Newer OFF fences
  // delayed ON; expiry bounds a missing session heartbeat. Motion stays disarmed.
  uint32_t test(unsigned index, bool on, uint64_t sequence, uint64_t deadline, uint64_t now, uint64_t native_generation, uint64_t application_generation, bool begin=false) noexcept {
    using namespace rosie::protocol;
    if (index >= config_.output_count || !sequence) return reason_invalid_command;
    if(config_.outputs[index].indicator) return reason_io_not_configured;
    if (test_native_ != native_generation || test_app_ != application_generation) {
      test_sequence_.fill(0); test_native_=native_generation; test_app_=application_generation;
    }
    if (sequence <= test_sequence_[index]) return reason_invalid_command;
    test_sequence_[index] = sequence;
    if (on && (deadline <= now || deadline-now > io_expiry_max_ns_unverified)) return reason_io_marker_late;
    if (on && !begin && (!state_.outputs[index].intent || state_.outputs[index].expiry_ns<=now)) return reason_io_not_armed;
    if (on && begin && !state_.armed) { const auto r=arm(native_generation,application_generation); if(r) return r; }
    const auto reason = apply_intent(index, on, now, state_.cycle, now, true);
    if (!reason) manual_=true;
    if (!reason && on) state_.outputs[index].expiry_ns = std::min(state_.outputs[index].expiry_ns, deadline);
    return reason;
  }
  uint32_t intent(unsigned index,bool on,uint64_t sample_ns,uint64_t sample_cycle,uint64_t now) noexcept {
    using namespace rosie::protocol;
    if(index>=config_.output_count) return reason_io_not_configured;
    if(config_.outputs[index].indicator) return reason_io_not_configured;
    if(config_.outputs[index].torch && !config_.torch_qualified) return reason_io_torch_unqualified;
    return apply_intent(index, on, sample_ns, sample_cycle, now, false);
  }
private:
  // Manual terminal testing is explicit, sequenced and expiry-bounded. It does
  // not qualify an output for automatic welding or grant motion authority.
  uint32_t apply_intent(unsigned index,bool on,uint64_t sample_ns,uint64_t sample_cycle,uint64_t now,bool manual) noexcept {
    using namespace rosie::protocol;
    if(on && !state_.armed) return reason_io_not_armed;
    if(on && !fast_satisfied()) return reason_io_fast_input_unsatisfied;
    if(sample_cycle>state_.cycle || state_.cycle-sample_cycle>io_marker_delivery_cycles_unverified) return reason_io_marker_late;
    const uint64_t lease=config_.outputs[index].expiry_ns;
    if(now>UINT64_MAX-lease) return reason_invalid_command;
    manual_=manual;
    auto& s=state_.outputs[index]; s.intent=on; s.expiry_ns=on?now+lease:0;
    s.marker_sample_ns=sample_ns; s.marker_cycle=state_.cycle;
    return reason_none;
  }
public:
  void observe(uint16_t inputs,bool fresh,uint64_t now,uint64_t cycle) noexcept {
    input_word_ = fresh ? inputs : 0;
    fresh_=fresh; state_.time_ns=now; state_.cycle=cycle;
    if(fresh) last_observed_cycle_=cycle;
    for(size_t i=0;i<config_.input_count;++i) {
      auto& s=state_.inputs[i]; const auto& c=config_.inputs[i]; s.valid=fresh;
      s.value=fresh && (bool(inputs&(1u<<c.bit))==c.active_high); if(fresh) s.observed_ns=now;
    }
    for(size_t i=0;i<config_.output_count;++i) {
      auto& s=state_.outputs[i]; const auto& c=config_.outputs[i];
      if(!c.wired) { s.valid=0; s.readback=0; continue; }
      s.valid=fresh;
      if(fresh) { s.readback=bool(inputs&(1u<<c.readback_bit))==c.readback_high; s.observed_ns=now; }
      if(fresh && s.readback!=s.commanded && cycle>=changed_cycle_[i] && cycle-changed_cycle_[i]>=io_readback_cycles_unverified) {
        state_.reason=rosie::protocol::reason_io_readback_disagreement; disarm();
      }
    }
    if(configured() && !fresh) { state_.reason=rosie::protocol::reason_io_exchange_lost; disarm(); clear_indicators(); }
    else if(state_.armed && !fast_satisfied()) { state_.reason=rosie::protocol::reason_io_fast_input_unsatisfied; disarm(); }
  }
  // Called once per native cycle from fresh PDO evidence, after the final arm
  // gate. No client lease, worker, allocation or external writer is involved.
  template<class Statuswords>
  void indicators(uint64_t now, const Statuswords& statuswords, unsigned axes,
                  uint32_t feedback_mask, bool available, bool armed, bool alarm_permit) noexcept {
    if(!available || !fresh_) { clear_indicators(); return; }
    const bool edge=armed && !indicator_was_armed_;
    if(edge) indicator_arm_ns_=now;
    indicator_was_armed_=armed;
    for(unsigned i=0;i<config_.output_count;++i) {
      const auto& c=config_.outputs[i]; if(!c.indicator) continue;
      auto& s=state_.outputs[i]; bool on=false;
      if(c.indicator==1) {
        bool powered=false;
        for(unsigned j=0;j<axes;++j)
          powered |= bool(feedback_mask&(1u<<j)) && (statuswords[j]&c.power_mask)==c.power_value;
        // At least one observed powered drive means the cell is energized.
        on=powered && (!armed || (now-indicator_arm_ns_)%c.indicator_period_ns<c.indicator_period_ns/2);
      } else {
        if(!alarm_permit) indicator_until_[i]=0;
        else if(edge && now<=UINT64_MAX-c.indicator_period_ns) indicator_until_[i]=now+c.indicator_period_ns;
        on=now<indicator_until_[i];
        s.expiry_ns=on?indicator_until_[i]:0;
      }
      s.intent=on;
    }
  }
  uint16_t output(uint64_t now,bool permit,uint64_t native_generation,uint64_t application_generation) noexcept {
    if(!permit || native_generation!=armed_native_ || application_generation!=armed_app_) disarm();
    expire(now);
    uint16_t bits=0;
    for(size_t i=0;i<config_.output_count;++i) {
      auto& s=state_.outputs[i];
      const bool value=config_.outputs[i].indicator ? bool(s.intent && fresh_) : state_.armed && s.intent && fresh_ && fast_satisfied();
      if(s.commanded!=value) { s.commanded=value; s.changed_ns=now; changed_cycle_[i]=state_.cycle; }
      if(value) bits|=uint16_t(1u<<config_.outputs[i].bit);
    }
    output_word_ = bits;
    return bits;
  }
 private:
  void clear_indicators() noexcept {
    indicator_was_armed_=false; indicator_arm_ns_=0; indicator_until_.fill(0);
    for(unsigned i=0;i<config_.output_count;++i) if(config_.outputs[i].indicator) {
      auto& s=state_.outputs[i]; s.intent=0; s.expiry_ns=0;
      if(s.commanded) { s.commanded=0; s.changed_ns=state_.time_ns; changed_cycle_[i]=state_.cycle; }
      output_word_ &= uint16_t(~(1u<<config_.outputs[i].bit));
    }
  }
  void safe() noexcept {
    for(size_t i=0;i<config_.output_count;++i) { if(config_.outputs[i].indicator) continue;
      output_word_ &= uint16_t(~(1u<<config_.outputs[i].bit));
      auto& s=state_.outputs[i]; s.intent=0; s.expiry_ns=0;
      if(s.commanded) { s.commanded=0; s.changed_ns=state_.time_ns; changed_cycle_[i]=state_.cycle; }
    }
  }
  const IoConfig& config_;
  gradient::ipc::v1::IoStateV1 state_{};
  std::array<uint64_t,io_channels_unverified> changed_cycle_{};
  std::array<uint64_t,io_channels_unverified> test_sequence_{};
  std::array<uint64_t,io_channels_unverified> indicator_until_{};
  uint64_t indicator_arm_ns_=0;
  bool indicator_was_armed_=false;
  uint64_t test_native_=0, test_app_=0;
  bool manual_=false;
  bool fresh_=false,have_fence_=false;
  uint16_t input_word_=0,output_word_=0;
  uint64_t fence_cycle_=0,last_observed_cycle_=0;
  uint64_t floor_native_=0,floor_app_=0,armed_native_=0,armed_app_=0;
};
} // namespace rosie::rtcore
