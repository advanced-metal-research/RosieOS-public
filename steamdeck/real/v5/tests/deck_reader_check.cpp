#define main decoderChecks
#include "deck_input_check.cpp"
#undef main
#include "deck_reader.hpp"
#include <fcntl.h>
#include <atomic>

int main() {
  int pipefd[2]; assert(pipe2(pipefd, O_NONBLOCK | O_CLOEXEC) == 0);
  std::atomic<bool> available{true}, feed{true};
  std::atomic<int> l2{32767}, button{0};
  std::thread source([&] {
    uint32_t frame=1;
    while (feed) {
      if (available) {
        auto r=report(frame++,l2.load()); put32(r,8,button.load());
        assert(write(pipefd[1],r.data(),r.size())==64);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
  });
  {
    DeckReader reader([&] { return dup(pipefd[0]); });
    auto wait=[&](auto predicate,int ms=2000) {
      const auto until=DeckReader::now()+ms;
      while(DeckReader::now()<until) {
        if(predicate(reader.snapshot()))return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      return false;
    };
    assert(wait([](auto s){return s.online;}));
    auto initial=reader.snapshot();
    // A blocked UI consumer cannot manufacture a HID timeout. The reader
    // keeps real arrival times, and retains releases/Stop/system buttons.
    l2=0; button=0x20|0x2000|0x100;
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    l2=32767; button=0;
    std::this_thread::sleep_for(std::chrono::milliseconds(260));
    auto next=reader.snapshot();
    assert(next.online && next.input.pad.enabled());
    // The status banner reads acquisition health, not the UI's old sample.
    // A delayed consumer must still reject that cached sample for action
    // admission, while an advancing stream stays visibly online.
    const auto delayedHealth=reader.health(initial.input);
    assert(!initial.input.fresh(DeckReader::now()));
    assert(delayedHealth.online && delayedHealth.reportAgeMs >= 0 && delayedHealth.reportAgeMs < DeckInput::staleMs);
    assert(delayedHealth.consumedAgeMs >= DeckInput::staleMs && delayedHealth.losses == initial.losses);
    assert(next.losses==initial.losses && next.releases==initial.releases+1);
    assert(next.stops==initial.stops+1 && next.systems==initial.systems+1);
    assert(next.hatReleases==initial.hatReleases+1);
    // Real source silence still revokes within the existing 100 ms bound.
    available=false;
    assert(wait([&](auto s){return !s.online && s.losses>next.losses;},250));
    const auto lostHealth=reader.health(next.input);
    assert(!lostHealth.online && lostHealth.losses > next.losses);
    available=true;
    assert(wait([](auto s){return s.online;}));
    assert(reader.health(next.input).online);
    ArmHoldGate gate; gate.cancel();
    assert(!gate.press(true,reader.snapshot().input.pad.enabled()));
    l2=0;assert(wait([](auto s){return !s.input.pad.enabled();}));
    assert(!gate.press(true,false));
    l2=32767;assert(wait([](auto s){return s.input.pad.enabled();}));
    assert(gate.press(true,true));
  }
  feed=false; source.join(); close(pipefd[0]);close(pipefd[1]);
  puts("deck-reader-check passed: delayed UI health, sticky releases/Stop, real silence, reconnect");
}
