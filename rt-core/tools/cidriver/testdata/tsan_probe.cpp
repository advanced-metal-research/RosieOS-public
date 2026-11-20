// A race-free threaded process probes TSan's runtime mapping support before qualification.
#include <thread>
int main() {
  int value = 0;
  std::thread worker([&] { value = 1; });
  worker.join();
  return value == 1 ? 0 : 1;
}
