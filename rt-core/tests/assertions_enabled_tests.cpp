// Makefile appends -UNDEBUG; make test runs this assertion probe first.
#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

// Test checks must remain effective when a caller supplies release-build flags.
// These checks deliberately use ordinary control flow, not assert itself.
int main() {
  int setup_calls = 0;
  assert(++setup_calls == 1);
  if (setup_calls != 1) {
    std::fputs("FAIL: assert erased its setup expression\n", stderr);
    return EXIT_FAILURE;
  }

  const pid_t child = fork();
  if (child < 0) {
    std::perror("fork assertion probe");
    return EXIT_FAILURE;
  }
  if (child == 0) {
    const rlimit no_core{0, 0};
    if (setrlimit(RLIMIT_CORE, &no_core) != 0) std::_Exit(2);
    // The child is expected to abort; avoid an alarming diagnostic on success.
    close(STDERR_FILENO);
    assert(false && "deliberate negative assertion probe");
    std::_Exit(0);
  }

  int status = 0;
  pid_t waited;
  do {
    waited = waitpid(child, &status, 0);
  } while (waited < 0 && errno == EINTR);
  if (waited != child || !WIFSIGNALED(status) || WTERMSIG(status) != SIGABRT) {
    std::fputs("FAIL: false assertion did not terminate its child with SIGABRT\n", stderr);
    return EXIT_FAILURE;
  }

  std::puts("PASS: assertion setup executes and a false assertion aborts");
  return EXIT_SUCCESS;
}
