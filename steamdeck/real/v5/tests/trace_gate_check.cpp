#include "trace_gate.hpp"
#include <cassert>
int main() {
 TraceGate g;
 assert(g.sample(false,1)==0); assert(g.sample(true,1)==0);
 assert(g.sample(true,0)==0); assert(g.sample(true,1)==1);
 for(int i=0;i<100;i++) assert(g.sample(true,1)==0);
 assert(g.keep(true,1)); assert(!g.keep(true,-1));
 g.cancel(); assert(g.sample(true,-1)==0);
 assert(g.sample(true,0)==0); assert(g.sample(true,-1)==-1);
 assert(!g.keep(false,-1)); g.sample(false,-1); assert(g.sample(true,-1)==0);
}
