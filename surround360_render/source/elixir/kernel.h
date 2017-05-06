#ifndef ELIXIR_KERNEL_H
#define ELIXIR_KERNEL_H

#include <vector>
#include "data.h"

namespace elixir {

  using namespace std;

  class Kernel {
  public:

    virtual unordered_map<string, void *> execute(vector<Data *>& dataList);

    virtual Kernel* clone();
  };


}

#endif /* ELIXIR_KERNEL_H */
