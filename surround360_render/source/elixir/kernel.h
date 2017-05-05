#ifndef ELIXIR_KERNEL_H
#define ELIXIR_KERNEL_H

#include <vector>
#include "data.h"

namespace elixir {

  using namespace std;

  class Kernel {
  public:
    virtual Data *execute(vector<Data *>& dataList);
  };


}

#endif /* ELIXIR_KERNEL_H */
