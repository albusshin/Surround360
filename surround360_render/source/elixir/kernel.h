#ifndef ELIXIR_KERNEL_H
#define ELIXIR_KERNEL_H

#include <vector>
#include "data.h"

namespace elixir {

  using namespace std;

  class Kernel {
  public:

    virtual unordered_map<string, void *> execute(vector<Data *>& dataList) = 0;

    virtual Kernel* clone() = 0;

    virtual void updateToNextLayer() = 0;
  };


}

#endif /* ELIXIR_KERNEL_H */
