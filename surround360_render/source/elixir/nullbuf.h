#ifndef ELIXIR_NULLBUF_H
#define ELIXIR_NULLBUF_H
#include <iostream>

class NullBuffer : public std::streambuf {
public:
  int overflow(int c) { return c; }
};

NullBuffer null_buffer;
std::ostream null_stream(&null_buffer);

#endif /* ELIXIR_NULLBUF_H */
