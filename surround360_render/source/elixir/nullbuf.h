#ifndef ELIXIR_NULLBUF_H
#define ELIXIR_NULLBUF_H
#include <iostream>

class NullBuffer : public std::streambuf {
public:
  int overflow(int c) { return c; }
};

extern NullBuffer null_buffer;

extern std::ostream null_stream;

#endif /* ELIXIR_NULLBUF_H */
