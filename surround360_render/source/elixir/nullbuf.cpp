#include <nullbuf.h>

extern NullBuffer null_buffer;
std::ostream null_stream(&null_buffer);
