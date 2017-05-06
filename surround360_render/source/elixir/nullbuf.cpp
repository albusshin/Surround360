#include "nullbuf.h"

NullBuffer null_buffer;
std::ostream null_stream(&null_buffer);
