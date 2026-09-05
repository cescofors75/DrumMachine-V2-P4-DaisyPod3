#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
inline void* ps_malloc(size_t n) { return malloc(n); }
inline void* ps_calloc(size_t n, size_t size) { return calloc(n, size); }
