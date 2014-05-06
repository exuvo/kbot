
#ifndef __dbg_h__
#define __dbg_h__

#include <cstdio>
#include <cerrno>
#include <cstring>

#ifdef NDEBUG
#define dbg_debug(M, ...)
#else
#define dbg_debug(M, ...) fprintf(stderr, "DEBUG %s:%d: " M "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#endif

#define dbg_clean_errno() (errno == 0 ? "None" : strerror(errno))

#define log_err(M, ...) fprintf(stderr, "[ERROR] (%s:%d: errno: %s) " M "\n", __FILE__, __LINE__, dbg_clean_errno(), ##__VA_ARGS__)

#define log_warn(M, ...) fprintf(stderr, "[WARN] (%s:%d: errno: %s) " M "\n", __FILE__, __LINE__, dbg_clean_errno(), ##__VA_ARGS__)

#define log_info(M, ...) fprintf(stderr, "[INFO] (%s:%d) " M "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define dbg_check(A, M, ...) if(!(A)) { log_err(M, ##__VA_ARGS__); errno=0; goto error; }

#define dbg_sentinel(M, ...)  { log_err(M, ##__VA_ARGS__); errno=0; goto error; }

#define dbg_check_mem(A) dbg_check((A), "Out of memory.")

#define dbg_check_debug(A, M, ...) if(!(A)) { dbg_debug(M, ##__VA_ARGS__); errno=0; goto error; }

#endif
