#ifndef UTILS_H

#include <string>

namespace util {

std::string format (const char *fmt, ...);
std::string vformat (const char *fmt, va_list ap);

}
#endif /* UTILS_H */
