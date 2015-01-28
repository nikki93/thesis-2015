#ifndef ERROR_H
#define ERROR_H

/* printf-style error formatting */
void errorf(const char *fmt, ...);

/* evaluates to a 'filename:linenumber: ' string */
#define line_str__(line)                        \
    __FILE__ ":" #line ": "
#define line_str_(line)                         \
    line_str__(line)
#define line_str()                              \
    line_str_(__LINE__)

/* printf-style error formatting with file name, line number */
#define error(...)                              \
    errorf(line_str() __VA_ARGS__)

/* printf-style error formatting for an assertion, also prints condition */
#define error_assert(cond, ...)                         \
    ((cond) ? 0: error("assertion '" #cond "' failed ... " __VA_ARGS__))

#endif
