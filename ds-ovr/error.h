#ifndef ERROR_H
#define ERROR_H

#include <exception>

#define error_assert(cond) \
    ((cond) ? 0 : throw Error("assertion '" #cond "' failed"))

class Error : public std::exception
{
protected:
    std::string msg;

public:
    explicit Error(const std::string &msg)
        : msg(msg)
    {
    }

    virtual const char *what() const throw() override
    {
        return msg.c_str();
    }
};

#endif
