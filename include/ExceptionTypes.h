#ifndef _EXCEPTIONTYPES_H_
#define _EXCEPTIONTYPES_H_

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"

#define code_line \
    std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": "

#define warning(msg)                          \
    std::cerr                                 \
        << YELLOW << code_line << msg << "\n" \
        << RESET

#define OUT_OF_RANGE out_of_range
#define LOGIC_ERROR logic_error

#define THROW_EXCEPTION(TYPE, msg) \
    throw std::TYPE(RED + code_line + msg + RESET)

#endif