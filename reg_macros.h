#pragma once

// REGISTER SYSTEM

#define __expand_args(...) __VA_ARGS__
#define __engulf(x)
#define __expand(x) x

#define REG_FUNCTIONS(prefix, params, reg_type, body) \
    prefix##_read( __expand_args params, reg_type *value ) { \
        body(__expand, __engulf) \
    } \
    prefix##_write( __expand_args params, reg_type value ) { \
        body(__engulf, __expand) \
    }

#define REG_LVALUE(R, W, LVALUE) \
    R(*value = (LVALUE);) W((LVALUE) = value;)

#define REG_CASE_RW(R, W, CASE, LVALUE) \
    case (CASE): REG_LVALUE(R, W, LVALUE) break;

#define REG_CASE_RO(R, W, CASE, VALUE) \
    R(case (CASE): *value = (VALUE); break;)

#define REG_FUNC(R, W, prefix) R(prefix##_read) W(prefix##_write)
