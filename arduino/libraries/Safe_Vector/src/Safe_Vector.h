//######################################
//=========== Safe_Vector.h ============
//######################################

/// <file>
/// Used for the Safe_Vector class to store
/// arrays without using dynamic memory allocation
/// </file>

#ifndef SAFE_VECTOR_H
#define SAFE_VECTOR_H

//============= INCLUDE ================
#include "Arduino.h"
#include <stdarg.h>

/// <summary>
/// Dynamic array implementation with bounds checking.
/// </summary>
template<typename T>
class S_VEC {

private:
    size_t v_cap; ///< Capacity of the vector
    size_t v_len; ///< Length of the vector
    T* p_data; ///< Pointer to the vector data

public:
    S_VEC();
    S_VEC(size_t);
    S_VEC(size_t, const T*);
    virtual ~S_VEC();
    size_t cap() const;
    size_t lng() const;
    T* data() const;
    T& operator[](size_t);
    T const& operator[](size_t) const;
    void sort(bool = true) const;

private:
    size_t check_idx(size_t) const;

};

#endif
