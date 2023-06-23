//######################################
//========== Safe_Vector.cpp ===========
//######################################

/// <file>
/// Used for the Safe_Vector class to store
/// arrays without using dynamic memory allocation
/// </file>

//============= INCLUDE ================
#include "Safe_Vector.h"

/// <summary>
/// Default constructor. Initializes the vector with zero capacity and length.
/// </summary>
template<typename T>
S_VEC<T>::S_VEC() : v_cap(0), v_len(0), p_data(0) {}
/// <summary>
/// Overload with option
/// </summary>
/// <param name="s">size of array to store </param>
template<typename T>
S_VEC<T>::S_VEC(size_t s) :
    v_cap(s),
    v_len(0),
    p_data(new T[s])
{
    for (size_t i = 0; i < s; i++)
    {
        p_data[i] = NULL;
    }
}
/// <summary>
/// Overload with option
/// </summary>
/// <param name="s">size of array to store </param>
/// <param name="a_in">input array </param>
template<typename T>
S_VEC<T>::S_VEC(size_t s, const T* a_in) :
    v_cap(s),
    v_len(s),
    p_data(new T[s])
{
    memcpy(p_data, a_in, v_cap * sizeof(T));
}

/// <summary>
/// Destructor. Frees the dynamically allocated memory for the vector.
/// </summary>
template<typename T>
S_VEC<T>::~S_VEC()
{
    delete[] p_data;
}

/// <summary>
/// Returns the capacity of the vector.
/// </summary>
/// <returns>The capacity of the vector.</returns>
template<typename T>
size_t S_VEC<T>::cap() const
{
    return v_cap;
}

/// <summary>
/// Returns the length of the vector.
/// </summary>
/// <returns>The length of the vector.</returns>
template<typename T>
size_t S_VEC<T>::lng() const
{
    return v_len;
}

/// <summary>
/// Returns a pointer to the data of the vector.
/// </summary>
/// <returns>A pointer to the data of the vector.</returns>
template<typename T>
T* S_VEC<T>::data() const
{
    return p_data;
}

/// <summary>
/// Provides access to the elements of the vector using the [] operator.
/// </summary>
/// <param name="idx">The index of the element to access.</param>
/// <returns>The reference to the element at the given index.</returns>
template<typename T>
T& S_VEC<T>::operator[](size_t idx)
{
    size_t idx_out = check_idx(idx);
    if (idx_out == idx)
    {
        v_len = max(idx + 1, v_len);
    }
    else {
        v_len = v_cap;
    }
    return p_data[idx_out];
}

/// <summary>
/// Provides read-only access to the elements of the vector using the [] operator.
/// </summary>
/// <param name="idx">The index of the element to access.</param>
/// <returns>The const reference to the element at the given index.</returns>
template<typename T>
T const& S_VEC<T>::operator[](size_t idx) const
{
    size_t idx_out = check_idx(idx);
    return p_data[idx_out];
}

/// <summary>
/// Sorts a given array.
/// </summary>
/// <param name="idx">The index of the element to access.</param>
template<typename T>
void S_VEC<T>::sort(bool ascending) const
{
    if (v_len <= 1) {
        // No need to sort if vector has 0 or 1 element
        return;
    }

    // Sorting in ascending order using bubble sort algorithm
    if (ascending) {
        for (size_t i = 0; i < v_len - 1; ++i) {
            for (size_t j = 0; j < v_len - i - 1; ++j) {
                if (p_data[j] > p_data[j + 1]) {
                    // Swap elements if they are out of order
                    T temp = p_data[j];
                    p_data[j] = p_data[j + 1];
                    p_data[j + 1] = temp;
                }
            }
        }
    }
    // Sorting in descending order
    else {
        for (size_t i = 0; i < v_len - 1; ++i) {
            for (size_t j = 0; j < v_len - i - 1; ++j) {
                if (p_data[j] < p_data[j + 1]) {
                    // Swap elements if they are out of order
                    T temp = p_data[j];
                    p_data[j] = p_data[j + 1];
                    p_data[j + 1] = temp;
                }
            }
        }
    }
}

/// <summary>
/// Checks if the given index is within the valid range and adjusts it if necessary.
/// </summary>
/// <param name="idx">The index to check.</param>
/// <returns>The adjusted index.</returns>
template<typename T>
size_t S_VEC<T>::check_idx(size_t idx) const
{
    size_t idx_out = idx < v_cap ?
        idx : v_cap - 1;
    return idx_out;
}


template class S_VEC<uint8_t>;
