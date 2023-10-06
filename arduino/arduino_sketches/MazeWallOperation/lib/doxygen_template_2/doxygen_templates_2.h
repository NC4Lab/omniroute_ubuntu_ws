/**
 * @file doxygen_template_2.h
 *
 * @author John Doe
 * 
 * @date July 29, 2023
 *
 * @version 1.0.0
 *
 * @copyright Copyright (c) 2023 John Doe, All Rights Reserved.
 *
 * @brief This is a brief description of what this file is for.
 * 
 * @details This is a more detailed description of this file. This is where you
 * could talk about the purpose of the file, the key classes and functions that
 * it defines, and any other information that someone reading the code might find
 * useful. Remember to keep this up to date as the file changes.
 * 
 * @todo List any planned improvements here.
 * 
 * @bug List any known bugs here.
 *
 * @deprecated List any deprecated features here.
 */


#pragma once

#include <map>
#include <string>
#include <vector>

// Macro definition
#define MACRO_1 10 ///< @brief A useful macro.
#define MACRO_2 20 ///< @brief Another useful macro.

// Enum definition
/// @brief Enum for representing choices.
enum Enum1
{
    ENUM1_VALUE_1, ///< @details This is the first value.
    ENUM1_VALUE_2, ///< @details This is the second value.
    ENUM1_VALUE_3  ///< @details This is the third value.
};

// Another Enum definition
/// @brief Strongly typed enum for representing states.
enum class Enum2
{
    VALUE_1, ///< @details This is the first state.
    VALUE_2, ///< @details This is the second state.
    VALUE_3  ///< @details This is the third state.
};

// Union definition
/// @brief A union for holding an int or float.
union Union1
{
    int a;     ///< @details Integer variant of the union.
    float b;   ///< @details Float variant of the union.
};

// Struct definition
/// @brief A simple struct for holding data.
struct Struct1
{
    int a;               ///< @details The integer member.
    float b;             ///< @details The floating point member.
    std::string c;       ///< @details The string member.
};

// Forward declaration of classes
class Class1; 
class Class2;

// Function prototypes with different types of arguments and return types
/// @brief Function for doing a thing.
/// @param a The first parameter.
/// @param b The second parameter.
/// @return The result of the operation.
int function1(int a, float b);

/// @brief Function for mapping things.
/// @param map A map of things to do.
/// @return A pair of results.
std::pair<int, int> function2(const std::map<int, std::string>& map);

/// @brief Function for doing something with a number.
/// @param a The number to do something with.
/// @param b The other number to do something with.
void function3(int& a, int&& b);

/// @brief Function for doing something with a list.
/// @param v The list to do something with.
void function4(const std::vector<int>& v);

/// @brief Function for doing something with a moved list.
/// @param v The list to do something with.
void function4(std::vector<int>&& v); // overloaded version

// Template class
/// @brief A class template for handling data.
template <typename T>
class TemplateClass1
{
public:
    /// @brief Constructor.
    TemplateClass1();
    /// @brief Adds an item to the data.
    /// @param item The item to add.
    void add(const T& item);
    /// @brief Gets an item from the data.
    /// @param index The index of the item to get.
    /// @return The item at the specified index.
    T get(int index) const;

private:
    std::vector<T> data; ///< @details The data container.
};

// Class definitions
/// @brief The first class.
class Class1
{
public:
    Class1(); ///< @brief Constructor.
    ~Class1(); ///< @brief Destructor.
    /// @brief A method that does a thing.
    /// @param param1 The first parameter.
    /// @param param2 The second parameter.
    /// @return The result of the operation. 
    /// @return can also be used with other types, like @c std::string. 
    /// @return can also be used with other types, like @c std::vector.
    /// @return can also include details about the returned data like this:
    /// @return The result of the operation, which is a number. 
    /// @details This is a more detailed description of the method.
    /// This is where you could talk about the purpose of the method, the key
    /// steps that it performs, and any other information that someone reading
    /// the code might find useful. Remember to keep this up to date as the
    /// method changes.
    /// @todo List any planned improvements here.
    /// @bug List any known bugs here.
    /// @deprecated List any deprecated features here.
    /// @warning List any warnings here.
    /// @note List any notes here.
    /// @attention List any attention items here.
    /// @pre List any preconditions here.
    /// @post List any postconditions here.
    /// @invariant List any invariants here.
    /// @parblock Which is useful for multi-line descriptions. like
    /// this one. You can use this to describe the method in more detail.
    /// @endparblock
    int method1(int param1, float param2);
    static void staticMethod1(); ///< @brief A static method.
    friend void friendFunction1(Class1& class1); ///< @brief A friend function.

private:
    int privateVar1; ///< @details A private variable.
};

/// @brief The second class.
class Class2
{
public:
    Class2(); ///< @brief Constructor.
    ~Class2(); ///< @brief Destructor.
    /// @brief A method that does a thing with a Class1 instance.
    /// @param param1 The first parameter.
    /// @param class1 The Class1 instance to do something with.
    void method1(int param1, Class1& class1);
    /// @brief A method that returns an Enum1 value.
    /// @return An Enum1 value the can be described with other @c enum types. 
    Enum1 method2() const;

private:
    Union1 privateVar2; ///< @details A private union.
    Enum2 privateEnum; ///< @details A private enum.
};

// Namespace with some elements
/// @brief A namespace for organizing code.
namespace Namespace1
{
    extern int externVar; ///< @brief An extern variable.

    void function1(); ///< @brief A function in the namespace.

    /// @brief A class in the namespace.
    class Class3
    {
    public:
        void method1(); ///< @brief A method in the class.
    };
}
