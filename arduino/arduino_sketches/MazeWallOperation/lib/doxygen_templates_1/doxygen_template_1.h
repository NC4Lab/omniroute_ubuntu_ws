'''
/// "doxygen_templates.h" is a file that contains snippets of code that can be used to generate doxygen documentation.
'''

///==============================================================================
/// 
/// FILE: doxygen_template_1.h
///
/// AUTHOR: ChatGPT
/// 
/// CREATED: July 29, 2023
/// 
/// REVISION: 1.0.0
/// 
/// COPYRIGHT: Copyright (c) 2023 John Doe, All Rights Reserved.
///
/// BRIEF: This is a brief description of what this file is for.
/// 
/// DETAILS: This is a more detailed description of this file. This is where you
/// could talk about the purpose of the file, the key classes and functions that
/// it defines, and any other information that someone reading the code might find
/// useful. Remember to keep this up to date as the file changes.
/// 
/// TODO: List any planned improvements here.
/// 
/// BUG: List any known bugs here.
/// 
/// DEPRECATED: List any deprecated features here.
///
/// LINKS: Documentation for Doxygen comment blocks:
/// https://www.doxygen.nl/manual/docblocks.html
///==============================================================================



#pragma once

#include <map>
#include <string>
#include <vector>

// Macro definition
#define MACRO_1 10 /// SUMMARY: A useful macro.
#define MACRO_2 20 /// SUMMARY: Another useful macro.

// Enum definition
/// BRIEF: Enum for representing choices.
enum Enum1
{
    ENUM1_VALUE_1, /// DETAILS: This is the first value.
    ENUM1_VALUE_2, /// DETAILS: This is the second value.
    ENUM1_VALUE_3  /// DETAILS: This is the third value.
};

// Another Enum definition
/// BRIEF: Strongly typed enum for representing states.
enum class Enum2
{
    VALUE_1, /// DETAILS: This is the first state.
    VALUE_2, /// DETAILS: This is the second state.
    VALUE_3  /// DETAILS: This is the third state.
};

// Union definition
/// BRIEF: A union for holding an int or float.
union Union1
{
    int a;     /// DETAILS: Integer variant of the union.
    float b;   /// DETAILS: Float variant of the union.
};

// Struct definition
/// BRIEF: A simple struct for holding data.
struct Struct1
{
    int a;               /// DETAILS: The integer member.
    float b;             /// DETAILS: The floating point member.
    std::string c;       /// DETAILS: The string member.
};

// Forward declaration of classes
class Class1;
class Class2;

// Function prototypes with different types of arguments and return types
/// BRIEF: Function for doing a thing.
/// PARAM: a - The first parameter.
/// PARAM: b - The second parameter.
/// RETURN: The result of the operation.
int function1(int a, float b);

/// BRIEF: Function for mapping things.
/// PARAM: map - A map of things to do.
/// RETURN: A pair of results.
std::pair<int, int> function2(const std::map<int, std::string>& map);

/// BRIEF: Function for doing something with a number.
/// PARAM: a - The number to do something with.
/// PARAM: b - The other number to do something with.
void function3(int& a, int&& b);

/// BRIEF: Function for doing something with a list.
/// PARAM: v - The list to do something with.
void function4(const std::vector<int>& v);

/// BRIEF: Function for doing something with a moved list.
/// PARAM: v - The list to do something with.
void function4(std::vector<int>&& v); // overloaded version

// Template class
/// BRIEF: A class template for handling data.
template <typename T>
class TemplateClass1
{
public:
    /// BRIEF: Constructor.
    TemplateClass1();
    /// BRIEF: Adds an item to the data.
    /// PARAM: item - The item to add.
    void add(const T& item);
    /// BRIEF: Gets an item from the data.
    /// PARAM: index - The index of the item to get.
    /// RETURN: The item at the specified index.
    T get(int index) const;

private:
    std::vector<T> data; /// DETAILS: The data container.
};

// Class definitions
/// BRIEF: The first class.
class Class1
{
public:
    Class1(); /// BRIEF: Constructor.
    ~Class1(); /// BRIEF: Destructor.
    /// BRIEF: A method that does a thing.
    /// PARAM: param1 - The first parameter.
    /// PARAM: param2 - The second parameter.
    /// RETURN: The result of the operation.
    int method1(int param1, float param2);
    static void staticMethod1(); /// BRIEF: A static method.
    friend void friendFunction1(Class1& class1); /// BRIEF: A friend function.

private:
    int privateVar1; /// DETAILS: A private variable.
};

/// BRIEF: The second class.
class Class2
{
public:
    Class2(); /// BRIEF: Constructor.
    ~Class2(); /// BRIEF: Destructor.
    /// BRIEF: A method that does a thing with a Class1 instance.
    /// PARAM: param1 - The first parameter.
    /// PARAM: class1 - The Class1 instance to do something with.
    void method1(int param1, Class1& class1);
    /// BRIEF: A method that returns an Enum1 value.
    /// RETURN: An Enum1 value.
    Enum1 method2() const;

private:
    Union1 privateVar2; /// DETAILS: A private union.
    Enum2 privateEnum; /// DETAILS: A private enum.
};

// Namespace with some elements
/// BRIEF: A namespace for organizing code.
namespace Namespace1
{
    extern int externVar; /// BRIEF: An extern variable.

    void function1(); /// BRIEF: A function in the namespace.

    /// BRIEF: A class in the namespace.
    class Class3
    {
    public:
        void method1(); /// BRIEF: A method in the class.
    };
}



