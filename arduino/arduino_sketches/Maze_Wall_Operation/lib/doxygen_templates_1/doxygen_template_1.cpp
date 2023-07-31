///==============================================================================
/// 
/// FILE: doxygen_template_1.cpp
///
/// AUTHOR: John Doe
/// 
/// CREATED: July 29, 2023
/// 
/// REVISION: 1.0.0
/// 
/// COPYRIGHT: Copyright (c) 2023 John Doe, All Rights Reserved.
///
/// BRIEF: Implementation file for the doxygen_template_1 header.
/// 
/// DETAILS: This file provides the function implementations for the functions 
/// declared in the doxygen_template_1 header file.
/// 
/// TODO: List any planned improvements here.
/// 
/// BUG: List any known bugs here.
/// 
/// DEPRECATED: List any deprecated features here.
/// 
///==============================================================================

#include "doxygen_template_1.h"

void Class1::method1(int param1, Class1& class1) {
    /// DETAILS: This function assigns the provided integer to the `value` member
    /// of this instance of Class1. It also assigns twice that integer to the `value`
    /// member of the provided instance of Class1.
    
    // Assign the provided integer to the `value` member of this instance
    this->value = param1;
    
    // Assign twice the provided integer to the `value` member of the provided instance
    class1.value = param1 * 2;
}

Class2::Enum1 Class2::method2() const {
    /// DETAILS: This function simply returns the Enum1::VALUE2 enumeration value.
    /// It could be used when a fixed value of Enum1 needs to be returned.

    // Return the Enum1::VALUE2 enumeration value
    return Enum1::VALUE2;
}

void Namespace1::Class3::method1() {
    /// DETAILS: This function simply sets the `value` member of this Class3 instance to zero.
    /// It could be used for resetting or initializing the `value` member.

    // Set `value` to zero
    value = 0;
}

