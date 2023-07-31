/// here is an example class with a constructor, destructor, methods, various types of variables, and a static variable.
/// @class Example_Class
/// @brief This is an example class for generating doxygen documentation.
/// @details This class is used to demonstrate how to generate doxygen documentation.
/// @note This class is not intended to be used for any other purpose.
/// @warning This class is not intended to be used for any other purpose.
/// @todo This class is not intended to be used for any other purpose.
/// @bug This class is not intended to be used for any other purpose.
/// @deprecated This class is not intended to be used for any other purpose.
/// @param p_wall_inc [0-7] max 8 entries.
/// @param s length of "p_wall_inc" array.
/// @param d_type Specifies the data type to print.
/// @param p_reg An array of existing register values.
/// @param x [0-7] max 8 entries.
/// @param nCham number of chambers being used [1-49]
/// @param pwmDuty PWM duty for all walls [0-255]
/// @param DB_VERBOSE set to control debugging behavior [0:silent, 1:verbose]
/// @param resp capture I2C comm flags from Wire::method calls [0:success, 1-4:errors]
/// @param W_OPR Wall_Operation class instance
/// @param C_COM Cypress_Com class instance
/// @param DB Maze_Debug class instance
/// @param LED_BUILTIN built in LED pin
/// @param Serial1 Serial1 port
/// @param Serial Serial port

/// QUESTION: is this the best way to return the array if I am going to be passing it to:
/// @class Esmacatshield: :write_reg(uint8_t id, uint16_t *reg16)
// Make a list of doxygen commands to use in the comments

/// QUESTION: TODO: OPTIONAL: DEFAULT: PARAM: RETURN: SUMMARY: BRIEF: DETAILS: NOTE: WARNING: TODO: BUG: DEPRECATED: PARAM: RETURN: 
/// @brief: @details: @note: @warning: @todo: @bug: @deprecated: @param: @return: @ref: @see: 
/// @bug: @deprecated: @param: @return: @ref: @see: @todo: @warning: @note: @details: @brief:

/// more examples below


/// <summary>
/// Used for debugging to print out all fields of a PMS struct.
/// </summary>
/// <param name="p_wall_inc">OPTIONAL: [0-7] max 8 entries. DEFAULT:[all walls] </param>
/// PARAM: p_wall_inc [0-7] max 8 entries. OPTIONAL: DEFAULT:[all walls]
/// <param name="s">OPTIONAL: length of "p_wall_inc" array. DEFAULT:[8] </param>

/// OVERLOAD: function for printing Ethercat register values.
/// This version of the function accepts an array of integer register values.
/// @param d_type: Specifies the data type to print. [0, 1] corresponds to [uint8, uint16].
/// @param p_reg: An array of existing register values. @def [x]:[0-7] max 8 entries.
/// RETURN: 0 if successful, 1 if error.

/// Break up these sections of code with a clear delimiting text feature
/// create a new section of code with a break line and a new section title:
/// @section title

/**
 * \class MyClass
 * \brief A brief description of the class.
 *
 * A more detailed description of the class.
 */
class MyClass
{
public:
    /**
     * \brief Brief description of the constructor.
     *
     * More detailed description of the constructor.
     */
    MyClass();

    /**
     * \brief Brief description of the method.
     * \param param1 Description of the first parameter.
     * \param param2 Description of the second parameter.
     * \return Description of the return value.
     *
     * More detailed description of the method.
     */
    int myMethod(int param1, float param2);
};

/**
 * \fn int myFunction(int param1, float param2)
 * \brief Brief description of the function.
 * \param param1 Description of the first parameter.
 * \param param2 Description of the second parameter.
 * \return Description of the return value.
 *
 * More detailed description of the function.
 */
int myFunction(int param1, float param2);

/**
 * \file myfile.cpp
 * \brief Brief description of the file.
 *
 * More detailed description of the file.
 */


/**
 * \def MY_MACRO(arg)
 * \brief Brief description of the macro.
 * \param arg Description of the argument.
 *
 * More detailed description of the macro.
 */
#define MY_MACRO(arg) ...


// This is great! Now reuse everything you have here making sure to include at least two examples of most type of element (class, enum, etc) so this would mean having, for example a function1() and a function2() which different prameters. 

// Then I want you to then generate 2 identical versions in terms of the code.

// The first usin the capitol style comments:
// /// QUESTION: TODO: OPTIONAL: DEFAULT: PARAM: RETURN: SUMMARY: BRIEF: DETAILS: NOTE: WARNING: TODO: BUG: DEPRECATED: PARAM: RETURN: 

// The second version useing  @ comments:
// /// @brief: @details: @note: @warning: @todo: @bug: @deprecated: @param: @return: @ref:  
// /// @bug: @deprecated: @param: @return: @ref: @see: @todo: @warning: @note: @details: @brief: @section: @indef

// try to use as many different types of documentation cammans and more if possible, with good examples of how to use them to explain things like types expected arguments usages ranges of values for arguments etc

// While each version will ideally using only one of these two styles, if there are other interestin types of formating I would like that as well. For example, any interesting ways there are of breaking up code blocks visually like:
//  """---TEXT---"""
// ///#pragma region ----------TEXT----------

// ///#pragma endregion 