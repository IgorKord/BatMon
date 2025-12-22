#ifndef BITOPS_HPP
#define BITOPS_HPP

#include <cstdint>
#include <type_traits>

#ifdef __cplusplus

// ===============================
// C++ template implementations
// ===============================

// setBit for non-volatile variables
template<typename VarType, typename BitType>
inline void setBit_impl(VarType* var, BitType bit)
{
static_assert(std::is_integral<VarType>::value, "VarType must be integral");
*var |= static_cast<VarType>(bit);
}

// setBit for volatile variables
template<typename VarType, typename BitType>
inline void setBit_impl(volatile VarType* var, BitType bit)
{
static_assert(std::is_integral<VarType>::value, "VarType must be integral");
*var |= static_cast<VarType>(bit);
}

// clearBit for non-volatile
template<typename VarType, typename BitType>
inline void clearBit_impl(VarType* var, BitType bit)
{
static_assert(std::is_integral<VarType>::value, "VarType must be integral");
*var &= ~static_cast<VarType>(bit);
}

// clearBit for volatile
template<typename VarType, typename BitType>
inline void clearBit_impl(volatile VarType* var, BitType bit)
{
static_assert(std::is_integral<VarType>::value, "VarType must be integral");
*var &= ~static_cast<VarType>(bit);
}

// toggleBit for non-volatile
template<typename VarType, typename BitType>
inline void toggleBit_impl(VarType* var, BitType bit)
{
static_assert(std::is_integral<VarType>::value, "VarType must be integral");
*var ^= static_cast<VarType>(bit);
}

// toggleBit for volatile
template<typename VarType, typename BitType>
inline void toggleBit_impl(volatile VarType* var, BitType bit)
{
static_assert(std::is_integral<VarType>::value, "VarType must be integral");
*var ^= static_cast<VarType>(bit);
}

// testBit for non-volatile variables
template<typename VarType, typename BitType>
inline bool testBit_impl(const VarType* var, BitType bit)
{
static_assert(std::is_integral<VarType>::value, "VarType must be integral");
return ((*var & static_cast<VarType>(bit)) != 0);
}

// testBit for volatile variables
template<typename VarType, typename BitType>
inline bool testBit_impl(volatile const VarType* var, BitType bit)
{
static_assert(std::is_integral<VarType>::value, "VarType must be integral");
return ((*var & static_cast<VarType>(bit)) != 0);
}


// ===============================
// Macros to keep original call syntax
// ===============================
#define setBit(var, bit)      setBit_impl(&(var), (bit))
#define clearBit(var, bit)    clearBit_impl(&(var), (bit))
#define toggleBit(var, bit)   toggleBit_impl(&(var), (bit))
#define testBit(var, bit) testBit_impl(&(var), (bit))

#else // C fallback

#include <stdint.h>
#define setBit(var, bit)      ((var) |= (bit))
#define clearBit(var, bit)    ((var) &= ~(bit))
#define toggleBit(var, bit)   ((var) ^= (bit))
#define testBit(var, bit_value) (var & bit_value)

#endif // __cplusplus

#endif // BITOPS_HPP
