/********************************************************************\
|*  This file is part of easy_config.                               *|
|*                                                                  *|
|*  Copyright (c) 2021-2022 Incloon                                 *|
|*                                                                  *|
|* easy_config is free software : you can redistribute it and/or    *|
|* modify it under the terms of the GNU Lesser General Public       *|
|* License as published by the Free Software Foundation, either     *|
|* version 3 of the License, or (at your option) any later version. *|
|*                                                                  *|
|* easy_config is distributed in the hope that it will be useful,   *|
|* but WITHOUT ANY WARRANTY; without even the implied warranty of   *|
|* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the      *|
|* GNU Lesser General Public License for more details.              *|
|*                                                                  *|
|* You should have received a copy of the GNU Lesser General Public *|
|* License along with easy_config.                                  *|
|* If not, see < https://www.gnu.org/licenses/>.                    *|
\********************************************************************/
#pragma once

#include <iostream>
#include <type_traits>

#define INTEGERT_DEFINE_OPERATOR(SIGN)                          	 \
IntegerT operator SIGN(const IntegerT& r_operator) const		   	 \
{																   	 \
	if (is_signed)												   	 \
		if (r_operator.is_signed)								   	 \
			return { signed_value SIGN r_operator.signed_value };	 \
		else													   	 \
			return { signed_value SIGN r_operator.unsigned_value };  \
	else														   	 \
		if (r_operator.is_signed)								   	 \
			return { unsigned_value SIGN r_operator.signed_value };  \
		else													  	 \
			return { unsigned_value SIGN r_operator.unsigned_value };\
}

#define INTEGERT_DEFINE_TYPE_CONVERSION(TYPE)\
operator TYPE() const						 \
{											 \
	if (is_signed) return signed_value;		 \
	else return unsigned_value;				 \
}

#define ARITHMETICT_DEFINE_OPERATOR(SIGN)                                             \
ArithmeticT operator SIGN(const ArithmeticT& r_operator) const		   				  \
{																   					  \
	if (is_float)												   					  \
		if (r_operator.is_float)								   					  \
			return { float_value SIGN r_operator.float_value };	   					  \
		else													  					  \
			return { float_value SIGN static_cast<double>(r_operator.integer_value) };\
	else														   					  \
		if (r_operator.is_float)								   					  \
			return { static_cast<double>(integer_value) SIGN r_operator.float_value };\
		else													   					  \
			return { integer_value SIGN r_operator.integer_value };					  \
}

#define ARITHMETICT_DEFINE_TYPE_CONVERSION(TYPE)\
operator TYPE() const							\
{												\
	if (is_float) return float_value;			\
	else return integer_value;					\
}

namespace ezcfg
{
	struct IntegerT
	{
		template<typename T>
		IntegerT(T value, bool is_signed = std::is_signed<T>::value)
			: is_signed{ is_signed }
		{
			static_assert(std::is_arithmetic<T>::value, "Expected a arithmetic type");
			static_assert(!std::is_floating_point<T>::value, "Expected a Integer type");

			if (is_signed) signed_value = value;
			else unsigned_value = value;
		}

		IntegerT(const IntegerT&) = default;
		IntegerT& operator=(const IntegerT&) = default;

		IntegerT operator +() const
		{ return *this; }

		IntegerT operator -() const
		{
			if (is_signed) return -signed_value;
			else return -unsigned_value;
		}

		INTEGERT_DEFINE_OPERATOR(+)
		INTEGERT_DEFINE_OPERATOR(-)
		INTEGERT_DEFINE_OPERATOR(*)
		INTEGERT_DEFINE_OPERATOR(/)
		INTEGERT_DEFINE_OPERATOR(%)

		INTEGERT_DEFINE_TYPE_CONVERSION(float)
		INTEGERT_DEFINE_TYPE_CONVERSION(double)

		INTEGERT_DEFINE_TYPE_CONVERSION(bool)

		INTEGERT_DEFINE_TYPE_CONVERSION(char)
		INTEGERT_DEFINE_TYPE_CONVERSION(unsigned char)
		INTEGERT_DEFINE_TYPE_CONVERSION(signed char)

		INTEGERT_DEFINE_TYPE_CONVERSION(short)
		INTEGERT_DEFINE_TYPE_CONVERSION(int)
		INTEGERT_DEFINE_TYPE_CONVERSION(long)
		INTEGERT_DEFINE_TYPE_CONVERSION(long long)
		INTEGERT_DEFINE_TYPE_CONVERSION(unsigned short)
		INTEGERT_DEFINE_TYPE_CONVERSION(unsigned int)
		INTEGERT_DEFINE_TYPE_CONVERSION(unsigned long)
		INTEGERT_DEFINE_TYPE_CONVERSION(unsigned long long)

		friend std::ostream& operator<<(std::ostream & output, const IntegerT& value)
		{
			if (value.is_signed) output << value.signed_value;
			else output << value.unsigned_value;
			return output;
		}

	private:
		bool is_signed;
		union
		{
			long long signed_value;
			unsigned long long unsigned_value;
		};
	};

	struct ArithmeticT
	{
		ArithmeticT(float value)
			: is_float(true)
			, float_value(value)
		{ }

		ArithmeticT(double value)
			: is_float(true)
			, float_value(value)
		{ }

		ArithmeticT(const IntegerT& value)
			: is_float(false)
			, integer_value(value)
		{ }

		template<typename T>
		ArithmeticT(T value)
			: is_float(false)
			, integer_value(value)
		{
			static_assert(std::is_arithmetic<T>::value, "Expected a arithmetic type");
			static_assert(!std::is_same<T, long double>::value, "Not support type: long double");
		}

		ArithmeticT(const ArithmeticT&) = default;
		ArithmeticT& operator=(const ArithmeticT&) = default;

		ArithmeticT operator +() const
		{ return *this; }

		ArithmeticT operator -() const
		{
			if (is_float) return -float_value;
			else return -integer_value;
		}

		ARITHMETICT_DEFINE_OPERATOR(+)
		ARITHMETICT_DEFINE_OPERATOR(-)
		ARITHMETICT_DEFINE_OPERATOR(*)
		ARITHMETICT_DEFINE_OPERATOR(/)

		ArithmeticT operator %(const ArithmeticT& r_operator) const
		{
			if (is_float || r_operator.is_float)
				exit(-1);
			return { integer_value % r_operator.integer_value };
		}

		ARITHMETICT_DEFINE_TYPE_CONVERSION(float)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(double)

        ARITHMETICT_DEFINE_TYPE_CONVERSION(bool)

		ARITHMETICT_DEFINE_TYPE_CONVERSION(char)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(unsigned char)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(signed char)

		ARITHMETICT_DEFINE_TYPE_CONVERSION(short)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(int)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(long)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(long long)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(unsigned short)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(unsigned int)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(unsigned long)
		ARITHMETICT_DEFINE_TYPE_CONVERSION(unsigned long long)

		friend std::ostream& operator<<(std::ostream& output, const ArithmeticT& value)
		{
			if (value.is_float) output << value.float_value;
			else output << value.integer_value;
			return output;
		}

	private:
		typedef std::conditional<sizeof(size_t) == sizeof(double), double, float>::type FloatT;

		bool is_float;
		union
		{
			IntegerT integer_value;
			double float_value;
		};
	};
} /* namespace: ezcfg */

#undef INTEGERT_DEFINE_OPERATOR
#undef INTEGERT_DEFINE_TYPE_CONVERSION
#undef ARITHMETICT_DEFINE_OPERATOR
#undef ARITHMETICT_DEFINE_TYPE_CONVERSION
