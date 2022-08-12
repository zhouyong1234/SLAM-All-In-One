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

namespace ezcfg
{
	enum class Token : unsigned char
	{
		L_BRACE             =  '{' ,
		R_BRACE             =  '}' ,
		L_BRACKET           =  '[' ,
		R_BRACKET           =  ']' ,
		L_PARENTHESIS       =  '(' ,
		R_PARENTHESIS       =  ')' ,
		L_ANGLE_BRACKET     =  '<' ,
		R_ANGLE_BRACKET     =  '>' ,
		DOT                 =  '.' ,
		COMMA               =  ',' ,
		SEMICOLON           =  ';' ,
		HASH                =  '#' ,

		EQU                 =  '=' ,
		ADD                 =  '+' ,
		SUB                 =  '-' ,
		MUL                 =  '*' ,
		DIV                 =  '/' ,
		REM                 =  '%' ,

		//not support
		COLON               =  ':' ,
		BIT_NOT             =  '~' ,
		LOG_NOT             =  '!' ,
		BIT_AND             =  '&' ,
		BIT_OR              =  '|' ,
		BIT_XOR             =  '^' ,
		INC                 =  128 , //  ++
		DEC,                //  --
		LOG_AND,            //  &&
		LOG_OR,             //  ||
		BIT_L_SHIFT,        //  <<
		//end not support

		SCOPE,              //  ::
		INT,                // true false
		FLOAT,
		STR,
		ID,

#ifdef COMPILER
		//keyword
		STRUCT,
		NAMESPACE,
		ENUM,
		CONSTANT,
#endif // COMPILER

		END
	};
} /* namespace: ezcfg */
