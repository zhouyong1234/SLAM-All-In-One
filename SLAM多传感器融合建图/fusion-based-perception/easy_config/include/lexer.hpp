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
#include <fstream>
#include <sstream>
#include <limits>
#include <climits>
#include <string>
#include <memory>
#include <map>

#include <token_info.hpp>
#include <arithmetic_type.hpp>

#define RAW_STRING_BEGIN '\2'

namespace ezcfg
{
	class Lexer
	{
		class FilterStream
		{
			class FormatFilterStream
			{
			public:
				FormatFilterStream(size_t& line, const std::unique_ptr<std::istream>& stream)
					: current_character{ 0 }
					, line{ line }
					, stream{ stream }
				{}

				char get()
				{
					char temp = current_character;
					while (true)
						switch (current_character = static_cast<char>(stream->get()))
						{
						case '\\':
							switch (stream->peek())
							{
							case '\r':
								stream->get();
								if (stream->peek() != '\n')
								{
									stream->unget();
									return temp;
								}
							case '\n':
								stream->get();
								++line;
								break;
							case static_cast<char>(std::ifstream::traits_type::eof()):
								current_character = '\n';
								return temp;
							default:
								return temp;
							}
							break;
                        case static_cast<char>(std::ifstream::traits_type::eof()):
                            if (temp == '\n' || temp == static_cast<char>(std::ifstream::traits_type::eof()))
                                current_character = static_cast<char>(std::ifstream::traits_type::eof());
                            else
                                current_character = '\n';
                            return temp;
						case '\r'://fallthrough
							if (stream->peek() == '\n')
							{
								stream->get();
                                current_character = '\n';
							}
							else
								return temp;
						case '\n':
							++line;
						default:
							return temp;
						}
				}

				char peek() const
				{ return current_character; }

			private:
				char current_character;
				size_t& line;
				const std::unique_ptr<std::istream>& stream;
			};

			class CommentFilterStream
			{
			public:
				CommentFilterStream(const std::string& file_name, FormatFilterStream& stream)
					: current_character{ 0 }
					, file_name{ file_name }
					, stream{ stream }
				{}

				char get()
				{
					char temp = current_character;
					while (true)
						switch (current_character = stream.get())
						{
						case '/':
							switch (stream.peek())
							{
							case '/':
								while (stream.get() != '\n');
                                    current_character = ' ';
								return temp;
							case '*':
								stream.get();
								while (true)
									switch (stream.get())
									{
									case '*':
										if (stream.peek() == '/')
										{
											stream.get();
                                            current_character = ' ';
											return temp;
										}
										break;
									case static_cast<char>(std::ifstream::traits_type::eof()):
										//std::cerr << file_name << ": " << line << ": " << "Lexical error: Multiline comment error" << std::endl;
										exit(-1);
									default:
										break;
									}
							default:
								break;
							}
							return temp;
						case 'R':	//very very special! pay attention!
							if (stream.peek() == '"')
                                current_character = RAW_STRING_BEGIN;
						default:
							return temp;
						}
				}

				char peek() const
				{ return current_character; }

			private:
				char current_character;
				const std::string& file_name;
				FormatFilterStream& stream;
			};

		public:
			FilterStream(const std::string& file_name)
				: line{ 1 }
				, file_name{ file_name }
				, base_stream{ nullptr }
				, format_filter_stream{ line, base_stream }
				, comment_filter_stream{ file_name, format_filter_stream }
			{}

			bool loadFile()
			{
				auto ifs_ptr = std::unique_ptr<std::ifstream>{ new std::ifstream(file_name) };
				if (!ifs_ptr->is_open())
					return false;
				base_stream = std::move(ifs_ptr);
				line = 1;
				format_filter_stream.get();
				comment_filter_stream.get();
				return true;
			}

			bool loadSource(const std::string& source)
			{
				if (source.empty())
					return false;

				base_stream.reset(new std::stringstream(source));
				line = 1;
				format_filter_stream.get();
				comment_filter_stream.get();
				return true;
			}

			inline char get()
			{ return comment_filter_stream.get(); }

			inline char peek()
			{ return comment_filter_stream.peek(); }

			inline char getRaw()
			{ return base_stream->get(); }

			inline char peekRaw()
			{ return base_stream->peek(); }

			inline void recover()
			{
				format_filter_stream.get();
				comment_filter_stream.get();
			}

			inline size_t getLineNum()
			{ return line; }

			explicit operator bool() const
			{ return comment_filter_stream.peek() != static_cast<char>(std::ifstream::traits_type::eof()); }

		private:
			size_t line;
			const std::string& file_name;
			std::unique_ptr<std::istream> base_stream;
			FormatFilterStream format_filter_stream;
			CommentFilterStream comment_filter_stream;
		};

		enum CharacterSetName
		{
			BIN,
			OCT,
			DEC,
			HEX
		};

		inline bool binarySet()
		{ return stream.peek() == '0' || stream.peek() == '1'; }

		inline bool octalSet()
		{ return stream.peek() >= '0' && stream.peek() <= '7'; }

		inline bool decimalSet()
		{ return stream.peek() >= '0' && stream.peek() <= '9'; }

		inline bool hexadecimalSet()
		{ return stream.peek() >= 'a' && stream.peek() <= 'f' || stream.peek() >= 'A' && stream.peek() <= 'F' || stream.peek() >= '0' && stream.peek() <= '9'; }

		inline bool identifierSet()
		{ return stream.peek() >= 'a' && stream.peek() <= 'z' || stream.peek() >= 'A' && stream.peek() <= 'Z' || stream.peek() >= '0' && stream.peek() <= '9' || stream.peek() == '_'; }

		inline bool dcharSet()
		{ return stream.peekRaw() != ' ' && stream.peekRaw() != '\t' && stream.peekRaw() != '(' && stream.peekRaw() != ')' && stream.peekRaw() != '\v' && stream.peekRaw() != '\f' && stream.peekRaw() != '\n'; }


		template<CharacterSetName name> struct CharacterSetInfo;

		template<CharacterSetName set_name>
		void recognizeCS(std::string& seq)
		{
			if ((this->*CharacterSetInfo<set_name>::isInSet)())
				do seq.push_back(stream.get());
				while ((this->*CharacterSetInfo<set_name>::isInSet)());
			else lexError(CharacterSetInfo<set_name>::info);
		}

		template<CharacterSetName set_name>
		void matchNumCS(std::string& seq)
		{
			while ((this->*CharacterSetInfo<set_name>::isInSet)()) seq.push_back(stream.get());
			while (stream.peek() == '\'')
			{
				stream.get();
				recognizeCS<set_name>(seq);
			}
		}

		template<CharacterSetName set_name>
		void recognizeNumCS(std::string& seq)
		{
			goto start_point;
			while (stream.peek() == '\'')
			{
				stream.get();
			start_point:
				recognizeCS<set_name>(seq);
			}
		}

		char charToNum(char c)
		{
			if (c >= '0' && c <= '9')
				return c - '0';
			if (c >= 'A' && c <= 'Z')
				return c - 'A' + 10;
			if (c >= 'a' && c <= 'z')
				return c - 'a' + 10;

			lexError("Internal error: charToNum");
		}

		char escapeSequences()
		{
			if (stream.get() != '\\')
				lexError("Expected the character \\");
			switch (stream.peek())
			{
			case 'a':
				stream.get();
				return '\x07';
			case 'b':
				stream.get();
				return '\x08';
			case 'f':
				stream.get();
				return '\x0c';
			case 'n':
				stream.get();
				return '\x0a';
			case 'r':
				stream.get();
				return '\x0d';
			case 't':
				stream.get();
				return '\x09';
			case 'v':
				stream.get();
				return '\x0b';
			case 'x':
			{
				stream.get();
				std::string result;
				recognizeCS<HEX>(result);
				if (result.size() == 1)
					return charToNum(result.back());
				else
					return charToNum(*(result.end() - 2)) * 16 + charToNum(result.back());
			}
			default:
				if (octalSet())
				{
					int result;
					result = stream.get() - '0';
					if (octalSet())
					{
						result = result * 8 + stream.get() - '0';
						if (octalSet())
						{
							result = result * 8 + stream.get() - '0';
							if (result > 255)
								lexError("Escape character octal number out of rang!");
						}
					}
					return result;
				}
				else
					return stream.get();
			}
		}

		bool matchIntegerSuffix()
		{
			bool res = true;
			switch (stream.peek())
			{
			case 'u':
			case 'U':
				res = false;
				token_text.push_back(stream.get());
				if (stream.peek() == 'l')
				{
					token_text.push_back(stream.get());
					if (stream.peek() == 'l')
						token_text.push_back(stream.get());
				}
				else if (stream.peek() == 'L')
				{
					token_text.push_back(stream.get());
					if (stream.peek() == 'L')
						token_text.push_back(stream.get());
				}
				break;
			case 'l':
				token_text.push_back(stream.get());
				if (stream.peek() == 'l')
					token_text.push_back(stream.get());
				if (stream.peek() == 'u' || stream.peek() == 'U')
					res = false, token_text.push_back(stream.get());
				break;
			case 'L':
				stream.get();
				if (stream.peek() == 'L')
					token_text.push_back(stream.get());
				if (stream.peek() == 'u' || stream.peek() == 'U')
					res = false, token_text.push_back(stream.get());
				break;
			default:
				break;
			}

			if (identifierSet())
				lexError("Integer suffix error!");

			return res;
		}

		void matchFloatSuffix()
		{
			switch (stream.peek())
			{
			case 'f':
			case 'F':
			case 'l':
			case 'L':
				token_text.push_back(stream.get());
			default:
				break;
			}

			if (identifierSet())
				lexError("Float suffix error!");
		}

		void recognizeCharacter()
		{
			if (stream.get() != '\'')
				lexError("Expected the character '");
			switch (stream.peek())
			{
			case '\'':
				lexError("Void character");
			case '\n':
				lexError("Received \\n between ''");
			case static_cast<char>(std::ifstream::traits_type::eof()):
				lexError("Expected the character '");
			case '\\':
				number = escapeSequences();
				break;
			default:
				number = stream.get();
				break;
			}
			if (stream.get() != '\'')
				lexError("Expected the character '");
		}

		void recognizeRawString()
		{
			if (stream.peek() != RAW_STRING_BEGIN)
				lexError("Expected the character R\"");

			std::string dchar_sequence;
			while (dcharSet()) dchar_sequence.push_back(stream.getRaw());
			if (stream.getRaw() != '(')
				lexError("Expected the character ( in raw string");
			std::string temp;
			while (true)
			{
				do token_text.push_back(stream.getRaw());
				while (token_text.back() != ')');
				while (temp.size() < dchar_sequence.size() && stream.peekRaw() != ')') temp.push_back(stream.getRaw());
				if (temp == dchar_sequence && stream.peekRaw() == '"')
				{
					stream.getRaw();
					token_text.pop_back();
					stream.recover();
					return;
				}
				else
				{
					token_text += temp;
					temp.clear();
				}
			}
		}

		void recognizeSingleString()
		{
			if (stream.get() != '"')
				lexError("Expected the character \"");

			while (true)
			{
				switch (stream.peek())
				{
				case '"':
					stream.get();
					return;
				case '\n':
					lexError(R"(Received \n between "")");
				case static_cast<char>(std::ifstream::traits_type::eof()):
					lexError("Expected the character \"");
				case '\\':
					token_text.push_back(escapeSequences());
					break;
				default:
					token_text.push_back(stream.get());
					break;
				}
			}
		}

		void recognizeMultiString()
		{
			if (stream.peek() != '"' && stream.peek() != RAW_STRING_BEGIN)
				lexError("Expected the character \"");
			while (true)
				switch (stream.peek())
				{
				case RAW_STRING_BEGIN:
					recognizeRawString();
					break;
				case '"':
					recognizeSingleString();
					break;
				case '\t':
				case '\v':
				case '\f':
				case '\n':
				case ' ':
					stream.get();
					break;
				default:
					return;
				}
		}

		void recognizeID()
		{
			if (stream.peek() >= 'a' && stream.peek() <= 'z' || stream.peek() >= 'A' && stream.peek() <= 'Z' || stream.peek() == '_')
				do token_text.push_back(stream.get());
				while (identifierSet());
			else
				lexError("Expected a identity");

			if (token_text == "true")
			{
				current_token = Token::INT;
				number = true;
			}
			else if (token_text == "false")
			{
				current_token = Token::INT;
				number = false;
			}
#ifdef COMPILER
			else if (token_text == "namespace")
				current_token = Token::NAMESPACE;
			else if (token_text == "struct")
				current_token = Token::STRUCT;
			else if (token_text == "enum")
				current_token = Token::ENUM;
			else if (token_text == "const")
				current_token = Token::CONSTANT;
#endif // COMPILER
			else
				current_token = Token::ID;
		}

		static bool isSigned(unsigned long long v)
		{
			if (v > std::numeric_limits<int>::max() && v <= std::numeric_limits<unsigned int>::max())
				return false;
			else if (v > std::numeric_limits<long>::max() && v <= std::numeric_limits<unsigned long>::max())
				return false;
			else if (v > std::numeric_limits<long long>::max() && v <= std::numeric_limits<unsigned long long>::max())
				return false;
			return true;
		}

		void recognizeNum()
		{
			if (!decimalSet())
				lexError("Expected a number");

			char* discarded_value;
			bool octal_check = false;
			current_token = Token::INT;
			token_text.push_back(stream.peek());
			if (stream.get() == '0')
			{
				switch (stream.peek())
				{
				case 'b':
				case 'B':
					token_text.push_back(stream.get());
					recognizeNumCS<BIN>(token_text);
					if (token_text.size() - 2 > CHAR_BIT * sizeof(size_t))
						lexError("binary number out of range");
					else
					{
						size_t temp = std::strtoull(token_text.c_str() + 2, &discarded_value, 2);
						number = IntegerT{ temp ,isSigned(temp) && matchIntegerSuffix() };
					}
					return;
				case 'x':
				case 'X':
					token_text.push_back(stream.get());
					matchNumCS<HEX>(token_text);
					if (stream.peek() != '.')
					{
						if (token_text.size() - 2 > CHAR_BIT * sizeof(size_t) / 4)
							lexError("hexadecimal number out of range");
						else
						{
							size_t temp = std::strtoull(token_text.c_str() + 2, &discarded_value, 16);
							number = IntegerT{ temp ,isSigned(temp) && matchIntegerSuffix() };
						}
					}
					else
					{
						current_token = Token::FLOAT;
						token_text.push_back(stream.get());
						matchNumCS<HEX>(token_text);
						if (token_text.size() < 4)
							lexError("Expected a hexadecimal number");

						if (stream.peek() == 'p' || stream.peek() == 'P')
						{
							token_text.push_back(stream.get());
							if (stream.peek() == '-' || stream.peek() == '+')
								token_text.push_back(stream.get());
							recognizeNumCS<HEX>(token_text);
						}
						number = std::stod(token_text);
						matchFloatSuffix();
					}
					return;
				default:
					octal_check = true;
					break;
				}
			}

			matchNumCS<DEC>(token_text);
			if (stream.peek() != '.')
			{
				if (octal_check)
				{
					for (char c : token_text)
						if (c > '7')
							lexError("Expected a octal number");
					size_t temp = std::stoull(token_text, nullptr, 8);
					number = IntegerT{ temp ,isSigned(temp) && matchIntegerSuffix() };
				}
				else
					number = IntegerT{ std::stoull(token_text) ,matchIntegerSuffix() };
			}
			else
			{
				current_token = Token::FLOAT;
				token_text.push_back(stream.get());
				matchNumCS<DEC>(token_text);
				if (stream.peek() == 'e' || stream.peek() == 'E')
				{
					token_text.push_back(stream.get());
					if (stream.peek() == '-' || stream.peek() == '+')
						token_text.push_back(stream.get());
					recognizeNumCS<DEC>(token_text);
				}
				number = std::stod(token_text);
				matchFloatSuffix();
			}
		}

		void recognizeDotBeginFloat()
		{
			recognizeNumCS<DEC>(token_text);
			if (stream.peek() == 'e' || stream.peek() == 'E')
			{
				token_text.push_back(stream.get());
				if (stream.peek() == '-' || stream.peek() == '+')
					token_text.push_back(stream.get());
				recognizeNumCS<DEC>(token_text);
			}
			number = std::stod(token_text);
		}

	public:
		Lexer()
			: file_name{}
			, stream{ file_name }
			, current_token{ Token::END }
			, token_text{}
			, number{ 0 }
		{};

		Lexer(const std::string& str, bool is_file = true)
			: file_name{ is_file ? str : "string" }
			, stream{ file_name }
			, current_token{ Token::END }
			, token_text{}
			, number{ 0 }
		{
			is_file ? stream.loadFile() : stream.loadSource(str);
			next();
		}

		bool loadFile(const std::string& file)
		{
			file_name = file;
			if (!stream.loadFile())
				return false;
			next();
			return true;
		}

		bool loadSource(const std::string& source)
		{
			if (!stream.loadSource(source))
				return false;
			file_name = "string";
			next();
			return true;
		}

		Token next()
		{
			token_text.clear();
			while (true)
			{
				switch (stream.peek())
				{
				case ':':    //  : ::
					stream.get();
					if (stream.peek() == ':')
					{
						stream.get();
						return current_token = Token::SCOPE;
					}
					else
					{
						return current_token = Token::COLON;
					}
				case '+':    //  + (++)
					stream.get();
					if (stream.peek() == '+')
					{
						lexError("Token ++ is not support!");
						stream.get();
						return current_token = Token::INC;
					}
					else
						return current_token = Token::ADD;
				case '-':    //  - (--)
					stream.get();
					if (stream.peek() == '-')
					{
						lexError("Token -- is not support!");
						stream.get();
						return current_token = Token::DEC;
					}
					else
						return current_token = Token::SUB;
				case '.':
					token_text.push_back(stream.get());
					if (decimalSet())
					{
						recognizeDotBeginFloat();
						return current_token = Token::FLOAT;
					}
					else
						return current_token = Token::DOT;

				case '\'':
                    recognizeCharacter();
					return current_token = Token::INT;

				case RAW_STRING_BEGIN:
				case '"':
					recognizeMultiString();
					return current_token = Token::STR;

				case '#':
					while (stream.get() != '\n');
					break;
				case '{':
				case '}':
				case '[':
				case ']':
				case '(':
				case ')':
				case '<':    //  < <<
				case '=':    //  = (==)
				case '>':
				case ',':
				case ';':
				case '*':
				case '/':
				case '%':
					return current_token = static_cast<Token>(stream.get());

				case '~':
				case '!':
				case '&':    //  & (&&)
				case '|':    //  | (||)
				case '^':
				case '?':
					lexError("Current symbol not support!");

				case ' ':
				case '\t':
				case '\v':
				case '\f':
				case '\n':
					stream.get();
					break;

				case static_cast<char>(std::ifstream::traits_type::eof()):
					return current_token = Token::END;

				default:
					if ((stream.peek() >= 'a' && stream.peek() <= 'z') || (stream.peek() >= 'A' && stream.peek() <= 'Z') || (stream.peek() == '_'))
					{
						recognizeID();
						return current_token;
					}

					if (decimalSet())
					{
						recognizeNum();
						return current_token;
					}

					lexError(std::string("Undefined symbol : ") + stream.peek());

					break;
				}
			}
		}

		ArithmeticT getNumber()
		{
			if (current_token == Token::INT || current_token == Token::FLOAT)
				return number;

			syntaxError("Get float value of a not number token");
		}

		Token match(Token t)
		{
			if (current_token == t)
				return next();
			else
				syntaxError(std::string("Expected token ") + tokenToString(t));
		}

		void matchID(const char* id)
		{
			if (current_token == Token::ID && token_text == id)
				next();
			else
				syntaxError(std::string("Expected identify ") + id);
		}

		bool option(Token t)
		{
			if (current_token == t)
			{
				next();
				return true;
			}

			return false;
		}

		Token getToken()
		{
			return current_token;
		}

		const std::string& getTokenText()
		{
			return token_text;
		}

		const std::string& currentFilename()
		{
			return file_name;
		}

		explicit operator bool() const
		{
			return static_cast<bool>(stream);
		}

		[[noreturn]] void syntaxError(const std::string& info)
		{
			std::cerr << file_name << " : " << stream.getLineNum() << " : " << "Syntax error: " << info << std::endl;
			std::cerr << "current token is " << tokenToString(current_token);
			switch (current_token)
			{
			case Token::INT:
			case Token::FLOAT:
				std::cerr << " value is " << number;
				break;
			case Token::ID:
			case Token::STR:
				std::cerr << " value is " << token_text;
				break;
			default:
				break;
			}
			std::cerr << std::endl;
			exit(-1);
		}

		[[noreturn]] void lexError(const std::string& info)
		{
			std::cerr << file_name << ": " << stream.getLineNum() << ": " << "Lexical error: " << info << std::endl;
			exit(-1);
		}

		static std::string tokenToString(Token t)
		{
			static const std::map<Token, std::string> reflex =
			{
				{Token::SCOPE,"::"},
				{Token::INT,  "integer number"},
				{Token::FLOAT,"float number"},
				{Token::STR,"string"},
				{Token::ID,"identify"},
				{Token::BIT_L_SHIFT,"<<"},
				{Token::LOG_AND,"&&"},
				{Token::LOG_OR,"||"},
				{Token::END,"EOF"},
#ifdef COMPILER
				{Token::STRUCT,"struct"},
				{Token::NAMESPACE,"namespace"},
				{Token::ENUM,"enum"},
				{Token::CONSTANT,"const"},
#endif // COMPILER
			};

			auto iter = reflex.find(t);
			if (iter != reflex.end())
				return iter->second;
			else
				return std::string(1, static_cast<char>(t));
		}

	private:
		FilterStream stream;
		std::string file_name;

		Token current_token;
		std::string token_text;
		ArithmeticT number;
	};

	template<> struct Lexer::CharacterSetInfo<Lexer::BIN>
	{
		constexpr static const char* info = "Expected a binary number";
		constexpr static bool (Lexer::* isInSet)() = &Lexer::binarySet;
	};

	template<> struct Lexer::CharacterSetInfo<Lexer::OCT>
	{
		constexpr static const char* info = "Expected a octal number";
		constexpr static bool (Lexer::* isInSet)() = &Lexer::octalSet;
	};

	template<> struct Lexer::CharacterSetInfo<Lexer::DEC>
	{
		constexpr static const char* info = "Expected a decimal number";
		constexpr static bool (Lexer::* isInSet)() = &Lexer::decimalSet;
	};

	template<> struct Lexer::CharacterSetInfo<Lexer::HEX>
	{
		constexpr static const char* info = "Expected a hexadecimal number";
		constexpr static bool (Lexer::* isInSet)() = &Lexer::hexadecimalSet;
	};
} /* namespace: ezcfg */

#undef RAW_STRING_BEGIN
