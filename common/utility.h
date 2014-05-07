#pragma once
#include <string>

namespace util
{
	std::wstring s2ws(const std::string str)
	{
		unsigned len = str.size() * 2;
		setlocale(LC_CTYPE, "");
		wchar_t *p = new wchar_t[len];
		mbstowcs(p, str.c_str(), len);
		std::wstring str1(p);
		delete[] p;
		return str1;
	}

	std::string CW2A(const std::wstring str)
	{
		unsigned len = str.size() * 4;
		setlocale(LC_CTYPE, "");
		char *p = new char[len];
		wcstombs(p, str.c_str(), len);
		std::string str1(p);
		delete[] p;
		return str1;
	}
};

