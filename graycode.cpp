#include "graycode.h"
#include <cmath>

GrayCode::GrayCode(int n) : m_n(n)
{
    if (n < 1) return;
    std::vector<std::string> code_temp = __createGrayCode(n);
    __formCodes(n, code_temp);

    // ÃÓ≥‰”≥…‰±Ì
    for (int k = 0; k < static_cast<int>(std::pow(2, n)); ++k)
    {
        int gray_value_as_int = __k2v(k);
        m_code2k_int[gray_value_as_int] = k;
        m_k2v[k] = gray_value_as_int;
    }
}

GrayCode::~GrayCode() {}

std::vector<std::string> GrayCode::__createGrayCode(int n)
{
    if (n < 1)
    {
        return std::vector<std::string>();
    }
    if (n == 1)
    {
        return { "0", "1" };
    }

    std::vector<std::string> prev_code = __createGrayCode(n - 1);
    std::vector<std::string> code;
    for (const auto& s : prev_code)
    {
        code.push_back("0" + s);
    }
    for (int i = static_cast<int>(prev_code.size()) - 1; i >= 0; --i)
    {
        code.push_back("1" + prev_code[i]);
    }
    return code;
}

void GrayCode::__formCodes(int n, const std::vector<std::string>& code_temp)
{
    int num_codes = static_cast<int>(code_temp.size());
    m_codes.create(n, num_codes, CV_8U);

    for (int row = 0; row < n; ++row)
    {
        for (int col = 0; col < static_cast<int>(num_codes); ++col)
        {
            m_codes.at<uchar>(row, col) = (code_temp[col][row] == '1' ? 1 : 0);
        }
    }
}

std::string GrayCode::__code2k(int k)
{
    std::string code = "";
    for (int i = 0; i < m_n; ++i)
    {
        code += std::to_string(m_codes.at<uchar>(i, k));
    }
    return code;
}

int GrayCode::__k2v(int k)
{
    std::string code = __code2k(k);
    return std::stoi(code, nullptr, 2);
}
