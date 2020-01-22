/**
 * Copyright (c) 2019 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <Windows.h>
#  define VTS_IO_GMTIME(t, tm) gmtime_s(tm, t)
#else
#  include <sys/time.h>
#  define VTS_IO_GMTIME(t, tm) gmtime_r(t, tm)
#endif

#include <ctime>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>

#include "io.hpp"

namespace ba = boost::algorithm;

namespace vtslibs { namespace registry {

namespace detail {
const Credit dummyCredit;
} // namespace detail

std::ostream& operator<<(std::ostream&os, const CreditHtmlizer &html)
{
    boost::optional<std::string> year;
    const auto &getYear([&]() -> const std::string&
    {
        if (!year) {
            const auto now(std::time(nullptr));
            std::tm tm;
            VTS_IO_GMTIME(&now, &tm);
            year = boost::lexical_cast<std::string>(tm.tm_year + 1900);
        }
        return *year;
    });

    const auto escaped([&](char c)
    {
        switch (c) {
        case '&': os << "&amp;"; break;
        case '<': os << "&lt;"; break;
        case '>': os << "&gt;"; break;
        case '"': os << "&quot;"; break;
        default: os << c; break;
        }
    });

    const auto escapedString([&](const std::string &text)
    {
        for (const auto c : text) { escaped(c); }
    });

    const auto &parseTo([&](char end, const char *&p) -> std::string
    {
        std::string out;
        while (const auto c = *p) {
            out.push_back(c);
            if (c == end) { return out; }
            ++p;
        }
        return out;
    });

    const auto &replace([&](const std::string &what)
    {
        if (what == "{copy}") { os << "&copy;"; return;  }
        if (what == "{Y}") { os << getYear(); return; }
    });

    const auto beginA([&](const std::string &url)
    {
        os << "<a href=\"";
        escapedString(url);
        os << "\">";
    });

    const auto endA([&]() { os << "</a>"; });

    const auto &url([&](const std::string &what)
    {
        std::string url;
        std::string text;
        bool split(false);
        for (const auto c : what) {
            switch (c) {
            case '[': case ']': continue;
            case ' ':
                if (!split) {
                    split = true;
                } else {
                    text.push_back(c);
                }
                break;
            default:
                (split ? text : url).push_back(c);
                break;
            }
        }

        beginA(url);
        escapedString(text.empty() ? url : text);
        endA();
    });

    const auto &credit(*html.credit);

    if (credit.url) { beginA(*credit.url); }

    for (const auto *p(credit.notice.c_str()); *p; ++p) {
        switch (const auto c = *p) {
        case '{': replace(parseTo('}', p)); break;
        case '[': url(parseTo(']', p));  break;
        default: escaped(c); break;
        }
    }

    if (credit.url) { endA(); }

    return os;
}

} } // namespace vtslibs::registry
