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

#ifndef vtslibs_vts_tileset_driver_runcallback_hpp_included_
#define vtslibs_vts_tileset_driver_runcallback_hpp_included_

#include "dbglog/dbglog.hpp"

#include "streams.hpp"

namespace vtslibs { namespace vts {

template <typename Generator, typename ValueType, typename Traits>
void runCallback(const Generator &generator
                 , const std::function<void(const utility::Expected
                                            <ValueType, Traits>&)> &cb);

template <typename ValueType, typename Traits>
void runCallback(const std::exception_ptr &exc
                 , const std::function<void(const utility::Expected
                                            <ValueType, Traits>&)> &cb);

template <typename ValueType, typename Traits>
void runCallback(const std::function<void(const utility::Expected
                                          <ValueType, Traits>&)> &cb);

// implementation

template <typename Generator, typename ValueType, typename Traits>
void runCallback(const Generator &generator
                 , const std::function<void(const utility::Expected
                                            <ValueType, Traits>&)> &cb)
{
    utility::Expected<ValueType> evalue;
    try {
        evalue.set(generator());
    } catch (...) {
        // forward failed attempt to generate source
        return runCallback(cb);
    }

    // we have valid input, use
    try {
        cb(evalue);
    } catch (const std::exception &e) {
        LOG(warn3)
            << "Failed to call a callback with a walue; reason: <"
            << e.what() << ">.";
    } catch (...) {
        LOG(warn3)
            << "Failed to call a callback with a value: reason unknown.";
    }
}

template <typename ValueType, typename Traits>
void runCallback(const std::exception_ptr &exc
                 , const std::function<void(const utility::Expected
                                            <ValueType, Traits>&)> &cb)
{
    // we have valid input, use
    try {
        cb(exc);
    } catch (const std::exception &e) {
        LOG(warn3)
            << "Failed to call a callback with an exception; reason: <"
            << e.what() << ">.";
    } catch (...) {
        LOG(warn3)
            << "Failed to call a callback with an exception: reason unknown.";
    }
}

template <typename ValueType, typename Traits>
void runCallback(const std::function<void(const utility::Expected
                                          <ValueType, Traits>&)> &cb)
{
    runCallback(std::current_exception(), cb);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_driver_runcallback_hpp_included_
