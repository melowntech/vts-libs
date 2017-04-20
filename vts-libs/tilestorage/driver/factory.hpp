/**
 * Copyright (c) 2017 Melown Technologies SE
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
#ifndef vtslibs_tilestorage_driver_factory_hpp_included_
#define vtslibs_tilestorage_driver_factory_hpp_included_

#include "utility/gccversion.hpp"

#include "../driver.hpp"

namespace vtslibs { namespace tilestorage {

#define VADSTENA_TILESTORAGE_DRIVER_FACTORY(DRIVER_TYPE, DRIVER_CLASS)  \
    class Factory : public Driver::Factory {                            \
    public:                                                             \
        Factory() : Driver::Factory(DRIVER_TYPE) {}                     \
                                                                        \
        virtual Driver::pointer                                         \
        create(const std::string location, CreateMode mode              \
               , const Driver::CreateProperties &properties)            \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return std::make_shared<DRIVER_CLASS>                       \
                (location, mode, properties);                           \
        }                                                               \
                                                                        \
        virtual Driver::pointer open(const std::string location         \
                                     , OpenMode mode                    \
                                     , const DetectionContext &context) \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return std::make_shared<DRIVER_CLASS>                       \
                (location, mode, context);                             \
        }                                                               \
                                                                        \
        virtual std::string help() const UTILITY_OVERRIDE               \
        {                                                               \
            return DRIVER_CLASS::help;                                  \
        }                                                               \
                                                                        \
        virtual std::string detectType(DetectionContext &context        \
                                       , const std::string &location)   \
            const UTILITY_OVERRIDE                                      \
        {                                                               \
            return DRIVER_CLASS::detectType_impl(context, location); \
        }                                                               \
                                                                        \
        static const char* staticType()                                 \
        {                                                               \
            return DRIVER_TYPE;                                         \
        }                                                               \
    }

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_driver_factory_hpp_included_
