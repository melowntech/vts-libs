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
/**
 * \file vts/storage/gluerules.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set glue creation rules
 */

#ifndef vtslibs_vts_storage_gluerules_hpp_included_
#define vtslibs_vts_storage_gluerules_hpp_included_

#include <memory>

#include "../storage.hpp"

namespace vtslibs { namespace vts {

/** Glue creation rule.
 */
class GlueRule {
public:
    typedef std::shared_ptr<GlueRule> pointer;
    typedef std::vector<pointer> list;

    virtual ~GlueRule() {}

    class MatcherBase {
    public:
        typedef std::shared_ptr<MatcherBase> pointer;
        typedef std::vector<pointer> list;

        virtual ~MatcherBase() {}

        /** Checks whether given tag set is allowed.
         */
        bool check(const StoredTileset &tileset);

        pointer clone() const;

    private:
        virtual bool check_impl(const StoredTileset &tileset) = 0;
        virtual pointer clone_impl() const = 0;
    };

    /** Generate Matcher
     */
    MatcherBase::pointer matcher();

    /** Dump information into stream. Should follow the same representation as
     *  is parsed from input.
     */
    std::ostream& dump(std::ostream &os, const std::string &prefix) const;

private:
    virtual MatcherBase::pointer matcher_impl() = 0;

    virtual std::ostream& dump_impl(std::ostream &os
                                    , const std::string &prefix) const = 0;
};

/** Iterative glue rule checker.
 */
class GlueRuleChecker {
public:
    /** Creates glue rule checker.
     */
    GlueRuleChecker(const GlueRule::list &rules);

    /** Deep copy constructor.
     */
    GlueRuleChecker(const GlueRuleChecker &other);

    /** Check whether this tileset can be added to glue.
     */
    bool operator()(const StoredTileset &tileset) const;

private:
    GlueRule::MatcherBase::list matchers_;
};

/** Check whether given list of stored tilesets can be glued together.
 */
bool check(const GlueRule::list &rules
           , const StoredTileset::constptrlist &tilesets);

/** Load glue rules file.
 *
 * \param path path to glue rules file
 * \param ignoreNoexistent ignore non-existent file (return empty list)
 * \return list of parsed rules
 */
GlueRule::list loadGlueRules(const boost::filesystem::path &path
                             , bool ignoreNoexistent = false);

// inlines

inline GlueRule::MatcherBase::pointer GlueRule::matcher()
{
    return matcher_impl();
}

inline std::ostream& GlueRule::dump(std::ostream &os
                                    , const std::string &prefix) const
{
    return dump_impl(os, prefix);
}

inline bool GlueRule::MatcherBase::check(const StoredTileset &tags)
{
    return check_impl(tags);
}

inline GlueRule::MatcherBase::pointer GlueRule::MatcherBase::clone() const
{
    return clone_impl();
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_storage_gluerules_hpp_included_
