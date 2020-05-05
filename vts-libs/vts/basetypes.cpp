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
#include <algorithm>

#include <boost/range/adaptor/reversed.hpp>

#include "dbglog/dbglog.hpp"

#include "glue.hpp"
#include "tileop.hpp"

namespace vtslibs { namespace vts {

namespace {
template <typename Iterator>
std::size_t detectAlien(Iterator &b, const Iterator &e
                        , const TilesetId &tilesetId
                        , std::size_t size)
{
    // starts with different tilesetId? -> increment
    if ((b != e) && (*b != tilesetId)) {
        ++b;
        return size - 1;
    }

    // original size
    return size;
}

} // namespace

TileSetGlues::list glueOrder(const TileSetGlues::list &in)
{
    // creata tileset alphabet
    std::map<TilesetId, int> alphabet;
    {
        int d(0);
        for (const auto &tsg : boost::adaptors::reverse(in)) {
            alphabet[tsg.tilesetId] = d;
            LOG(info2) << "Depth <" << tsg.tilesetId << ">: " << d << ".";
            ++d;
        }
    }

    // build tileset stack
    TilesetIdList stack;
    for (const auto &tsg : in) { stack.push_back(tsg.tilesetId); }

    TileSetGlues::list out;

    typedef TileSetGlues::EnhancedGlue EGlue;

    for (const auto &tsg : in) {
        auto compareGlues([&](const EGlue &l, const EGlue &r) -> bool
        {
            auto lr(boost::adaptors::reverse(l.id));
            auto rr(boost::adaptors::reverse(r.id));
            auto lrb(begin(lr)), lre(end(lr));
            auto rrb(begin(rr)), rre(end(rr));

            auto lsize(detectAlien(lrb, lre, tsg.tilesetId, l.id.size()));
            auto rsize(detectAlien(rrb, rre, tsg.tilesetId, r.id.size()));

            while ((lrb != lre) && (rrb != rre)) {
                // grab characters
                auto lc(alphabet[*lrb++]);
                auto rc(alphabet[*rrb++]);

                if (lc < rc) {
                    return true;
                } else if (rc < lc) {
                    return false;
                }
                // same character at the same position -> next one
            }

            // one id is prefix or the other (or both are the same, which is
            // unlikely) -> longest is less

            // what to do if we have normal/alien ID or alien/alien ID of the
            // same length?
            return rsize < lsize;
        });

        out.push_back(tsg);
        std::sort(out.back().glues.begin(), out.back().glues.end()
                  , compareGlues);

        for (auto &glue : out.back().glues) {
            std::size_t i(0);
            for (const auto &id : glue.id) {
                if (i >= stack.size()) {
                    LOGTHROW(err2, vtslibs::storage::Error)
                        << "Glue <" << utility::join(id, ", ")
                        << "> doesn't belong into tileset stack stack <"
                        << utility::join(stack, ",") << ">.";
                }

                while (i < stack.size()) {
                    if (stack[i] == id) {
                        glue.indices.push_back(int(i));
                        ++i;
                        break;
                    } else {
                        ++i;
                    }
                }
            }
        }
    }

    return out;
}

Ranges::Ranges(const LodRange &lodRange, const TileRange &tileRange)
    : lodRange_(lodRange)
{
    populate(tileRange);
}

Ranges::Ranges(const LodTileRange &range, Lod bottomLod)
    : lodRange_(range.lod, bottomLod)
{
    populate(range.range);
}

void Ranges::populate(const TileRange &tileRange)
{
    if (!math::valid(tileRange)) {
        LOGTHROW(err2, vtslibs::storage::Error)
            << "Invalid tile range: " << tileRange << ".";
    }

    if (lodRange_.empty()) { return; }

    // fill in ranges
    for (auto lod : lodRange_) {
        (void) lod;
        if (tileRanges_.empty()) {
            // original argument
            tileRanges_.push_back(tileRange);
        } else {
            // child range of previous lod
            tileRanges_.push_back(childRange(tileRanges_.back()));
        }
    }
}

Ranges::Ranges(const LodRange &lodRange, const TileRange &tileRange
               , const FromBottom&)
    : lodRange_(lodRange)
{
    if (!math::valid(tileRange)) {
        LOGTHROW(err2, vtslibs::storage::Error)
            << "Invalid tile range: " << tileRange << ".";
    }

    if (lodRange.empty()) { return; }

    // fill in ranges, in reverse order
    for (auto lod : lodRange) {
        (void) lod;
        if (tileRanges_.empty()) {
            // original argument
            tileRanges_.push_back(tileRange);
        } else {
            // child range of previous lod
            tileRanges_.push_back(parentRange(tileRanges_.back()));
        }
    }

    // reverse ranges
    std::reverse(tileRanges_.begin(), tileRanges_.end());
}

const TileRange& Ranges::tileRange(Lod lod) const
{
    if (!in(lod, lodRange_)) {
        LOGTHROW(err2, vtslibs::storage::Error)
            << "Lod <" << lod << "> outside of lod range <"
            << lodRange_ << ">.";
    }

    return tileRanges_[lod - lodRange_.min];
}

const TileRange* Ranges::tileRange(Lod lod, std::nothrow_t) const
{
    if (!in(lod, lodRange_)) { return nullptr; }
    return &tileRanges_[lod - lodRange_.min];
}

std::vector<LodTileRange> Ranges::ranges() const
{
    std::vector<LodTileRange> ranges;
    auto itileRanges(tileRanges_.begin());
    for (auto lod : lodRange_) {
        ranges.emplace_back(lod, *itileRanges++);
    }
    return ranges;
}

namespace {
const TileRange emptyTileRange;
} //namespace

const TileRange& Ranges::tileRange() const
{
    return (tileRanges_.empty() ? emptyTileRange : tileRanges_.front());
}

void Ranges::update(const Ranges &other)
{
    LodRange lr(unite(lodRange_, other.lodRange_));
    if (lr.empty()) { return; }

    // fill in ranges
    std::vector<TileRange> tileRanges;
    for (auto lod : lr) {
        const auto *tr1(tileRange(lod, std::nothrow));
        const auto *tr2(other.tileRange(lod, std::nothrow));
        if (tr1 && tr2) {
            // merge two ranges
            tileRanges.push_back(*tr1);
            math::update(tileRanges.back(), tr2->ll);
            math::update(tileRanges.back(), tr2->ur);
        } else if (tr1) {
            // one range
            tileRanges.push_back(*tr1);
        } else if (tr2) {
            // one range
            tileRanges.push_back(*tr2);
        } else if (!tileRanges.empty()) {
            // single range and something present
            tileRanges.push_back(childRange(tileRanges.back()));
        }
    }

    // store
    lodRange_ = lr;
    std::swap(tileRanges_, tileRanges);
}

} } // namespace vtslibs::vts
