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

#ifndef vts_libs_tools_progress_hpp_included
#define vts_libs_tools_progress_hpp_included

#include <atomic>
#include <ctime>
#include <mutex>
#include <utility>

#include <boost/noncopyable.hpp>
#include <boost/program_options.hpp>

#include "utility/filedes.hpp"

#include "../vts/ntgenerator.hpp"

namespace vtslibs { namespace tools {

/**
 * Progress with external reporting 0.0 -- 100.0 with phases. Report goes into
 * configured file descriptor.
 *
 * Expected usage:
 *
 * ExternalProgress ep(..., ..., { weight, ... });
 *
 * // single thread
 * ep.expect(...);
 *
 * // multiple threads doing some work:
 * ++ep;
 *
 * // single thread
 * // ep.expect(...);
 *
 * // multiple threads doing some work:
 * ++ep;
 *
 * // single thread
 * done();
 *
 */
class ExternalProgress
    : boost::noncopyable
    , public vts::NtGenerator::Reporter
{
public:
    struct Config {
        utility::Filedes fd;
        std::time_t period;

        Config(utility::Filedes &&fd = utility::Filedes()
               , std::time_t period = 0)
            : fd(std::move(fd)), period(period)
        {}
    };

    typedef std::vector<double> Weights;

    /** Create external progress reporter.
     *
     * Phase weights are adjusted to make 100% together:
     * weights (1000, 40, 100) are transformed into (87.72%, 3.51%, 8.77%).
     *
     * \param fd file descriptor to report to
     * \param period time period between reports (in seconds)
     * \param weights of individual phases
     */
    ExternalProgress(utility::Filedes &&fd, std::time_t period
                     , const Weights &phaseWeights);

    ExternalProgress(Config &&config, const Weights &phaseWeights);

    /** Switch to next phase. If current accumulated value is less than expected
     *  total current value is updated.
     *
     *  THREAD SAFETY: unsafe, only one thread can call this function;
     *  done() and operator++() cannot be called concurently
     *
     *  Implementation of NtGenerator::Reporter::expect.
     *
     */
    void expect(std::size_t total);

    /** It's 100%.
     *
     *  THREAD SAFETY: unsafe, only one thread can call this function;
     *  expect() and operator++() cannot be called concurently
     */
    void done();

    /** Add one more tick and report when time is right.
     *  If there are more thicks than expected, do nothing.
     *
     *  THREAD SAFETY: safe, multiple threads can call this function but not
     *  expect() and done()!
     */
    ExternalProgress& operator++();

    /** Implementation of NtGenerator::Reporter::report.
     */
    void report() { ++(*this); }

private:
    void reportPercentage(std::size_t value, bool force = false);

    const utility::Filedes fd_;
    const std::time_t period_;

    /** Adjusted accumulated weights.
     */
    const Weights weights_;

    double accumulator_;

    /** Current adjusted accumulated weight.
     */
    double weight_;

    /** Current total.
     */
    std::size_t total_;

    /** Current value.
     */
    std::atomic<std::size_t> value_;

    /** Index of current phase, -1 means none.
     */
    int currentPhase_;

    /** Last report timestamp.
     */
    std::atomic<std::time_t> lastReport_;

    /** Report mutex.
     */
    std::mutex reportMutex_;
};

void progressConfiguration(boost::program_options::options_description
                           &options);

ExternalProgress::Config
configureProgress(const boost::program_options::variables_map &vars);

} } // namespace vtslibs::tools

#endif // vts_libs_tools_progress_hpp_included
