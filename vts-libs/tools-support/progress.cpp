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

#include <unistd.h>

#include <utility>
#include <numeric>

#include "dbglog/dbglog.hpp"

#include "progress.hpp"

namespace po = boost::program_options;

namespace vtslibs { namespace tools {

namespace {
std::vector<double> adjustWeights(const std::vector<double> &phaseWeights)
{
    /** Compute total unadjusted weight.
     */
    auto total(std::accumulate(phaseWeights.begin(), phaseWeights.end(), 0.0));

    // adjust weights
    std::vector<double> weights;
    for (auto w : phaseWeights) {
        weights.push_back((100 * w) / total);
    }

    return weights;
}

} // namespace

ExternalProgress::ExternalProgress(utility::Filedes &&fd, std::time_t period
                                   , const std::vector<double> &phaseWeights)
    : fd_(std::move(fd)), period_(period)
    , weights_(adjustWeights(phaseWeights))
    , accumulator_(0.0)
    , weight_(0.0), total_(0), value_(0)
    , currentPhase_(-1)
    , lastReport_(0)
{
}

ExternalProgress::ExternalProgress(Config &&config
                                   , const std::vector<double> &phaseWeights)
    : fd_(std::move(config.fd)), period_(config.period)
    , weights_(adjustWeights(phaseWeights))
    , accumulator_(0.0)
    , weight_(0.0), total_(0), value_(0)
    , currentPhase_(-1)
    , lastReport_(0)
{}

void ExternalProgress::expect(std::size_t total)
{
    ++currentPhase_;
    if (currentPhase_ < 0) {
        LOGTHROW(err2, std::runtime_error)
            << "Inconsistent external progress: negative phase index.";
    }

    if (currentPhase_ >= int(weights_.size())) {
        LOGTHROW(err2, std::runtime_error)
            << "Inconsistent external progress: too large index.";
    }

    // update accumulator
    accumulator_ += weight_;

    // update total and reset value
    total_ = total;
    value_ = 0;

    // update weights
    weight_ = weights_[currentPhase_];

    // force report at start
    reportPercentage(value_, (currentPhase_ == 1));
}

void ExternalProgress::done()
{
    ++currentPhase_;
    if (currentPhase_ != int(weights_.size())) {
        LOGTHROW(err2, std::runtime_error)
            << "Inconsistent external progress: done not called "
            "after last phase (" << (weights_.size() - currentPhase_)
            << " more to go).";
    }

    // force report of 100%
    accumulator_ += weight_;
    value_ = 0;
    reportPercentage(value_, true);
}

ExternalProgress& ExternalProgress::operator++()
{
    reportPercentage(++value_);
    return *this;
}

void ExternalProgress::reportPercentage(std::size_t value, bool force)
{
    if (!fd_) { return; }

    const auto now(std::time(nullptr));

    // double locking... atomic check
    if (force || (now >= lastReport_)) {
        std::lock_guard<std::mutex> lock(reportMutex_);
        if (!force && (now < lastReport_)) { return; }

        // fix total
        if (value > total_) { value = total_; }

        const double toReport(accumulator_ + (value * weight_) / total_);

        // we should report
        const auto tmp(str(boost::format("%.2f\n") % toReport));
        const auto *data(tmp.data());
        auto left(tmp.size());

        while (left) {
            auto written(TEMP_FAILURE_RETRY(::write(fd_, data, left)));
            if (-1 == written) {
                LOG(warn2) << "Error writing to progress fd ("
                           << fd_.get() << "): " << errno;
                break;
            }

            left -= written;
            data += written;
        }

        // update next report
        lastReport_ = now + period_;
    }
}

void progressConfiguration(po::options_description &options)
{
    options.add_options()
        ("progress.fd"
         , po::value<int>()->default_value(-1)->required()
         , "File descriptor (number) where merge progress is reported.")
        ("progress.period"
         , po::value<std::time_t>()->default_value(10)->required()
         , "Period between individual progress reports.")
        ;
}

ExternalProgress::Config configureProgress(const po::variables_map &vars)
{
    utility::Filedes fd(vars["progress.fd"].as<int>());
    auto period(vars["progress.period"].as<std::time_t>());
    if (!fd.valid()) {
        LOG(info2) << "Progress fd (" << fd.get() << ") is not valid. "
            "Disabling progress logging.";
        return {};
    }

    if (period <= 0) {
        throw po::validation_error
            (po::validation_error::invalid_option_value
             , "progress.period");
    }

    // create fd duplicate and ignore fd
    auto dup(fd.dup());
    fd.release();

    return ExternalProgress::Config(std::move(dup), period);
}

} } // namespace vtslibs::tools
