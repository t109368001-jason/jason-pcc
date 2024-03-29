/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2018, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include <chrono>

namespace jpcc {
namespace chrono {
/**
 * Measurement of cumulative elapsed time intervals.
 *
 * This timer acts like a stopwatch and may be used to measure the
 * cumulative periods between successive calls to start() and stop().
 */
template <typename Clock>
class Stopwatch {
 public:
  typedef typename Clock::duration duration;

  /// Reset the accumulated interval count.
  void reset();

  /// Mark the beginning of a measurement period.
  void start();

  /// Mark the end of a measurement period.
  ///
  /// @return  the duration of the elapsed Clock time since start()
  duration stop();

  /// The sum of the previous elapsed time intervals.
  ///
  /// NB: this excludes any currently active period marked by start().
  ///
  /// @return  cumulative elapsed time
  constexpr duration count() const { return cumulative_time_; }

 private:
  typename Clock::time_point start_time_;
  duration                   cumulative_time_{duration::zero()};
};
}  // namespace chrono
}  // namespace jpcc

//===========================================================================

template <typename Clock>
void jpcc::chrono::Stopwatch<Clock>::reset() {
  cumulative_time_ = cumulative_time_.zero();
}

//---------------------------------------------------------------------------

template <typename Clock>
void jpcc::chrono::Stopwatch<Clock>::start() {
  start_time_ = Clock::now();
}

//---------------------------------------------------------------------------

template <typename Clock>
typename jpcc::chrono::Stopwatch<Clock>::duration jpcc::chrono::Stopwatch<Clock>::stop() {
  const auto& delta = duration(Clock::now() - start_time_);
  cumulative_time_ += delta;
  return delta;
}

//===========================================================================
