#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

#include <boost/program_options.hpp>

#include <jpcc/common/Common.h>
#include <jpcc/common/ParameterOStream.h>

namespace jpcc {

namespace po = boost::program_options;

class Parameter {
 protected:
  std::string             prefix_;
  std::string             caption_;
  po::options_description opts_;

 public:
  Parameter(std::string prefix, const std::string& caption);

  [[nodiscard]] virtual const po::options_description& getOpts() const;

  [[nodiscard]] virtual std::vector<std::array<std::string, 2>> getConflicts() const;

  [[nodiscard]] virtual std::vector<std::array<std::string, 2>> getDependencies() const;

  [[nodiscard]] virtual std::vector<std::string> getShowTexts() const;

  virtual void getShowTexts(std::vector<std::string>& showTexts) const;

  virtual void notify();

  ParameterOStream coutParameters(std::ostream& out) const;
};

}  // namespace jpcc
