#ifndef JPCC_COMMON_PARAMETER_H_
#define JPCC_COMMON_PARAMETER_H_

#include <array>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include <jpcc/common/Common.h>

namespace jpcc {

namespace po = boost::program_options;

namespace common {

class Parameter {
 protected:
  std::string             prefix_;
  std::string             caption_;
  po::options_description opts_;

 public:
  Parameter(std::string prefix, const std::string& caption);

  [[nodiscard]] virtual po::options_description& getOpts();

  [[nodiscard]] virtual std::vector<std::array<std::string, 2>> getConflicts() const;

  [[nodiscard]] virtual std::vector<std::array<std::string, 2>> getDependencies() const;

  virtual void notify();
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_PARAMETER_H_