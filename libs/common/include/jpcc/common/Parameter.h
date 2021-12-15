#ifndef JPCC_COMMON_PARAMETER_H_
#define JPCC_COMMON_PARAMETER_H_

#include <array>
#include <string>
#include <vector>

#include <boost/program_options.hpp>

namespace jpcc {
namespace common {

namespace po = boost::program_options;

class Parameter {
 public:
  virtual po::options_description getOpts();

  virtual std::vector<std::array<std::string, 2>> getConflicts();

  virtual std::vector<std::array<std::string, 2>> getDependencies();

  virtual void notify();
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_PARAMETER_H_