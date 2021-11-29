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
 protected:
  po::options_description opts_;

 public:
  Parameter(const std::string optsName);

  po::options_description& getOpts();

  virtual std::vector<std::array<std::string, 2>> getConflicts();

  virtual std::vector<std::array<std::string, 2>> getDependencies();
};

}  // namespace common
}  // namespace jpcc

#endif  // JPCC_COMMON_PARAMETER_H_