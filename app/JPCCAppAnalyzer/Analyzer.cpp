#include "Analyzer.h"

#include <filesystem>
#include <fstream>

using namespace std;
using namespace std::filesystem;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
Analyzer::Analyzer(const float&       frequency,
                   const double&      resolution,
                   const string&      outputDir,
                   const std::string& title,
                   const std::string& otherParameters) :
    frequency_(frequency), resolution_(resolution), filepath_(), hasLock(false) {
  path outputPath = outputDir;

  stringstream subFolder;
  subFolder << "[" << to_string(frequency_) << "]"    //
            << "[" << to_string(resolution_) << "]";  //

  outputPath /= subFolder.str();

  if (!filesystem::exists(outputPath)) { create_directories(outputPath); }

  string filename = title + otherParameters + ".csv";

  filepath_ = outputPath / filename;
}

//////////////////////////////////////////////////////////////////////////////////////////////
Analyzer::~Analyzer() {
  if (hasLock) { remove(filepath_.string() + ".lock"); }
}

//////////////////////////////////////////////////////////////////////////////////////////////
const std::filesystem::path& Analyzer::getFilepath() const { return filepath_; }

//////////////////////////////////////////////////////////////////////////////////////////////
bool Analyzer::exists() { return filesystem::exists(filepath_); }

//////////////////////////////////////////////////////////////////////////////////////////////
bool Analyzer::tryLockFile() {
  bool result = !std::ifstream(filepath_.string() + ".lock");
  if (result) {
    ofstream(filepath_.string() + ".lock").put(' ');
    hasLock = true;
  }
  return result;
}

}  // namespace jpcc