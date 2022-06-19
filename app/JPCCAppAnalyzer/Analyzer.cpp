#include "Analyzer.h"

#include <filesystem>

using namespace std;
using namespace std::filesystem;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
Analyzer::Analyzer(const float&       frequency,
                   const double&      resolution,
                   const string&      outputDir,
                   const std::string& title,
                   const std::string& otherParameters) :
    frequency_(frequency), resolution_(resolution), filepath_() {
  path outputPath = outputDir;

  stringstream subFolder;
  subFolder << "[" << to_string(frequency_) << "]"   //
            << "[" << to_string(resolution_) << "]"  //
            << otherParameters                       //
            << ".csv";                               //

  outputPath /= subFolder.str();

  if (!filesystem::exists(outputPath)) { create_directories(outputPath); }

  filepath_ = outputPath / (title + ".csv");
}

//////////////////////////////////////////////////////////////////////////////////////////////
const std::filesystem::path& Analyzer::getFilepath() const { return filepath_; }

//////////////////////////////////////////////////////////////////////////////////////////////
bool Analyzer::exists() { return filesystem::exists(filepath_); }

}  // namespace jpcc