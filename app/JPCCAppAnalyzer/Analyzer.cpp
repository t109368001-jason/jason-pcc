#include "Analyzer.h"

#include <filesystem>

using namespace std;
using namespace std::filesystem;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
Analyzer::Analyzer(const string& outputDir, const string& filename) : filepath_() {
  filepath_ = outputDir;
  filepath_ /= filename;
}

//////////////////////////////////////////////////////////////////////////////////////////////
const std::filesystem::path& Analyzer::getFilepath() const { return filepath_; }

//////////////////////////////////////////////////////////////////////////////////////////////
bool Analyzer::exists() { return filesystem::exists(filepath_); }

}  // namespace jpcc