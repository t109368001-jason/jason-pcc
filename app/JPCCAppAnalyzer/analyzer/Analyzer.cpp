#include "Analyzer.h"

#include <filesystem>
#include <fstream>

#include <pcl/io/auto_io.h>

using namespace std;
using namespace std::filesystem;

namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
Analyzer::Analyzer(const float&  frequency,
                   const double& resolution,
                   const string& outputDir,
                   const string& title,
                   const string& otherParameters) :
    frequency_(frequency), resolution_(resolution), filepath_(), hasLock(false) {
  path outputPath = outputDir;

  stringstream subFolder;
  subFolder << "[" << to_string(frequency_) << "]"    //
            << "[" << to_string(resolution_) << "]";  //

  outputPath /= subFolder.str();

  if (!filesystem::exists(outputPath)) {
    create_directories(outputPath);
  }

  string filename = title + otherParameters + ".csv";

  filepath_ = outputPath / filename;
}

//////////////////////////////////////////////////////////////////////////////////////////////
Analyzer::~Analyzer() {
  releaseLockFile();
}

//////////////////////////////////////////////////////////////////////////////////////////////
const filesystem::path& Analyzer::getFilepath() const {
  return filepath_;
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool Analyzer::exists() {
  return filesystem::exists(filepath_);
}

//////////////////////////////////////////////////////////////////////////////////////////////
bool Analyzer::tryLockFile() {
  bool result = !ifstream(filepath_.string() + ".lock");
  if (result) {
    ofstream(filepath_.string() + ".lock").put(' ');
    hasLock = true;
  }
  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void Analyzer::releaseLockFile() {
  if (hasLock) {
    remove(filepath_.string() + ".lock");
    hasLock = false;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void Analyzer::reset() {
  releaseLockFile();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void Analyzer::saveCloud() {
  PclFramePtr<PointAnalyzer> cloud;
  getCloud(cloud);
  if (cloud) {
    path cloudPath = filepath_;
    cloudPath.replace_extension(".ply");

    pcl::io::savePLYFileASCII(cloudPath.string(), *cloud);
  }
}

}  // namespace jpcc