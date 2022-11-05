#include <jpcc/common/ParameterOStream.h>

#include <utility>

using namespace std;
using namespace Eigen;

namespace jpcc {

ParameterOStream::ParameterOStream(ostream& out, string prefix) : prefix_(std::move(prefix)), out_(out) {}

void ParameterOStream::operator()(const vector<jpcc::shared_ptr<Matrix4f>>& matrixVector) {
  out_ << "[" << endl;
  for (size_t i = 0; i < matrixVector.size(); i++) {
    if (i != 0) { out_ << ", " << endl; }
    out_ << "\t\t[";
    (*this)(matrixVector[i]);
  }
  out_ << endl
       << "\t"
       << "]";
}

void ParameterOStream::operator()(const jpcc::shared_ptr<Matrix4f>& matrix) {
  for (auto i = 0; i < matrix->rows(); i++) {
    for (auto j = 0; j < matrix->cols(); j++) {
      if (j != 0) { out_ << ","; }
      out_ << (*matrix)(i, j);
    }
    if (i != matrix->rows() - 1) { out_ << "; "; }
  }
  out_ << "]";
}

void ParameterOStream::operator()(bool value) { out_ << (value ? "true" : "false"); }

}  // namespace jpcc
