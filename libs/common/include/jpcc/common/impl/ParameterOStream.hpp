#pragma once

namespace jpcc {

template <typename T>
ParameterOStream& ParameterOStream::operator()(const std::string& optName, T value) {
  out_ << "\t" << prefix_ << optName << "=";
  (*this)(value);
  out_ << std::endl;
  return *this;
}

template <typename T>
void ParameterOStream::operator()(std::vector<T> value) {
  out_ << "[";
  for (size_t i = 0; i < value.size(); i++) {
    if (i != 0) { out_ << ", "; }
    (*this)(value.at(i));
  }
  out_ << "]";
}

template <typename T>
void ParameterOStream::operator()(T value) {
  out_ << value;
}

}  // namespace jpcc
