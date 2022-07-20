#pragma once

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <jpcc/common/Parameter.h>

namespace jpcc::io {

enum class Sensor { NONE, MID_100, MID_70, VLP_16, HI_RES, HDL_32 };
enum class Type { NONE, PLY, PCAP, LVX };
enum class EncodeType { NONE, DYNAMIC_STATIC, DYNAMIC_STATIC_ADDED_STATIC_REMOVED };

class DatasetParameter : public virtual Parameter {
 protected:
  std::string              sensor_;
  std::string              type_;
  std::string              encodedType_;
  std::vector<std::string> transforms_;

 public:
  std::string                              name;
  Sensor                                   sensor;
  Type                                     type;
  bool                                     preProcessed;
  EncodeType                               encodedType;
  std::string                              folderPrefix;
  std::string                              folder;
  std::filesystem::path                    folderPath;
  std::vector<std::string>                 files;
  std::vector<std::filesystem::path>       filePaths;
  std::vector<size_t>                      frameCounts;
  std::vector<size_t>                      startFrameNumbers;
  std::vector<size_t>                      endFrameNumbers;
  std::vector<shared_ptr<Eigen::Matrix4f>> transforms;
  bool                                     haveGpsTime;

  DatasetParameter();

  DatasetParameter(const std::string& prefix, const std::string& caption);

  [[nodiscard]] std::vector<std::array<std::string, 2>> getDependencies() const override;

  void getShowTexts(std::vector<std::string>& showTexts) const override;

  void notify() override;

  void notify(bool isInput);

  [[nodiscard]] size_t count() const;

  [[nodiscard]] std::string getFilePath(size_t index = 0) const;

  [[nodiscard]] size_t getFrameCounts(size_t index = 0) const;

  [[nodiscard]] size_t getFrameCount() const;

  [[nodiscard]] size_t getStartFrameNumbers(size_t index = 0) const;

  [[nodiscard]] size_t getStartFrameNumber() const;

  [[nodiscard]] size_t getEndFrameNumbers(size_t index = 0) const;

  [[nodiscard]] size_t getEndFrameNumber() const;

  [[nodiscard]] shared_ptr<Eigen::Matrix4f> getTransforms(size_t index = 0) const;

  friend std::ostream& operator<<(std::ostream& out, const DatasetParameter& obj);
};

Sensor getSensor(const std::string& sensor);

Type getType(const std::string& type);

EncodeType getEncodeType(const std::string& encodeType);

}  // namespace jpcc::io
