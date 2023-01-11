#pragma once

#include <fstream>

#include <jpcc/common/CoderBackendType.h>
#include <jpcc/common/Common.h>
#include <jpcc/common/IJPCCEncoderContext.h>
#include <jpcc/common/IJPCCDecoderContext.h>
#include <jpcc/common/JPCCHeader.h>

namespace jpcc {

class JPCCCoderContext : public IJPCCEncoderContext, public IJPCCDecoderContext {
 public:
  using Ptr = std::shared_ptr<JPCCCoderContext>;

 protected:
  Index        startFrameNumber_;
  JPCCHeader   header_;
  std::string  compressedStreamPath_;
  std::fstream fs_;
  GroupOfFrame frames_;
  // coder specified type
  std::vector<std::shared_ptr<void>> coderFrames_;

 public:
  JPCCCoderContext(const std::string& compressedStreamPath);

  void writeHeader(double resolution, CoderBackendType backendType);

  void readHeader();

  void clear();

  void flush();

  void ifsSeekgEnd();

  [[nodiscard]] bool eof() const;

  [[nodiscard]] JPCCHeader          getHeader() const { return header_; };
  [[nodiscard]] Index               getStartFrameNumber() const override { return startFrameNumber_; };
  [[nodiscard]] const GroupOfFrame& getFrames() const override { return frames_; };
  [[nodiscard]] const std::vector<std::shared_ptr<void>>& getCoderFrames() const override { return coderFrames_; }
  [[nodiscard]] const std::ostream&                       getOs() const override { return fs_; };
  [[nodiscard]] const std::istream&                       getIs() const override { return fs_; };

  [[nodiscard]] Index&                              getStartFrameNumber() override { return startFrameNumber_; };
  [[nodiscard]] GroupOfFrame&                       getFrames() override { return frames_; };
  [[nodiscard]] std::vector<std::shared_ptr<void>>& getCoderFrames() override { return coderFrames_; }
  [[nodiscard]] std::ostream&                       getOs() override { return fs_; };
  [[nodiscard]] std::istream&                       getIs() override { return fs_; };
};

}  // namespace jpcc
