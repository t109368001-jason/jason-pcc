#pragma once

#include <memory>
#include <vector>

#include <jpcc/common/Common.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/DatasetReaderParameter.h>
#include <jpcc/io/DatasetStreamReader.h>
#include <jpcc/io/Pcap.h>

namespace jpcc::io {

class PcapReader : public DatasetStreamReader {
 protected:
  int                                                      maxNumLasers_;
  std::vector<float>                                       verticals_;
  std::vector<float>                                       sinVerticals_;
  std::vector<float>                                       cosVerticals_;
  std::vector<std::unique_ptr<void, decltype(&pcapClose)>> pcaps_;
  std::vector<uint16_t>                                    lastAzimuth100s_;

 public:
  PcapReader(DatasetReaderParameter param, DatasetParameter datasetParam);

 protected:
  void open_(size_t datasetIndex, size_t startFrameNumber) override;

  [[nodiscard]] bool isOpen_(size_t datasetIndex) const override;

  [[nodiscard]] bool isEof_(size_t datasetIndex) const override;

  void load_(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize) override;

  void parseDataPacket(size_t        startFrameNumber,
                       size_t&       currentFrameNumber,
                       void*         pcap,
                       uint16_t&     lastAzimuth100,
                       GroupOfFrame& frameBuffer);
};

}  // namespace jpcc::io
