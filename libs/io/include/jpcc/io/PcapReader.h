#pragma once

#include <vector>

#include <pcap/pcap.h>

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetReaderBase.h>

namespace jpcc::io {

class PcapReader : public DatasetReaderBase {
 protected:
  int                   maxNumLasers_;
  std::vector<float>    verticals_;
  std::vector<float>    sinVerticals_;
  std::vector<float>    cosVerticals_;
  std::vector<pcap_t*>  pcaps_;
  std::vector<uint16_t> lastAzimuth100s_;

 public:
  PcapReader(DatasetReaderParameter param, DatasetParameter datasetParam);

 protected:
  void open_(size_t datasetIndex, size_t startFrameNumber) override;

  [[nodiscard]] bool isOpen_(size_t datasetIndex) override;

  [[nodiscard]] bool isEof_(size_t datasetIndex) override;

  void load_(size_t datasetIndex, size_t startFrameNumber, size_t groupOfFramesSize, GroupOfFrame& frames) override;

  void close_(size_t datasetIndex) override;

  [[nodiscard]] int parseDataPacket(size_t                   startFrameNumber,
                                    size_t&                  currentFrameNumber,
                                    pcap_t*                  pcap,
                                    uint16_t&                lastAzimuth100,
                                    std::vector<Frame::Ptr>& frameBuffer);
};

}  // namespace jpcc::io
