#pragma once

#include <vector>

#include <pcap/pcap.h>

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/io/PcapReaderParameter.h>

namespace jpcc::io {

class PcapReader : public DatasetReader {
 protected:
  PcapReaderParameter   param_;
  int                   maxNumLasers_;
  std::vector<float>    verticals_;
  std::vector<float>    sinVerticals_;
  std::vector<float>    cosVerticals_;
  std::vector<pcap_t*>  pcaps_;
  std::vector<uint16_t> lastAzimuth100s_;

 public:
  PcapReader(PcapReaderParameter param, DatasetParameter datasetParam);

  [[nodiscard]] const PcapReaderParameter& getPcapReaderParameter();

 protected:
  void open_(size_t datasetIndex, size_t startFrameIndex) override;

  [[nodiscard]] bool isOpen_(size_t datasetIndex) override;

  [[nodiscard]] bool isEof_(size_t datasetIndex) override;

  void load_(size_t datasetIndex, size_t startFrameIndex, size_t groupOfFramesSize, GroupOfFrame& frames) override;

  void close_(size_t datasetIndex) override;

  [[nodiscard]] int parseDataPacket(size_t                   startFrameIndex,
                                    size_t&                  currentFrameIndex,
                                    pcap_t*                  pcap,
                                    uint16_t&                lastAzimuth100,
                                    std::vector<Frame::Ptr>& frameBuffer);
};

}  // namespace jpcc::io
