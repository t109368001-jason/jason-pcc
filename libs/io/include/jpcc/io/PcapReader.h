#ifndef JPCC_IO_PCAP_READER_H_
#define JPCC_IO_PCAP_READER_H_

#include <vector>

#include <pcap/pcap.h>

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/io/PcapReaderParameter.h>

namespace jpcc {
namespace io {

using Frame        = common::Frame;
using GroupOfFrame = common::GroupOfFrame;

constexpr auto LASER_PER_FIRING = 32;
constexpr auto FIRING_PER_PKT   = 12;

#pragma pack(push, 1)
typedef struct LaserReturn {
  uint16_t distance;
  uint8_t  intensity;
} LaserReturn;
#pragma pack(pop)

#pragma pack(push, 1)
struct FiringData {
  uint16_t    blockIdentifier;
  uint16_t    rotationalPosition;
  LaserReturn laserReturns[LASER_PER_FIRING];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct DataPacket {
  FiringData firingData[FIRING_PER_PKT];
  uint32_t   gpsTimestamp;
  uint8_t    mode;
  uint8_t    sensorType;
};
#pragma pack(pop)

class PcapReader : public DatasetReader {
 protected:
  const PcapReaderParameter& param_;
  int                        maxNumLasers_;
  std::vector<float>         verticals_;
  std::vector<float>         sinVerticals_;
  std::vector<float>         cosVerticals_;

  std::vector<size_t>                  currentFrameIndices;
  std::vector<pcap_t*>                 pcaps_;
  std::vector<uint16_t>                lastAzimuth100s_;
  std::vector<std::vector<Frame::Ptr>> frameBuffers_;

 public:
  PcapReader(const DatasetParameter& datasetParam, const PcapReaderParameter& param);

  ~PcapReader();

  void load(const size_t  datasetIndex,
            GroupOfFrame& frames,
            const size_t  startFrameIndex,
            const size_t  groupOfFramesSize,
            const bool    parallel = false) override;

  const PcapReaderParameter& getPcapReaderParameter();

 protected:
  void open(const size_t datasetIndex, const size_t startFrameIndex = 0);

  void close(const size_t datasetIndex);

  void load_(const size_t  datasetIndex,
             GroupOfFrame& frames,
             const size_t  startFrameIndex,
             const size_t  groupOfFramesSize,
             const bool    parallel = false);

  int parseDataPacket(const size_t             startFrameIndex,
                      size_t&                  currentFrameIndex,
                      pcap_t*                  pcap,
                      uint16_t&                lastAzimuth100,
                      std::vector<Frame::Ptr>& frameBuffer);
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_PCAP_READER_H_