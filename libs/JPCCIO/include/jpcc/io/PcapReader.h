#ifndef JPCC_IO_PCAP_READER_H_
#define JPCC_IO_PCAP_READER_H_

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/io/PcapReaderParameter.h>

namespace jpcc {
namespace io {

using GroupOfFrame = common::GroupOfFrame;

class PcapReader : public DatasetReader {
 protected:
  const PcapReaderParameter&                      param_;
  std::vector<shared_ptr<velodyne::VLP16Capture>> captures;

 public:
  PcapReader(const DatasetParameter& datasetParam, const PcapReaderParameter& param);

  void load(const size_t  datasetIndex,
            GroupOfFrame& frames,
            const size_t  startFrameIndex,
            const size_t  groupOfFramesSize,
            const bool    parallel = false) override;

  const PcapReaderParameter& getPcapReaderParameter();

 protected:
  void open(const size_t datasetIndex, const size_t startFrameIndex);

  void reset(const size_t datasetIndex);
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_PCAP_READER_H_