#ifndef JPCC_IO_PCAP_READER_H_
#define JPCC_IO_PCAP_READER_H_

#include <mutex>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetParameter.h>
#include <jpcc/io/PcapReaderParameter.h>

namespace jpcc {
namespace io {

using GroupOfFrame = common::GroupOfFrame;

class PcapReader {
 protected:
  PcapReaderParameter param_;
  DatasetParameter    datasetParam_;

 public:
  void setPcapReaderParameter(const PcapReaderParameter& param);

  void setDatasetParameter(const DatasetParameter& datasetParam);

  void load(std::vector<GroupOfFrame>& sources, const size_t startFrameIndex, const size_t groupOfFramesSize, const bool parallel = false) const;

  GroupOfFrame load(const size_t datasetIndex, const size_t startFrameIndex, const size_t groupOfFramesSize, const bool parallel = false) const;
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_PCAP_READER_H_