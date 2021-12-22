#ifndef JPCC_IO_LVX_READER_H_
#define JPCC_IO_LVX_READER_H_

#include <vector>

#include <lvx_file.h>

#include <jpcc/common/Common.h>
#include <jpcc/common/GroupOfFrame.h>
#include <jpcc/io/DatasetReader.h>
#include <jpcc/io/LvxReaderParameter.h>

namespace jpcc {
namespace io {

using Frame        = common::Frame;
using GroupOfFrame = common::GroupOfFrame;

class LvxReader : public DatasetReader {
 protected:
  const LvxReaderParameter& param_;
  size_t                    capacity_;

  std::vector<size_t>                               currentFrameIndices_;
  std::vector<shared_ptr<livox_ros::LvxFileHandle>> lvxs_;
  std::vector<std::vector<Frame::Ptr>>              frameBuffers_;

 public:
  LvxReader(const DatasetParameter& datasetParam, const LvxReaderParameter& param);

  ~LvxReader();

  void load(const size_t  datasetIndex,
            GroupOfFrame& frames,
            const size_t  startFrameIndex,
            const size_t  groupOfFramesSize,
            const bool    parallel = false) override;

  const LvxReaderParameter& getLvxReaderParameter();

 protected:
  void open(const size_t datasetIndex, const size_t startFrameIndex = 0);

  void close(const size_t datasetIndex);

  void load_(const size_t  datasetIndex,
             GroupOfFrame& frames,
             const size_t  startFrameIndex,
             const size_t  groupOfFramesSize,
             const bool    parallel = false);
};

}  // namespace io
}  // namespace jpcc

#endif  // JPCC_IO_LVX_READER_H_