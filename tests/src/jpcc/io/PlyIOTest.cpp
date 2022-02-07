#include <gtest/gtest.h>

#include <jpcc/io/PlyIO.h>

namespace jpcc::io {

TEST(PlyIOTest, save_load) {
  //  given
  size_t       frameNumber = 123;
  const char*  filePath    = "PlyIOTest_save_load_%05d.ply";
  GroupOfFrame groupOfFrame;
  groupOfFrame.resize(1);
  Frame::Ptr& frame = groupOfFrame.at(0);
  frame.reset(new Frame());
  frame->emplace_back(0.0, 0.0, 0.0);
  frame->emplace_back(1.0, 1.0, 1.0);
  frame->header.seq   = frameNumber;
  frame->header.stamp = 111;

  // when
  savePly(groupOfFrame, filePath);
  GroupOfFrame result;
  loadPly(result, filePath, frameNumber, frameNumber + 1);

  EXPECT_EQ(result.size(), groupOfFrame.size());
  Frame::Ptr& resultFrame = result.at(0);
  EXPECT_EQ(resultFrame->header.seq, frame->header.seq);
  EXPECT_EQ(resultFrame->size(), frame->size());
  for (size_t i = 0; i < resultFrame->size(); i++) {
    EXPECT_EQ(resultFrame->at(i).x, frame->at(i).x);
    EXPECT_EQ(resultFrame->at(i).y, frame->at(i).y);
    EXPECT_EQ(resultFrame->at(i).z, frame->at(i).z);
  }
}

}  // namespace jpcc::io
