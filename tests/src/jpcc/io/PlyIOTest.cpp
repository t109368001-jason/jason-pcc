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
  EXPECT_EQ(result.at(0)->header.seq, groupOfFrame.at(0)->header.seq);
  EXPECT_EQ(result.at(0)->points.size(), groupOfFrame.at(0)->points.size());
  for (size_t i = 0; i < result.at(0)->points.size(); i++) {
    EXPECT_EQ(result.at(0)->points.at(i).x, groupOfFrame.at(0)->points.at(i).x);
    EXPECT_EQ(result.at(0)->points.at(i).y, groupOfFrame.at(0)->points.at(i).y);
    EXPECT_EQ(result.at(0)->points.at(i).z, groupOfFrame.at(0)->points.at(i).z);
  }
}

}  // namespace jpcc::io
