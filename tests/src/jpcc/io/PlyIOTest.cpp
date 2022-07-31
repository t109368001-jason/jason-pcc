#include <gtest/gtest.h>

#include <filesystem>

#include <jpcc/io/PlyIO.h>

namespace jpcc::io {

using namespace std;

TEST(PlyIOTest, save_load) {
  //  given
  size_t                      frameCount       = 3;
  size_t                      startFrameNumber = 123;
  const char*                 filePath         = "PlyIOTest_save_load_%05d.ply";
  GroupOfFrame<pcl::PointXYZ> frames;
  size_t                      ii = 1;
  for (size_t i = 0; i < frameCount; i++) {
    frames.push_back(jpcc::make_shared<Frame<pcl::PointXYZ>>());
    frames.at(i)->header.seq = startFrameNumber + i;
    for (size_t j = 0; j < i + 1; j++, ii++) { frames.at(i)->emplace_back(ii * 1.1, ii * 2.2, ii * 3.3); }
  }

  // when
  savePly<pcl::PointXYZ>(frames, filePath);
  GroupOfFrame<pcl::PointXYZ> results;
  loadPly<pcl::PointXYZ>(results, filePath, startFrameNumber, startFrameNumber + frameCount);
  filesystem::remove(filePath);

  EXPECT_EQ(results.size(), frameCount);
  EXPECT_EQ(results.size(), frames.size());
  for (size_t i = 0; i < frameCount; i++) {
    const FramePtr<pcl::PointXYZ>& result = results.at(i);
    const FramePtr<pcl::PointXYZ>& frame  = frames.at(i);
    EXPECT_EQ(result->header.seq, frame->header.seq);
    EXPECT_EQ(result->size(), frame->size());
    for (size_t j = 0; j < frame->size(); j++) {
      EXPECT_EQ(result->at(j).x, frame->at(j).x);
      EXPECT_EQ(result->at(j).y, frame->at(j).y);
      EXPECT_EQ(result->at(j).z, frame->at(j).z);
    }
  }
}

}  // namespace jpcc::io
