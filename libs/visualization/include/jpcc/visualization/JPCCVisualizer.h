#pragma once

#include <map>
#include <queue>
#include <string>

#include <jpcc/common/Common.h>
#include <jpcc/visualization/JPCCVisualizerBase.h>

namespace jpcc::visualization {

template <typename PointT>
class JPCCVisualizer : public JPCCVisualizerBase {
 public:
  using Ptr                = shared_ptr<JPCCVisualizer>;
  using PclFrameQueue      = std::queue<PclFramePtr<PointT>>;
  using PointCloudColor    = pcl::visualization::PointCloudColorHandler<PointT>;
  using PointCloudColorPtr = typename PointCloudColor::Ptr;

 protected:
  std::map<std::string, PclFramePtr<PointT>> frameMap_;
  std::map<std::string, PclFrameQueue>       queueMap_;

 public:
  JPCCVisualizer(const VisualizerParameter& param);

  void updateOrAddCloud(const PclFramePtr<PointT>& cloud, const PointCloudColor& color, const std::string& id);

  int updateText(int* windowSize) override;

  void updateCloud();

  void updateQueue();

  void updateAll() override;

  void nextFrame() override;

  void handleKeyboardEvent(const pcl::visualization::KeyboardEvent& event) override;

  void enqueue(const GroupOfPclFrameMap<PointT>& framesMap);

  void saveScreenshot();

  [[nodiscard]] PointCloudColorPtr getCloudColor(const std::string& id, const PclFramePtr<PointT>& cloud);

  [[nodiscard]] bool isFull();

  [[nodiscard]] bool isEmpty();
};

}  // namespace jpcc::visualization

#include <jpcc/visualization/impl/JPCCVisualizer.hpp>
