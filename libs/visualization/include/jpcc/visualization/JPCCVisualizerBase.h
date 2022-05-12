#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <vector>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>

#include <jpcc/visualization/VisualizerParameter.h>

namespace jpcc::visualization {

class JPCCVisualizerBase : public pcl::visualization::PCLVisualizer {
 public:
  using KeyboardEvent = std::function<void(const pcl::visualization::KeyboardEvent& event)>;

 protected:
  VisualizerParameter                  param_;
  int                                  fontSize_;
  int                                  lineHeight_;
  std::string                          primaryId_;
  std::map<std::string, std::string>   textMap_;
  std::map<std::string, int>           textHeightMap;
  std::map<std::string, std::string>   fieldColorMap_;
  std::map<std::string, RGBColor>      rgbColorMap_;
  mutable std::recursive_mutex         mutex_;
  shared_ptr<int>                      lastWindowHeight_;
  std::map<std::string, KeyboardEvent> keyboardCallbacks_;
  std::vector<std::string>             parameterTexts_;

 public:
  JPCCVisualizerBase(const VisualizerParameter& param);

  void updateOrAddText(const std::string& text, int ypos, const std::string& id);

  virtual int updateText(int* windowSize);

  virtual void updateAll();

  virtual void nextFrame() = 0;

  void registerKeyboardEvent(const KeyboardEvent& callback, const std::string& id);

  virtual void handleKeyboardEvent(const pcl::visualization::KeyboardEvent& event);

  [[nodiscard]] RGBColor getTextColor(const std::string& id);

  void setPrimaryId(const std::string& primaryId);

  void setColor(const std::string& id, const std::string& field);

  void setColor(const std::string& id, double r, double g, double b);

  void addParameter(const Parameter& parameter);
};

}  // namespace jpcc::visualization
