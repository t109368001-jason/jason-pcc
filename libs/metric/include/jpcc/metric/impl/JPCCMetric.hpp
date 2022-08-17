namespace jpcc::metric {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::add(const std::string& name, const FramePtr<PointT>& frame) {
  const auto& points = frame->size();
  pointsMap_[name] += points;
  bytesMap_[name] += points * sizeof(float) * 3;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::add(const std::string& name, const GroupOfFrame<PointT>& frames) {
  for (const auto& frame : frames) { add<PointT>(name, frame); }
}

}  // namespace jpcc::metric