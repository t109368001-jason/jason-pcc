namespace jpcc::metric {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::addPoints(const std::string& name, const FramePtr<PointT>& frame) {
  const auto& points = frame->size();
  frameNumberSet_.insert(frame->header.seq);
  pointsNameSet_.insert(name);
  pointsMapMap_[frame->header.seq][name] += points;
  addBytes(name, frame->header.seq, points * sizeof(float) * 3);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCMetric::addPoints(const std::string& name, const GroupOfFrame<PointT>& frames) {
  for (const auto& frame : frames) { addPoints<PointT>(name, frame); }
}

}  // namespace jpcc::metric