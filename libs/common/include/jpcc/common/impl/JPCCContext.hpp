namespace jpcc {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
JPCCContext<PointT>::JPCCContext(double                 resolution,
                                 SegmentationType       segmentationType,
                                 SegmentationOutputType segmentationOutputType,
                                 CoderBackendType       dynamicBackendType,
                                 CoderBackendType       staticBackendType) {
  header_.resolution             = resolution;
  header_.segmentationType       = segmentationType;
  header_.segmentationOutputType = segmentationOutputType;
  header_.dynamicBackendType     = dynamicBackendType;
  header_.staticBackendType      = staticBackendType;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void JPCCContext<PointT>::clear() {
  pclFrames_.clear();
  dynamicPclFrames_.clear();
  staticPclFrames_.clear();
  staticAddedPclFrames_.clear();
  staticRemovedPclFrames_.clear();
  dynamicFrames_.clear();
  staticFrames_.clear();
  staticAddedFrames_.clear();
  staticRemovedFrames_.clear();
  dynamicEncodedBytesVector_.clear();
  staticEncodedBytesVector_.clear();
  staticAddedEncodedBytesVector_.clear();
  staticRemovedEncodedBytesVector_.clear();
  dynamicReconstructFrames_.clear();
  staticReconstructFrames_.clear();
  staticAddedReconstructFrames_.clear();
  staticRemovedReconstructFrames_.clear();
  dynamicReconstructPclFrames_.clear();
  staticReconstructPclFrames_.clear();
  staticAddedReconstructPclFrames_.clear();
  staticRemovedReconstructPclFrames_.clear();
  reconstructPclFrames_.clear();
}

template <typename PointT>
void writeJPCCContext(const JPCCContext<PointT>& context, std::ostream& os) {
  for (int i = 0; i < context.getDynamicEncodedBytesVector().size(); i++) {
    os.write(context.getDynamicEncodedBytesVector()[i].data(),
             (std::streamsize)context.getDynamicEncodedBytesVector()[i].size());
    if (context.getHeader().segmentationOutputType == jpcc::SegmentationOutputType::DYNAMIC_STATIC) {
      os.write(context.getStaticEncodedBytesVector()[i].data(),
               (std::streamsize)context.getStaticEncodedBytesVector()[i].size());
    } else if (context.getHeader().segmentationOutputType ==
               jpcc::SegmentationOutputType::DYNAMIC_STATIC_ADDED_STATIC_REMOVED) {
      os.write(context.getStaticAddedEncodedBytesVector()[i].data(),
               (std::streamsize)context.getStaticAddedEncodedBytesVector()[i].size());
      os.write(context.getStaticRemovedEncodedBytesVector()[i].data(),
               (std::streamsize)context.getStaticRemovedEncodedBytesVector()[i].size());
    }
  }
}

}  // namespace jpcc