namespace jpcc::segmentation {

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename GMMContainerT, typename StaticPointContainerT>
OctreeContainerSegmentation<GMMContainerT, StaticPointContainerT>::OctreeContainerSegmentation() :
    GMMContainerT(), StaticPointContainerT() {
  static_assert(std::is_base_of_v<IOctreeContainerGMM, GMMContainerT>, "invalid template type");
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename GMMContainerT, typename StaticPointContainerT>
void OctreeContainerSegmentation<GMMContainerT, StaticPointContainerT>::reset() {
  GMMContainerT::reset();
  StaticPointContainerT::reset();
}

}  // namespace jpcc::segmentation