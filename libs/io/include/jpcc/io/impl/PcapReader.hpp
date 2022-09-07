#include <stdexcept>
#include <utility>

namespace jpcc::io {

constexpr auto LASER_PER_FIRING = 32;
constexpr auto FIRING_PER_PKT   = 12;

constexpr float PI_DIV18000 = M_PI / 18000.0;

//////////////////////////////////////////////////////////////////////////////////////////////
constexpr int VLP16_MAX_NUM_LASERS = 16;

[[maybe_unused]] constexpr float VLP16_VERTICAL_DEGREE[] = {-15.0, 1.0, -13.0, 3.0,  -11.0, 5.0,  -9.0, 7.0,  //
                                                            -7.0,  9.0, -5.0,  11.0, -3.0,  13.0, -1.0, 15.0};

constexpr float VLP16_VERTICAL_RADIAN[] = {
    -0.2617993878,  0.01745329252, -0.2268928028,  0.05235987756,  //
    -0.1919862177,  0.0872664626,  -0.1570796327,  0.1221730476,   //
    -0.1221730476,  0.1570796327,  -0.0872664626,  0.1919862177,   //
    -0.05235987756, 0.2268928028,  -0.01745329252, 0.2617993878,   //
};
constexpr float VLP16_VERTICAL_COS[] = {
    0.9659258263, 0.9998476952, 0.9743700648, 0.9986295348, 0.9816271834, 0.9961946981, 0.9876883406, 0.9925461516,
    0.9925461516, 0.9876883406, 0.9961946981, 0.9816271834, 0.9986295348, 0.9743700648, 0.9998476952, 0.9659258263,
};
constexpr float VLP16_VERTICAL_SIN[] = {
    -0.2588190451,  0.01745240644, -0.2249510543,  0.05233595624, -0.1908089954,  0.08715574275,
    -0.156434465,   0.1218693434,  -0.1218693434,  0.156434465,   -0.08715574275, 0.1908089954,
    -0.05233595624, 0.2249510543,  -0.01745240644, 0.2588190451,
};

//////////////////////////////////////////////////////////////////////////////////////////////
constexpr int HDL32_MAX_NUM_LASERS = 32;

[[maybe_unused]] constexpr float HDL32_VERTICAL_DEGREE[] = {
    -30.67,     -9.3299999, -29.33, -8.0,      -28,    -6.6700001, -26.67, -5.3299999, -25.33,    -4.0,   -24.0,
    -2.6700001, -22.67,     -1.33,  -21.33,    0.0,    -20.0,      1.33,   -18.67,     2.6700001, -17.33, 4.0,
    -16,        5.3299999,  -14.67, 6.6700001, -13.33, 8.0,        -12.0,  9.3299999,  -10.67,    10.67,
};

constexpr float HDL32_VERTICAL_RADIAN[] = {
    -0.5352924816, -0.1628392175,  -0.5119050696, -0.1396263402,
    -0.4886921906, -0.1164134629,  -0.4654793115, -0.09302604739,
    -0.4420918995, -0.06981317008, -0.4188790205, -0.04660029277,
    -0.3956661414, -0.02321287905, -0.3722787295, 0,
    -0.3490658504, 0.02321287905,  -0.3258529713, 0.04660029277,
    -0.3024655594, 0.06981317008,  -0.2792526803, 0.09302604739,
    -0.2560398013, 0.1164134629,   -0.2326523893, 0.1396263402,
    -0.2094395102, 0.1628392175,   -0.1862266312, 0.1862266312,
};
constexpr float HDL32_VERTICAL_SIN[] = {
    0.8601194734, 0.9867709659, 0.8718129128, 0.9902680687, 0.8829475929, 0.9932316018, 0.8936065287, 0.9956761967,
    0.9038586617, 0.9975640503, 0.9135454576, 0.9989144028, 0.9227400229, 0.9997305932, 0.931500902,  1,
    0.9396926208, 0.9997305932, 0.9473780205, 0.9989144028, 0.9546049635, 0.9975640503, 0.9612616959, 0.9956761967,
    0.9674004875, 0.9932316018, 0.9730582856, 0.9902680687, 0.9781476007, 0.9867709659, 0.9827098767, 0.9827098767,
};
constexpr float HDL32_VERTICAL_COS[] = {
    -0.5100926304, -0.1621205137,  -0.489838999,  -0.139173101,
    -0.4694715628, -0.1161506999,  -0.4488511689, -0.09289193326,
    -0.4278311813, -0.06975647374, -0.4067366431, -0.0465834285,
    -0.3854229496, -0.02321079445, -0.363739013,  0,
    -0.3420201433, 0.02321079445,  -0.3201169885, 0.0465834285,
    -0.2978747449, 0.06975647374,  -0.2756373558, 0.09289193326,
    -0.2532514496, 0.1161506999,   -0.2305592609, 0.139173101,
    -0.2079116908, 0.1621205137,   -0.1851520951, 0.1851520951,
};

//////////////////////////////////////////////////////////////////////////////////////////////
constexpr int HI_RES_MAX_NUM_LASERS = 16;

[[maybe_unused]] constexpr float HI_RES_VERTICAL_DEGREE[] = {-10.00, 0.67, -8.67, 2.00, -7.33, 3.33, -6,    4.67,  //
                                                             -4.67,  6.00, -3.33, 7.33, -2.00, 8.67, -0.67, 10.00};

constexpr float HI_RES_VERTICAL_RADIAN[] = {
    -0.1745329252,  0.01169370599, -0.1513200461,  0.03490658504,  //
    -0.1279326342,  0.05811946409, -0.1047197551,  0.08150687607,  //
    -0.08150687607, 0.1047197551,  -0.05811946409, 0.1279326342,   //
    -0.03490658504, 0.1513200461,  -0.01169370599, 0.1745329252    //
};
constexpr float HI_RES_VERTICAL_COS[] = {
    0.984807753,  0.9999316294, 0.9885729513, 0.999390827,  0.9918277758, 0.9983115393, 0.9945218954, 0.9966801531,
    0.9966801531, 0.9945218954, 0.9983115393, 0.9918277758, 0.999390827,  0.9885729513, 0.9999316294, 0.984807753,
};
constexpr float HI_RES_VERTICAL_SIN[] = {
    -0.1736481777,  0.01169343949, -0.1507432253,  0.0348994967,   //
    -0.1275839459,  0.0580867496,  -0.1045284633,  0.08141665931,  //
    -0.08141665931, 0.1045284633,  -0.0580867496,  0.1275839459,   //
    -0.0348994967,  0.1507432253,  -0.01169343949, 0.1736481777,   //
};

//////////////////////////////////////////////////////////////////////////////////////////////
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#endif  // defined(__GNUC__)

#pragma pack(push, 1)
typedef struct LaserReturn {
  uint16_t distance;
  uint8_t  intensity;
} LaserReturn;
#pragma pack(pop)

#pragma pack(push, 1)
struct FiringData {
  [[maybe_unused]] uint16_t blockIdentifier;
  uint16_t                  rotationalPosition;
  LaserReturn               laserReturns[LASER_PER_FIRING];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct DataPacket {
  FiringData                firingData[FIRING_PER_PKT];
  [[maybe_unused]] uint32_t gpsTimestamp;
  uint8_t                   mode;
  uint8_t                   sensorType;
};
#pragma pack(pop)

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif  // defined(__GNUC__)

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
PcapReader<PointT>::PcapReader(DatasetReaderParameter param, DatasetParameter datasetParam) :
    DatasetStreamReader<PointT>(std::move(param), std::move(datasetParam)) {
  THROW_IF_NOT(this->datasetParam_.type == Type::PCAP);
  if (this->datasetParam_.sensor == Sensor::VLP_16) {
    maxNumLasers_ = VLP16_MAX_NUM_LASERS;
    verticals_.resize(maxNumLasers_);
    sinVerticals_.resize(maxNumLasers_);
    cosVerticals_.resize(maxNumLasers_);
    for (size_t i = 0; i < maxNumLasers_; i++) {
      verticals_.at(i)    = VLP16_VERTICAL_RADIAN[i];
      sinVerticals_.at(i) = VLP16_VERTICAL_SIN[i];
      cosVerticals_.at(i) = VLP16_VERTICAL_COS[i];
    }
    this->capacity_ = (size_t)((double)(300000) / this->param_.frequency);
  } else if (this->datasetParam_.sensor == Sensor::HI_RES) {
    maxNumLasers_ = HI_RES_MAX_NUM_LASERS;
    verticals_.resize(maxNumLasers_);
    sinVerticals_.resize(maxNumLasers_);
    cosVerticals_.resize(maxNumLasers_);
    for (size_t i = 0; i < maxNumLasers_; i++) {
      verticals_.at(i)    = HI_RES_VERTICAL_RADIAN[i];
      sinVerticals_.at(i) = HI_RES_VERTICAL_SIN[i];
      cosVerticals_.at(i) = HI_RES_VERTICAL_COS[i];
    }
    this->capacity_ = (size_t)((double)(300000) / this->param_.frequency);
  } else if (this->datasetParam_.sensor == Sensor::HDL_32) {
    maxNumLasers_ = HDL32_MAX_NUM_LASERS;
    verticals_.resize(maxNumLasers_);
    sinVerticals_.resize(maxNumLasers_);
    cosVerticals_.resize(maxNumLasers_);
    for (size_t i = 0; i < maxNumLasers_; i++) {
      verticals_.at(i)    = HDL32_VERTICAL_RADIAN[i];
      sinVerticals_.at(i) = HDL32_VERTICAL_SIN[i];
      cosVerticals_.at(i) = HDL32_VERTICAL_COS[i];
    }
    this->capacity_ = (size_t)((double)(695000) / this->param_.frequency);
  } else {
    throw std::logic_error("sensor not support");
  }
  this->currentFrameNumbers_.resize(this->datasetParam_.count());
  for (size_t i = 0; i < this->datasetParam_.count(); i++) { pcaps_.emplace_back(nullptr, &pcapClose); }
  this->frameBuffers_.resize(this->datasetParam_.count());

  std::for_each(this->datasetIndices_.begin(), this->datasetIndices_.end(),
                [this](const size_t& datasetIndex) { open_(datasetIndex, 0); });
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void PcapReader<PointT>::open_(const size_t datasetIndex, const size_t startFrameNumber) {
  if (pcaps_.at(datasetIndex) && this->currentFrameNumbers_.at(datasetIndex) <= startFrameNumber) { return; }
  const std::string pcapPath = this->datasetParam_.getFilePath(datasetIndex);

  this->eof_.at(datasetIndex) = false;
  pcaps_.at(datasetIndex).reset(pcapOpen(pcapPath));
  this->currentFrameNumbers_.at(datasetIndex) = this->datasetParam_.getStartFrameNumbers(datasetIndex);
  this->frameBuffers_.at(datasetIndex).clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool PcapReader<PointT>::isOpen_(const size_t datasetIndex) const {
  return static_cast<bool>(pcaps_.at(datasetIndex));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool PcapReader<PointT>::isEof_(const size_t datasetIndex) const {
  return DatasetStreamReader<PointT>::isEof_(datasetIndex);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void PcapReader<PointT>::load_(const size_t datasetIndex,
                               const size_t startFrameNumber,
                               const size_t groupOfFramesSize) {
  assert(groupOfFramesSize > 0);
  void* const           pcap        = pcaps_.at(datasetIndex).get();
  GroupOfFrame<PointT>& frameBuffer = this->frameBuffers_.at(datasetIndex);

  int ret = parseDataPacket(pcap, frameBuffer);
  if (ret <= 0) { this->eof_.at(datasetIndex) = true; }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
int PcapReader<PointT>::parseDataPacket(void* const pcap, GroupOfFrame<PointT>& frameBuffer) {
  // Retrieve Header and Data from PCAP
  const unsigned char* data;
  int64_t              timestampUS;
  const int            ret       = pcapNextEx(pcap, &data, &timestampUS);
  const int64_t        timestamp = timestampUS / 1000;
  if (ret != 1) {
    if (ret <= 0) {
      for (const FramePtr<PointT>& frame : frameBuffer) {
        frame->width  = static_cast<uint32_t>(frame->size());
        frame->height = 1;
      }
    }
    return ret;
  }

  // Convert to DataPacket Structure ( Cut Header 42 bytes )
  // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16, 0x24 is Hi-Res
  const auto* const packet = reinterpret_cast<const DataPacket*>(data + 42);

  if (packet->sensorType != 0x21 && packet->sensorType != 0x22 && packet->sensorType != 0x24) {
    throw(std::runtime_error("This sensor is not supported"));
  }
  if (packet->mode != 0x37 && packet->mode != 0x38) {
    throw(std::runtime_error("Sensor can't be set in dual return mode"));
  }

  // Processing Packet
  for (auto firing_data : packet->firingData) {
    // Retrieve Firing Data
    // Retrieve Rotation Azimuth
    const uint16_t azimuth100 = firing_data.rotationalPosition;

    for (int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++) {
      const float distance = static_cast<float>(firing_data.laserReturns[laser_index].distance) * 2.0f;
      if (distance < 1) { continue; }
      const float azimuth = static_cast<float>(azimuth100) * PI_DIV18000;
      //      float   vertical  = verticals_.at(laser_index % maxNumLasers_);
      uint8_t     intensity = firing_data.laserReturns[laser_index].intensity;
      const auto  id        = static_cast<uint8_t>(laser_index % maxNumLasers_);
      const float rCosV     = distance * cosVerticals_.at(id);
      const auto  x         = static_cast<float>(rCosV * sin(azimuth));
      const auto  y         = static_cast<float>(rCosV * cos(azimuth));
      const auto  z         = static_cast<float>(distance * sinVerticals_.at(id));
      {
        if (frameBuffer.empty()) {
          // new frame
          const auto frame    = jpcc::make_shared<Frame<PointT>>();
          frame->header.stamp = timestamp;
          frame->reserve(this->capacity_);
          frameBuffer.push_back(frame);
        }
        int64_t index = (timestamp - (int64_t)frameBuffer.front()->header.stamp) / (int64_t)this->param_.interval;
        if (index < 0) {
          for (int i = -1; i >= index; i--) {
            // new frame
            const auto frame    = jpcc::make_shared<Frame<PointT>>();
            frame->header.stamp = frameBuffer.front()->header.stamp + (int64_t)(this->param_.interval * (float)i);
            frame->reserve(this->capacity_);
            frameBuffer.insert(frameBuffer.begin(), frame);
          }
          index = 0;
        } else if (index >= frameBuffer.size()) {
          for (size_t i = frameBuffer.size(); i <= index; i++) {
            // new frame
            const auto frame    = jpcc::make_shared<Frame<PointT>>();
            frame->header.stamp = frameBuffer.front()->header.stamp + (int64_t)(this->param_.interval * (float)i);
            frame->reserve(this->capacity_);
            frameBuffer.push_back(frame);
          }
        }
        // emplace_back points only, improve performance
        // frameBuffer.at(index)->emplace_back(x, y, z);
        PointT point(x, y, z);
        if constexpr (pcl::traits::has_intensity_v<PointT>) { point.intensity = intensity / 255.0; }
        frameBuffer.at(index)->points.push_back(point);
      }
    }
  }

  for (const FramePtr<PointT>& frame : frameBuffer) {
    if ((frame->header.stamp + (int64_t)this->param_.interval) > timestamp) { break; }
    frame->width  = static_cast<uint32_t>(frame->size());
    frame->height = 1;
  }
  return ret;
}

}  // namespace jpcc::io
