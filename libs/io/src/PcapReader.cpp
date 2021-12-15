#include <jpcc/io/PcapReader.h>

#include <iomanip>
#include <sstream>

namespace jpcc {
namespace io {

using namespace std;
using namespace jpcc::common;

constexpr float PI_DIV180   = M_PI / 180.0;
constexpr float PI_DIV18000 = M_PI / 18000.0;
constexpr float TOTAL_POINTS_MULTIPLE_MAX_AZIMUTH_DIFF_OF_PACKET =
    36000.0 * (LASER_PER_FIRING + 1) * (FIRING_PER_PKT - 1);
constexpr int   VLP16_MAX_NUM_LASERS    = 16;
constexpr float VLP16_VERTICAL_DEGREE[] = {-15.0, 1.0, -13.0, 3.0,  -11.0, 5.0,  -9.0, 7.0,
                                           -7.0,  9.0, -5.0,  11.0, -3.0,  13.0, -1.0, 15.0};
constexpr float VLP16_VERTICAL_RADIAN[] = {
    -0.2617993878,  0.01745329252, -0.2268928028,  0.05235987756,  //
    -0.1919862177,  0.0872664626,  -0.1570796327,  0.1221730476,   //
    -0.1221730476,  0.1570796327,  -0.0872664626,  0.1919862177,   //
    -0.05235987756, 0.2268928028,  -0.01745329252, 0.2617993878,   //
};
constexpr float VLP16_VERTICAL_SIN[] = {
    0.9659258263, 0.9998476952, 0.9743700648, 0.9986295348, 0.9816271834, 0.9961946981, 0.9876883406, 0.9925461516,
    0.9925461516, 0.9876883406, 0.9961946981, 0.9816271834, 0.9986295348, 0.9743700648, 0.9998476952, 0.9659258263,
};
constexpr float VLP16_VERTICAL_COS[] = {
    -0.2588190451,  0.01745240644, -0.2249510543,  0.05233595624, -0.1908089954,  0.08715574275,
    -0.156434465,   0.1218693434,  -0.1218693434,  0.156434465,   -0.08715574275, 0.1908089954,
    -0.05233595624, 0.2249510543,  -0.01745240644, 0.2588190451,
};

constexpr int   HDL32_MAX_NUM_LASERS    = 32;
constexpr float HDL32_VERTICAL_DEGREE[] = {
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

PcapReader::PcapReader(const DatasetParameter& datasetParam, const PcapReaderParameter& param) :
    DatasetReader(datasetParam), param_(param) {
  assert(datasetParam_.type == "pcap");
  if (datasetParam_.sensor == "vlp16") {
    maxNumLasers_ = VLP16_MAX_NUM_LASERS;
    verticals_.resize(maxNumLasers_);
    sinVerticals_.resize(maxNumLasers_);
    cosVerticals_.resize(maxNumLasers_);
    for (size_t i = 0; i < maxNumLasers_; i++) {
      if (param_.useRadian) {
        verticals_.at(i) = VLP16_VERTICAL_RADIAN[i];
      } else {
        verticals_.at(i) = VLP16_VERTICAL_DEGREE[i];
      }
      sinVerticals_.at(i) = VLP16_VERTICAL_SIN[i];
      cosVerticals_.at(i) = VLP16_VERTICAL_COS[i];
    }
  } else if (datasetParam_.sensor == "hdl32") {
    maxNumLasers_ = HDL32_MAX_NUM_LASERS;
    verticals_.resize(maxNumLasers_);
    sinVerticals_.resize(maxNumLasers_);
    cosVerticals_.resize(maxNumLasers_);
    for (size_t i = 0; i < maxNumLasers_; i++) {
      if (param_.useRadian) {
        verticals_.at(i) = HDL32_VERTICAL_RADIAN[i];
      } else {
        verticals_.at(i) = HDL32_VERTICAL_DEGREE[i];
      }
      sinVerticals_.at(i) = HDL32_VERTICAL_SIN[i];
      cosVerticals_.at(i) = HDL32_VERTICAL_COS[i];
    }
  } else {
    throw logic_error("Not support dataset.sensor " + datasetParam_.sensor);
  }
  currentFrameIndices.resize(datasetParam_.totalFiles);
  pcaps_.resize(datasetParam_.totalFiles);
  lastAzimuth100s_.resize(datasetParam_.totalFiles);
  frameBuffers_.resize(datasetParam_.totalFiles);

  for (size_t i = 0; i < datasetParam_.totalFiles; i++) { open(i); }
}

PcapReader::~PcapReader() {
  for (size_t i = 0; i < datasetParam_.totalFiles; i++) { close(i); }
}

void PcapReader::load(const size_t  datasetIndex,
                      GroupOfFrame& frames,
                      const size_t  startFrameIndex,
                      const size_t  groupOfFramesSize,
                      const bool    parallel) {
  open(datasetIndex, startFrameIndex);
  load_(datasetIndex, frames, startFrameIndex, groupOfFramesSize, parallel);
}

const PcapReaderParameter& PcapReader::getPcapReaderParameter() { return param_; }

void PcapReader::open(const size_t datasetIndex, const size_t startFrameIndex) {
  if (pcaps_.at(datasetIndex) && currentFrameIndices.at(datasetIndex) <= startFrameIndex) { return; }
  if (pcaps_.at(datasetIndex)) { close(datasetIndex); }
  string pcapPath = datasetParam_.getFilePath(datasetIndex);

  char    error[PCAP_ERRBUF_SIZE];
  pcap_t* pcap = pcap_open_offline(pcapPath.c_str(), error);
  if (!pcap) { throw std::runtime_error(error); }

  struct bpf_program filter;
  std::ostringstream oss;
  if (pcap_compile(pcap, &filter, oss.str().c_str(), 0, 0xffffffff) == -1) {
    throw std::runtime_error(pcap_geterr(pcap));
  }

  if (pcap_setfilter(pcap, &filter) == -1) { throw std::runtime_error(pcap_geterr(pcap)); }
  pcaps_.at(datasetIndex) = pcap;
}

void PcapReader::close(const size_t datasetIndex) {
  currentFrameIndices.at(datasetIndex) = 0;
  pcap_t* pcap                         = pcaps_.at(datasetIndex);
  if (pcap) {
    pcap_close(pcap);
    pcap = nullptr;
  }
  lastAzimuth100s_.at(datasetIndex) = 0;
  std::queue<Frame::Ptr>().swap(frameBuffers_.at(datasetIndex));
}

void PcapReader::load_(const size_t  datasetIndex,
                       GroupOfFrame& frames,
                       const size_t  startFrameIndex,
                       const size_t  groupOfFramesSize,
                       const bool    parallel) {
  assert(groupOfFramesSize > 0);
  size_t            endFrameIndex     = startFrameIndex + groupOfFramesSize;
  size_t&           currentFrameIndex = currentFrameIndices.at(datasetIndex);
  pcap_t*           pcap              = pcaps_.at(datasetIndex);
  uint16_t&         lastAzimuth       = lastAzimuth100s_.at(datasetIndex);
  queue<Frame::Ptr> frameBuffer       = frameBuffers_.at(datasetIndex);

  frames.resize(groupOfFramesSize);
  if (frameBuffer.size() == 0) {
    Frame::Ptr frame(new Frame());
    frame->addPointTypes(param_.pointTypes);
    frame->reserve(29200);  // VLP 16 10 Hz (600 rpm)
    frameBuffer.push(frame);
  }
  while (currentFrameIndex < endFrameIndex) {
    if (frameBuffer.size() > 1) {
      if (currentFrameIndex < startFrameIndex) {
        frameBuffer.pop();
        currentFrameIndex++;
        continue;
      }
      frames.at(currentFrameIndex - startFrameIndex) = std::move(frameBuffer.front());
      cout << datasetParam_.getFilePath(datasetIndex) << ":" << currentFrameIndex << " "
           << *frames.at(currentFrameIndex - startFrameIndex) << endl;
      frameBuffer.pop();
      currentFrameIndex++;
    }
    // Retrieve Header and Data from PCAP
    struct pcap_pkthdr*  header;
    const unsigned char* data;
    const int            ret = pcap_next_ex(pcap, &header, &data);
    if (ret <= 0) { break; }

    // Check Packet Data Size
    // Data Blocks ( 100 bytes * 12 blocks ) + Time Stamp ( 4 bytes ) + Factory ( 2 bytes )
    if ((header->len - 42) != 1206) { continue; }

    uint64_t headerTime;
    if (!datasetParam_.haveGpsTime) {
      // Retrieve Unix Time ( microseconds )
      headerTime = static_cast<uint64_t>(header->ts.tv_sec) * 1000000 + header->ts.tv_usec;
    }
    // Convert to DataPacket Structure ( Cut Header 42 bytes )
    // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
    const DataPacket* packet = reinterpret_cast<const DataPacket*>(data + 42);
    parseDataPacket(startFrameIndex, currentFrameIndex, packet, lastAzimuth, frameBuffer, headerTime);
  }
  if (currentFrameIndex >= startFrameIndex) {
    frames.resize(currentFrameIndex - startFrameIndex);
  } else {
    frames.resize(0);
  }
}

void PcapReader::parseDataPacket(const size_t            startFrameIndex,
                                 size_t&                 currentFrameIndex,
                                 const DataPacket*       packet,
                                 uint16_t&               lastAzimuth100,
                                 std::queue<Frame::Ptr>& frameBuffer,
                                 const uint64_t          headerTime) {
  if (packet->sensorType != 0x21 && packet->sensorType != 0x22) {
    throw(std::runtime_error("This sensor is not supported"));
  }
  if (packet->mode != 0x37 && packet->mode != 0x38) {
    throw(std::runtime_error("Sensor can't be set in dual return mode"));
  }

  // Processing Packet
  for (int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++) {
    // Retrieve Firing Data
    const FiringData firing_data = packet->firingData[firing_index];
    // Retrieve Rotation Azimuth
    uint16_t azimuth100 = firing_data.rotationalPosition;
    // Complete Retrieve Capture One Rotation Data
    if (lastAzimuth100 > azimuth100) {
      frameBuffer.back()->shrink_to_fit();
      // Push One Rotation Data to Queue
      Frame::Ptr frame(new Frame());
      frame->addPointTypes(param_.pointTypes);
      float size = static_cast<float>(packet->firingData[11].rotationalPosition) -
                   static_cast<float>(packet->firingData[0].rotationalPosition);
      if (size < 0) { size += 36000.0; }
      size = TOTAL_POINTS_MULTIPLE_MAX_AZIMUTH_DIFF_OF_PACKET / size;
      frame->reserve((size_t)size);
      frameBuffer.push(frame);
    }
    if (currentFrameIndex + frameBuffer.size() < startFrameIndex) {
      // Update Last Rotation Azimuth
      lastAzimuth100 = azimuth100;
      continue;
    }
    for (int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++) {
      if (firing_data.laserReturns[laser_index].distance < param_.epsilon) { continue; }
      float azimuth;
      float phi = azimuth100 * PI_DIV18000;
      if (param_.useRadian) {
        azimuth = phi;
      } else {
        azimuth = azimuth100 / 100.0f;
      }
      float    vertical  = verticals_.at(laser_index % maxNumLasers_);
      float    distance  = static_cast<float>(firing_data.laserReturns[laser_index].distance * 2.0f);
      uint8_t  intensity = firing_data.laserReturns[laser_index].intensity;
      uint8_t  id        = static_cast<uint8_t>(laser_index % maxNumLasers_);
      uint64_t time;
      if (datasetParam_.haveGpsTime) {
        time = packet->gpsTimestamp;
      } else {
        time = headerTime;
      }
      float rSinV = distance * sinVerticals_.at(id);
      float x     = static_cast<float>(rSinV * cos(phi));
      float y     = static_cast<float>(rSinV * sin(phi));
      float z     = static_cast<float>(distance * cosVerticals_.at(id));
      frameBuffer.back()->add(azimuth, vertical, distance, intensity, id, time, x, y, z);
    }
    // Update Last Rotation Azimuth
    lastAzimuth100 = azimuth100;
  }
}

}  // namespace io
}  // namespace jpcc