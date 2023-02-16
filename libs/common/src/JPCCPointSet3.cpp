#include <jpcc/common/JPCCPointSet3.h>

#include <fstream>
#include <iomanip>

#include <boost/log/trivial.hpp>

namespace jpcc {

const uint32_t g_undefined_index = -1;

static bool        compareSeparators(char aChar, const char* const sep);
static inline bool getTokens(const char* str, const char* const sep, std::vector<std::string>& tokens);

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCPointSet3::addPoint(const PointType& point) {
  const size_t index = getPointCount();
  resize(index + 1);
  (*this)[index] = point;
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCPointSet3::addPositionNormal(const PointType& point, const NormalType& normal) {
  const size_t index = getPointCount();
  resize(index + 1);
  (*this)[index] = point;
  this->setNormal(index, normal);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void JPCCPointSet3::subset(JPCCPointSet3& frame, const Indices& indices) const {
  frame.addRemoveAttributes(*this);
  frame.resize(indices.size());
  Index indexFrame = 0;
  for (Index index : indices) {
    frame[indexFrame] = (*this)[index];
    if (hasReflectances()) { frame.getReflectance(indexFrame) = this->getReflectance(index); }
    if (hasNormals()) { frame.getNormal(indexFrame) = this->getNormal(index); }
    indexFrame++;
  }
}

bool JPCCPointSet3::write(const std::string& fileName, const bool asAscii) {
  std::ofstream fout(fileName, std::ofstream::out);
  if (!fout.is_open()) { return false; }
  const size_t pointCount = getPointCount();
  fout << "ply" << std::endl;

  if (asAscii) {
    fout << "format ascii 1.0" << std::endl;
  } else {
    JPCCEndianness endianess = JPCC_LITTLE_ENDIAN;
    if (endianess == JPCC_BIG_ENDIAN) {
      fout << "format binary_big_endian 1.0" << std::endl;
    } else {
      fout << "format binary_little_endian 1.0" << std::endl;
    }
  }
  fout << "element vertex " << pointCount << std::endl;
  if (asAscii) {
    fout << "property float x" << std::endl;
    fout << "property float y" << std::endl;
    fout << "property float z" << std::endl;
  } else {
    // fout << "property int16 x" << std::endl;
    // fout << "property int16 y" << std::endl;
    // fout << "property int16 z" << std::endl;
    fout << "property float x" << std::endl;
    fout << "property float y" << std::endl;
    fout << "property float z" << std::endl;
  }
  if (hasNormals()) {
    fout << "property float nx" << std::endl;
    fout << "property float ny" << std::endl;
    fout << "property float nz" << std::endl;
  }
  if (hasReflectances()) { fout << "property uint16 refc" << std::endl; }
  fout << "element face 0" << std::endl;
  fout << "property list uint8 int32 vertex_index" << std::endl;
  fout << "end_header" << std::endl;
  if (asAscii) {
    fout << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (size_t i = 0; i < pointCount; ++i) {
      const auto& position = (*this)[i];
      fout << position[0] << " " << position[1] << " " << position[2];
      if (hasNormals()) {
        const auto& normal = getNormal(i);
        fout << " " << static_cast<float>(normal[0]) << " " << static_cast<float>(normal[1]) << " "
             << static_cast<float>(normal[2]);
      }
      if (hasReflectances()) { fout << " " << static_cast<int>(getReflectance(i)); }
      fout << std::endl;
    }
  } else {
    fout.clear();
    fout.close();
    fout.open(fileName, std::ofstream::binary | std::ofstream::out | std::ofstream::app);
    for (size_t i = 0; i < pointCount; ++i) {
      const auto& position = (*this)[i];
      // fout.write( reinterpret_cast<const char* const>( &position ), sizeof( PCCType ) * 3 );
      float value[3];
      value[0] = position[0];
      value[1] = position[1];
      value[2] = position[2];
      fout.write(reinterpret_cast<const char*>(&value), sizeof(float) * 3);
      if (hasNormals()) {
        const auto& normal = getNormal(i);
        value[0]           = normal[0];
        value[1]           = normal[1];
        value[2]           = normal[2];
        fout.write(reinterpret_cast<const char*>(&value), sizeof(float) * 3);
      }
      if (hasReflectances()) {
        const uint16_t& reflectance = getReflectance(i);
        fout.write(reinterpret_cast<const char*>(&reflectance), sizeof(uint16_t));
      }
    }
  }
  fout.close();
  return true;
}
bool JPCCPointSet3::read(const std::string& fileName, const bool readNormals) {
  std::ifstream ifs(fileName, std::ifstream::in);
  if (!ifs.is_open()) { return false; }
  enum AttributeType {
    ATTRIBUTE_TYPE_FLOAT64 = 0,
    ATTRIBUTE_TYPE_FLOAT32 = 1,
    ATTRIBUTE_TYPE_UINT64  = 2,
    ATTRIBUTE_TYPE_UINT32  = 3,
    ATTRIBUTE_TYPE_UINT16  = 4,
    ATTRIBUTE_TYPE_UINT8   = 5,
    ATTRIBUTE_TYPE_INT64   = 6,
    ATTRIBUTE_TYPE_INT32   = 7,
    ATTRIBUTE_TYPE_INT16   = 8,
    ATTRIBUTE_TYPE_INT8    = 9,
  };
  struct AttributeInfo {
    std::string   name;
    AttributeType type;
    size_t        byteCount;
  };

  std::vector<AttributeInfo> attributesInfo;
  attributesInfo.reserve(16);
  const size_t             MAX_BUFFER_SIZE = 4096;
  char                     tmp[MAX_BUFFER_SIZE];
  const char*              sep = " \t\r";
  std::vector<std::string> tokens;

  ifs.getline(tmp, MAX_BUFFER_SIZE);
  getTokens(tmp, sep, tokens);
  if (tokens.empty() || tokens[0] != "ply") {
    BOOST_LOG_TRIVIAL(error) << "Error: corrupted file!";
    return false;
  }
  bool   isAscii          = false;
  double version          = 1.0;
  size_t pointCount       = 0;
  bool   isVertexProperty = true;
  while (true) {
    if (ifs.eof()) {
      BOOST_LOG_TRIVIAL(error) << "Error: corrupted header!";
      return false;
    }
    ifs.getline(tmp, MAX_BUFFER_SIZE);
    getTokens(tmp, sep, tokens);
    if (tokens.empty() || tokens[0] == "comment") { continue; }
    if (tokens[0] == "format") {
      if (tokens.size() != 3) {
        BOOST_LOG_TRIVIAL(error) << "Error: corrupted format info!";
        return false;
      }
      isAscii = tokens[1] == "ascii";
      version = atof(tokens[2].c_str());
    } else if (tokens[0] == "element") {
      if (tokens.size() != 3) {
        BOOST_LOG_TRIVIAL(error) << "Error: corrupted element info!";
        return false;
      }
      if (tokens[1] == "vertex") {
        pointCount = atoi(tokens[2].c_str());
      } else {
        isVertexProperty = false;
      }
    } else if (tokens[0] == "property" && isVertexProperty) {
      if (tokens.size() != 3) {
        BOOST_LOG_TRIVIAL(error) << "Error: corrupted property info!";
        return false;
      }
      const std::string& propertyType   = tokens[1];
      const std::string& propertyName   = tokens[2];
      const size_t       attributeIndex = attributesInfo.size();
      attributesInfo.resize(attributeIndex + 1);
      AttributeInfo& attributeInfo = attributesInfo[attributeIndex];
      attributeInfo.name           = propertyName;
      if (propertyType == "float64") {
        attributeInfo.type      = ATTRIBUTE_TYPE_FLOAT64;
        attributeInfo.byteCount = 8;
      } else if (propertyType == "float" || propertyType == "float32") {
        attributeInfo.type      = ATTRIBUTE_TYPE_FLOAT32;
        attributeInfo.byteCount = 4;
      } else if (propertyType == "uint64") {
        attributeInfo.type      = ATTRIBUTE_TYPE_UINT64;
        attributeInfo.byteCount = 8;
      } else if (propertyType == "uint32") {
        attributeInfo.type      = ATTRIBUTE_TYPE_UINT32;
        attributeInfo.byteCount = 4;
      } else if (propertyType == "uint16") {
        attributeInfo.type      = ATTRIBUTE_TYPE_UINT16;
        attributeInfo.byteCount = 2;
      } else if (propertyType == "uchar" || propertyType == "uint8") {
        attributeInfo.type      = ATTRIBUTE_TYPE_UINT8;
        attributeInfo.byteCount = 1;
      } else if (propertyType == "int64") {
        attributeInfo.type      = ATTRIBUTE_TYPE_INT64;
        attributeInfo.byteCount = 8;
      } else if (propertyType == "int32" || propertyType == "int") {
        attributeInfo.type      = ATTRIBUTE_TYPE_INT32;
        attributeInfo.byteCount = 4;
      } else if (propertyType == "int16") {
        attributeInfo.type      = ATTRIBUTE_TYPE_INT16;
        attributeInfo.byteCount = 2;
      } else if (propertyType == "char" || propertyType == "int8") {
        attributeInfo.type      = ATTRIBUTE_TYPE_INT8;
        attributeInfo.byteCount = 1;
      }
    } else if (tokens[0] == "end_header") {
      break;
    }
  }
  if (version != 1.0) {
    BOOST_LOG_TRIVIAL(error) << "Error: non-supported version!";
    return false;
  }

  size_t       indexX           = g_undefined_index;
  size_t       indexY           = g_undefined_index;
  size_t       indexZ           = g_undefined_index;
  size_t       indexR           = g_undefined_index;
  size_t       indexG           = g_undefined_index;
  size_t       indexB           = g_undefined_index;
  size_t       indexReflectance = g_undefined_index;
  size_t       indexNX          = g_undefined_index;
  size_t       indexNY          = g_undefined_index;
  size_t       indexNZ          = g_undefined_index;
  const size_t attributeCount   = attributesInfo.size();
  for (size_t a = 0; a < attributeCount; ++a) {
    const auto& attributeInfo = attributesInfo[a];
    if (attributeInfo.name == "x" &&
        (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4 || attributeInfo.byteCount == 2)) {
      indexX = a;
    } else if (attributeInfo.name == "y" &&
               (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4 || attributeInfo.byteCount == 2)) {
      indexY = a;
    } else if (attributeInfo.name == "z" &&
               (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4 || attributeInfo.byteCount == 2)) {
      indexZ = a;
    } else if (attributeInfo.name == "red" && attributeInfo.byteCount == 1) {
      indexR = a;
    } else if (attributeInfo.name == "green" && attributeInfo.byteCount == 1) {
      indexG = a;
    } else if (attributeInfo.name == "blue" && attributeInfo.byteCount == 1) {
      indexB = a;
    } else if (attributeInfo.name == "nx" && attributeInfo.byteCount == 4 && readNormals) {
      indexNX = a;
    } else if (attributeInfo.name == "ny" && attributeInfo.byteCount == 4 && readNormals) {
      indexNY = a;
    } else if (attributeInfo.name == "nz" && attributeInfo.byteCount == 4 && readNormals) {
      indexNZ = a;
    } else if ((attributeInfo.name == "reflectance" || attributeInfo.name == "refc") && attributeInfo.byteCount <= 2) {
      indexReflectance = a;
    }
  }
  if (indexX == g_undefined_index || indexY == g_undefined_index || indexZ == g_undefined_index) {
    BOOST_LOG_TRIVIAL(error) << "Error: missing coordinates!";
    return false;
  }
  withReflectances_ = indexReflectance != g_undefined_index;
  withNormals_      = indexNX != g_undefined_index && indexNY != g_undefined_index && indexNZ != g_undefined_index;
  resize(pointCount);
  if (isAscii) {
    size_t pointCounter = 0;
    while (!ifs.eof() && pointCounter < pointCount) {
      ifs.getline(tmp, MAX_BUFFER_SIZE);
      getTokens(tmp, sep, tokens);
      if (tokens.empty()) { continue; }
      if (tokens.size() < attributeCount) { return false; }
      auto& position = positions_[pointCounter];
      position[0]    = atof(tokens[indexX].c_str());
      position[1]    = atof(tokens[indexY].c_str());
      position[2]    = atof(tokens[indexZ].c_str());
      if (hasReflectances()) { reflectances_[pointCounter] = uint16_t(atoi(tokens[indexReflectance].c_str())); }
      ++pointCounter;
    }
  } else {
    ifs.close();
    ifs.open(fileName, std::ifstream::binary | std::ifstream::in);
    ifs.read(tmp, MAX_BUFFER_SIZE);
    char* str       = strstr(tmp, "end_header");
    str             = strstr(str, "\n");
    int headerCount = str - tmp + 1;
    ifs.close();
    ifs.open(fileName, std::ifstream::binary | std::ifstream::in);
    ifs.read(tmp, headerCount);
    for (size_t pointCounter = 0; pointCounter < pointCount && !ifs.eof(); ++pointCounter) {
      auto& position = positions_[pointCounter];
      for (size_t a = 0; a < attributeCount && !ifs.eof(); ++a) {
        const auto& attributeInfo = attributesInfo[a];
        if (a == indexX) {
          if (attributeInfo.byteCount == 2) {
            uint16_t x;
            ifs.read(reinterpret_cast<char*>(&x), sizeof(uint16_t));
            position[0] = x;
          } else if (attributeInfo.byteCount == 4) {
            float x;
            ifs.read(reinterpret_cast<char*>(&x), sizeof(float));
            position[0] = x;
          } else {
            double x;
            ifs.read(reinterpret_cast<char*>(&x), sizeof(double));
            position[0] = x;
          }
        } else if (a == indexY) {
          if (attributeInfo.byteCount == 2) {
            uint16_t y;
            ifs.read(reinterpret_cast<char*>(&y), sizeof(uint16_t));
            position[1] = y;
          } else if (attributeInfo.byteCount == 4) {
            float y;
            ifs.read(reinterpret_cast<char*>(&y), sizeof(float));
            position[1] = y;
          } else {
            double y;
            ifs.read(reinterpret_cast<char*>(&y), sizeof(double));
            position[1] = y;
          }
        } else if (a == indexZ) {
          if (attributeInfo.byteCount == 2) {
            uint16_t z;
            ifs.read(reinterpret_cast<char*>(&z), sizeof(uint16_t));
            position[2] = z;
          } else if (attributeInfo.byteCount == 4) {
            float z;
            ifs.read(reinterpret_cast<char*>(&z), sizeof(float));
            position[2] = z;
          } else {
            double z;
            ifs.read(reinterpret_cast<char*>(&z), sizeof(double));
            position[2] = z;
          }
        } else if (a == indexNX) {
          if (attributeInfo.byteCount == 4) {
            float nx;
            ifs.read(reinterpret_cast<char*>(&nx), sizeof(float));
            normals_[pointCounter][0] = nx;
          } else {
            double nx;
            ifs.read(reinterpret_cast<char*>(&nx), sizeof(double));
            normals_[pointCounter][0] = nx;
          }
        } else if (a == indexNY) {
          if (attributeInfo.byteCount == 4) {
            float ny;
            ifs.read(reinterpret_cast<char*>(&ny), sizeof(float));
            normals_[pointCounter][1] = ny;
          } else {
            double ny;
            ifs.read(reinterpret_cast<char*>(&ny), sizeof(double));
            normals_[pointCounter][1] = ny;
          }
        } else if (a == indexNZ) {
          if (attributeInfo.byteCount == 4) {
            float nz;
            ifs.read(reinterpret_cast<char*>(&nz), sizeof(float));
            normals_[pointCounter][2] = nz;
          } else {
            double nz;
            ifs.read(reinterpret_cast<char*>(&nz), sizeof(double));
            normals_[pointCounter][2] = nz;
          }
        } else if (a == indexReflectance && attributeInfo.byteCount <= 2) {
          if (indexReflectance == 1) {
            uint8_t reflectance;
            ifs.read(reinterpret_cast<char*>(&reflectance), sizeof(uint8_t));
            reflectances_[pointCounter] = reflectance;
          } else {
            auto& reflectance = reflectances_[pointCounter];
            ifs.read(reinterpret_cast<char*>(reflectance), sizeof(uint16_t));
          }
        } else {
          char buffer[128];
          ifs.read(buffer, attributeInfo.byteCount);
        }
      }
    }
  }
  return true;
}

static bool compareSeparators(char aChar, const char* const sep) {
  int i = 0;
  while (sep[i] != '\0') {
    if (aChar == sep[i]) return false;
    i++;
  }
  return true;
}

static inline bool getTokens(const char* str, const char* const sep, std::vector<std::string>& tokens) {
  if (!tokens.empty()) tokens.clear();
  std::string buf    = "";
  size_t      i      = 0;
  size_t      length = ::strlen(str);
  while (i < length) {
    if (compareSeparators(str[i], sep)) {
      buf += str[i];
    } else if (buf.length() > 0) {
      tokens.push_back(buf);
      buf = "";
    }
    i++;
  }
  if (!buf.empty()) tokens.push_back(buf);
  return !tokens.empty();
}

}  // namespace jpcc