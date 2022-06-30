#pragma once

#include <string>

namespace jpcc::io {

[[nodiscard]] void* pcapOpen(const std::string& pcapPath);

[[nodiscard]] int pcapNextEx(void* pcap, const unsigned char** data, int64_t* timestampUS);

void pcapClose(void* pcap);

}  // namespace jpcc::io