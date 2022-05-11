#pragma once

#include <string>

namespace jpcc::io {

void* pcapOpen(const std::string& pcapPath);

int pcapNextEx(void* pcap, const unsigned char** data);

void pcapClose(void* pcap);

}  // namespace jpcc::io