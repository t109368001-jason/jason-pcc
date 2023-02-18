#include <jpcc/common/JPCCMemory.h>

#include <cstdlib>
#include <cstdio>
#include <cstring>

namespace jpcc {

// ********************************************************************* //
// ******************** Note: this value is in KB! ********************* //
// ********************************************************************* //

#if defined(WIN32)
#include <Windows.h>
#include <Psapi.h>
int getUsedMemory() {
  PROCESS_MEMORY_COUNTERS pmc;
  GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));
  return pmc.WorkingSetSize / 1024;
}
uint64_t getPeakMemory() {
  PROCESS_MEMORY_COUNTERS pmc;
  GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc));
  return (uint64_t)pmc.PeakWorkingSetSize / 1024;
}
#elif defined(__APPLE__) && defined(__MACH__)
int getUsedMemory() {
  struct mach_task_basic_info info;
  mach_msg_type_number_t      infoCount = MACH_TASK_BASIC_INFO_COUNT;
  if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info, &infoCount) != KERN_SUCCESS) { return 0; }
  return static_cast<size_t>(info.resident_size);
}
uint64_t getPeakMemory() {
  struct rusage rusage;
  getrusage(RUSAGE_SELF, &rusage);
  return static_cast<size_t>(rusage.ru_maxrss / 1024);
}
#else
int parseLine(char* pLine) {
  int         iLen = static_cast<int>(strlen(pLine));
  const char* pTmp = pLine;
  while (*pTmp < '0' || *pTmp > '9') { pTmp++; }
  pLine[iLen - 3] = '\0';
  iLen            = atoi(pTmp);
  return iLen;
}
int getUsedMemory() {
  FILE* pFile   = fopen("/proc/self/status", "r");
  int   iResult = 0;
  if (pFile != NULL) {
    char pLine[128];
    while (fgets(pLine, 128, pFile) != NULL) {
      if (strncmp(pLine, "VmSize:", 7) == 0) {
        iResult          = static_cast<int>(strlen(pLine));
        const char* pTmp = pLine;
        while (*pTmp < '0' || *pTmp > '9') { pTmp++; }
        pLine[iResult - 3] = '\0';
        iResult            = atoi(pTmp);
        break;
      }
    }
    fclose(pFile);
  }
  return iResult;
}
uint64_t getPeakMemory() {
  FILE*    pFile   = fopen("/proc/self/status", "r");
  uint64_t iResult = 0;
  if (pFile != NULL) {
    char pLine[128];
    while (fgets(pLine, 128, pFile) != NULL) {
      if (strncmp(pLine, "VmPeak:", 7) == 0) {
        const char* pTmp = pLine;
        while (*pTmp < '0' || *pTmp > '9') { pTmp++; }
        pLine[static_cast<int>(strlen(pLine)) - 3] = '\0';
        iResult                                    = atoi(pTmp);
        break;
      }
    }
    fclose(pFile);
  }
  return iResult;
}
#endif

}  // namespace jpcc
