#include <iostream>
#include <fstream>
#include <unistd.h>


/**
 * https://gist.github.com/thirdwing/da4621eb163a886a03c5
 * @param vm_usage Virtual memory size in bytes
 * @param resident_set Resident Set Size: number of pages the process has in real memory (in bytes)
 */
inline void unixProcessMemUsage(double& vm_usage, double& resident_set)
{
    vm_usage     = 0.0;
    resident_set = 0.0;

    // the two fields we want
    unsigned long vsize;
    long rss;
    {
        std::string ignore;
        std::ifstream ifs("/proc/self/stat", std::ios_base::in);
        ifs >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
            >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore >> ignore
            >> ignore >> ignore >> vsize >> rss;
    }

    static long page_size_bytes = sysconf(_SC_PAGE_SIZE); // in case x86-64 is configured to use 2MB pages
    vm_usage = double(vsize);
    resident_set = double(rss) * double(page_size_bytes);
}
