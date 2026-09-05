#pragma once
#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cstring>
#include <algorithm>
#define FILE_READ "r"
#define FILE_WRITE "w"
inline long writeBudget = -1;
inline bool failOpenWrite = false;
struct File {
    std::shared_ptr<std::vector<uint8_t>> bytes;
    size_t pos = 0;
    operator bool() const { return bool(bytes); }
    size_t read(uint8_t* out, size_t n) {
        if(!bytes) return 0;
        n = std::min(n, bytes->size() - pos);
        memcpy(out, bytes->data() + pos, n); pos += n; return n;
    }
    size_t write(const uint8_t* in, size_t n) {
        if(!bytes) return 0;
        if(writeBudget >= 0) { n = std::min(n, size_t(writeBudget)); writeBudget -= n; }
        bytes->insert(bytes->end(), in, in+n); return n;
    }
    void flush() {}
    void close() {}
};
struct FakeFS {
    std::map<std::string, std::shared_ptr<std::vector<uint8_t>>> files;
    File open(const char* path, const char* mode) {
        if(*mode == 'w') {
            if(failOpenWrite) return {};
            files[path] = std::make_shared<std::vector<uint8_t>>();
        }
        auto it = files.find(path);
        return it == files.end() ? File{} : File{it->second};
    }
};
