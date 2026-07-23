//
//  io.cpp
//  engine::core / io
//

#include "engine/core/io/io.h"

#include <fstream>
#include <ios>

namespace engine::core::io {

std::vector<std::byte> readFile(std::string_view path) {
    std::ifstream f(std::string(path), std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto size = static_cast<std::streamsize>(f.tellg());
    if (size <= 0) return {};
    f.seekg(0);
    std::vector<std::byte> data(static_cast<std::size_t>(size));
    f.read(reinterpret_cast<char*>(data.data()), size);
    if (!f) return {};
    return data;
}

std::string readTextFile(std::string_view path) {
    std::ifstream f(std::string(path), std::ios::binary | std::ios::ate);
    if (!f) return {};
    const auto size = static_cast<std::streamsize>(f.tellg());
    if (size < 0) return {};
    f.seekg(0);
    std::string data(static_cast<std::size_t>(size), '\0');
    f.read(data.data(), size);
    if (!f) return {};
    return data;
}

bool fileExists(std::string_view path) {
    std::ifstream f(std::string(path), std::ios::binary);
    return static_cast<bool>(f);
}

} // namespace engine::core::io
