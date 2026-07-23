//
//  io.h
//  engine::core / io
//
//  Minimal filesystem reads. Dependency-free (std only) so it is available in every build
//  configuration, including the headless training build. Loaders that need third-party
//  decoders (images, meshes) live next to their decoder and build only in a full engine build.
//

#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace engine::core::io {

// Reads an entire file as raw bytes. Returns an empty vector if the file can't be opened.
std::vector<std::byte> readFile(std::string_view path);

// Reads an entire file as text. Returns an empty string if the file can't be opened.
std::string readTextFile(std::string_view path);

// True if the path names an existing, openable regular file.
bool fileExists(std::string_view path);

} // namespace engine::core::io
