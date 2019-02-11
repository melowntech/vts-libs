/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "vts-libs/vts0.hpp"

#include "commands.hpp"

namespace fs = boost::filesystem;
namespace vs = vtslibs::storage;

int runCommand(int argc, char *argv[]
               , const fs::path &path, const std::string &command)
{
    if (command == "create") {
        return create(argc - 2, argv + 2, path);
    } else if (command == "add") {
        return add(argc - 2, argv + 2, path);
    } else if (command == "info") {
        return info(argc - 2, argv + 2, path);
    } else if (command == "tileset-info") {
        return tileSetInfo(argc - 2, argv + 2, path);
    } else if (command == "tileset-paste") {
        return tileSetPaste(argc - 2, argv + 2, path);
    } else if (command == "tileset-clone") {
        return tileSetClone(argc - 2, argv + 2, path);
    }

    std::cerr << argv[0] << ": unknown command <" << command << ">"
              << std::endl;
    return EXIT_FAILURE;
}

int main(int argc, char *argv[])
{
    if (argc < 3) {
        return fallback(argc, argv);
    }

    fs::path path(argv[1]);
    std::string command(argv[2]);

    // run command
    if (std::getenv("NO_CATCH")) {
        // let uncaught exception to fall through
        return runCommand(argc, argv, path, command);
    } else {
        try {
            // catch uncaught exception (regular)
            return runCommand(argc, argv, path, command);
        } catch (const vs::Error &e) {
            std::cerr << "Operation failed: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }
}
