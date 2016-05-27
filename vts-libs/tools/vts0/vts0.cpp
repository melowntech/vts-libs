#include <cstdlib>
#include <string>
#include <iostream>

#include "dbglog/dbglog.hpp"

#include "vts-libs/vts0.hpp"

#include "./commands.hpp"

namespace fs = boost::filesystem;
namespace vs = vadstena::storage;

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
