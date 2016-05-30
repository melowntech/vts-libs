#ifndef vts_storage_commands_hpp_included_
#define vts_storage_commands_hpp_included_

#include <string>

#include <boost/filesystem/path.hpp>

#include "utility/buildsys.hpp"

int fallback(int argc, char *argv[]);

int create(int argc, char *argv[], const boost::filesystem::path &root);

int info(int argc, char *argv[], const boost::filesystem::path &root);

int add(int argc, char *argv[], const boost::filesystem::path &root);

int tileSetInfo(int argc, char *argv[], const boost::filesystem::path &root);

int tileSetPaste(int argc, char *argv[], const boost::filesystem::path &root);

int tileSetClone(int argc, char *argv[], const boost::filesystem::path &root);

#endif // vts_storage_commands_hpp_included_
