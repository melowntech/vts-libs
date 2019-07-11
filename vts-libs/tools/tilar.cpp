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
#include <boost/uuid/uuid_io.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/enum-io.hpp"
#include "utility/gccversion.hpp"
#include "utility/streams.hpp"
#include "utility/time.hpp"
#include "utility/buildsys.hpp"

#include "service/cmdline.hpp"

#include "vts-libs/storage/tilar.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace vs = vtslibs::storage;

UTILITY_GENERATE_ENUM(Command,
                      ((list))
                      ((create))
                      ((append))
                      ((remove))
                      ((extract))
                      )


struct FileIndex : vs::Tilar::FileIndex {
    typedef std::vector<FileIndex> list;
};

struct File : FileIndex {
    fs::path path;

    typedef std::vector<File> list;
};

typedef service::UnrecognizedParser UP;

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, FileIndex &index)
{
    if (!(is >> index.col
          >> utility::expect<CharT>(',')
          >> index.row))
    {
        return is;
    }

    auto comma(utility::match<CharT>(','));
    if (!(is >> comma)) { return is; }
    if (!comma.matched) {
        index.type = 0;
        return is;
    }
    return (is >> index.type);
}

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, File &file)
{
    return is >> static_cast<FileIndex&>(file)
              >> utility::expect<CharT>(':') >> file.path;
}

class Tilar : public service::Cmdline {
public:
    Tilar()
        : service::Cmdline("tilar", BUILD_TARGET_VERSION
                           , (service::DISABLE_EXCESSIVE_LOGGING
                              | service::ENABLE_UNRECOGNIZED_OPTIONS))
        , command_(Command::list)
        , createOptions_{ 5, 1 }
    {
    }

    ~Tilar() {}

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual UP::optional
    configure(const po::variables_map &vars
              , const std::vector<std::string> &unrecognized)
        UTILITY_OVERRIDE;

    virtual po::ext_parser extraParser() UTILITY_OVERRIDE;

    virtual std::vector<std::string> listHelps() const UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    template<typename Body> void createParser(po::options_description &cmdline
                                              , Command command
                                              , const std::string &help
                                              , Body body)
    {
        auto p(std::make_shared<UP>(help));
        body(*p);
        commandParsers_[command] = p;

        auto name(boost::lexical_cast<std::string>(command));
        cmdline.add_options()
            (name.c_str(), ("alias for --command=" + name).c_str())
            ;
    }

    UP::optional getParser(Command command)
        const;

    void optionsConfiguration(po::options_description &options);

    void offsetConfiguration(po::options_description &options);

    int list();

    int create();

    int append();

    int remove();

    int extract();

    fs::path file_;
    Command command_;

    vs::Tilar::Options createOptions_;
    File::list files_;
    FileIndex::list indices_;
    boost::optional<std::uint32_t> indexOffset_;

    std::map<Command, std::shared_ptr<UP> >
    commandParsers_;
};

void Tilar::configuration(po::options_description &cmdline
                          , po::options_description&
                          , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("file", po::value(&file_)->required()
         , "Path to tilar file to work with.")
        ("command", po::value(&command_)
         ->default_value(Command::list)->required()
         , "Command to run.")
        ;

    pd.add("file", 1);

    createParser(cmdline, Command::list
                 , "--command=list: lists content of file"
                 , [&](UP &p)
    {
        offsetConfiguration(p.options);
    });

    createParser(cmdline, Command::create
                 , "--command=create: creates new file"
                 , [&](UP &p)
    {
        optionsConfiguration(p.options);
        p.options.add_options()
            ("files", po::value(&files_)
             , "List of files to add to the archive (col,row[,type]:path).")
            ;
        p.positional.add("files", -1);
    });

    createParser(cmdline, Command::append
                 , "--command=append: appends data to existing file"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("files", po::value(&files_)
             , "List of files to add to the archive (col,row[,type]:path).")
            ;
        p.positional.add("files", -1);
    });

    createParser(cmdline, Command::remove
                 , "--command=remove: removes data from file"
                 , [&](UP &p)
    {
        p.options.add_options()
            ("files", po::value(&indices_)
             , "List of files to remove from the archive (col,row[,type]).")
            ;
        p.positional.add("files", -1);
    });

    createParser(cmdline, Command::extract
                 , "--command=extract: extract one or more files "
                 "from the archive"
                 , [&](UP &p)
    {
        offsetConfiguration(p.options);
        p.options.add_options()
            ("files", po::value(&files_)
             , "List of files to extract from the archive "
             "(col,row[,type]:path).")
            ;
        p.positional.add("files", -1);
    });
}

po::ext_parser Tilar::extraParser()
{
    return [&](const std::string &s) -> std::pair<std::string, std::string>
    {
        if ((s.size() < 3) || (s[0] != '-') || (s[1] != '-')) {
            return {};
        }

        // translate standalone --COMMAND_NAME into --command=COMMAND_NAME
        for (const auto &p : commandParsers_) {
            const auto &name(boost::lexical_cast<std::string>(p.first));
            if (!s.compare(2, std::string::npos, name)) {
                return { "command", name };
            }
        }
        return {};
    };
}

UP::optional
Tilar::configure(const po::variables_map &vars
                 , const std::vector<std::string>&)
{
    if (!vars.count("command")) { return {}; }
    return getParser(vars["command"].as<Command>());
}

UP::optional Tilar::getParser(Command command)
    const
{
    auto fcommandParsers(commandParsers_.find(command));
    if ((fcommandParsers != commandParsers_.end())
         && fcommandParsers->second)
    {
        return *fcommandParsers->second;
    }

    return {};
}

void Tilar::configure(const po::variables_map &vars)
{
    if (vars.count("offset")) {
        indexOffset_ = vars["offset"].as<std::uint32_t>();
    }
}

std::vector<std::string> Tilar::listHelps() const
{
    std::vector<std::string> out;

    for (const auto &p : commandParsers_) {
        out.push_back(boost::lexical_cast<std::string>(p.first));
    }

    return out;
}

bool Tilar::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        // program help
        out << ("Tile archive manipulator\n"
                );

        return true;
    }

    try {
        if (auto p = getParser(boost::lexical_cast<Command>(what))) {
            out << p->options;
        }
        return true;
    } catch (const boost::bad_lexical_cast&) {}

    return false;
}

void Tilar::optionsConfiguration(po::options_description &options)
{
    options.add_options()
        ("binaryOrder", po::value(&createOptions_.binaryOrder)
         ->default_value(createOptions_.binaryOrder)->required()
         , "Binary order, size of archive grid is pow(2, binaryOrder).")
        ("filesPerTile", po::value(&createOptions_.filesPerTile)
         ->default_value(createOptions_.filesPerTile)->required()
         , "Number of files per tile.")
        ("uuid", po::value(&createOptions_.uuid)
         ->default_value(createOptions_.uuid)->required()
         , "File's UUID.")
        ;
}

void Tilar::offsetConfiguration(po::options_description &options)
{
    options.add_options()
        ("offset", po::value<std::uint32_t>()
         , "Offset of file index to use (defaults to the index at "
         "the end of the file).")
        ;
}

int Tilar::run()
{
    try {
        switch (command_) {
        case Command::list: return list();
        case Command::create: return create();
        case Command::append: return append();
        case Command::remove: return remove();
        case Command::extract: return extract();
        }
    } catch (const std::exception &e) {
        std::cerr << "tilar: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cerr << "tilar: no operation requested" << std::endl;
    return EXIT_FAILURE;
}

void writeFiles(vs::Tilar &arch, const File::list &files)
{
    for (const auto &file : files) {
        utility::ifstreambuf in(file.path.string());
        auto out(arch.output(file));

        out->get() << in.rdbuf();
        in.close();
        out->close();
    }
}

void extractFiles(vs::Tilar &arch, const File::list &files)
{
    for (const auto &file : files) {
        auto in(arch.input(file));
        utility::ofstreambuf out(file.path.string());

        out << in->get().rdbuf();
        in->close();
        out.close();
    }
}

int Tilar::list()
{
    auto arch(indexOffset_
              ? vs::Tilar::open(file_, *indexOffset_)
              : vs::Tilar::open(file_, vs::Tilar::OpenMode::readOnly));

    auto options(arch.options());
    auto info(arch.info());
    std::cout << "File: " << file_.string()
              << "\nBinary order: " << options.binaryOrder
              << "\nFiles per tile: " << options.filesPerTile
              << "\nUUID: " << options.uuid
              << "\nOverhead: " << info.overhead << " bytes"
              << "\nModified at: " << utility::formatDateTime(info.modified)
              << "\nIndex offset: " << info.offset
              << "\nPrevious index offset: " << info.previousOffset
              << "\n\n";

    for (const auto &entry : arch.list()) {
        std::cout << '[' << entry.index.col << ',' << entry.index.row
                  << ',' << entry.index.type << "]: "
                  << entry.size << " bytes at " << entry.start << ".\n";
    }
    std::cout.flush();

    return EXIT_SUCCESS;
}

int Tilar::create()
{
    auto arch(vs::Tilar::create
              (file_, createOptions_, vs::Tilar::CreateMode::failIfExists));
    writeFiles(arch, files_);
    arch.commit();

    return EXIT_SUCCESS;
}

int Tilar::append()
{
    auto arch(vs::Tilar::open(file_));
    writeFiles(arch, files_);
    arch.commit();

    return EXIT_SUCCESS;
}

int Tilar::remove()
{
    auto arch(vs::Tilar::open(file_, vs::Tilar::OpenMode::readWrite));
    for (const auto &index : indices_) {
        arch.remove(index);
    }
    arch.commit();

    return EXIT_SUCCESS;
}

int Tilar::extract()
{
    auto arch(indexOffset_
              ? vs::Tilar::open(file_, *indexOffset_)
              : vs::Tilar::open(file_, vs::Tilar::OpenMode::readOnly));
    extractFiles(arch, files_);

    return EXIT_SUCCESS;
}

int main(int argc, char *argv[])
{
    return Tilar()(argc, argv);
}
