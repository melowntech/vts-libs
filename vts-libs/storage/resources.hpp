#ifndef vadstena_libs_storage_resources_hpp_included_
#define vadstena_libs_storage_resources_hpp_included_

namespace vadstena { namespace storage {

struct Resources {
    std::size_t openFiles;
    std::size_t memory;

    Resources(std::size_t openFiles = 0, std::size_t memory = 0)
        : openFiles(openFiles), memory(memory)
    {}

    Resources& operator+=(Resources &o);
    Resources& operator-=(Resources &o);
    bool operator<(const Resources &o) const;
    bool operator>(const Resources &o) const;
};

// inlines

inline Resources& Resources::operator+=(Resources &o)
{
    openFiles += o.openFiles;
    memory += o.memory;
    return *this;
}

inline Resources& Resources::operator-=(Resources &o)
{
    openFiles -= o.openFiles;
    memory -= o.memory;
    return *this;
}

inline bool Resources::operator<(const Resources &o) const
{
    if (openFiles < o.openFiles) {
        return true;
    } else if (o.openFiles < openFiles) {
        return false;
    }
    return memory < o.memory;
}

inline bool Resources::operator>(const Resources &o) const
{
    if (openFiles > o.openFiles) {
        return true;
    } else if (o.openFiles > openFiles) {
        return false;
    }
    return memory > o.memory;
}

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_resources_hpp_included_

