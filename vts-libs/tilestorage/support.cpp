#include "./support.hpp"

#include "tilestorage/browser/index.html.hpp"
#include "tilestorage/browser/index-offline.html.hpp"
#include "tilestorage/browser/skydome.jpg.hpp"

namespace vadstena { namespace tilestorage {

const SupportFile::Files SupportFile::files =
{
    { "index.html"
      , { browser::index_html
          , sizeof(browser::index_html) } }
    , { "index-offline.html"
        , { browser::index_offline_html
            , sizeof(browser::index_offline_html) } }
    , { "skydome.jpg"
        , { browser::skydome_jpg
            , sizeof(browser::skydome_jpg) } }

};

} } // namespace vadstena::tilestorage
