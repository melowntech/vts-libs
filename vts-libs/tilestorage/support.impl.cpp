// NB: do not include manually
// included from support.cpp and support.pic.cpp

const SupportFile::Files SupportFile::files =
{
    { "index.html"
      , { browser::index_html
          , sizeof(browser::index_html)
          , browser::index_html_attr_lastModified } }
    , { "index-offline.html"
        , { browser::index_offline_html
            , sizeof(browser::index_offline_html)
            , browser::index_offline_html_attr_lastModified } }
    , { "skydome.jpg"
        , { browser::skydome_jpg
            , sizeof(browser::skydome_jpg)
            , browser::skydome_jpg_attr_lastModified } }

};
