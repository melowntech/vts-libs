// NB: do not include manually
// included from support.cpp and support.pic.cpp

const storage::SupportFile::Files supportFiles =
{
    { "index.html"
      , {
            browser::index_html
            , sizeof(browser::index_html)
            , browser::index_html_attr_lastModified
            , "text/html; charset=utf-8"
            , true
        }
    }, {
        "skydome.jpg"
        , {
            browser::skydome_jpg
            , sizeof(browser::skydome_jpg)
            , browser::skydome_jpg_attr_lastModified
            , "image/jpeg"
            , false
        }
    }, { "debugger.html"
      , {
            browser::debugger_html
            , sizeof(browser::debugger_html)
            , browser::debugger_html_attr_lastModified
            , "text/html; charset=utf-8"
            , false
        }
    }, { "debugger.js"
      , {
            browser::debugger_js
            , sizeof(browser::debugger_js)
            , browser::debugger_js_attr_lastModified
            , "application/javascript; charset=utf-8"
            , false
        }
    }
};
