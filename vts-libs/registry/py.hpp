/**
 * \file registry/py.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_registry_py_hpp_included_
#define vtslibs_registry_py_hpp_included_

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>

#include "../registry.hpp"

namespace vtslibs { namespace registry {

void fromPython(BoundLayer &boundLayer, const boost::python::dict &value);
void fromPython(BoundLayer::dict &boundLayers
                , const boost::python::dict &value);

void fromPython(Credit &credit, const boost::python::dict &value);
void fromPython(Credit::dict &credits, const boost::python::dict &value);

void fromPython(Credits &credits, const boost::python::object &value);

void fromPython(RegistryBase &rb, const boost::python::dict &value);

void fromPython(Position &position, const boost::python::object &value);

} } // namespace vtslibs::registry

#endif // vtslibs_registry_py_hpp_included_

