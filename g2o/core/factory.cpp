// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "factory.h"

#include "creators.h"

#include <iostream>
#include <typeinfo>
#include <cassert>

using namespace std;

namespace g2o {

Factory* Factory::factoryInstance = 0;

Factory::Factory()
{
}

Factory::~Factory()
{
  for (CreatorMap::iterator it = creator.begin(); it != creator.end(); ++it)
    delete it->second;
}

Factory* Factory::instance()
{
  if (factoryInstance == 0) {
    factoryInstance = new Factory();
  }

  return factoryInstance;
}

void Factory::registerType(const std::string& tag, AbstractHyperGraphElementCreator* c)
{
  CreatorMap::const_iterator foundIt = creator.find(tag);
  if (foundIt != creator.end()) {
    cerr << "FACTORY WARNING: Overwriting Vertex tag " << tag << endl;
    assert(0);
  }
  TagLookup::const_iterator tagIt = tagLookup.find(c->name());
  if (tagIt != tagLookup.end()) {
    cerr << "FACTORY WARNING: Registering same class for two tags " << c->name() << endl;
    assert(0);
  }

  creator[tag] = c;
  tagLookup[c->name()] = tag;
}

HyperGraph::HyperGraphElement* Factory::construct(const std::string& tag)
{
  CreatorMap::const_iterator foundIt = creator.find(tag);
  if (foundIt != creator.end()) {
    //cerr << "tag " << tag << " -> " << foundIt->second->name() << endl;
    return foundIt->second->construct();
  }
  return 0;
}

const std::string& Factory::tag(HyperGraph::HyperGraphElement* e)
{
  static std::string emptyStr("");
  TagLookup::const_iterator foundIt = instance()->tagLookup.find(typeid(*e).name());
  if (foundIt != instance()->tagLookup.end())
    return foundIt->second;
  return emptyStr;
}

void Factory::fillKnownTypes(std::vector<std::string>& types)
{
  types.clear();
  for (CreatorMap::const_iterator it = creator.begin(); it != creator.end(); ++it)
    types.push_back(it->first);
}

bool Factory::knowsTag(const std::string& tag)
{
  return (creator.find(tag) != creator.end());
}

void Factory::destroy()
{
  delete factoryInstance;
  factoryInstance = 0;
}

void Factory::printRegisteredTypes(std::ostream& os, bool comment)
{
  if (comment)
    os << "# ";
  os << "types:" << endl;
  for (CreatorMap::const_iterator it = creator.begin(); it != creator.end(); ++it) {
    if (comment)
      os << "#";
    cerr << "\t" << it->first << endl;
  }
}

} // end namespace
