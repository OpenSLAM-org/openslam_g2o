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

#ifndef FACTORY_H
#define FACTORY_H

#include "hyper_graph.h"
#include <string>
#include <map>

namespace g2o {

  class AbstractHyperGraphElementCreator;
  
  /**
   * \brief create vertices and edges based on TAGs in, for example, a file
   */
  class Factory
  {
    public:

      //! return the instance
      static Factory* instance();

      //! free the instance
      static void destroy();

      /**
       * register a tag for a specific creator
       */
      void registerType(const std::string& tag, AbstractHyperGraphElementCreator* c);

      /**
       * construct a vertex based on its tag
       */
      HyperGraph::HyperGraphElement* construct(const std::string& tag);

      /**
       * return whether the factory knows this tag or not
       */
      bool knowsTag(const std::string& tag);

      //! return the TAG given a vertex
      const std::string& tag(HyperGraph::HyperGraphElement* v);

      /**
       * get a list of all known types
       */
      void fillKnownTypes(std::vector<std::string>& types);

      /**
       * print a list of the known registered types to the given stream
       */
      void printRegisteredTypes(std::ostream& os, bool comment = false);

    protected:
      typedef std::map<std::string, AbstractHyperGraphElementCreator*>      CreatorMap;
      typedef std::map<std::string, std::string>                            TagLookup;
      Factory();
      ~Factory();

      CreatorMap creator;     ///< look-up map for the existing creators
      TagLookup tagLookup;    ///< reverse look-up, class name to tag

    private:
      static Factory* factoryInstance;
  };

}

#endif
