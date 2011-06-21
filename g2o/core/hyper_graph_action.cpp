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

#include "hyper_graph_action.h"

#include <iostream>

namespace g2o {
  using namespace std;

  HyperGraphActionLibrary* HyperGraphActionLibrary::actionLibInstance = 0;

  HyperGraphAction::Parameters::~Parameters()
  {
  }

  HyperGraphAction::ParametersIteration::ParametersIteration(int iter) :
    HyperGraphAction::Parameters(),
    iteration(iter)
  {
  }

  HyperGraphAction::~HyperGraphAction()
  {
  }

  HyperGraphAction* HyperGraphAction::operator()(const HyperGraph*, Parameters*)
  {
    return 0;
  }

  HyperGraphElementAction::Parameters::~Parameters()
  {
  }

  HyperGraphElementAction::HyperGraphElementAction(const std::string& typeName_)
  {
    _typeName = typeName_;
  }

  HyperGraphElementAction* HyperGraphElementAction::operator()(HyperGraph::HyperGraphElement* , HyperGraphElementAction::Parameters* )
  {
    return 0;
  }
  
  HyperGraphElementAction* HyperGraphElementAction::operator()(const HyperGraph::HyperGraphElement* , HyperGraphElementAction::Parameters* )
  {
    return 0;
  }
  
  HyperGraphElementAction::~HyperGraphElementAction()
  {
  }

  HyperGraphElementActionCollection::HyperGraphElementActionCollection(const std::string& name_)
  {
    _name = name_;
  }

  HyperGraphElementActionCollection::~HyperGraphElementActionCollection()
  {
    _actionMap.clear();
  }

  HyperGraphElementAction* HyperGraphElementActionCollection::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params)
  {
    ActionMap::iterator it=_actionMap.find(typeid(*element).name());
    //cerr << typeid(*element).name() << endl;
    if (it==_actionMap.end())
      return 0;
    HyperGraphElementAction* action=it->second;
    return (*action)(element, params);
  }

  HyperGraphElementAction* HyperGraphElementActionCollection::operator()(const HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params)
  {
    ActionMap::iterator it=_actionMap.find(typeid(*element).name());
    if (it==_actionMap.end())
      return 0;
    HyperGraphElementAction* action=it->second;
    return (*action)(element, params);
  }

  bool HyperGraphElementActionCollection::registerAction(HyperGraphElementAction* action)
  {
    if (action->name()!=name()){
      cerr << __PRETTY_FUNCTION__  << ": invalid attempt to register an action in a collection with a different name " <<  name() << " " << action->name() << endl;
    }
    _actionMap.insert(make_pair ( action->typeName(), action) );
    return true;
  }

  HyperGraphActionLibrary::HyperGraphActionLibrary()
  {
  }

  HyperGraphActionLibrary* HyperGraphActionLibrary::instance()
  {
    if (! actionLibInstance)
      actionLibInstance = new HyperGraphActionLibrary();
    return actionLibInstance;
  }

  void HyperGraphActionLibrary::destroy()
  {
    delete actionLibInstance;
    actionLibInstance = 0;
  }

  HyperGraphActionLibrary::~HyperGraphActionLibrary()
  {
    _actionMap.clear();
  }
  
  HyperGraphElementAction* HyperGraphActionLibrary::actionByName(const std::string& name)
  {

    HyperGraphElementAction::ActionMap::iterator it=_actionMap.find(name);
    if (it!=_actionMap.end())
      return it->second;
    return 0;
  }

  bool HyperGraphActionLibrary::registerAction(HyperGraphElementAction* action)
  {
    HyperGraphElementAction* oldAction = actionByName(action->name());
    HyperGraphElementActionCollection* collection = 0;
    if (oldAction) {
      collection = dynamic_cast<HyperGraphElementActionCollection*>(oldAction);
      if (! collection) {
        cerr << __PRETTY_FUNCTION__ << ": fatal error, a collection is not at the first level in the library" << endl;
        return 0;
      }
    }
    if (! collection) {
      cerr << __PRETTY_FUNCTION__ << ": creating collection for \"" << action->name() << "\"" << endl;
      collection = new HyperGraphElementActionCollection(action->name());
      _actionMap.insert(make_pair(action->name(), collection));
    }
    return collection->registerAction(action);
  }
  

  WriteGnuplotAction::WriteGnuplotAction(const std::string& typeName_)
    : HyperGraphElementAction(typeName_)
  {
    _name="writeGnuplot";
  }

  DrawAction::DrawAction(const std::string& typeName_) 
    : HyperGraphElementAction(typeName_)
  {
    _name="draw";
  }

  void applyAction(HyperGraph* graph, HyperGraphElementAction* action, HyperGraphElementAction::Parameters* params, const std::string& typeName)
  {
    for (HyperGraph::VertexIDMap::iterator it=graph->vertices().begin(); 
        it!=graph->vertices().end(); it++){
      if ( typeName.empty() || typeid(*it->second).name()==typeName)
        (*action)(it->second, params);
    }
    for (HyperGraph::EdgeSet::iterator it=graph->edges().begin(); 
        it!=graph->edges().end(); it++){
      if ( typeName.empty() || typeid(**it).name()==typeName)
        (*action)(*it, params);
    }
  }

} // end namespace
