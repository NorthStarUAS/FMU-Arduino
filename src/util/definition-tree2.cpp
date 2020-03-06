/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Curtis Olson
*/

#include "definition-tree2.h"

// create a global instance of the deftree
DefinitionTree2 deftree;

ElementPtr DefinitionTree2::initElement(String name, String desc)
{
  def_tree_t::iterator it;
  it = data.find(name);
  if ( it != data.end() ) {
    // cout << "NOTICE: publisher found existing def-tree element: " << name << endl;
    it->second->description = desc;
    return it->second;
  } else {
    ElementPtr ele = make_shared<Element>();
    ele->description = desc;
    data[name] = ele;
    return ele;
  }
}

ElementPtr DefinitionTree2::getElement(String name, bool create) {
  def_tree_t::iterator it;
  it = data.find(name);
  if ( it != data.end() ) {
    return it->second;
  } else if ( create ) {
    Serial.print("NOTICE: subscriber created def-tree element: ");
    Serial.println(name.c_str());
    ElementPtr ele = make_shared<Element>();
    data[name] = ele;
    return ele;
  } else {
    Serial.print("NOTICE: subscriber FAILED TO GET def-tree element: ");
    Serial.println(name.c_str());
    return NULL;
  }
}

/* Gets list of definition tree member keys at a given tree level */
void DefinitionTree2::GetKeys(String Name, vector<String> *KeysPtr) {
  KeysPtr->clear();
  for (auto const& element : data) {
    if ( element.first.startsWith(Name) ) {
      KeysPtr->push_back(element.first);
    }
  }
}

/* Gets number of definition tree members at a given tree level */
size_t DefinitionTree2::Size(String Name) {
  size_t retval = 0;
  for ( auto const& element : data ) {
    if ( element.first.startsWith(Name) ) {
      retval++;
    }
  }
  return retval;
}

/* print definition tree member keys at a given tree level */
void DefinitionTree2::PrettyPrint(String Prefix) {
  Serial.print("Base path: ");
  Serial.println(Prefix.c_str());
  size_t len = Prefix.length();
  for ( auto const& it : data ) {
      if ( it.first.startsWith(Prefix) ) {
      String tail = it.first.substring(len + 1);
      ElementPtr ele = it.second;
      Serial.print("    ");
      Serial.print(tail.c_str());
      Serial.print(" (");
      Serial.print(ele->getType().c_str());
      Serial.print(") = ");
      Serial.println(ele->getValueAsString().c_str());
    }
  }
}

void DefinitionTree2::Erase(String name) {
  def_tree_t::iterator it;
  it = data.find(name);
  if ( it != data.end() ) {
    data.erase(it);
  } else {
    Serial.print("NOTICE: attempting to erase non-existent element: ");
    Serial.println(name.c_str());
  }
}
