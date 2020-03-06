/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Curtis Olson
*/

#include "definition-tree2.h"

// create a global instance of the deftree
DefinitionTree2 deftree;

int DefinitionTree2::find(String name) {
    for (unsigned int i = 0; i < data.size(); i++) {
        if ( data[i]->name == name ) {
            return i;
        }
    }
    return -1;
}

ElementPtr DefinitionTree2::initElement(String name, String desc)
{
    int i = find(name);
    if ( i  >= 0 ) {
        // cout << "NOTICE: publisher found existing def-tree element: " << name << endl;
        data[i]->description = desc;
        return data[i];
    } else {
        ElementPtr ele = make_shared<Element>();
        ele->name = name;
        ele->description = desc;
        data.push_back(ele);
        return ele;
    }
}

ElementPtr DefinitionTree2::getElement(String name, bool create) {
    int i = find(name);
    if ( i >= 0 ) {
        return data[i];
    } else if ( create ) {
        Serial.print("NOTICE: subscriber created def-tree element: ");
        Serial.println(name.c_str());
        ElementPtr ele = make_shared<Element>();
        ele->name = name;
        data.push_back(ele);
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
    for ( unsigned int i = 0; i < data.size(); i++ ) {
        if ( data[i]->name.startsWith(Name) ) {
            KeysPtr->push_back(data[i]->name);
        }
    }
}

/* Gets number of definition tree members at a given tree level */
size_t DefinitionTree2::Size(String Name) {
  size_t retval = 0;
    for ( unsigned int i = 0; i < data.size(); i++ ) {
        if ( data[i]->name.startsWith(Name) ) {
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
    for ( unsigned int i = 0; i < data.size(); i++ ) {
        if ( data[i]->name.startsWith(Prefix) ) {
            String tail = data[i]->name.substring(len + 1);
            Serial.print("    ");
            Serial.print(tail.c_str());
            Serial.print(" (");
            Serial.print(data[i]->getType().c_str());
            Serial.print(") = ");
            Serial.println(data[i]->getValueAsString().c_str());
        }
    }
}
