/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basic definitions.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <obnsim_basic.h>

using namespace std;

std::string OBNsim::Utils::trim(const std::string& s0) {
    string s(s0);
    size_t found = s.find_last_not_of(" \t\f\v\n\r");
    if (found != string::npos) {
        s.erase(found+1);
        found = s.find_first_not_of(" \t\f\v\n\r");
        s.erase(0, found);
    } else {
        s.clear();
    }
    
    return s;
}


bool OBNsim::Utils::isValidIdentifier(const std::string &name) {
    if (name.empty() || name[0] == '_') {
        return false;
    }
    return name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") == string::npos;
}