/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Basics of nodechai.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <cstring>
#include "nodechai.h"
#include <chaiscript/chaiscript.hpp>

namespace NodeChai {
    const char* process_commandline_args(int argc, char **argv,
                                         std::map<std::string, chaiscript::Boxed_Value>& argmap)
    {
        if (argc < 2) {
            // Script file not provided -> return nullptr
            return nullptr;
        }
        
        // We consider argument #2 onwards as named arguments to the node script; Argument #1 is the script file name.
        char* equalsign = nullptr;
        for (auto i=2; i < argc; ++i) {
            // Each named argument must have the form "keyword=value" and the keyword must be unique
            
            equalsign = std::strchr(argv[i], '=');  // Find the equal sign position
            if (!equalsign || equalsign == argv[i]) {
                // '=' not found or keyword is empty -> invalid
                throw nodechai_exception(std::string("Invalid named argument: ") + argv[i]);
            }
            
            // Extract the key string and trim it
            std::string keystr(OBNsim::Utils::trim(std::string(argv[i], equalsign-argv[i])));
            if (keystr.empty()) {
                throw nodechai_exception(std::string("Named argument has empty key: ") + argv[i]);
            }
            
            // Check if the key already exists
            if (argmap.count(keystr) > 0) {
                throw nodechai_exception("Key of named argument is not unique: " + keystr);
            }
            
            // Now we can register the key-value pair to the map
            argmap.emplace(keystr, chaiscript::Boxed_Value(std::string(equalsign+1)));
        }
        
        return argv[1];
    }
}