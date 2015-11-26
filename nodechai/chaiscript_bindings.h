/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Bindings for Chaiscript.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef NODECHAI_CHAISCRIPT_BINDINGS_H
#define NODECHAI_CHAISCRIPT_BINDINGS_H

#include <memory>

namespace chaiscript {
    class Module;
    class ChaiScript;
}

namespace NodeChai {
    struct NodeFactory;
    
    /** Create the API bindings before a node is created. */
    std::shared_ptr<chaiscript::Module> create_bindings_before_node(std::shared_ptr<chaiscript::Module> m = std::make_shared<chaiscript::Module>());
    
    /** Bind API for Eigen. */
    std::shared_ptr<chaiscript::Module> nodechai_api_eigen(std::shared_ptr<chaiscript::Module> m = std::make_shared<chaiscript::Module>());
}

#endif /* CHAISCRIPT_BINDINGS_H */
