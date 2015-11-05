/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Include the stdlib in Chaiscript.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef NODECHAI_CHAISCRIPT_STDLIB
#define NODECHAI_CHAISCRIPT_STDLIB

namespace chaiscript {
  class Module;
}

namespace SMNChai {
  std::shared_ptr<chaiscript::Module> create_chaiscript_stdlib();
}

#endif
