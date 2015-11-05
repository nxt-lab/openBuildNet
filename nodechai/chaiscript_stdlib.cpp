/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Include the stdlib in Chaiscript.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <chaiscript/chaiscript_stdlib.hpp>
#include "chaiscript_stdlib.h"

#ifdef NODECHAI_CHAISCRIPT_STATIC
namespace SMNChai {
  std::shared_ptr<chaiscript::Module> create_chaiscript_stdlib()
  {
    return chaiscript::Std_Lib::library();
  }
}
#endif
