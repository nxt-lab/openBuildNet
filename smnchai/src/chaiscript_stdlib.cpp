#include <chaiscript/chaiscript_stdlib.hpp>
#include "chaiscript_stdlib.h"

#ifdef SMNCHAI_CHAISCRIPT_STATIC
namespace SMNChai {
  std::shared_ptr<chaiscript::Module> create_chaiscript_stdlib()
  {
    return chaiscript::Std_Lib::library();
  }
}
#endif
