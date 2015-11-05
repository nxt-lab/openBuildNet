#ifndef SMNCHAI_CHAISCRIPT_STDLIB
#define SMNCHAI_CHAISCRIPT_STDLIB

namespace chaiscript {
  class Module;
}

namespace SMNChai {
  std::shared_ptr<chaiscript::Module> create_chaiscript_stdlib();
}

#endif
