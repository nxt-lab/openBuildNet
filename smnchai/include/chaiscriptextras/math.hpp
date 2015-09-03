#ifndef _CHAISCRIPT_EXTRAS_MATH_H
#define _CHAISCRIPT_EXTRAS_MATH_H

#include <cmath>
#include <memory>

#include <chaiscript/chaiscript.hpp>

namespace chaiscript {
  namespace extras {
    namespace math {
      // TRIG FUNCTIONS
      template<typename Ret, typename Param>
      ModulePtr cos(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::cos)), "cos");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr sin(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::sin)), "sin");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr tan(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::tan)), "tan");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr acos(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::acos)), "acos");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr asin(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::asin)), "asin");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr atan(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::atan)), "atan");
        return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr atan2(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::atan2)), "atan2");
        return m;
      }

      // HYPERBOLIC FUNCTIONS
      template<typename Ret, typename Param>
      ModulePtr cosh(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::cosh)), "cosh");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr sinh(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::sinh)), "sinh");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr tanh(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::tanh)), "tanh");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr acosh(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::acosh)), "acosh");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr asinh(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::asinh)), "asinh");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr atanh(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::atanh)), "atanh");
        return m;
      }

      // EXPONENTIAL AND LOGARITHMIC FUNCTIONS
      template<typename Ret, typename Param>
      ModulePtr exp(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::exp)), "exp");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr frexp(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::frexp)), "frexp");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr ldexp(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::ldexp)), "ldexp");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr log(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::log)), "log");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr log10(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::log10)), "log10");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr modf(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::modf)), "modf");
    	  return m;
      }
      template<typename Ret, typename Param>
      ModulePtr exp2(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::exp2)), "exp2");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr expm1(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::expm1)), "expm1");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr ilogb(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::ilogb)), "ilogb");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr log1p(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::log1p)), "log1p");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr log2(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::log2)), "log2");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr logb(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::logb)), "logb");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr scalbn(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::scalbn)), "scalbn");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr scalbln(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::scalbln)), "scalbln");
    	  return m;
      }

      // POWER FUNCTIONS
      template<typename Ret, typename Param1, typename Param2>
      ModulePtr pow(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::pow)), "pow");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr sqrt(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::sqrt)), "sqrt");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr cbrt(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::cbrt)), "cbrt");
        return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr hypot(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::hypot)), "hypot");
        return m;
      }

      // ERROR AND GAMMA FUNCTIONS
      template<typename Ret, typename Param>
      ModulePtr erf(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::erf)), "erf");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr erfc(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::erfc)), "erfc");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr tgamma(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::tgamma)), "tgamma");
        return m;
      }

      template<typename Ret, typename Param>
      ModulePtr lgamma(ModulePtr m = std::make_shared<Module>())
      {
        m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::lgamma)), "lgamma");
        return m;
      }

      // ROUNDING AND REMAINDER FUNCTIONS
      template<typename Ret, typename Param>
      ModulePtr ceil(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::ceil)), "ceil");
		  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr floor(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::floor)), "floor");
		  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr fmod(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::fmod)), "fmod");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr trunc(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::trunc)), "trunc");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr round(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::round)), "round");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr lround(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::lround)), "lround");
    	  return m;
      }

      // long long ints do not work
      template<typename Ret, typename Param>
      ModulePtr llround(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::llround)), "llround");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr rint(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::rint)), "rint");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr lrint(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::lrint)), "lrint");
    	  return m;
      }

      // long long ints do not work
      template<typename Ret, typename Param>
      ModulePtr llrint(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::llrint)), "llrint");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr nearbyint(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::nearbyint)), "nearbyint");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr remainder(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::remainder)), "remainder");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2, typename Param3>
      ModulePtr remquo(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2, Param3)>(&std::remquo)), "remquo");
    	  return m;
      }

      // FLOATING-POINT MANIPULATION FUNCTIONS
      template<typename Ret, typename Param1, typename Param2>
      ModulePtr copysign(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::copysign)), "copysign");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr nan(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::nan)), "nan");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr nextafter(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::nextafter)), "nextafter");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr nexttoward(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::nexttoward)), "nexttoward");
    	  return m;
      }

      // MINIMUM, MAXIMUM, DIFFERENCE FUNCTIONS
      template<typename Ret, typename Param1, typename Param2>
      ModulePtr fdim(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::fdim)), "fdim");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr fmax(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::fmax)), "fmax");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr fmin(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::fmin)), "fmin");
    	  return m;
      }

      // OTHER FUNCTIONS
      template<typename Ret, typename Param>
      ModulePtr fabs(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::fabs)), "fabs");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr abs(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::abs)), "abs");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2, typename Param3>
      ModulePtr fma(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2, Param3)>(&std::fma)), "fma");
    	  return m;
      }

      // CLASSIFICATION FUNCTIONS
      template<typename Ret, typename Param>
      ModulePtr fpclassify(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::fpclassify)), "fpclassify");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr isfinite(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::isfinite)), "isfinite");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr isinf(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::isinf)), "isinf");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr isnan(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::isnan)), "isnan");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr isnormal(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::isnormal)), "isnormal");
    	  return m;
      }

      template<typename Ret, typename Param>
      ModulePtr signbit(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param)>(&std::signbit)), "signbit");
    	  return m;
      }


      // COMPARISON FUNCTIONS
      template<typename Ret, typename Param1, typename Param2>
      ModulePtr isgreater(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::isgreater)), "isgreater");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr isgreaterequal(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::isgreaterequal)), "isgreaterequal");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr isless(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::isless)), "isless");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr islessequal(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::islessequal)), "islessequal");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr islessgreater(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::islessgreater)), "islessgreater");
    	  return m;
      }

      template<typename Ret, typename Param1, typename Param2>
      ModulePtr isunordered(ModulePtr m = std::make_shared<Module>())
      {
    	  m->add(chaiscript::fun(static_cast<Ret (*)(Param1, Param2)>(&std::isunordered)), "isunordered");
    	  return m;
      }
    }
  }
}

#endif // _CHAISCRIPT_EXTRAS_MATH_H
