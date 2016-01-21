/** \file
 * Support for Eigen linear algebra library in Chaiscript.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef _CHAISCRIPT_EXTRAS_EIGEN_H
#define _CHAISCRIPT_EXTRAS_EIGEN_H

#include <vector>
#include <Eigen/Dense>
#include <chaiscript/chaiscript.hpp>

namespace chaiscript {
    namespace extras {
        namespace eigenlinalg {
            typedef std::vector<chaiscript::Boxed_Value> TChaiVector;   ///< The C++ type of a Chaiscript's vector object
            
            Eigen::IOFormat EigenMatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
            
            // Register with Chaiscript a new Eigen vector type
            template<typename CLS, typename Scalar>
            ModulePtr eigen_vector_type(const char* CLSNAME, ModulePtr m = std::make_shared<Module>()) {
                //// 1.1. Type and Construction
                m->add(user_type<CLS>(), CLSNAME);
                m->add(constructor<CLS ()>(), CLSNAME);
                m->add(constructor<CLS (typename CLS::Index)>(), CLSNAME);
                m->add(bootstrap::copy_constructor<CLS>(CLSNAME));
                m->add(fun(static_cast<CLS& (CLS::*)(const CLS&)>(&CLS::operator=)), "=");
                
                // Construct vector from a Chaiscript vector of elements of EXACTLY the type Scalar
                m->add(fun([](const TChaiVector& cv) {
                    auto n = cv.size();
                    CLS v(n);
                    if (n > 0) {
                        int k = 0;
                        for (auto& elem: cv) {
                            v(k++) = chaiscript::boxed_cast<Scalar>(elem);
                        }
                    }
                    return v;
                }), CLSNAME);

                
                //// 1.2. Accessors
                m->add(fun(static_cast<Scalar& (CLS::*)(typename CLS::Index)>(&CLS::operator())), "[]");
                m->add(fun(static_cast<Scalar& (CLS::*)(typename CLS::Index)>(&CLS::operator())), "coeff");
                
                //// 1.3. Size and resize
                m->add(fun(static_cast<typename CLS::Index (CLS::*)() const>(&CLS::size)), "size");
                m->add(fun(static_cast<typename CLS::Index (CLS::*)() const>(&CLS::rows)), "rows");
                m->add(fun(static_cast<typename CLS::Index (CLS::*)() const>(&CLS::cols)), "cols");
                m->add(fun(static_cast<void (CLS::*)(typename CLS::Index)>(&CLS::resize)), "resize"); // destructive resize
                
                //// 1.4. Misc
                
                // Convert to string to print
                m->add(fun([](const CLS& v) {
                    std::stringstream s;
                    s << v.format(EigenMatlabFmt);
                    return s.str();
                }), "to_string");
                
                return m;
            }
         
            
            // Register with Chaiscript a new Eigen matrix type
            template<typename CLS, typename Scalar>
            ModulePtr eigen_matrix_type(const char* CLSNAME, ModulePtr m = std::make_shared<Module>()) {
                //// 1.1. Type and Construction
                m->add(user_type<CLS>(), CLSNAME);
                m->add(constructor<CLS ()>(), CLSNAME);
                // Somehow Eigen does not work nicely with Chaiscript's constructor utility, so I must use a workaround
                m->add(fun([](typename CLS::Index r, typename CLS::Index c) {
                    return CLS(r, c);
                }), CLSNAME);
                m->add(bootstrap::copy_constructor<CLS>(CLSNAME));
                m->add(fun(static_cast<CLS& (CLS::*)(const CLS&)>(&CLS::operator=)), "=");
                
                // Conversions from vector to matrix
                //m->add(fun([](const Eigen::Matrix<Scalar,Eigen::Dynamic,1>& v) { return CLS(v); }), CLSNAME);
                //m->add(fun([](const Eigen::Matrix<Scalar,1,Eigen::Dynamic>& v) { return CLS(v); }), CLSNAME);
                
                // Construct a matrix of given size from a Chaiscript vector of elements of EXACTLY the type Scalar.
                // This is done ROW-WISE.
                // Only r*c elements are copied from the vector to the matrix. If the vector contains fewer elements, only those will be copied, the remaining elements of the matrix are undefined.
                m->add(fun([](typename CLS::Index r, typename CLS::Index c, const TChaiVector& cv) {
                    auto n = cv.size();
                    CLS v(r, c);
                    if (n > 0 && r*c > 0) {
                        int k = 0;
                        for (auto ri = 0; ri < r; ++ri) {
                            for (auto ci = 0; ci < c; ++ci) {
                                v(ri, ci) = chaiscript::Boxed_Number(cv[k++]).get_as<Scalar>();
                                if (k == n) {
                                    break;
                                }
                            }
                            if (k == n) {
                                break;
                            }
                        }
                    }
                    return v;
                }), CLSNAME);
                
                
                //// 1.2. Accessors
                m->add(fun(static_cast<Scalar& (CLS::*)(typename CLS::Index, typename CLS::Index)>(&CLS::operator())), "coeff");
                m->add(fun(static_cast<Scalar& (CLS::*)(typename CLS::Index)>(&CLS::operator())), "[]");

                
                //// 1.3. Size and resize
                m->add(fun(static_cast<typename CLS::Index (CLS::*)() const>(&CLS::size)), "size");
                m->add(fun(static_cast<typename CLS::Index (CLS::*)() const>(&CLS::rows)), "rows");
                m->add(fun(static_cast<typename CLS::Index (CLS::*)() const>(&CLS::cols)), "cols");
                m->add(fun(static_cast<void (CLS::*)(typename CLS::Index, typename CLS::Index)>(&CLS::resize)), "resize"); // destructive resize
                
                //// 1.4. Misc
                // Convert to string to print
                m->add(fun([](const CLS& v) {
                    std::stringstream s;
                    s << v.format(EigenMatlabFmt);
                    return s.str();
                }), "to_string");
                
                return m;
            }
            
            // Predefined matrices
            template<typename CLS>
            ModulePtr eigen_matrix_predefined(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun([](typename CLS::Index r, typename CLS::Index c) { return CLS(CLS::Zero(r,c)); }), "zeros");  // Zero matrix
                m->add(fun([](CLS &m) { m.setZero(); }), "setZero");  // set all coefficients to 0
                
                m->add(fun([](typename CLS::Index r, typename CLS::Index c) { return CLS(CLS::Ones(r,c)); }), "ones");  // matrix of 1's
                m->add(fun([](CLS &m) { m.setOnes(); }), "setOnes");  // set all coefficients to 1
                
                m->add(fun(&CLS::fill), "fill");  // set all coefficients to a given constant
                
                m->add(fun([](typename CLS::Index r, typename CLS::Index c) { return CLS(CLS::Identity(r,c));}), "eyes");  // Identity matrix
                m->add(fun([](CLS &m) { m.setIdentity(); }), "setIdentity");  // set the matrix to identity
                m->add(fun([](CLS &m, typename CLS::Index r, typename CLS::Index c) { m.setIdentity(r,c); }), "setIdentity");  // resize and set the matrix to identity
                
                return m;
            }
            
            // Binary Addition and subtraction
            // CLS1 and CLS2 should be different types, where CLS1 should be "larger" in the sense that CLS2 is a special case of CLS1 (e.g. Matrix and Vector)
            template<typename CLS1, typename CLS2>
            ModulePtr binary_addition_substraction(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun([](const CLS1& a, const CLS2& b) { return CLS1(a + b); }), "+");
                m->add(fun([](const CLS2& a, const CLS1& b) { return CLS1(a + b); }), "+");
                m->add(fun([](const CLS1& a, const CLS2& b) { return CLS1(a - b); }), "-");
                m->add(fun([](const CLS2& a, const CLS1& b) { return CLS1(a - b); }), "-");
                return m;
            }
            
            // Binary Addition and subtraction for objects of same class
            template<typename CLS1>
            ModulePtr binary_addition_substraction(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun([](const CLS1& a, const CLS1& b) { return (a + b).eval(); }), "+");
                m->add(fun([](const CLS1& a, const CLS1& b) { return (a - b).eval(); }), "-");
                m->add(fun([](CLS1& a, const CLS1& b) { return (a += b); }), "+=");
                
                // Between matrix and scalar
                m->add(fun([](const CLS1& a, const typename CLS1::Scalar b) { return CLS1(a.array() + b); }), "+");
                //m->add(fun([](const typename CLS1::Scalar b, const CLS1& a) { return CLS1(a.array() + b); }), "+");
                m->add(fun([](const CLS1& a, const typename CLS1::Scalar b) { return CLS1(a.array() - b); }), "-");
                //m->add(fun([](const typename CLS1::Scalar b, const CLS1& a) { return CLS1((-a).array() + b); }), "-");

                return m;
            }
            
            // Unary Addition and subtraction
            template<typename CLS>
            ModulePtr unary_addition_substraction(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun([](const CLS& a) { return a; }), "+");
                m->add(fun([](const CLS& a) { return (-a).eval(); }), "-");
                return m;
            }
            
            // Scalar multiplication and division
            template<typename CLS, typename Scalar>
            ModulePtr scalar_mult_div(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun([](const CLS& a, const Scalar b) { return (a * b).eval(); }), "*");
                m->add(fun([](const Scalar a, const CLS& b) { return (a * b).eval(); }), "*");
                m->add(fun([](const CLS& a, const Scalar b) { return (a / b).eval(); }), "/");
                m->add(fun([](CLS& a, const Scalar b) { return (a *= b); }), "*=");
                m->add(fun([](CLS& a, const Scalar b) { return (a /= b); }), "/=");
                return m;
            }
            
            // Transpose and conjugation
            template<typename CLS, const bool withConjugation>
            ModulePtr transpose_conjugation(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun([](const CLS& a) { return CLS(a.transpose()); }), "transpose");
                m->add(fun(static_cast<void (CLS::*)()>(&CLS::transposeInPlace)), "transposeInPlace");    // in-place transpose: void transposeInPlace()
                
                if (withConjugation) {
                    m->add(fun([](const CLS& a) { return CLS(a.conjugate()); }), "conjugate");
                    m->add(fun([](const CLS& a) { return CLS(a.adjoint()); }), "adjoint");
                    m->add(fun(static_cast<void (CLS::*)()>(&CLS::adjointInPlace)), "adjointInPlace");    // in-place adjoint: void adjointInPlace()
                }
                return m;
            }
            
            // Matrix multiplication
            template<typename CLS1, typename CLS2 = CLS1>
            ModulePtr matrix_mult(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun([](const CLS1& a, const CLS2& b) { return (a * b).eval(); }), "*");
                m->add(fun([](CLS1& a, const CLS2& b) { return a *= b; }), "*=");
                return m;
            }
            
            // Dot product (only for vector types)
            template<typename CLS1>
            ModulePtr dot_product(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun([](const CLS1& a, const CLS1& b) { return a.dot(b); }), "dot");
                return m;
            }
            
            // Basic arithmetic reduction operations
            template<typename CLS>
            ModulePtr arith_reduction(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun(static_cast<typename CLS::Scalar (CLS::*)() const>(&CLS::sum)), "sum");
                m->add(fun(static_cast<typename CLS::Scalar (CLS::*)() const>(&CLS::prod)), "prod");
                m->add(fun(static_cast<typename CLS::Scalar (CLS::*)() const>(&CLS::mean)), "mean");
                m->add(fun(static_cast<typename CLS::Scalar (CLS::*)() const>(&CLS::minCoeff)), "min");
                m->add(fun(static_cast<typename CLS::Scalar (CLS::*)() const>(&CLS::maxCoeff)), "max");
                m->add(fun(static_cast<typename CLS::Scalar (CLS::*)() const>(&CLS::trace)), "trace");
                return m;
            }
            
            // Coefficient-wise operations
            template<typename CLS>
            ModulePtr coefficient_wise_operations(ModulePtr m = std::make_shared<Module>()) {
                m->add(fun([](const CLS& a) { return a.cwiseAbs().eval(); }), "abs");
                m->add(fun([](const CLS& a) { return a.cwiseAbs2().eval(); }), "abs2");
                m->add(fun([](const CLS& a) { return a.cwiseSqrt().eval(); }), "sqrt");
                m->add(fun([](const CLS& a) { return a.array().log().matrix().eval(); }), "log");
                m->add(fun([](const CLS& a) { return a.array().exp().matrix().eval().eval(); }), "exp");
                m->add(fun([](const CLS& a, double e) { return a.array().pow(e).matrix().eval(); }), "pow");
                m->add(fun([](const CLS& a) { return a.array().square().matrix().eval(); }), "square");
                m->add(fun([](const CLS& a) { return a.array().sin().matrix().eval(); }), "sin");
                m->add(fun([](const CLS& a) { return a.array().cos().matrix().eval(); }), "cos");
                m->add(fun([](const CLS& a) { return a.array().tan().matrix().eval(); }), "tan");
                m->add(fun([](const CLS& a) { return a.array().asin().matrix().eval(); }), "asin");
                m->add(fun([](const CLS& a) { return a.array().acos().matrix().eval(); }), "acos");
                
                return m;
            }

        }
    }
}
#endif // _CHAISCRIPT_EXTRAS_EIGEN_H
