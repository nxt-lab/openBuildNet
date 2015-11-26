/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Bindings for Eigen linear algebra library in Chaiscript.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include "chaiscript_bindings.h"
#include <chaiscript/chaiscript.hpp>
#include <chaiscript/dispatchkit/bootstrap.hpp>
#include <chaiscriptextras/eigen.hpp>

using namespace chaiscript;

namespace NodeChai {
    /** Add support for Eigen linear algebra library.
     Only Matrix types are supported, because vectors are special matrices.
     */
    chaiscript::ModulePtr nodechai_api_eigen(chaiscript::ModulePtr m) {
        
        //// Matrix of doubles
        chaiscript::extras::eigenlinalg::eigen_matrix_type<Eigen::MatrixXd, double>("MatrixD", m);
        
        //// Column vector of doubles
        //chaiscript::extras::eigenlinalg::eigen_vector_type<Eigen::VectorXd, double>("VectorD", m);
        
        //// Row vector of doubles
        //chaiscript::extras::eigenlinalg::eigen_vector_type<Eigen::RowVectorXd, double>("RowVectorD", m);
        
        //// Linear algebra operations
        chaiscript::extras::eigenlinalg::unary_addition_substraction<Eigen::MatrixXd>(m);
        
        chaiscript::extras::eigenlinalg::binary_addition_substraction<Eigen::MatrixXd>(m);
        //chaiscript::extras::eigenlinalg::binary_addition_substraction<Eigen::VectorXd>(m);
        //chaiscript::extras::eigenlinalg::binary_addition_substraction<Eigen::RowVectorXd>(m);
        
        //chaiscript::extras::eigenlinalg::binary_addition_substraction<Eigen::MatrixXd, Eigen::VectorXd>(m);
        //chaiscript::extras::eigenlinalg::binary_addition_substraction<Eigen::MatrixXd, Eigen::RowVectorXd>(m);
        
        chaiscript::extras::eigenlinalg::scalar_mult_div<Eigen::MatrixXd, double>(m);
        
        chaiscript::extras::eigenlinalg::transpose_conjugation<Eigen::MatrixXd, false>(m);
        
        chaiscript::extras::eigenlinalg::matrix_mult<Eigen::MatrixXd>(m);
        
        chaiscript::extras::eigenlinalg::arith_reduction<Eigen::MatrixXd>(m);
        
        chaiscript::extras::eigenlinalg::coefficient_wise_operations<Eigen::MatrixXd>(m);
        
        return m;
    }
}