/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Header file for the C dynamic library of the SMN library.
 *
 * The DLL is used by scripting languages such as Python, Julia, or by non-C++ languages such as C, to implement the SMN.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNSMNLIB_H
#define OBNSMNLIB_H

/** Types and functions the external interface can use. */
#ifdef __cplusplus
extern "C" {
#endif
    

#ifdef __cplusplus
}
#endif


// Macros defined for exporting functions in DLL

#if defined _WIN32 || defined __CYGWIN__
#ifdef BUILDING_DLL
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __declspec(dllimport)
#endif
#elif defined __APPLE__ || defined __linux__
#define EXPORT __attribute__ ((visibility ("default")))
#else
#error "Unknown platform."
#endif


#endif /* OBNSMNLIB_H */
