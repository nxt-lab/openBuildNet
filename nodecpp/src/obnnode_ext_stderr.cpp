/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnnode_ext_stderr.cpp
 * \brief Standard error reporting used by the external interface library.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <string.h>
#include <obnnode_ext.h>

#define MAXMSGLEN 255
static char error_message[MAXMSGLEN+1] = "";
static char warning_message[MAXMSGLEN+1] = "";


// Report an error and may terminate.
void reportError(const char* msg) {
    strncpy(error_message, msg, MAXMSGLEN); // Copy up to MAXMSGLEN characters
    error_message[MAXMSGLEN] = '\0';    // make sure it's null terminated
}

// Report a warning
void reportWarning(const char* msg) {
    strncpy(warning_message, msg, MAXMSGLEN); // Copy up to MAXMSGLEN characters
    warning_message[MAXMSGLEN] = '\0';    // make sure it's null terminated
}

const char* lastErrorMessage() {
    return error_message;
}

const char* lastWarningMessage() {
    return warning_message;
}