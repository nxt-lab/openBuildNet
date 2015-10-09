/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Meta include file for creating a node in C++ (node.cpp)
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifdef OBNNODE_COMM_YARP
#include <obnnode_yarpport.h>
#endif

#ifdef OBNNODE_COMM_MQTT
#include <obnnode_mqttport.h>
#include <obnnode_mqttnode.h>
#endif
