/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief All bindings with Chaiscript should be put in this file to allow faster compilation.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#define __STDC_WANT_LIB_EXT1__ 1
#include <time.h>   // Time utilities, with extension

#include "chaiscript_bindings.h"
#include <chaiscript/chaiscript.hpp>
#include <chaiscript/dispatchkit/bootstrap.hpp>

#include <chaiscriptextras/math.hpp>

#ifdef OBNNODE_COMM_MQTT
#include "nodechai_mqtt.h"
#endif

#include "chaiscript_io.h"

using namespace chaiscript;

Eigen::IOFormat EigenMatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");

typedef std::vector<chaiscript::Boxed_Value> TChaiVector;   ///< The C++ type of a Chaiscript's vector object;

// Eztracts a time_t POSIX time value to Chaiscript values
// Returns a vector of numbers [year, month, day, weekday, hour, minute, second]
// where weekday is between 0 and 6 with 0 = Sunday.
TChaiVector convert_posixtime_chaiscript(std::time_t t) {
#ifdef __STDC_LIB_EXT1__
    std::tm buf;
    std::tm* ptm = std::localtime_s(&t, &buf);
#else
    std::tm* ptm = std::localtime(&t);
#endif
    if (ptm) {
        TChaiVector v;
        v.emplace_back(ptm->tm_year);
        v.emplace_back(ptm->tm_mon);
        v.emplace_back(ptm->tm_mday);
        v.emplace_back(ptm->tm_wday);
        v.emplace_back(ptm->tm_hour);
        v.emplace_back(ptm->tm_min);
        v.emplace_back(ptm->tm_sec);
        return v;
    } else {
        throw NodeChai::nodechai_exception("Failed to convert wallclock time.");
    }
}

namespace NodeChai {
    
    /* All the global variables. */
    GlobalVariables global_variables;
    
    chaiscript::ModulePtr nodechai_api_utils_math(chaiscript::ModulePtr m)
    {
        using namespace chaiscript::extras::math;
        
        // TRIG FUNCTIONS
        cos<double, double>(m);
        cos<float, float>(m);
        cos<long double, long double>(m);
        
        sin<double, double>(m);
        sin<float, float>(m);
        sin<long double, long double>(m);
        
        tan<double, double>(m);
        tan<float, float>(m);
        tan<long double, long double>(m);
        
        acos<double, double>(m);
        acos<float, float>(m);
        acos<long double, long double>(m);
        
        asin<double, double>(m);
        asin<float, float>(m);
        asin<long double, long double>(m);
        
        atan<double, double>(m);
        atan<float, float>(m);
        atan<long double, long double>(m);
        
        atan2<double, double, double>(m);
        atan2<float, float, float>(m);
        atan2<long double, long double, long double>(m);
        
        // HYPERBOLIC FUNCTIONS
        cosh<double, double>(m);
        cosh<float, float>(m);
        cosh<long double, long double>(m);
        
        sinh<double, double>(m);
        sinh<float, float>(m);
        sinh<long double, long double>(m);
        
        tanh<double, double>(m);
        tanh<float, float>(m);
        tanh<long double, long double>(m);
        
        acosh<double, double>(m);
        acosh<float, float>(m);
        acosh<long double, long double>(m);
        
        asinh<double, double>(m);
        asinh<float, float>(m);
        asinh<long double, long double>(m);
        
        atanh<double, double>(m);
        atanh<float, float>(m);
        atanh<long double, long double>(m);
        
        // EXPONENTIAL AND LOGARITHMIC FUNCTIONS
        exp<double, double>(m);
        exp<float, float>(m);
        exp<long double, long double>(m);
        
        frexp<double, double, int *>(m);
        frexp<float, float, int *>(m);
        frexp<long double, long double, int *>(m);
        
        ldexp<double, double, int>(m);
        ldexp<float, float, int>(m);
        ldexp<long double, long double, int>(m);
        
        log<double, double>(m);
        log<float, float>(m);
        log<long double, long double>(m);
        
        log10<double, double>(m);
        log10<float, float>(m);
        log10<long double, long double>(m);
        
        modf<double, double, double *>(m);
        modf<float, float, float *>(m);
        modf<long double, long double, long double *>(m);
        
        exp2<double, double>(m);
        exp2<float, float>(m);
        exp2<long double, long double>(m);
        
        expm1<double, double>(m);
        expm1<float, float>(m);
        expm1<long double, long double>(m);
        
        ilogb<int, double>(m);
        ilogb<int, float>(m);
        ilogb<int, long double>(m);
        
        log1p<double, double>(m);
        log1p<float, float>(m);
        log1p<long double, long double>(m);
        
        log2<double, double>(m);
        log2<float, float>(m);
        log2<long double, long double>(m);
        
        logb<double, double>(m);
        logb<float, float>(m);
        logb<long double, long double>(m);
        
        scalbn<double, double, int>(m);
        scalbn<float, float, int>(m);
        scalbn<long double, long double, int>(m);
        
        scalbln<double, double, long int>(m);
        scalbln<float, float, long int>(m);
        scalbln<long double, long double, long int>(m);
        
        // POWER FUNCTIONS
        pow<double, double, double>(m);
        pow<float, float, float>(m);
        pow<long double, long double, long double>(m);
        
        sqrt<double, double>(m);
        sqrt<float, float>(m);
        sqrt<long double, long double>(m);
        
        cbrt<double, double>(m);
        cbrt<float, float>(m);
        cbrt<long double, long double>(m);
        
        hypot<double, double, double>(m);
        hypot<float, float, float>(m);
        hypot<long double, long double, long double>(m);
        
        // ERROR AND GAMMA FUNCTIONS
        erf<double, double>(m);
        erf<float, float>(m);
        erf<long double, long double>(m);
        
        erfc<double, double>(m);
        erfc<float, float>(m);
        erfc<long double, long double>(m);
        
        tgamma<double, double>(m);
        tgamma<float, float>(m);
        tgamma<long double, long double>(m);
        
        lgamma<double, double>(m);
        lgamma<float, float>(m);
        lgamma<long double, long double>(m);
        
        // ROUNDING AND REMAINDER FUNCTIONS
        ceil<double, double>(m);
        ceil<float, float>(m);
        ceil<long double, long double>(m);
        
        floor<double, double>(m);
        floor<float, float>(m);
        floor<long double, long double>(m);
        
        fmod<double, double, double>(m);
        fmod<float, float, float>(m);
        fmod<long double, long double, long double>(m);
        
        trunc<double, double>(m);
        trunc<float, float>(m);
        trunc<long double, long double>(m);
        
        round<double, double>(m);
        round<float, float>(m);
        round<long double, long double>(m);
        
        lround<long int, double>(m);
        lround<long int, float>(m);
        lround<long int, long double>(m);
        
        // long long ints do not work
        llround<long long int, double>(m);
        llround<long long int, float>(m);
        llround<long long int, long double>(m);
        
        rint<double, double>(m);
        rint<float, float>(m);
        rint<long double, long double>(m);
        
        lrint<long int, double>(m);
        lrint<long int, float>(m);
        lrint<long int, long double>(m);
        
        // long long ints do not work
        llrint<long long int, double>(m);
        llrint<long long int, float>(m);
        llrint<long long int, long double>(m);
        
        nearbyint<double, double>(m);
        nearbyint<float, float>(m);
        nearbyint<long double, long double>(m);
        
        remainder<double, double, double>(m);
        remainder<float, float, float>(m);
        remainder<long double, long double, long double>(m);
        
        remquo<double, double, double, int *>(m);
        remquo<float, float, float, int *>(m);
        remquo<long double, long double, long double, int *>(m);
        
        // FLOATING-POINT MANIPULATION FUNCTIONS
        copysign<double, double, double>(m);
        copysign<float, float, float>(m);
        copysign<long double, long double, long double>(m);
        
        nan<double, const char*>(m);
        
        nextafter<double, double, double>(m);
        nextafter<float, float, float>(m);
        nextafter<long double, long double, long double>(m);
        
        nexttoward<double, double, long double>(m);
        nexttoward<float, float, long double>(m);
        nexttoward<long double, long double, long double>(m);
        
        // MINIMUM, MAXIMUM, DIFFERENCE FUNCTIONS
        fdim<double, double, double>(m);
        fdim<float, float, float>(m);
        fdim<long double, long double, long double>(m);
        
        fmax<double, double, double>(m);
        fmax<float, float, float>(m);
        fmax<long double, long double, long double>(m);
        
        fmin<double, double, double>(m);
        fmin<float, float, float>(m);
        fmin<long double, long double, long double>(m);
        
        // OTHER FUNCTIONS
        fabs<double, double>(m);
        fabs<float, float>(m);
        fabs<long double, long double>(m);
        
        abs<double, double>(m);
        abs<float, float>(m);
        abs<long double, long double>(m);
        
        fma<double, double, double, double>(m);
        fma<float, float, float, float>(m);
        fma<long double, long double, long double, long double>(m);
        
        // CLASSIFICATION FUNCTIONS
        fpclassify<int, float>(m);
        fpclassify<int, double>(m);
        fpclassify<int, long double>(m);
        
        isfinite<bool, float>(m);
        isfinite<bool, double>(m);
        isfinite<bool, long double>(m);
        
        isinf<bool, float>(m);
        isinf<bool, double>(m);
        isinf<bool, long double>(m);
        
        isnan<bool, float>(m);
        isnan<bool, double>(m);
        isnan<bool, long double>(m);
        
        isnormal<bool, float>(m);
        isnormal<bool, double>(m);
        isnormal<bool, long double>(m);
        
        signbit<bool, float>(m);
        signbit<bool, double>(m);
        signbit<bool, long double>(m);
        
        // COMPARISON FUNCTIONS
        isgreater<bool, double, double>(m);
        isgreater<bool, float, float>(m);
        isgreater<bool, long double, long double>(m);
        
        isgreaterequal<bool, double, double>(m);
        isgreaterequal<bool, float, float>(m);
        isgreaterequal<bool, long double, long double>(m);
        
        isless<bool, double, double>(m);
        isless<bool, float, float>(m);
        isless<bool, long double, long double>(m);
        
        islessequal<bool, double, double>(m);
        islessequal<bool, float, float>(m);
        islessequal<bool, long double, long double>(m);
        
        islessgreater<bool, double, double>(m);
        islessgreater<bool, float, float>(m);
        islessgreater<bool, long double, long double>(m);
        
        isunordered<bool, double, double>(m);
        isunordered<bool, float, float>(m);
        isunordered<bool, long double, long double>(m);
        
        return m;
    }
    
    /** API bindings for general IO utility functions. */
    std::shared_ptr<chaiscript::Module> nodechai_api_utils_io(std::shared_ptr<chaiscript::Module> m) {
        m->add(fun(&NodeChai::extras::load_csv_into_chai), "load_csv_file");
        NodeChai::extras::bind_CSVFileWriter(m);
        
        return m;
    }
    
    /** Add the API bindings after a node has been created, e.g. to create inputs, outputs, etc. */
    std::shared_ptr<chaiscript::Module> create_bindings_after_node(std::shared_ptr<chaiscript::Module> m = std::make_shared<chaiscript::Module>()) {
        if (!global_variables.node_created || !global_variables.node_factory) {
            // Node not yet created
            throw nodechai_exception("The node has not been created.");
        }
        
        // Create the specific bindings
        global_variables.node_factory->create_bindings(m);
        
        // Create the common bindings
        NodeFactory* pNF = global_variables.node_factory.get();
        OBNnode::NodeBase* pNode = pNF->get_node_object();
        if (!pNode) {
            throw nodechai_exception("Internal error: node object is null.");
        }
        
        m->add(fun(&NodeFactory::callback_init, pNF), "callback_init");
        m->add(fun(&NodeFactory::callback_term, pNF), "callback_term");
        m->add(fun(&NodeFactory::callback_x, pNF), "callback_x");
        m->add(fun(&NodeFactory::callback_y, pNF), "callback_y");
        
        // General methods to control the simulation
        m->add(fun(&OBNnode::NodeBase::stopSimulation, pNode), "stop_simulation");
        m->add(fun(&OBNnode::NodeBase::requestStopSimulation, pNode), "request_stop_simulation");
        
        // Functions to get the current simulation time in different units
        m->add(fun(&OBNnode::NodeBase::currentSimulationTime<std::chrono::seconds>, pNode), "current_time_s"); // in seconds
        m->add(fun(&OBNnode::NodeBase::currentSimulationTime<std::chrono::minutes>, pNode), "current_time_m"); // in minutes
        m->add(fun(&OBNnode::NodeBase::currentSimulationTime<std::chrono::hours>, pNode), "current_time_h"); // in hours
        
        // Get the current wallclock time: returns a vector of numbers [year, month, day, weekday, hour, minute, second]
        // where weekday is between 0 and 6 with 0 = Sunday; year is since 1900; month is [0,11]; day is [1,31]; hour is [0,23]; minute is [0,59]; second is [0,60].
        m->add(fun([pNode](){ return convert_posixtime_chaiscript(pNode->currentWallClockTime()); }), "current_wallclock");

        return m;
    }
    
    /** Actually create the node. */
    void create_node() {
        if (global_variables.node_created || global_variables.node_factory) {
            // Node is already created
            throw nodechai_exception("Node has already been created; only one node is allowed.");
        }
        if (global_variables.node_name.empty() || !OBNsim::Utils::isValidNodeName(global_variables.node_name)) {
            throw nodechai_exception("Node name must be set and valid.");
        }
        if (!global_variables.chaiscript_engine) {
            throw nodechai_exception("Internal error: Chaiscript engine has not been initialized.");
        }

        // Create the factory object based on the chosen comm protocol
        switch (global_variables.comm_protocol) {
            case OBNnode::COMM_MQTT:
#ifdef OBNNODE_COMM_MQTT
                global_variables.node_factory.reset(new NodeFactoryMQTT(global_variables.mqtt_server));
#else
                throw nodechai_exception("MQTT is not supported.");
#endif
                break;
                
            default:
                throw nodechai_exception("Unsupported communication protocol.");
        }
        if (!global_variables.node_factory) {
            throw nodechai_exception("Failed to create the factory for the chosen communication protocol.");
        }
        
        // Actually create the node using the factory
        global_variables.node_created = global_variables.node_factory->create_node(global_variables.node_name, global_variables.workspace);
        if (!global_variables.node_created) {
            throw nodechai_exception("Failed to create the node.");
        }
        
        // Bind the API for running the node
        global_variables.chaiscript_engine->add(create_bindings_after_node());
    }
    
    std::shared_ptr<chaiscript::Module> create_bindings_before_node(std::shared_ptr<chaiscript::Module> m) {
        ////// Common, generic purpose functions
        nodechai_api_utils_math(m);
        nodechai_api_eigen(m);
        nodechai_api_utils_io(m);
        
        ////// Functions to set properties before creating a node
        // Some of these can only be used before the node is created
        m->add(fun([](const std::string& s) {
            if (global_variables.node_created) {
                throw nodechai_exception("set_node_name can only be called before a node is created.");
            }
            global_variables.node_name = s;
        }), "set_node_name");
        
        m->add(fun([](const std::string& s) {
            if (global_variables.node_created) {
                throw nodechai_exception("set_workspace can only be called before a node is created.");
            }
            global_variables.workspace = s;
        }), "set_workspace");
        
        m->add(fun([]() {
            if (global_variables.node_created) {
                throw nodechai_exception("set_comm_yarp can only be called before a node is created.");
            }
            global_variables.comm_protocol = OBNnode::COMM_YARP;
        }), "set_comm_yarp");
        
        m->add(fun([](const std::string& s) {
            if (global_variables.node_created) {
                throw nodechai_exception("set_comm_mqtt can only be called before a node is created.");
            }
            global_variables.comm_protocol = OBNnode::COMM_MQTT;
            global_variables.mqtt_server = s;
        }), "set_comm_mqtt");
        
        m->add(fun([](const double to) { global_variables.timeout = to; }), "set_timeout");
        
        ////// Function to actually create the node
        m->add(fun(&create_node), "create_node");
        
        return m;
    }
    
    
    
#ifdef OBNNODE_COMM_MQTT
    template <typename T>
    void bindings_for_nonstrict_input(const char* TNAME, std::shared_ptr<chaiscript::Module> m) {
        m->add(user_type<T>(), TNAME);
        m->add(fun(&T::isValuePending), "pending");
        m->add(fun([](T& p, const std::function<void ()>& f) { p.setMsgRcvCallback(f, true); }), "callback_msgrcv");
        m->add(fun(&T::clearMsgRcvCallback), "clear_callback_msgrcv");
    }
    
    template <typename T>
    void bindings_for_strict_input(const char* TNAME, std::shared_ptr<chaiscript::Module> m) {
        m->add(user_type<T>(), TNAME);
        m->add(fun(&T::size), "size");
        m->add(fun(&T::isValuePending), "pending");
        m->add(fun([](T& p, const std::function<void ()>& f) { p.setMsgRcvCallback(f, true); }), "callback_msgrcv");
        m->add(fun(&T::clearMsgRcvCallback), "clear_callback_msgrcv");
    }
    
    std::shared_ptr<chaiscript::Module> NodeFactoryMQTT::create_bindings(std::shared_ptr<chaiscript::Module> m) {
        
        ////////////////////////////////////////////////////////////////
        // Add the input port types and their methods to access
        ////////////////////////////////////////////////////////////////
        
        bindings_for_nonstrict_input<NodeFactoryMQTT::InputScalarDouble>("InputScalarDouble", m);
        m->add(fun(&NodeFactoryMQTT::InputScalarDouble::get), "get");
        
        bindings_for_strict_input<NodeFactoryMQTT::InputScalarDoubleStrict>("InputScalarDoubleStrict", m);
        m->add(fun(&NodeFactoryMQTT::InputScalarDoubleStrict::pop), "pop");
        
        m->add(user_type<NodeFactoryMQTT::OutputScalarDouble>(), "OutputScalarDouble");
        m->add(fun(&NodeFactoryMQTT::OutputScalarDouble::operator()), "get");
        m->add(fun([](NodeFactoryMQTT::OutputScalarDouble& p, const double v) { p = v; }), "set");
        
        bindings_for_nonstrict_input<NodeFactoryMQTT::InputVectorDouble>("InputVectorDouble", m);
        m->add(fun([](NodeFactoryMQTT::InputVectorDouble& p) { return Eigen::MatrixXd(p.get()); }), "get");
        
        bindings_for_strict_input<NodeFactoryMQTT::InputVectorDoubleStrict>("InputVectorDoubleStrict", m);
        m->add(fun([](NodeFactoryMQTT::InputVectorDoubleStrict& p) { return Eigen::MatrixXd(*p.pop()); }), "pop");
        
        m->add(user_type<NodeFactoryMQTT::OutputVectorDouble>(), "OutputVectorDouble");
        m->add(fun([](NodeFactoryMQTT::OutputVectorDouble& p) { return Eigen::MatrixXd(p()); }), "get");
        // Write to vector output: the value must be a row or column vector
        m->add(fun([](NodeFactoryMQTT::OutputVectorDouble& p, const Eigen::MatrixXd& v) {
            if (v.cols() == 1) {
                p = Eigen::VectorXd(v);
            } else if (v.rows() == 1) {
                p = Eigen::VectorXd(v.transpose());
            } else {
                throw nodechai_exception(std::string("Vector output port ") + p.getPortName() + " expects a vector but got a matrix.");
            }
        }), "set");
        
        bindings_for_nonstrict_input<NodeFactoryMQTT::InputMatrixDouble>("InputMatrixDouble", m);
        m->add(fun(&NodeFactoryMQTT::InputMatrixDouble::get), "get");
        
        bindings_for_strict_input<NodeFactoryMQTT::InputMatrixDoubleStrict>("InputMatrixDoubleStrict", m);
        m->add(fun([](NodeFactoryMQTT::InputMatrixDoubleStrict& p) { return *p.pop(); }), "pop");

        m->add(user_type<NodeFactoryMQTT::OutputMatrixDouble>(), "OutputMatrixDouble");
        m->add(fun([](NodeFactoryMQTT::OutputMatrixDouble& p) { return p(); }), "get");
        m->add(fun([](NodeFactoryMQTT::OutputMatrixDouble& p, const Eigen::MatrixXd& v) { p = v; }), "set");
        
        ////////////////////////////////////////////////////////////////
        // Methods to create ports
        ////////////////////////////////////////////////////////////////
        
        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputScalarDouble>, this), "new_input_double");
        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputScalarDoubleStrict>, this), "new_input_double_strict");
        m->add(fun(&NodeFactoryMQTT::chai_create_output<OutputScalarDouble>, this), "new_output_double");

        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputVectorDouble>, this), "new_input_double_vector");
        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputVectorDoubleStrict>, this), "new_input_double_vector_strict");
        m->add(fun(&NodeFactoryMQTT::chai_create_output<OutputVectorDouble>, this), "new_output_double_vector");

        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputMatrixDouble>, this), "new_input_double_matrix");
        m->add(fun(&NodeFactoryMQTT::chai_create_input<InputMatrixDoubleStrict>, this), "new_input_double_matrix_strict");
        m->add(fun(&NodeFactoryMQTT::chai_create_output<OutputMatrixDouble>, this), "new_output_double_matrix");

        return m;
    }
#endif
}