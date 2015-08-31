/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief A generic bus node with multiple users.
 *
 * Requires YARP.
 *
 * This file is part of the openBuildNet example simulating distributed optimization algorithms for collective tracking.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <cstdlib>      // String utilities (atoi,...)
#include <cmath>
#include <memory>
#include <vector>

#include <obnnode.h>


#ifndef OBNSIM_COMM_YARP
#error This test requires YARP to run
#endif


/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNSIM_COMM_YARP: if YARP is supported for communication.
 - OBNSIM_SMN_COMM_MQTT: if MQTT is supported for communication.
 */

using namespace OBNnode;

// The main update that receives requests from users and sends the aggregate request to the grid
#define MAIN_UPDATE 0

// The update that gets the values returned from the grid node, disaggregates them, and forwards them to the user nodes.
#define FEEDBACK_UPDATE 1


/* The controller node class */
class ConstBus: public YarpNode {
private:
    // The constant vector to be output, at least of length 4
    Eigen::VectorXd m_const_value;
    
    // One input: V from the grid node, a vector of 4 doubles
    YarpInput<OBN_PB, obn_vector_fixed<double, 4> > m_input_grid;
    
    // Output: V from bus to grid, vector of doubles, at least 4
    YarpOutput<OBN_PB, obn_vector<double> > m_output_grid;
public:
    ConstBus(const char* t_name, const char* t_workspace, const std::vector<double>& t_values):
    YarpNode(t_name, t_workspace), m_input_grid("VfromGrid"), m_output_grid("VtoGrid")
    {
        assert(t_values.size() >= 4);
        
        // Copy the values to the internal vector
        auto n = t_values.size();
        m_const_value.resize(n);
        for (auto i = 0; i < n; ++i) {
            m_const_value[i] = t_values[i];
        }
    }
    
    /* Add ports to node, hardware components may be started, etc. */
    bool initialize() {
        bool success;
        
        // Add the ports to the node
        if (!(success = addInput(&m_input_grid))) {
            std::cerr << "Error while adding input: " << m_input_grid.getPortName() << std::endl;
        }
        
        if (success && !(success = addOutput(&m_output_grid))) {
            std::cerr << "Error while adding output: " << m_output_grid.getPortName() << std::endl;
        }
        
        // Add the updates
        success = success && (addUpdate(MAIN_UPDATE, std::bind(&ConstBus::doMainUpdate, this)) >= 0);
        
        // Open the SMN port
        success = success && openSMNPort();
        
        return success;
    }
    
    void doMainUpdate() {
        // Send the constant values to the grid node
        m_output_grid = m_const_value;
        
        //std::cout << "At " << _current_sim_time << " UPDATE_Y" << std::endl;
    }
    
    /* This node has no state update */
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual void onInitialization() {
        // Initial state and output
        m_output_grid = m_const_value;
        
        std::cout << "At " << _current_sim_time << " INIT" << std::endl;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() {
        std::cout << "At " << _current_sim_time << " TERMINATED" << std::endl;
    }
    
    /* There are other callbacks for reporting errors, etc. */
};


// The generic bus node, with at least one user attached to it
class GenericBus: public YarpNode {
private:
    unsigned int m_nUsers;
    
    using UserInputType = YarpInput< OBN_PB, obn_vector<double> >;
    
    // Vector of inputs from user nodes.
    std::vector< std::unique_ptr<UserInputType> > m_input_users;
    
    // Output: V from bus to grid, vector of doubles
    YarpOutput<OBN_PB, obn_vector<double> > m_output_grid;
    
    // Input: V from the grid node, a vector of 4 doubles
    YarpInput<OBN_PB, obn_vector_fixed<double, 4> > m_input_grid;
    
    using UserOutputType = YarpOutput< OBN_PB, obn_vector_fixed<double, 4> >;
    
    // Vector of outputs to user nodes.
    std::vector< std::unique_ptr<UserOutputType> > m_output_users;
    
    // Memory variables to store the requested values P's and Q's of users
    struct VWithLimits {
        double value, minvalue, maxmindiff;
    };
    std::vector<VWithLimits> m_Ps, m_Qs;
    
    // The number of NaN values in Ps and Qs
    int m_nPNaNs, m_nQNaNs;
    
    // Sum of the non-NaN P/Q values
    double m_Psum, m_Qsum;
    
    // Sums of the Pmin, Pmax, Qmin, Qmax of those Ps and Qs having NaN values
    double m_Pmin, m_Pmax, m_Qmin, m_Qmax;
    
    /** Process the input from a user node.
     
     This function processes the input values from user node #i (>=1), where the current aggregate E value is t_curE and the current aggregate Theta value is t_curTheta.
     This function will save the P and Q values to m_Ps and m_Qs, and update t_curE and t_curTheta.
     \return true if successful, false if there was an error (a message will be displayed).
     */
    bool processUserInput(int idx, double& t_curE, double& t_curTheta);
    
    void doMainUpdate() {
        double curE = NAN, curTheta = NAN;  // Current aggregate values of E and Theta
        
        // Reset variables used in computation
        m_nPNaNs = m_nQNaNs = 0;
        m_Psum = m_Qsum = 0.0;
        m_Pmin = m_Pmax = m_Qmin = m_Qmax = 0.0;
        
        // Get and process the requests from users
        for (auto i = 1; i <= m_nUsers; i++) {
            if (!processUserInput(i, curE, curTheta)) {
                std::cout << "[ERROR] At " << currentSimulationTime() << " encountered an input error; continue with previous values.\n";
                return;
            }
        }

        // Send out the aggregate values
        auto& output = (*m_output_grid);
        bool bPNaN = m_nPNaNs>0, bQNaN = m_nQNaNs>0;
        output.resize(4 + (bPNaN?2:0) + (bQNaN?2:0));
        
        output[0] = bPNaN?NAN:m_Psum;   // P
        output[1] = bQNaN?NAN:m_Qsum;   // Q
        output[2] = curE;               // E
        output[3] = curTheta;           // Theta
        
        // P limits
        if (bPNaN) {
            output[4] = m_Psum + m_Pmin;
            output[5] = m_Psum + m_Pmax;
        }
        
        // Q limits
        if (bQNaN) {
            output[4 + (bPNaN?2:0)] = m_Qsum + m_Qmin;
            output[5 + (bPNaN?2:0)] = m_Qsum + m_Qmax;
        }
        
        //std::cout << "At " << _current_sim_time << " UPDATE_Y" << std::endl;
    }
    
    void doFeedbackUpdate() {
        auto result = m_input_grid();
        
        // Ratios for distributing P/Q to flexible users (i.e. NaN users)
        double Pr = m_nPNaNs>0?((result[0] - m_Psum) - m_Pmin)/(m_Pmax - m_Pmin):0;
        double Qr = m_nQNaNs>0?((result[1] - m_Qsum) - m_Qmin)/(m_Qmax - m_Qmin):0;
        
        // Disaggregate the result to individual users
        for (auto i = 0; i < m_nUsers; ++i) {
            auto& user_v = **m_output_users[i];
            user_v[0] = std::isfinite(m_Ps[i].value)?m_Ps[i].value:(m_Ps[i].maxmindiff*Pr + m_Ps[i].minvalue);
            user_v[1] = std::isfinite(m_Qs[i].value)?m_Qs[i].value:(m_Qs[i].maxmindiff*Qr + m_Qs[i].minvalue);
            user_v[2] = result[2];
            user_v[3] = result[3];
        }
        
        //std::cout << "At " << _current_sim_time << " UPDATE_Y" << std::endl;
    }
    
public:
    /** Construct the bus node.
     
     \param t_name Name of the bus node
     \param t_workspace The workspace name
     \param t_nUsers Number of users (>= 1)
     */
    GenericBus(const char* t_name, const char* t_workspace, unsigned int t_nUsers):
    YarpNode(t_name, t_workspace), m_nUsers(t_nUsers),
    m_output_grid("VtoGrid"), m_input_grid("VfromGrid")
    {
        assert(t_nUsers > 0);
        m_input_users.reserve(m_nUsers);
        m_output_users.reserve(m_nUsers);
    }
    
    /* Add ports to node, register updates, etc. */
    bool initialize() {
        bool success;
        
        // Add the ports to the node
        if (!(success = addInput(&m_input_grid))) {
            std::cerr << "Error while adding input: " << m_input_grid.getPortName() << std::endl;
        }
        
        if (success && !(success = addOutput(&m_output_grid))) {
            std::cerr << "Error while adding output: " << m_output_grid.getPortName() << std::endl;
        }
        
        if (success) {
            for (auto i = 1; i <= m_nUsers; ++i) {
                m_input_users.emplace_back(new UserInputType("Vfrom" + std::to_string(i)));
                auto p = m_input_users.back().get();
                if (!(success = addInput(p))) {
                    std::cerr << "Error while adding input: " << p->getPortName() << std::endl;
                    break;
                }
            }
        }
        
        if (success) {
            for (auto i = 1; i <= m_nUsers; ++i) {
                m_output_users.emplace_back(new UserOutputType("Vto" + std::to_string(i)));
                auto p = m_output_users.back().get();
                if (!(success = addOutput(p))) {
                    std::cerr << "Error while adding output: " << p->getPortName() << std::endl;
                    break;
                }
            }
        }
        
        // Add the updates
        success = success && (addUpdate(MAIN_UPDATE, std::bind(&GenericBus::doMainUpdate, this)) >= 0);
        success = success && (addUpdate(FEEDBACK_UPDATE, std::bind(&GenericBus::doFeedbackUpdate, this)) >= 0);
        
        // Open the SMN port
        success = success && openSMNPort();
        
        return success;
    }
    
    /* This node has no state update */
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual void onInitialization() {
        // Clear the memory for individual P's and Q's
        VWithLimits v{0.0, 0.0, 0.0};
        m_Ps.assign(m_nUsers, v);
        m_Qs.assign(m_nUsers, v);
        
        std::cout << "At " << currentSimulationTime() << " INIT" << std::endl;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() {
        std::cout << "At " << _current_sim_time << " TERMINATED" << std::endl;
    }
    
    /* There are other callbacks for reporting errors, etc. */
};


bool GenericBus::processUserInput(int idx, double& t_curE, double& t_curTheta) {
    int i = idx - 1;
    const auto& values = (*m_input_users[i])();
    
    auto n = values.size();
    
    // At least P is given
    if (n < 1) {
        std::cout << "At " << currentSimulationTime() << " User#" << idx << " gave an empty vector.\n";
        return false;
    }
    
    double P = values[0];
    
    // Extract Q, E, Theta
    double Q = NAN, E = (n>2?values[2]:NAN), Theta = (n>3?values[3]:NAN);
    if (n > 1) {
        // Q value
        Q = values[1];
        m_Qs[i].value = Q;
    }
    
    // Extract limits if necessary
    bool isPNaN = !std::isfinite(P);
    if (isPNaN) {
        m_Ps[i].value = NAN;
        m_nPNaNs++;
        
        if (n < 6) {
            // Not enough length for P limits
            std::cout << "At " << currentSimulationTime() << " User#" << idx << " failed to give P limits.\n";
            return false;
        }
        double Pmin = values[4], Pmax = values[5];
        if (!std::isfinite(Pmin) || !std::isfinite(Pmax) || Pmin >= Pmax) {
            // Invalid Pmin and Pmax
            std::cout << "At " << currentSimulationTime() << " User#" << idx << " gave invalid P limits.\n";
            return false;
        }
        m_Ps[i].minvalue = Pmin;
        m_Pmin += Pmin;
        m_Ps[i].maxmindiff = Pmax - Pmin;
        m_Pmax += Pmax;
    } else {
        m_Ps[i].value = P;
        m_Psum += P;
    }
    
    if (!std::isfinite(Q)) {
        m_Qs[i].value = NAN;
        m_nQNaNs++;
        
        if (n < 6 || (isPNaN && n < 8)) {
            // Not enough length for Q limits
            std::cout << "At " << currentSimulationTime() << " User#" << idx << " failed to give Q limits.\n";
            return false;
        }
        double Qmin = values[isPNaN?6:4], Qmax = values[isPNaN?7:5];
        if (!std::isfinite(Qmin) || !std::isfinite(Qmax) || Qmin >= Qmax) {
            // Invalid Qmin and Qmax
            std::cout << "At " << currentSimulationTime() << " User#" << idx << " gave invalid Q limits.\n";
            return false;
        }
        m_Qs[i].minvalue = Qmin;
        m_Qmin += Qmin;
        m_Qs[i].maxmindiff = Qmax - Qmin;
        m_Qmax += Qmax;
    } else {
        m_Qs[i].value = Q;
        m_Qsum += Q;
    }
    
    // Update current E and theta
    if (std::isfinite(E)) {
        if (std::isfinite(t_curE)) {
            // Check if E is similar to current E
            if (std::abs(E - t_curE) > 1e-6) {
                // E is different
                std::cout << "At " << currentSimulationTime() << " User#" << idx << " gave E incompatible with other users.\n";
                return false;
            }
            // else, because they are similar, we keep the current value
        } else {
            // The first E that is not NaN --> save it
            t_curE = E;
        }
    }
    
    if (std::isfinite(Theta)) {
        if (std::isfinite(t_curTheta)) {
            // Check if Theta is similar to current Theta
            if (std::abs(Theta - t_curTheta) > 1e-6) {
                // Theta is different
                std::cout << "At " << currentSimulationTime() << " User#" << idx << " gave theta incompatible with other users.\n";
                return false;
            }
            // else, because they are similar, we keep the current value
        } else {
            // The first Theta that is not NaN --> save it
            t_curTheta = Theta;
        }
    }
    
    return true;
}

void show_usage(char *prog) {
    std::cout << "USAGE:" << std::endl <<
    prog << " node_name nUsers [P Q E theta ...] [--workspace <workspace_name>]" << std::endl <<
    R"args(
where
   node_name is the name of the bus node
   nUsers is the number of user nodes attached to this bus node (must be a non-negative integer)
   [P Q E theta ...] are optional values in the case when nUsers = 0 (see below)
   workspace_name is the name of the simulation workspace (default: "powernet").

If nUsers = 0, the bus node will be a "constant" bus, which always sends constant values P, Q, E, theta, ... to the grid.
In this case, the four values P, Q, E, theta are required, but optional extra values can be specified after them in the command line.
All real values can be either a finite real number (e.g. 0, 0.5) or one of the words "NaN", "Inf", "-Inf".
If an invalid number is given, 0.0 will be used instead.

If nUsers > 0, it is the number of user nodes attached to this bus (numberred from 1 to nUsers).
Appropriate input and output ports for each user will be created automatically.
In this case, the values P, Q, E, theta, ..., if supplied, will not be used.
)args";
}

void show_banner() {
    std::cout << R"banner(
+-------------------------------------------------------------------+
|                         Generic Bus Node                          |
|        Part of the Power Network library for openBuildNet.        |
+-------------------------------------------------------------------+
        
This program is part of the openBuildNet framework developed at EPFL.

)banner";
}

int main(int argc, char **argv) {
    show_banner();
    
    //////////////////////////
    // Extract the arguments
    //////////////////////////
    
    const char *node_name = nullptr;        // The node's name
    int nUsers = -1;
    std::vector<double> const_values;       // The constants sent to the grid node
    
    const char *workspace_name = nullptr;     // The workspace name
    const std::string WORKSPACE_OPTION("--workspace");
    
    int k = 1;
    while (k < argc) {
        if (std::strlen(argv[k]) > 2 && argv[k][0] == '-' && argv[k][1] == '-') {
            // This is an option
            if (WORKSPACE_OPTION.compare(argv[k]) == 0) {
                // Save the workspace name
                if (workspace_name) {
                    // Workspace already set
                    std::cout << "Workspace name has already been set." << std::endl;
                    show_usage(argv[0]);
                    exit(1);
                }
                
                if (++k < argc) {
                    workspace_name = argv[k++];
                } else {
                    std::cout << "Workspace name must be given after the option --workspace." << std::endl;
                    show_usage(argv[0]);
                    exit(1);
                }
            } else {
                std::cout << "Unrecognized options: " << argv[k] << std::endl;
                show_usage(argv[0]);
                exit(1);
            }
        } else {
            if (node_name == nullptr) {
                // This is the node's name
                node_name = argv[k++];
            } else if (nUsers < 0) {
                // This is the number of user nodes
                nUsers = atoi(argv[k++]);
                if (nUsers < 0) {
                    std::cout << "The number of users must be a non-negative integer.\n";
                    show_usage(argv[0]);
                    exit(1);
                }
            } else {
                // These will be the constant values sent to the grid node (when nUsers = 0)
                const char *p = argv[k++];
                char *pend;
                double d = std::strtod(p, &pend);
                if (errno == ERANGE){
                    std::cout << "Range error while converting to number for argument #" << (k-1);
                    errno = 0;
                    show_usage(argv[0]);
                    exit(1);
                } else if (pend == p) {
                    std::cout << "Argument #" << (k-1) << " is an invalid real number.";
                    show_usage(argv[0]);
                    exit(1);
                } else {
                    // save the number
                    const_values.push_back(d);
                }
            }
        }
    }

    if (node_name == nullptr || nUsers < 0) {
        // Not enough input args
        std::cout << "Not enough input arguments." << std::endl;
        show_usage(argv[0]);
        exit(1);
    }
    
    if (workspace_name == nullptr) {
        workspace_name = "powernet";
    }
    
    if (nUsers == 0 && const_values.size() < 4) {
        std::cout << "Not enough constant values (P, Q, E, theta) for a constant bus (nUsers = 0)." << std::endl;
        show_usage(argv[0]);
        exit(1);
    }
    
    // Display info
    if (nUsers == 0) {
        std::cout << "This is constant bus node named \"" << node_name
        << "\" in the workspace \"" << workspace_name << "\"\n"
        << "The constant values to be sent are: " << const_values[0];
        
        for (auto i = 1; i < const_values.size(); ++i) {
            std::cout << ' ' << const_values[i];
        }
    } else {
        std::cout << "This is a bus node named \"" << node_name
        << "\" in the workspace \"" << workspace_name << "\"\n"
        << "There are " << nUsers << " users attached to this bus.";
        if (!const_values.empty()) {
            std::cout << "\nWarning: there are constant values provided but they will not be used.";
        }
    }
    std::cout << "\n\n";
    
    // Create the node
    std::unique_ptr<YarpNode> pbus;
    bool success;
    if (nUsers > 0) {
        GenericBus *p = new GenericBus(node_name, workspace_name, nUsers);
        success = p->initialize();
        pbus.reset(p);
    } else {
        ConstBus *p = new ConstBus(node_name, workspace_name, const_values);
        success = p->initialize();
        pbus.reset(p);
    }
    if (!success) {
        std::cout << "There was/were error(s) while initializing the node.\n";
        return 2;
    }

    
    // Here we will not connect the node to the GC, let the SMN do it
    std::cout << "Starting simulation of node " << pbus->name() << std::endl;
    
    pbus->run();
    
    std::cout << "Simulation finished. Goodbye!" << std::endl;
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    google::protobuf::ShutdownProtobufLibrary();
    
    return pbus->hasError()?3:0;
}
