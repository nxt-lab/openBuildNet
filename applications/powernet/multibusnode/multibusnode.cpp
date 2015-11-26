/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief A generic node that implements multiple buses, whose configurations are loaded from a CSV file.
 *
 * Requires YARP.
 *
 * This file is part of the openBuildNet example simulating distributed optimization algorithms for collective tracking.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <cstdlib>      // String utilities (atoi,...)
#include <limits>
#include <memory>
#include <vector>

#include "csvparser.h"

#include <obnnode.h>


/** The following macros are defined by CMake to indicate which libraries this SMN build supports:
 - OBNNODE_COMM_YARP: if YARP is supported for communication.
 - OBNNODE_COMM_MQTT: if MQTT is supported for communication.
 */

using namespace OBNnode;

// The main update that receives requests from users and sends the aggregate request to the grid
#define MAIN_UPDATE 0

// The update that gets the values returned from the grid node, disaggregates them, and forwards them to the user nodes.
#define FEEDBACK_UPDATE 1


/* The abstract bus node, which implements methods to calculate the values sent to the grid node and to distribute the results from the grid to the users. */
template <typename BaseCLS>
class Bus {
    std::string m_name;  // The name of the bus
public:
    Bus(const std::string &t_name): m_name(t_name) { }
    virtual ~Bus() { }  // This virtual destructor is IMPORTANT to make member variables destroyed properly.
    
    std::string get_name() const { return m_name; }
    
    virtual bool init(BaseCLS *pnode) = 0;  // Initialize the ports associated with this bus in the given node
    virtual bool calculateV2Grid(typename BaseCLS::Output2GridPort::ValueType &v, OBNnode::simtime_t t) = 0;  // calculate the values to send to the grid node
    virtual void distributeResults(const typename BaseCLS::InputFromGridPort::ValueType &v, OBNnode::simtime_t t) {}
};


/* The const bus class */
template <typename BaseCLS>
class ConstBus: public Bus<BaseCLS> {
private:
    // The constant vector to be output, at least of length 4
    typename BaseCLS::Output2GridPort::ValueType m_const_value;
public:
    /** Construct the bus object from a vector of constant values. */
    ConstBus(const std::string &name, const std::vector<double>& t_values): Bus<BaseCLS>(name)  {
        assert(t_values.size() >= 4);
        
        // Copy the values to the internal vector
        auto n = t_values.size();
        m_const_value.resize(n);
        for (auto i = 0; i < n; ++i) {
            m_const_value[i] = t_values[i];
        }
    }
    
    /* This const bus doesn't have any user inputs. */
    virtual bool init(BaseCLS *pnode) override {
        return true;
    }
    
    /* Output the constant values to the grid. */
    virtual bool calculateV2Grid(typename BaseCLS::Output2GridPort::ValueType &v, OBNnode::simtime_t t) override {
        v = m_const_value;
        return true;
    }
};


// The generic bus node, with at least one user attached to it
template <typename BaseCLS>
class GenericBus: public Bus<BaseCLS> {
private:
    unsigned int m_nUsers;
    
    // Vector of inputs from user nodes.
    std::vector< std::unique_ptr<typename BaseCLS::UserInputType> > m_input_users;
    
    // Vector of outputs to user nodes.
    std::vector< std::unique_ptr<typename BaseCLS::UserOutputType> > m_output_users;
    
    // Memory variables to store the requested values P's and Q's of users
    struct VWithLimits {
        double value, minvalue, maxvalue;
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
    bool processUserInput(int idx, double& t_curE, double& t_curTheta, OBNnode::simtime_t t);
    
    
public:
    
    // calculate the values to send to the grid node
    virtual bool calculateV2Grid(typename BaseCLS::Output2GridPort::ValueType &output, OBNnode::simtime_t t) {
        double curE = NAN, curTheta = NAN;  // Current aggregate values of E and Theta
        
        // Reset variables used in computation
        m_nPNaNs = m_nQNaNs = 0;
        m_Psum = m_Qsum = 0.0;
        m_Pmin = m_Pmax = m_Qmin = m_Qmax = 0.0;
        
        // Get and process the requests from users
        for (auto i = 1; i <= m_nUsers; i++) {
            if (!processUserInput(i, curE, curTheta, t)) {
                std::cout << "[ERROR] At " << t << " bus " << this->get_name() << " encountered an input error.\n";
                return false;
            }
        }
        
        // Send out the aggregate values
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
        
        return true;
    }
    
    virtual void distributeResults(const typename BaseCLS::InputFromGridPort::ValueType &v, OBNnode::simtime_t t) override;
    
    /* Add ports to node, register updates, etc. */
    virtual bool init(BaseCLS *pnode) {
        bool success;
        
        // Add the ports to the node
        for (auto i = 1; i <= m_nUsers; ++i) {
            m_input_users.emplace_back(pnode->createUserInput("VfU" + this->get_name() + '_' + std::to_string(i)));
            auto p = m_input_users.back().get();
            if (!(success = pnode->addInput(p))) {
                std::cerr << "Error while adding input: " << p->getPortName() << std::endl;
                break;
            }
        }
        
        if (success) {
            for (auto i = 1; i <= m_nUsers; ++i) {
                m_output_users.emplace_back(pnode->createUserOutput("VtU" + this->get_name() + '_' + std::to_string(i)));
                auto p = m_output_users.back().get();
                if (!(success = pnode->addOutput(p))) {
                    std::cerr << "Error while adding output: " << p->getPortName() << std::endl;
                    break;
                }
            }
        }
        
        return success;
    }
    
    /** Construct the bus node.
     \param t_nUsers Number of users (>= 1)
     */
    GenericBus(const std::string &name, unsigned int t_nUsers): Bus<BaseCLS>(name), m_nUsers(t_nUsers)
    {
        assert(t_nUsers > 0);
        m_input_users.reserve(m_nUsers);
        m_output_users.reserve(m_nUsers);
        
        // Clear the memory for individual P's and Q's
        VWithLimits v{0.0, 0.0, 0.0};
        m_Ps.assign(m_nUsers, v);
        m_Qs.assign(m_nUsers, v);
    }
};

template <typename BaseCLS>
bool GenericBus<BaseCLS>::processUserInput(int idx, double& t_curE, double& t_curTheta, OBNnode::simtime_t t) {
    int i = idx - 1;
    auto values = (*m_input_users[i])();
    
    auto n = values.size();
    
    // At least P is given
    if (n < 1) {
        std::cout << "At " << t << " User#" << idx << " of bus " << this->get_name() << " gave an empty vector.\n";
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
            std::cout << "At " << t << " User#" << idx << " of bus " << this->get_name() << " failed to give P limits.\n";
            return false;
        }
        double Pmin = values[4], Pmax = values[5];
        if (std::isnan(Pmin) || std::isnan(Pmax) || (std::isfinite(Pmin) && std::isfinite(Pmax) && Pmin > Pmax)) {
            // Invalid Pmin and Pmax: they must be non-NAN (can be INF) and Pmin <= Pmax
            std::cout << "At " << t << " User#" << idx << " of bus " << this->get_name() << " gave invalid P limits.\n";
            return false;
        }
        m_Ps[i].minvalue = Pmin;
        if (std::isfinite(Pmin)) {
            m_Pmin += Pmin;
        } else {
            // take care of INF case
            m_Pmin = -INFINITY;
        }
        m_Ps[i].maxvalue = Pmax;
        if (std::isfinite(Pmax)) {
            m_Pmax += Pmax;
        } else {
            // take care of INF case
            m_Pmax = INFINITY;
        }
    } else {
        m_Ps[i].value = P;
        m_Psum += P;
    }
    
    if (!std::isfinite(Q)) {
        m_Qs[i].value = NAN;
        m_nQNaNs++;
        
        if (n < 6 || (isPNaN && n < 8)) {
            // Not enough length for Q limits
            std::cout << "At " << t << " User#" << idx << " of bus " << this->get_name() << " failed to give Q limits.\n";
            return false;
        }
        double Qmin = values[isPNaN?6:4], Qmax = values[isPNaN?7:5];
        if (std::isnan(Qmin) || std::isnan(Qmax) || (std::isfinite(Qmin) && std::isfinite(Qmax) && Qmin > Qmax)) {
            // Invalid Qmin and Qmax: they must be non-NAN (can be INF) and Qmin <= Qmax
            std::cout << "At " << t << " User#" << idx << " of bus " << this->get_name() << " gave invalid Q limits.\n";
            return false;
        }
        m_Qs[i].minvalue = Qmin;
        if (std::isfinite(Qmin)) {
            m_Qmin += Qmin;
        } else {
            // take care of INF case
            m_Qmin = -INFINITY;
        }
        m_Qs[i].maxvalue = Qmax;
        if (std::isfinite(Qmax)) {
            m_Qmax += Qmax;
        } else {
            // take care of INF case
            m_Qmax = INFINITY;
        }
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
                std::cout << "At " << t << " User#" << idx << " of bus " << this->get_name() << " gave E incompatible with other users.\n";
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
                std::cout << "At " << t << " User#" << idx << " of bus " << this->get_name() << " gave theta incompatible with other users.\n";
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

template <typename BaseCLS>
void GenericBus<BaseCLS>::distributeResults(const typename BaseCLS::InputFromGridPort::ValueType &result, OBNnode::simtime_t t) {
    // For E and theta
    for (auto i = 0; i < m_nUsers; ++i) {
        auto& user_v = **m_output_users[i];
        user_v[2] = result[2];
        user_v[3] = result[3];
    }
    
    /* There are 4 cases for each P and Q:
     1) Both m_Pmin and m_Pmax are finite: in this case, we distribute the value among variable Pi's by a ratio Pr.
     2) Only m_Pmin is finite: for all variable Pi's except the last one, we assign Pi_min. The rest goes to the last one.
     3) Only m_Pmax is finite: similar to case 2 but use the upper bounds.
     4) Both m_Pmax and m_Pmin are infinite: this case is tricky because of all the constraints. There are two sub-cases:
     4a) One Pi is completely unconstrained (-Inf..Inf): assign everyone with their min/max/midpoint; the rest goes to the unconstrained one.
     4b) Otherwise, there must be Pi, Pj such that one is upper bounded, one is lower bounded. We distribute the others as min/max/midpoint; the rest x goes to Pi <= A, Pj >= B as: Pi = x - Pj <= A => Pj >= x - A. If x-A > B then we take Pj = x - A; o.w. Pj = B. Then Pi = x - Pj.
     */
    
    if (std::isfinite(m_Pmin) && std::isfinite(m_Pmax)) {
        // Case 1
        // Ratios for distributing P/Q to flexible users (i.e. NaN users)
        double Pr = m_nPNaNs>0?((result[0] - m_Psum) - m_Pmin)/(m_Pmax - m_Pmin):0;
        for (auto i = 0; i < m_nUsers; ++i) {
            auto& user_v = **m_output_users[i];
            user_v[0] = std::isfinite(m_Ps[i].value)?m_Ps[i].value:((m_Ps[i].maxvalue-m_Ps[i].minvalue)*Pr + m_Ps[i].minvalue);
        }
    } else if (std::isfinite(m_Pmin)) {
        // Case 2
        // Find one user whose P is unbounded above
        auto Pr = result[0];
        int theOne = -1;
        for (auto i = 0; i < m_nUsers; ++i) {
            auto& user_v = **m_output_users[i];
            if (std::isfinite(m_Ps[i].value)) {
                auto tmp = m_Ps[i].value;
                user_v[0] = tmp;
                Pr -= tmp;
            } else if (theOne < 0 && std::isinf(m_Ps[i].maxvalue)) {
                theOne = i; // Found the one
            } else {
                auto tmp = m_Ps[i].minvalue;
                user_v[0] = tmp;
                Pr -= tmp;
            }
            (**m_output_users[theOne])[0] = Pr;
        }
    } else if (std::isfinite(m_Pmax)) {
        // Case 3
        // Find one user whose P is unbounded below
        auto Pr = result[0];
        int theOne = -1;
        for (auto i = 0; i < m_nUsers; ++i) {
            auto& user_v = **m_output_users[i];
            if (std::isfinite(m_Ps[i].value)) {
                auto tmp = m_Ps[i].value;
                user_v[0] = tmp;
                Pr -= tmp;
            } else if (theOne < 0 && std::isinf(m_Ps[i].minvalue)) {
                theOne = i; // Found the one
            } else {
                auto tmp = m_Ps[i].maxvalue;
                user_v[0] = tmp;
                Pr -= tmp;
            }
            (**m_output_users[theOne])[0] = Pr;
        }
    } else {
        // Case 4
        int theI = -1, theJ = -1;
        double A, B;
        
        // Find case 4a or 4b
        for (auto i = 0; i < m_nUsers; ++i) {
            auto maxInf = std::isinf(m_Ps[i].maxvalue);
            auto minInf = std::isinf(m_Ps[i].minvalue);
            
            if (maxInf && minInf) {
                theI = theJ = i;
                break;
            } else if (minInf && theI < 0) {
                theI = i;
                A = m_Ps[i].maxvalue;
            } else if (maxInf && theJ < 0) {
                theJ = i;
                B = m_Ps[i].minvalue;
            }
            if (theI >= 0 && theJ >= 0) {
                break;
            }
        }
        
        assert(theI >= 0 && theJ >= 0);
        
        auto Pr = result[0];
        for (auto i = 0; i < m_nUsers; ++i) {
            auto& user_v = **m_output_users[i];
            if (std::isfinite(m_Ps[i].value)) {
                auto tmp = m_Ps[i].value;
                user_v[0] = tmp;
                Pr -= tmp;
            } else if (std::isfinite(m_Ps[i].maxvalue) && std::isfinite(m_Ps[i].minvalue)) {
                auto tmp = (m_Ps[i].maxvalue + m_Ps[i].minvalue) / 2.0;
                user_v[0] = tmp;
                Pr -= tmp;
            } else if (std::isinf(m_Ps[i].maxvalue) && std::isinf(m_Ps[i].minvalue)) {
                if (i != theI) {
                    user_v[0] = 0.0;
                }
            } else if (std::isinf(m_Ps[i].maxvalue)) {
                // maxvalue must be inf and minvalue must be finite
                if (i != theJ) {
                    auto tmp = m_Ps[i].minvalue;
                    user_v[0] = tmp;
                    Pr -= tmp;
                }
            } else {
                // minvalue must be inf and maxvalue must be finite
                if (i != theI) {
                    auto tmp = m_Ps[i].maxvalue;
                    user_v[0] = tmp;
                    Pr -= tmp;
                }
            }
            if (theI == theJ) {
                // Case 4a
                (**m_output_users[theI])[0] = Pr;
            } else {
                // Case 4b
                // If x-A > B then we take Pj = x - A; o.w. Pj = B. Then Pi = x - Pj.
                double Pj = (Pr-A > B)?Pr-A:B;
                (**m_output_users[theI])[0] = Pr - Pj;
                (**m_output_users[theJ])[0] = Pj;
            }
        }
    }
    
    
    // Same thing but for Q
    if (std::isfinite(m_Qmin) && std::isfinite(m_Qmax)) {
        // Case 1
        // Ratios for distributing P/Q to flexible users (i.e. NaN users)
        double Qr = m_nQNaNs>0?((result[1] - m_Qsum) - m_Qmin)/(m_Qmax - m_Qmin):0;
        for (auto i = 0; i < m_nUsers; ++i) {
            auto& user_v = **m_output_users[i];
            user_v[1] = std::isfinite(m_Qs[i].value)?m_Qs[i].value:((m_Qs[i].maxvalue - m_Qs[i].minvalue)*Qr + m_Qs[i].minvalue);
        }
    } else if (std::isfinite(m_Qmin)) {
        // Case 2
        // Find one user whose Q is unbounded above
        auto Qr = result[1];
        int theOne = -1;
        for (auto i = 0; i < m_nUsers; ++i) {
            auto& user_v = **m_output_users[i];
            if (std::isfinite(m_Qs[i].value)) {
                auto tmp = m_Qs[i].value;
                user_v[1] = tmp;
                Qr -= tmp;
            } else if (theOne < 0 && std::isinf(m_Qs[i].maxvalue)) {
                theOne = i; // Found the one
            } else {
                auto tmp = m_Qs[i].minvalue;
                user_v[1] = tmp;
                Qr -= tmp;
            }
            (**m_output_users[theOne])[1] = Qr;
        }
    } else if (std::isfinite(m_Qmax)) {
        // Case 3
        // Find one user whose Q is unbounded below
        auto Qr = result[1];
        int theOne = -1;
        for (auto i = 0; i < m_nUsers; ++i) {
            auto& user_v = **m_output_users[i];
            if (std::isfinite(m_Qs[i].value)) {
                auto tmp = m_Qs[i].value;
                user_v[1] = tmp;
                Qr -= tmp;
            } else if (theOne < 0 && std::isinf(m_Qs[i].minvalue)) {
                theOne = i; // Found the one
            } else {
                auto tmp = m_Qs[i].maxvalue;
                user_v[1] = tmp;
                Qr -= tmp;
            }
            (**m_output_users[theOne])[1] = Qr;
        }
    } else {
        // Case 4
        int theI = -1, theJ = -1;
        double A, B;
        
        // Find case 4a or 4b
        for (auto i = 0; i < m_nUsers; ++i) {
            auto maxInf = std::isinf(m_Qs[i].maxvalue);
            auto minInf = std::isinf(m_Qs[i].minvalue);
            
            if (maxInf && minInf) {
                theI = theJ = i;
                break;
            } else if (minInf && theI < 0) {
                theI = i;
                A = m_Qs[i].maxvalue;
            } else if (maxInf && theJ < 0) {
                theJ = i;
                B = m_Qs[i].minvalue;
            }
            if (theI >= 0 && theJ >= 0) {
                break;
            }
        }
        
        assert(theI >= 0 && theJ >= 0);
        
        auto Qr = result[1];
        for (auto i = 0; i < m_nUsers; ++i) {
            auto& user_v = **m_output_users[i];
            if (std::isfinite(m_Qs[i].value)) {
                auto tmp = m_Qs[i].value;
                user_v[1] = tmp;
                Qr -= tmp;
            } else if (std::isfinite(m_Qs[i].maxvalue) && std::isfinite(m_Qs[i].minvalue)) {
                auto tmp = (m_Qs[i].maxvalue + m_Qs[i].minvalue) / 2.0;
                user_v[1] = tmp;
                Qr -= tmp;
            } else if (std::isinf(m_Qs[i].maxvalue) && std::isinf(m_Qs[i].minvalue)) {
                if (i != theI) {
                    user_v[1] = 0.0;
                }
            } else if (std::isinf(m_Qs[i].maxvalue)) {
                // maxvalue must be inf and minvalue must be finite
                if (i != theJ) {
                    auto tmp = m_Qs[i].minvalue;
                    user_v[1] = tmp;
                    Qr -= tmp;
                }
            } else {
                // minvalue must be inf and maxvalue must be finite
                if (i != theI) {
                    auto tmp = m_Qs[i].maxvalue;
                    user_v[1] = tmp;
                    Qr -= tmp;
                }
            }
            if (theI == theJ) {
                // Case 4a
                (**m_output_users[theI])[1] = Qr;
            } else {
                // Case 4b
                double Qj = (Qr-A > B)?Qr-A:B;
                (**m_output_users[theI])[1] = Qr - Qj;
                (**m_output_users[theJ])[1] = Qj;
            }
        }
    }
    
    //std::cout << "At " << t << " UPDATE_Y" << std::endl;
}


struct BusInfo {
    int nUsers;
    std::vector<double> const_values;       // The constants sent to the grid node
};


std::map<std::string, BusInfo> load_csv_bus_defs(const char* t_file) {
    CsvParser *csvparser = CsvParser_new(t_file, ",", false);
    
    std::map<std::string, BusInfo> v, empty_result;
    CsvRow *row;
    int krow = 0;
    
    while ((row = CsvParser_getRow(csvparser)) ) {
        char **rowFields = CsvParser_getFields(row);
        ++krow;
        
        int n = CsvParser_getNumFields(row);
        if (n < 2) {
            // At least two fields must be provided
            std::cout << "[ERROR] Row #" << krow << " in the CSV file has fewer than 2 required fields." << std::endl;
            return empty_result;
        }
        
        BusInfo curBus;
        std::string curBusName = OBNsim::Utils::trim(std::string(rowFields[0]));
        
        // Check if the bus's name already existed in the map
        if (v.find(curBusName) != v.end()) {
            std::cout << "[ERROR] Row #" << krow << " in the CSV file defines an already-existing bus." << std::endl;
            return empty_result;
        }
        
        // The number of users
        int nUsers = std::strtol(rowFields[1], nullptr, 0);
        if (nUsers < 0) {
            std::cout << "[ERROR] Row #" << krow << " in the CSV file has a negative number of users." << std::endl;
            return empty_result;
        }
        
        // Extract more values for the case of const bus
        if (nUsers == 0) {
            // At least 4 values must be provided
            if (n < 6) {
                std::cout << "[ERROR] At least four values (P,Q,E,theta) must be provided for the const bus in row #" << krow << std::endl;
                return empty_result;
            }
            for (int i = 2; i < n; ++i) {
                curBus.const_values.emplace_back(std::strtod(rowFields[i], nullptr));
            }
        }
        
        curBus.nUsers = nUsers;
        v.emplace(curBusName, curBus);
        
        CsvParser_destroy_row(row);
    }
    
    CsvParser_destroy(csvparser);
    return v;
}


#ifdef OBNNODE_COMM_YARP
class MultiBusYarp: public YarpNode {
public:
    /* Type of the output port from the bus to the grid. */
    typedef YarpOutput<OBN_PB, obn_vector<double> > Output2GridPort;
    
    /* Type of the input port from the grid to the bus. */
    typedef YarpInput<OBN_PB, obn_vector_fixed<double,4> > InputFromGridPort;
    
    /* Type of the Bus base class. */
    typedef Bus<MultiBusYarp> BusType;
    
    typedef ConstBus<MultiBusYarp> ConstBusType;
    typedef GenericBus<MultiBusYarp> GenericBusType;
    
    using UserInputType = YarpInput< OBN_PB, obn_vector<double> >;
    
    UserInputType* createUserInput(const std::string& t_name) {
        return new UserInputType(t_name);
    }

    using UserOutputType = YarpOutput< OBN_PB, obn_vector_fixed<double, 4> >;
    
    UserOutputType* createUserOutput(const std::string& t_name) {
        return new UserOutputType(t_name);
    }
    
    // Information about a bus
    struct BusInfo {
        std::unique_ptr<BusType> m_bus;
        InputFromGridPort m_input;
        Output2GridPort m_output;
        BusInfo(BusType* pbus, MultiBusYarp* pnode): m_bus(pbus), m_input("VfG" + pbus->get_name()), m_output("VtG" + pbus->get_name())
        { }
    };
    
    MultiBusYarp(const std::string& _name, const std::string& ws): YarpNode(_name, ws) { }
};
#endif


#ifdef OBNNODE_COMM_MQTT
class MultiBusMQTT: public MQTTNode {
public:
    /* Type of the output port from the bus to the grid. */
    typedef MQTTOutput<OBN_PB, obn_vector<double> > Output2GridPort;
    
    /* Type of the input port from the grid to the bus. */
    typedef MQTTInput<OBN_PB, obn_vector_fixed<double,4> > InputFromGridPort;
    
    /* Type of the Bus base class. */
    typedef Bus<MultiBusMQTT> BusType;
    
    typedef ConstBus<MultiBusMQTT> ConstBusType;
    typedef GenericBus<MultiBusMQTT> GenericBusType;

    
    using UserInputType = MQTTInput< OBN_PB, obn_vector<double> >;
    
    UserInputType* createUserInput(const std::string& t_name) {
        return new UserInputType(t_name);
    }
    
    using UserOutputType = MQTTOutput< OBN_PB, obn_vector_fixed<double, 4> >;

    UserOutputType* createUserOutput(const std::string& t_name) {
        return new UserOutputType(t_name);
    }
    
    // Information about a bus
    struct BusInfo {
        std::unique_ptr<BusType> m_bus;
        InputFromGridPort m_input;
        Output2GridPort m_output;
        BusInfo(BusType* pbus, MultiBusMQTT* pnode): m_bus(pbus), m_input("VfG" + pbus->get_name()), m_output("VtG" + pbus->get_name())
        { }
    };
    
    MultiBusMQTT(const std::string& _name, const std::string& ws): MQTTNode(_name, ws) { }
};
#endif

/** The node class for this multi-bus node. */
template <typename BaseCLS>
class MultiBus: public BaseCLS {
private:
    // The list of buses
    // We use pointers because when BusInfo is created and copied, m_bus can't be copied (unique_ptr's copy constructor is deleted) and m_input and m_output can't be copied too (Yarp forbids that).
    std::vector< std::unique_ptr<typename BaseCLS::BusInfo> > m_buses;
public:
    MultiBus(const char* t_name, const char* t_workspace): BaseCLS(t_name, t_workspace) { }
    
    /* Add a bus to the node */
    bool addBus(typename BaseCLS::BusType *pbus) {
        assert(pbus);
        auto name = pbus->get_name();
        
        // Find if this bus already exist
        for (auto& bus: m_buses) {
            if (bus->m_bus->get_name() == name) {
                std::cout << "The bus " << name << " already existed in this node." << std::endl;
                return false;
            }
        }
        
        m_buses.emplace_back(new typename BaseCLS::BusInfo(pbus, this));
        return true;
    }
    
    bool loadCSV(const char* csv_file) {
        // Load the CSV file into a vector of BusInfo records
        auto csvbuses = load_csv_bus_defs(csv_file);
        if (csvbuses.empty()) {
            return false;
        }
        
        // Loop through the vector to create the buses
        for (auto& bus: csvbuses) {
            if (bus.second.nUsers == 0) {
                // Const bus
                addBus(new typename BaseCLS::ConstBusType(bus.first, bus.second.const_values));
            } else {
                // Generic bus
                addBus(new typename BaseCLS::GenericBusType(bus.first, bus.second.nUsers));
            }
        }
        
        return true;
    }
    
    int numBuses() const {
        return m_buses.size();
    }
    
    bool initialize() {
        if (!this->openSMNPort()) {
            std::cerr << "Error while opening the GC port; check the network or server.\n";
            return false;
        }
        
        // Initialize all the buses
        for (auto& bus: m_buses) {
            // Add the grid input and output for the bus
            if (!(this->addInput(&(bus->m_input)) && this->addOutput(&(bus->m_output)) && bus->m_bus->init(this))) {
                return false;
            }
        }
        
        // Add the updates
        bool success = (this->addUpdate(MAIN_UPDATE, std::bind(&MultiBus::doMainUpdate, this)) >= 0) &&
            (this->addUpdate(FEEDBACK_UPDATE, std::bind(&MultiBus::doFeedbackUpdate, this)) >= 0);
        
        return success;
    }
    
    void doMainUpdate() {
        auto t = this->currentSimulationTime();
        for (auto& bus: m_buses) {
            if (!bus->m_bus->calculateV2Grid(*(bus->m_output), t)) {
                // Error: in this case we will request the simulation to stop and exit
                this->requestStopSimulation();
                this->stopSimulation();
                return;
            }
        }
        
        //std::cout << "At " << currentSimulationTime() << " UPDATE_Y" << std::endl;
    }
    
    /* This node has no state update */
    
    void doFeedbackUpdate() {
        auto t = this->currentSimulationTime();
        for (auto& bus: m_buses) {
            bus->m_bus->distributeResults(bus->m_input.get(), t);
        }
    }
    
    /* This callback is called everytime this node's simulation starts or restarts.
     This is different from initialize() above. */
    virtual void onInitialization() {
        // Initial state and output
        
        std::cout << "At " << this->currentSimulationTime() << " INIT" << std::endl;
    }
    
    /* This callback is called when the node's current simulation is about to be terminated. */
    virtual void onTermination() {
        std::cout << "At " << this->currentSimulationTime() << " TERMINATED" << std::endl;
    }
    
    /* There are other callbacks for reporting errors, etc. */
};

void show_usage(char *prog) {
    std::cout << "USAGE:" << std::endl <<
    prog << " node_name bus_data_file_name [--workspace <workspace_name>] [--mqtt [serveraddr]]" << std::endl <<
    R"args(
where
   node_name is the name of the bus node
   bus_data_file_name is the name of the CSV file that contains all the buses' configurations.
   workspace_name is the name of the simulation workspace (default: "powernet").
   --mqtt specifies that the MQTT communication framework will be used, and the optional serveraddr is the address of the MQTT server/broker.
    
The default communication framework is YARP.

Each of the rows in the CSV file defines a bus in the following format:
   bus_name, no_of_users, P, Q, E, Th, ...
bus_name is a unique string that identifies the bus.
no_of_users is the number of users connected to the bus.
If no_of_users = 0, this is a "constant" bus, which always sends constant values P, Q, E, theta, ... to the grid.
In this case, the four values P, Q, E, theta are required, but optional extra values can be specified after them.
All real values can be either a finite real number (e.g. 0, 0.5) or one of the words "NaN", "Inf", "-Inf".
If an invalid number is given, 0.0 will be used instead.

If no_of_users > 0, appropriate input and output ports for each user will be created automatically.
In this case, the values P, Q, E, theta, ..., if supplied, will not be used.
)args";
}

void show_banner() {
    std::cout << R"banner(
+-------------------------------------------------------------------+
|                      Generic Multi-Bus Node                       |
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
    const char *csv_file = nullptr;         // The CSV file name
    
    const char *workspace_name = nullptr;     // The workspace name
    const std::string WORKSPACE_OPTION("--workspace");
    
    std::string mqtt_server{""};     // The MQTT server address
    const std::string MQTT_OPTION("--mqtt");
    bool mqtt_used = false;
    
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
            } else if (MQTT_OPTION.compare(argv[k]) == 0) {
                // Get the server address if given
                mqtt_used = true;
                if (++k < argc) {
                    // Check that this is not another option
                    if (std::strlen(argv[k]) <= 2 || argv[k][0] != '-' || argv[k][1] != '-') {
                        mqtt_server = argv[k++];
                    }
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
            } else if (csv_file == nullptr) {
                // This is the CSV file's name
                csv_file = argv[k++];
            } else {
                // Unrecognized argument
                std::cout << "Unrecognized argument: " << argv[k] << std::endl;
                show_usage(argv[0]);
                exit(1);
            }
        }
    }

    if (node_name == nullptr || csv_file == nullptr) {
        // Not enough input args
        std::cout << "Not enough input arguments." << std::endl;
        show_usage(argv[0]);
        exit(1);
    }
    
    if (workspace_name == nullptr) {
        workspace_name = "powernet";
    }
    
    // The bus node object
    std::unique_ptr<NodeBase> pbus;
    bool success;
    int numBuses = 0;
    
    if (mqtt_used) {
        // MQTT is used
#ifdef OBNNODE_COMM_MQTT
        // Create the node
        auto *p = new MultiBus<MultiBusMQTT>(node_name, workspace_name);
        pbus.reset(p);
        
        if (!mqtt_server.empty()) {
            p->setServerAddress(mqtt_server);
        }
        
        success = p->loadCSV(csv_file); // Load the bus definitions
        if (!success) {
            std::cout << "Could not load the CSV file of bus definitions, or the file is empty/invalid." << std::endl;
            show_usage(argv[0]);
            return 1;
        }
        
        success = p->initialize();
        if (!success) {
            std::cout << "There was/were error(s) while initializing the node.\n";
            return 2;
        }
        
        numBuses = p->numBuses();
#else
        std::cerr << "MQTT communication is specified but not supported by this node.\n";
        return 1;
#endif
    } else {
        // Yarp is used
#ifdef OBNNODE_COMM_YARP
        // Set verbosity level
        yarp::os::Network::setVerbosity(-1);

        auto *p = new MultiBus<MultiBusYarp>(node_name, workspace_name);
        pbus.reset(p);
        
        success = p->loadCSV(csv_file); // Load the bus definitions
        if (!success) {
            std::cout << "Could not load the CSV file of bus definitions, or the file is empty/invalid." << std::endl;
            show_usage(argv[0]);
            return 1;
        }
        
        success = p->initialize();
        if (!success) {
            std::cout << "There was/were error(s) while initializing the node.\n";
            return 2;
        }
        
        numBuses = p->numBuses();
#else
        std::cerr << "YARP communication is specified but not supported by this node.\n";
        return 1;
#endif
    }
    
    // Display info
    std::cout << "This is a multi-bus node named \"" << node_name
    << "\" in the workspace \"" << workspace_name << "\"\n"
    << "There are " << numBuses << " buses on this node.\n\n";
    
    // Here we will not connect the node to the GC, let the SMN do it
    std::cout << "Starting simulation of node " << pbus->name() << std::endl;
    
    pbus->run();
    
    std::cout << "Simulation finished. Goodbye!" << std::endl;
    
    //////////////////////
    // Clean up before exiting
    //////////////////////
    
    pbus->delayBeforeShutdown();
    
    google::protobuf::ShutdownProtobufLibrary();
    
    return pbus->hasError()?3:0;
}
