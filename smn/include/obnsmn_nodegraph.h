/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file
 * \brief Define node graph classes to be used by the GC.
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#ifndef OBNSIM_NODEGRAPH_H
#define OBNSIM_NODEGRAPH_H

#include <iostream>
#include <memory>
#include <vector>
#include <utility>

#include <obnsmn_basic.h>

// Boost Graph Library
#include <boost/graph/adjacency_list.hpp>


namespace OBNsmn {
    /* A class that will be defined later in obnsim_gc.h, that implements iterator for regular updating nodes. */
    class RegUpdateListIterator;
    
    /* NOT USED
     A base class for iterating over a list of updating nodes.
    class UpdateListIterator {
    public:
        virtual UpdateListIterator& operator++() = 0;
        //virtual bool operator==(const UpdateListIterator& rhs) = 0;
        //virtual bool operator!=(const UpdateListIterator& rhs) = 0;
        virtual int getID() const = 0;
        virtual outputmask_t getMask() const = 0;
    }; */
    
    /** \brief Graph of nodes' dependency, used in runtime.
     
     This abstract class defines the interface for a graph of nodes' dependency, which is used in runtime to determine which nodes are to be updated next.
     The graph is simple: each node is associated with the node's ID and a simple directed edge with no weight indicates that the target node's update depends on the to-be-updated outputs of the source node.
     During run time, this graph will be pruned of independent nodes until it becomes empty. It is therefore often quite sparse.
     */
    class RTNodeDepGraph {
    public:
        RTNodeDepGraph() {}
        virtual ~RTNodeDepGraph() {}
        
        /** \brief Return and remove independent nodes.
         
         Return a list of IDs of independent nodes in the graph, then remove them as well as all adjacent edges.
         \return Vector of IDs of the independent nodes.
         */
        virtual std::vector<int> getAndRemoveIndependentNodes() = 0;
        
        /** \brief Check if the graph is empty.
         \return True if empty.
         */
        virtual bool empty() = 0;
    };
    
    /** \brief Graph of nodes' dependency, with weights on edges.
     
     This abstract class defines the interface for a graph of nodes' dependency.
     An edge from node s to node t with weight w means that some outputs of node t depends on the output groups specified by w of node s.
     Nodes and edges can be added to this graph, but their removal is not supported; therefore a highly efficient but not feature-rich graph structure can be used.
     Nodes are always identified by unique IDs. Internally, the object may need to maintain a map from ID values to node references.
     
     These graphs will not check the validity of the node dependency graph (e.g. acyclic property). That is the task of the system configuration functions/tools.
     */
    class NodeDepGraph {
    public:
        NodeDepGraph() {}
        virtual ~NodeDepGraph() {}
        
        /** \brief Add a new node with a unique ID.
         
         Add a new node associated with a unique ID to the graph.
         The ID is the index of the node in the array of all nodes, so it is between 0 and N-1, where N is the number of nodes. The IDs of nodes are guaranteed to be contiguous integers.
         \param id The unique integer ID of the node.
         */
        // virtual void addNode(int id) = 0;
        
        /** \brief Add dependency of a node on another node.
         
         Add that some outputs of node t depend on the values of the output groups specified in w of node s.
         The ID of a node is the index of the node in the array of all nodes, so it is between 0 and N-1, where N is the number of nodes. The IDs of nodes are guaranteed to be contiguous integers.
         \param s ID of the source node.
         \param t ID of the target node.
         \param w Bit mask of the output groups of node s on which node t depends.
         */
        virtual void addDependency(int s, int t, outputmask_t w) = 0;
        
        /** \brief Return a runtime node dependency graph, keeping only updating nodes.
         
         Copy the full node dependency graph to a run-time node dependency graph after removing all non-updating nodes and their associated edges.
         All relevant data (e.g. mappings between IDs and nodes) must be copied also.
         \param it First iterator
         \param n Exact number of elements (nodes to be updated)
         \return Unique pointer to a RTNodeDepGraph object
         */
        virtual std::unique_ptr<RTNodeDepGraph> getRTNodeDepGraph(RegUpdateListIterator it, size_t n) = 0;
    };
    
    
    // =============================================
    // Implementation using Boost graph library (BGL)
    // =============================================
    
    class NodeDepGraph_BGL;
    
    /** \brief Implementation of RTNodeDepGraph in Boost Graph Library.
     
     The most important part is choosing the right containers for the adjacency_list graph, which stores the dependency graph.
     Properties:
     - RT graphs are created and destroyed many times (once in each update iteration).
     - Frequent operations are add_edge or remove_edge, clear_vertex and remove_vertex.
     - Speed is more important than space.
     
     Therefore we choose listS for VertexList and OutEdgeList.
     \see http://www.boost.org/doc/libs/1_57_0/libs/graph/doc/using_adjacency_list.html
     */
    class RTNodeDepGraph_BGL: public RTNodeDepGraph {
    public:
        RTNodeDepGraph_BGL(int numNodes): _graph(numNodes) {}
        // virtual ~RTNodeDepGraph_BGL() { std::cout << "RT Node graph deleted." << std::endl; }
        
        /** \brief Return and remove independent nodes. */
        virtual std::vector<int> getAndRemoveIndependentNodes();
        
        /** \brief Check if the graph is empty.
         \return True if empty.
         */
        virtual bool empty() {
            return boost::num_vertices(_graph) == 0;
        }
        
    private:
        /**
         Here we use listS to store the list of vertices and the list of edges because the RT graph will be modified frequently.
         To each vertex, we associate the ID of the corresponding node, therefore a bundled property of type int is used for the vertices.
         */
        typedef boost::adjacency_list<boost::listS, boost::listS, boost::directedS, int> GraphT;
        GraphT _graph;
        
        RTNodeDepGraph_BGL() {}  // prevent being constructed outside of NodeDepGraph_BGL

        friend class NodeDepGraph_BGL;
    };
    
    
    /** \brief Implementation of NodeDepGraph in Boost Graph Library.
     
     The most important part is choosing the right containers for the adjacency_list graph, which stores the dependency graph.
     Properties:
     - The graph is constructed once at the beginning, and remains constant during the simulation.
     - Potentially large. For constructing the graph, space is more important than speed.
     - We frequently need to traverse its vertex list and the **out-edge list** of each vertex, in constructing the RT graph in each update iteration. In this case, speed is more important than space.
     
     We choose vecS for both VertexList and OutEdgeList.
     \see http://www.boost.org/doc/libs/1_57_0/libs/graph/doc/using_adjacency_list.html
     */
    class NodeDepGraph_BGL: public NodeDepGraph {
    public:
        /** \brief Construct a graph of a given number of nodes.
         \param numNodes Number of nodes, whose indices are from 0 to (numNodes-1).
         */
        NodeDepGraph_BGL(int numNodes): _graph(numNodes) {}
        
        // virtual ~NodeDepGraph_BGL() { std::cout << "Node graph deleted." << std::endl; }
        
        /** \brief Add a new node with a unique ID.
         
         Add a new node associated with a unique ID to the graph.
         The ID is the index of the node in the array of all nodes, so it is between 0 and N-1, where N is the number of nodes. The IDs of nodes are guaranteed to be contiguous integers.
         \param id The unique integer ID of the node.
         */
        // virtual void addNode(int id);
        
        
        /** \brief Add dependency of a node on another node.
         
         Add that some outputs of node t depend on the values of the output groups specified in w of node s.
         The ID is the index of the node in the array of all nodes, so it is between 0 and N-1, where N is the number of nodes. The IDs of nodes are guaranteed to be contiguous integers.
         \param s ID of the source node.
         \param t ID of the target node.
         \param w Bit mask of the output groups of node s on which node t depends.
         */
        virtual void addDependency(int s, int t, outputmask_t w) {
            boost::add_edge(s, t, w, _graph);
        }
        
        /** \brief Return a runtime node dependency graph, keeping only updating nodes. */
        virtual std::unique_ptr<RTNodeDepGraph> getRTNodeDepGraph(RegUpdateListIterator it, size_t n);

    private:
        /**
         Here we use vecS to store the list of vertices, which is essentially a vector of integers (IDs of the vertices) starting from 0.
         Because this coincides with the IDs of the nodes (which are guaranteed to be integers from 0 to N-1 where N is the total number of nodes), we can directly associate the ID of a node with the ID (descriptor) of a vertex.
         However, Otherwise we would have to use an <int> as the property of a vertex.
         The property of an edge is simply its weight (output mask).
         */
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, boost::no_property, outputmask_t> GraphT;
        GraphT _graph;
    };
}

#endif /* defined(OBNSIM_NODEGRAPH_H) */
