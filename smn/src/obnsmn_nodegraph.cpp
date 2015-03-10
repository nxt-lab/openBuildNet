/* -*- mode: C++; indent-tabs-mode: nil; -*- */
/** \file obnsim_nodegraph.cpp
 * \brief Implement node graph classes using LEMON graph library
 *
 * This file is part of the openBuildNet simulation framework
 * (OBN-Sim) developed at EPFL.
 *
 * \author Truong X. Nghiem (xuan.nghiem@epfl.ch)
 */

#include <unordered_set>
#include <unordered_map>
#include <obnsmn_iterators.h>
#include <obnsmn_nodegraph.h>

using namespace boost;
using namespace OBNsmn;


/** Return a list of IDs of independent nodes in the graph, then remove them as well as all adjacent edges.
 \return Vector of IDs of the independent nodes.
 */
std::vector<int> RTNodeDepGraph_BGL::getAndRemoveIndependentNodes() {
    if (empty()) {
        return std::vector<int>();
    }
    
    // Set of all vertices
    std::unordered_set<GraphT::vertex_descriptor> independents;
    
    GraphT::vertex_iterator vit, vend;
    tie(vit, vend) = vertices(_graph);
    for (; vit != vend; ++vit) {
        independents.insert(*vit);
    }
    
    // Traverse the list of edges and remove the target nodes from the set of independent nodes
    GraphT::edge_iterator eit, eend;
    tie(eit, eend) = edges(_graph);
    for (; eit != eend; ++eit) {
        independents.erase(target(*eit, _graph));
    }
    
    // Because the graph is acyclic (guaranteed by the parser), there must be at least one independent node,
    // so the set must be non-empty.
    
    // Construct the list of nodes' IDs
    std::vector<int> results(independents.size());
    auto resultIt = results.begin();
    
    // Remove independent vertices (and their associated edges) from the graph
    // After this loop, all values in independents are invalid.
    for (auto it: independents) {
        *(resultIt++) = _graph[it];
        clear_vertex(it, _graph);
        remove_vertex(it, _graph);
    }
    
    return results;
}

/** Copy the full node dependency graph to a run-time node dependency graph after removing all non-updating nodes and their associated edges.
 All relevant data (e.g. mappings between IDs and nodes) must be copied also.
 \param it First iterator
 \param n Exact number of elements (nodes to be updated)
 \return Unique pointer to a RTNodeDepGraph object
 */
std::unique_ptr<RTNodeDepGraph> NodeDepGraph_BGL::getRTNodeDepGraph(RegUpdateListIterator itNodes, size_t nNodes) {
    
    /* Algorithm:
     - Create RT graph with number of nodes.
     - Loop updateList, create a map from updating nodes in main graph to RT graph's nodes + output mask.
     - Loop through the keys of the map, for each node in the main graph, loop through its out-edges, if weight intersects output_mask, create an edge in RT graph from source node to target node (1 lookup for target node in the map).
     */
    
    // Create the resulting graph, pre-allocated with right number of vertices
    RTNodeDepGraph_BGL* result = new RTNodeDepGraph_BGL(nNodes);
    
    // Construct a map that associates a vertex of result with a vertex (ID) in updateNodes
    std::unordered_map<GraphT::vertex_descriptor, std::pair<RTNodeDepGraph_BGL::GraphT::vertex_descriptor, outputmask_t> > nodeMap;
    RTNodeDepGraph_BGL::GraphT::vertex_iterator vit, vend;
    tie(vit, vend) = vertices(result->_graph);
    for (; vit != vend && !itNodes.atEnd(); ++vit, ++itNodes) {
        // Assign the node's ID to the vertex in result
        auto ID = itNodes.getID();
        result->_graph[*vit] = ID;
        
        // Associate the node in _graph with a vertex in result as well as the updating output mask
        nodeMap.insert(std::make_pair(static_cast<GraphT::vertex_descriptor>(ID),
                                      std::make_pair(*vit, itNodes.getMask())));
    }

    // Create the edges in the result graph
    GraphT::out_edge_iterator oeit, oeend;
    for (auto it: nodeMap) {
        auto s = it.second.first;  // source node in result
        outputmask_t m = it.second.second;
        
        tie(oeit, oeend) = out_edges(it.first, _graph);
        for (; oeit != oeend; ++oeit) {
            if (_graph[*oeit] & m) {
                // Copy the edge to the result graph
                add_edge(s, nodeMap[target(*oeit, _graph)].first, result->_graph);
            }
        }
    }
    
    return std::unique_ptr<RTNodeDepGraph>(result);
}