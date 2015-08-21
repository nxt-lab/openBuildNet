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
#include <obnsmn_nodegraph.h>
#include <obnsmn_gc.h>

using namespace boost;
using namespace OBNsmn;
using namespace std;


/** Return a list of IDs of independent nodes in the graph, then remove them as well as all adjacent edges.
 \return Vector of (ID, update-mask) of the independent nodes.
 */
std::vector< std::pair<int, updatemask_t> > const& NodeDepGraph_BGL::getAndRemoveIndependentNodes() {
    rtResult.clear();

    if (empty()) {
        return rtResult;
    }

    // Traverse the vertices and extract independent updates of active nodes
    GraphT::vertex_iterator vit, vend;
    tie(vit, vend) = vertices(_graph);
    GraphT::vertex_descriptor theVertex;
    
    GraphT::out_edge_iterator oeit, oeend;
    
    for (; vit != vend; ++vit) {
        theVertex = *vit;
        VertexLabel& thisNode = _graph[theVertex];  // Directly change the label if needs to

        if (thisNode.active) {
            // Find if there exist updates that are not in any of the active input edges (represented by the inputMask field)
            updatemask_t independentUpdates = thisNode.updateMask & (~thisNode.inputMask);
            if (independentUpdates) {
                // Add these updates to the result
                rtResult.emplace_back(static_cast<int>(theVertex), independentUpdates);
                
                thisNode.updateMask &= (~independentUpdates);   // Remove these updates from the flag
                
                if (thisNode.updateMask == 0) {
                    // This node becomes inactive because there are no more updates
                    thisNode.active = false;
                    rtNodesLeft--;  // Decrease the number of nodes left in the RT graph
                    
                    // Mark all its out-edges as inactive
                    tie(oeit, oeend) = out_edges(theVertex, _graph);
                    for (; oeit != oeend; ++oeit) {
                        _graph[*oeit].active = false;
                    }
                }
            }
            // It's still active, reset its inputMask so it will get updated in the later part
            thisNode.inputMask = 0;
        }
    }
    
    // Loop through all active edges and determine if each of them is still active or inactive
    // For remaining active ones, also re-calculate the inputMask of its target node
    GraphT::edge_iterator eit, eitend;
    tie(eit, eitend) = edges(_graph);
    
    for (; eit != eitend; ++eit) {
        if (_graph[*eit].active) {
            // The source node is guaranteed to be active, because if it's inactive, it would have deactivated this edge (see above)
            auto sourceNode = source(*eit, _graph), destNode = target(*eit, _graph);
            bool active_edge = false;
            if (_graph[destNode].active) {
                // Both source and target nodes are active
                // Check if at least one link is active
                // Also calculate the inputMask of the target node
                
                updatemask_t inputMask = 0;
                auto sourceUpdateMask = _graph[sourceNode].updateMask, targetUpdateMask = _graph[destNode].updateMask;
                for (auto alink: _graph[*eit].links) {
                    if (alink.second & targetUpdateMask) {
                        // This link has target mask that intersects with its target node's update mask
                        inputMask |= alink.second;
                        
                        active_edge = active_edge || ((alink.first & sourceUpdateMask) != 0);
                    }
                }
                if (active_edge) {
                    // This edge is still active, so we will update the inputMask of its target node
                    _graph[destNode].inputMask |= inputMask;
                }
            }
            _graph[*eit].active = active_edge;  // update the status of this edge
        }
    }
    
    return rtResult;
}

// OLD CODE
//std::vector<int> NodeDepGraph_BGL::getAndRemoveIndependentNodes() {
//    if (empty()) {
//        return std::vector<int>();
//    }
//    
//    // Traverse the edges and mark vertices that are dependent, also counting the number of marked vertices
//    int markedCount = 0;
//    
//    GraphT::edge_iterator eit, eend;
//    tie(eit, eend) = edges(_graph);
//    
//    GraphT::vertex_descriptor theVertex;
//    
//    for (; eit != eend; ++eit) {
//        if (!_graph[*eit].removed) {
//            theVertex = target(*eit, _graph);
//            if (_graph[theVertex].status == VertexLabel::UNMARKED) {
//                _graph[theVertex].status = VertexLabel::MARKED;
//                ++markedCount;
//            }
//        }
//    }
//    
//    // At this point, markedCount is the number of nodes that have been switched to MARKED, so the number of independent nodes is (rtNodesLeft - markedCount) because rtNodesLeft = number of nodes considering and markedCount = number of dependent nodes.
//
//    // Construct the list of independent nodes' IDs
//    int indepedentCount = rtNodesLeft - markedCount;
//    if (indepedentCount <= 0) {
//        // Error or algebraic loop
//        return std::vector<int>();
//    }
//    
//    std::vector<int> results(indepedentCount);
//    auto resultIt = results.begin();
//    
//    // Traverse the vertices and extract independent nodes
//    GraphT::vertex_iterator vit, vend;
//    tie(vit, vend) = vertices(_graph);
//    
//    GraphT::out_edge_iterator oeit, oeend;
//    
//    for (; vit != vend; ++vit) {
//        switch (_graph[*vit].status) {
//            case VertexLabel::MARKED:
//                _graph[*vit].status = VertexLabel::UNMARKED;
//                break;
//                
//            case VertexLabel::UNMARKED:
//                /* This is an independent node, we need to:
//                 - Extract it
//                 - Mark it as REMOVED
//                 - Iterate its out-edges and mark them as REMOVED
//                 */
//                theVertex = *vit;
//                *(resultIt++) = static_cast<int>(theVertex);
//                _graph[theVertex].status = VertexLabel::REMOVED;
//                
//                tie(oeit, oeend) = out_edges(theVertex, _graph);
//                for (; oeit != oeend; ++oeit) {
//                    _graph[*oeit].removed = true;
//                }
//                
//                break;
//                
//            default:
//                break;
//        }
//    }
//    
//    // NUmber of nodes left = number of dependent nodes
//    rtNodesLeft = markedCount;
//    
//    return results;
//}


/** Return a list of IDs of independent nodes in the graph, then remove them as well as all adjacent edges.
 \return Vector of IDs of the independent nodes.
 */
std::vector< std::pair<int,updatemask_t> > NodeDepGraph_BGL::getCurrentNodes() const {
    if (rtNodesLeft <= 0) {
        return std::vector< std::pair<int,updatemask_t> >();
    }
    
    std::vector< std::pair<int,updatemask_t> > result;
    result.reserve(rtNodesLeft);
    
    // Traverse the vertices and return the nodes
    GraphT::vertex_iterator vit, vend;
    tie(vit, vend) = vertices(_graph);
    
    for (; vit != vend; ++vit) {
        if (_graph[*vit].active) {
            result.emplace_back(static_cast<int>(*vit), _graph[*vit].updateMask);
        }
    }
    
    return result;
}

/**
 This method tries to combine two links (link1 and link2). If they can be combined, the combined link will be placed in link1, and link2 can be removed.
\return true if they were combined; false otherwise.
*/
bool NodeDepGraph_BGL::combineLinks(NodeDepGraph_BGL::LinkLabel& link1, const NodeDepGraph_BGL::LinkLabel& link2) {
    // Check if link2 is a special case of link1 (i.e. if both its masks are contained in the masks of the other link) or vice versa
    if (((link1.first & link2.first) == link2.first) && ((link1.second & link2.second) == link2.second)) {
        // link2 is a member of link1
        return true;
    }
    else if (((link1.first | link2.first) == link2.first) && ((link1.second | link2.second) == link2.second)) {
        // link1 is a member of link2 => replace link1 with link2
        link1 = link2;
        return true;
    }
    else if ((link1.first == link2.first) || (link1.second == link2.second)) {
        // Either their sources or targets are the same, then we can combine them
        link1.first |= link2.first;
        link1.second |= link2.second;
        return true;
    }
    return false;
}

/* Add dependency of a node on another node.
 
 Add that some outputs of node t depend on the values of the output groups specified in w of node s.
 The ID of a node is the index of the node in the array of all nodes, so it is between 0 and N-1, where N is the number of nodes. The IDs of nodes are guaranteed to be contiguous integers.
 Dependency links are combined if possible, so that the resulted links in the graph between any two nodes/vertices are minimal.
 
 \param s ID of the source node.
 \param t ID of the target node.
 \param smask Bit mask of the update types of node s on which node t depends.
 \param tmask Bit mask of the update types of node t for which node t will depend on the update types smask of node s.
 */
void NodeDepGraph_BGL::addDependency(int s, int t, updatemask_t smask, updatemask_t tmask) {
    auto sdesc = static_cast<GraphT::vertex_descriptor>(s);
    auto tdesc = static_cast<GraphT::vertex_descriptor>(t);

    // Check if an edge already exists between s and t; if not then create one.
    GraphT::edge_descriptor theEdge;
    bool edgeExists;
    tie(theEdge, edgeExists) = edge(sdesc, tdesc, _graph);
    
    if (edgeExists) {
        // Check if the new link can be combined with any current links
        auto it = _graph[theEdge].links.begin(), itend = _graph[theEdge].links.end();
        bool combined = false;
        NodeDepGraph_BGL::LinkLabel newlink = make_pair(smask, tmask);
        for (; !combined && it != itend; ++it) {
            combined = combineLinks(*it, newlink);
        }
        
        if (combined) {
            // If combined then we need to run a reduction round on the set of links until a minimal set of links (i.e. no further reduction is possible).
            while (combined) {
                combined = false;
                for (it = _graph[theEdge].links.begin(), itend = _graph[theEdge].links.end(); !combined && it != itend; ++it) {
                    for (auto itnext = it+1; !combined && itnext != itend; ++itnext) {
                        combined = combineLinks(*it, *itnext);
                        if (combined) {
                            // Remove the second link
                            _graph[theEdge].links.erase(itnext);
                        }
                    }
                }
                // at this point, if combined = true then we need to continue the reduction, otherwise we are done
            }
        }
        else {
            // If not combined then we simply add the new link
            _graph[theEdge].links.push_back(newlink);
        }
    }
    else {
        // Create a new edge between s and t with the new link
        tie(theEdge, edgeExists) = boost::add_edge(sdesc, tdesc, NodeDepGraph_BGL::EdgeLabel(smask, tmask), _graph);
        assert(edgeExists);
    }
}


/** Prepare the run-time dependency graph for specified updating nodes.
 \param itbegin Iterator to beginning of the list of updating nodes.
 \param n Exact number of elements (nodes to be updated)
 \return Unique pointer to a RTNodeDepGraph object
 */
RTNodeDepGraph* NodeDepGraph_BGL::getRTNodeDepGraph(GCUpdateListIterator itnode, size_t nNodes) {
    
    /* Algorithm: the run-time graph is the same graph, except that non-updating nodes are marked as REMOVED, updating nodes are marked as UNMARKED, and inactive edges are marked as "removed".
     - Mark all vertices as REMOVED.
     - Loop through list of updating nodes, switch respective vertices as UNMARKED and set its updating mask, set rtNodesLeft
     - Loop through all edges and set its "removed" property unless it is an active edge (i.e. both its source and targer are not REMOVED and at least one of its links is active).
     */
    
    GraphT::vertex_iterator vit, vitend;
    tie(vit, vitend) = vertices(_graph);
    
    // Mark all vertices as REMOVED
    for (; vit != vitend; ++vit) {
        _graph[*vit].active = false;
    }
    
    // Mark updating nodes and set their updating masks
    rtNodesLeft = nNodes;
    for (; nNodes > 0; --nNodes) {
        auto updateInfo = *(itnode++);
        assert(updateInfo.updateMask != 0); // only if the update mask is non-zero
        _graph[static_cast<GraphT::vertex_descriptor>(updateInfo.nodeID)] = VertexLabel{true, updateInfo.updateMask, 0};  // fields: active, updateMask, inputMask
    }
    
    // Loop through all edges and determine if each of them is active or inactive
    // For active ones, also calculate the inputMask of its target node
    GraphT::edge_iterator eit, eitend;
    tie(eit, eitend) = edges(_graph);
    
    for (; eit != eitend; ++eit) {
        _graph[*eit].active = false;  // mark it as inactive, only turn it to active if there is at least one active link
        
        // Get the vertices
        auto sourceNode = source(*eit, _graph);
        if (_graph[sourceNode].active) {
            auto destNode = target(*eit, _graph);
            if (_graph[destNode].active) {
                // Both source and target nodes are active
                // Check if at least one link is active
                // Also calculate the inputMask of the target node
                bool active_edge = false;
                updatemask_t inputMask = 0;
                auto sourceUpdateMask = _graph[sourceNode].updateMask, targetUpdateMask = _graph[destNode].updateMask;
                for (auto alink: _graph[*eit].links) {
                    if (alink.second & targetUpdateMask) {
                        // This link has target mask that intersects with its target node's update mask
                        inputMask |= alink.second;
                    
                        active_edge = active_edge || ((alink.first & sourceUpdateMask) != 0);
                    }
                }
                if (active_edge) {
                    // This edge is active, so we will set its status and update the inputMask of its target node
                    _graph[*eit].active = true;
                    _graph[destNode].inputMask |= inputMask;
                }
            }
        }
    }
    
    return this;
}