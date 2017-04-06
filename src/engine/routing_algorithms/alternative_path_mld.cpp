#include "engine/routing_algorithms/alternative_path.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"

#include "util/integer_range.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <iterator>
#include <memory>
#include <unordered_set>
#include <vector>

// TODO: debug, remove
#include <iostream>

namespace osrm
{
namespace engine
{
namespace routing_algorithms
{

// Alternative Routes for MLD.
//
// Start search from s and continue "for a while" when t was found. Save all vertices.
// Start search from t and continue "for a while" when s was found. Save all vertices.
// Intersect both vertex sets: these are the candidate vertices.
// For all candidate vertices c a (potentially arbitrarily bad) alternative route is (s, c, t).
// Apply heuristic to evaluate alternative route based on stretch, overlap, how reasonable it is.
//
// For MLD specifically we can pull off some tricks to make evaluating alternatives fast:
//   Only consider (s, c, t) with c border vertex: re-use MLD search steps.
//   Add meta data to border vertices: consider (s, c, t) only when c is e.g. on a highway.
//   Prune based on vertex cell id
//
// https://github.com/Project-OSRM/osrm-backend/issues/3905
InternalRouteResult
alternativePathSearch(SearchEngineData<mld::Algorithm> &search_engine_data,
                      const datafacade::ContiguousInternalMemoryDataFacade<mld::Algorithm> &facade,
                      const PhantomNodes &phantom_node_pair)
{
    search_engine_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());

    auto &forward_heap = *search_engine_data.forward_heap_1;
    auto &reverse_heap = *search_engine_data.reverse_heap_1;

    insertNodesInHeaps(forward_heap, reverse_heap, phantom_node_pair);

    // TODO: use for pruning
    // const auto &partition = facade.GetMultiLevelPartition();
    // const auto &cells = facade.GetCellStorage();

    NodeID middle = SPECIAL_NODEID;
    EdgeWeight weight = INVALID_EDGE_WEIGHT;

    EdgeWeight forward_heap_min = forward_heap.MinKey();
    EdgeWeight reverse_heap_min = reverse_heap.MinKey();

    // Let forward and reverse search space overlap by a third for searching via candidate nodes.
    const auto search_space_overlap = 1.66;

    const auto force_loop_forward = needsLoopForward(phantom_node_pair);
    const auto force_loop_backward = needsLoopBackwards(phantom_node_pair);

    std::vector<NodeID> candidates;

    while (forward_heap.Size() + reverse_heap.Size() > 0 &&
           forward_heap_min + reverse_heap_min < weight * search_space_overlap)
    {
        if (!forward_heap.Empty())
        {
            mld::routingStep<FORWARD_DIRECTION>(facade,
                                                forward_heap,
                                                reverse_heap,
                                                middle,
                                                weight,
                                                force_loop_forward,
                                                force_loop_backward,
                                                phantom_node_pair);

            if (!forward_heap.Empty())
                forward_heap_min = forward_heap.MinKey();
        }

        if (!reverse_heap.Empty())
        {
            mld::routingStep<REVERSE_DIRECTION>(facade,
                                                reverse_heap,
                                                forward_heap,
                                                middle,
                                                weight,
                                                force_loop_forward,
                                                force_loop_backward,
                                                phantom_node_pair);

            if (!reverse_heap.Empty())
                reverse_heap_min = reverse_heap.MinKey();
        }

        // Search spaces are meeting, these are the potential candidate via nodes
        if (middle != SPECIAL_NODEID)
        {
            candidates.push_back(middle);
        }
    }

    std::cout << ">>> number of candidates: " << candidates.size() << std::endl;

    std::sort(begin(candidates), end(candidates));
    auto it = std::unique(begin(candidates), end(candidates));
    candidates.erase(it, end(candidates));

    std::cout << ">>> number of unique candidates: " << candidates.size() << std::endl;

    if (weight == INVALID_EDGE_WEIGHT || middle == SPECIAL_NODEID)
        return {};

    return {};
}

} // namespace routing_algorithms
} // namespace engine
} // namespace osrm}
