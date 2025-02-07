#include "cfg.h"
using namespace std;
using namespace llvm;

node::node(const string& name) : name(name) {}

cfg::cfg(const string& funcname): funcname(funcname), ENTRY(), EXIT() {}

std::set<node *> cfg::getFullNodes() {
    std::set<node *> fullnodes = {};
    for (auto pair : nodes) {
        fullnodes.insert(pair.second);
    }
    return std::move(fullnodes);
}

void cfg::writeDotFile(const std::string& filename, std::map<node *, std::set<node *>>& OutEdges) {
    std::ofstream dotFile(filename);
    if (!dotFile.is_open()){
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    dotFile << "digraph G {" << std::endl;
    for (auto outedge : OutEdges) {
        for (auto outnode: outedge.second) {
            dotFile << "    " << outedge.first->name << " -> " << outnode->name << std::endl;
        }
    }
    dotFile << "}" << std::endl;
    dotFile.close();
}

cfg::~cfg() {
    for (auto it = nodes.begin(); it != nodes.end(); it++) {
        delete it->second;
    }
}

void cfg::insertNode(const string &name) {
    if(nodes.find(name) == nodes.end()) {
        nodes[name] = new node(name);
    }
}

void cfg::insertEdge(const string &from, const string &to) {
    node *src = nodes[from], *dst = nodes[to];
    if(outEdges.find(src) == outEdges.end()) {
        outEdges[src] = set<node *>();
    }
    outEdges[src].insert(dst);
    if(inEdges.find(dst) == inEdges.end()) {
        inEdges[dst] = set<node *>();
    }
    inEdges[dst].insert(src);
}

void cfg::findENTRY() {
    for (auto node : nodes) {
        if (inEdges[node.second].size() == 0) {
            ENTRY = node.second;
            return;
        }
    }
}

void cfg::findEXIT() {
    for (auto node :nodes) {
        if (outEdges[node.second].size() == 0) {
            EXIT = node.second;
            return;
        }
    }
}

std::set<std::string> cfg::findNonSelfLoops(){
    // Use Depth First Search to identify all non-self-loop(s) u
    std::set<std::string> non_self_loop = {};
    for (auto u : nodes) {
        std::vector<std::string> visited = {u.first};
        std::vector<std::string> stack = {};
        auto u_outedges = outEdges[u.second];

        for (auto u_outedge : u_outedges) {
            if (u_outedge->name == u.first) {
                // u is a self-loop, pass
                break;
            }
            stack.push_back(u_outedge->name);
        }
        if (isNonSelfLoopU(u.first, visited, stack)) {
            non_self_loop.insert(u.first);
        }
    }
    return std::move(non_self_loop);
}

bool cfg::isNonSelfLoopU(std::string u_name, std::vector<std::string> &visited, std::vector<std::string> &stack) {
    if (stack.empty()) {
        return false;
    }
    std::string current_v = stack.back();
    stack.pop_back();
    visited.push_back(current_v);
    auto v_outedges = outEdges[nodes[current_v]];
    for (auto v_outedge : v_outedges) {
        if (v_outedge->name == u_name) {
            return true;
        }
        if (std::find(stack.begin(), stack.end(), v_outedge->name) == stack.end()) {
            if (std::find(visited.begin(), visited.end(), v_outedge->name) == visited.end()) {
                stack.push_back(v_outedge->name);
            }
        }
    }
    return isNonSelfLoopU(u_name, visited, stack);
}

// Use Breadth-First Search to check if d is reachable from s.
// Return true if there is an s-d path, false otherwise.
bool cfg::isReachable(node* s, node* d) {
    if (s==d) {
        return true;
    }
    std::vector<node*> visited = {};
    std::queue<node*> queue = {};
    visited.push_back(s);
    queue.push(s);
    while (!queue.empty()){
        s = queue.front();
        queue.pop();
        for (auto dst : outEdges[s]) {
            if (dst == d) {
                return true;
            }
            if (std::find(visited.begin(), visited.end(), dst) == visited.end()){
                visited.push_back(dst);
                queue.push(dst);
            }
        }
    }
    return false;
}

// Yaodan's implementation. This returns the minimum path recovery set.
std::set<node *> cfg::findMinimumPRS() {
    // CREATE ENVIRONMENT AND MODEL
    GRBEnv env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.start();
    GRBModel model = GRBModel(env);

    // GET PROBLEM SIZE N
    unsigned n = nodes.size();

    // RESULT INITIALIZATION
    std::set<node *> minimumPRS = {};

    // Create an "isReachable" map for all u,v pairs.
    std::map<std::string, std::map<std::string, bool>> isReachableMap = {};

    // Initialize the 'isReachable' map.
    for (auto u : nodes) {
        for (auto v : nodes) {
            if (isReachable(u.second, v.second)) {
                isReachableMap[u.first][v.first] = true;
            } else {
                isReachableMap[u.first][v.first] = false;
            }
        }
    }

    // VARIABLE INITIALIZATION
    std::map<std::string, GRBVar> x_w;
    std::map<std::string, std::map<std::string, GRBVar>> d_u_v;

    try{
    // VARIABLE CREATION
    // For each w in V, CREATE x_w.
    for (auto node : nodes) {
        x_w[node.first] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
    }

    // For each u,v pair in V, CREATE d^u_v if there is a u-v path.
    for (auto u : nodes) {
        for (auto v : nodes) {
            if (isReachableMap[u.first][v.first]) {
                d_u_v[u.first][v.first] = model.addVar(0.0, n+1, 0.0, GRB_CONTINUOUS);
            }
            // Else do not create d^u_v for this particular u,v.
        }
    }

    // ADD CONSTRAINTS
    // For all self-loop node u, ADD CONSTRAINT x_u = 1
    for (auto u: nodes) {
        for (auto u_dst : outEdges[u.second]) {
            if (u.second == u_dst) {
                auto u_var = x_w[u.first];
                model.addConstr(u_var == 1);
            }
        }
    }

    // ADD CONSTRAINT d^u_u = 0, for all u
    for (auto u : nodes){
        model.addConstr(d_u_v[u.first][u.first]==0);
    }

    //  For all u, ADD CONSTRAINT d^u_d - d^u_s <= wt(s,d) (or x_d), for all (s,d) in E s.t. both d^u_s and d^u_d exist.
    for (auto u : nodes) {
        auto isReachableMap_from_u = isReachableMap[u.first];
        for (auto src_dsts : outEdges) {
            auto src = src_dsts.first;
            auto dsts = src_dsts.second;
            for (auto dst : dsts) {
                if (isReachableMap_from_u[src->name] && isReachableMap_from_u[dst->name]) {
                    model.addConstr(d_u_v[u.first][dst->name] - d_u_v[u.first][src->name] <= x_w[dst->name]);
                }
            }
        }
    }

    // For all u,v pair s.t. v is reachable from u, and pi != pj are v's parents, ADD CONSTRAINT d^u_pi + d^u_pj >= 1 where both d^u_pk exist.
    for (auto u : nodes) {
        for (auto v : nodes) {
            if (isReachableMap[u.first][v.first]) {
                auto p_s = inEdges[v.second];
                std::set<std::string> visited_pi_pj = {};
                for (auto pi : p_s) {
                    if (isReachableMap[u.first][pi->name]){
                        for (auto pj : p_s) {
                            if (isReachableMap[u.first][pj->name]){
                                std::string pi_pj = pi->name + pj->name;
                                std::string pj_pi = pj->name + pi->name;
                                if ((pi != pj) && (visited_pi_pj.find(pi_pj) == visited_pi_pj.end())) {
                                    visited_pi_pj.insert(pi_pj);
                                    visited_pi_pj.insert(pj_pi);
                                    model.addConstr(d_u_v[u.first][pi->name]+d_u_v[u.first][pj->name]>=1);
                                }
                                // Else do nothing.
                            }
                        }
                    }
                }
            }
            // Else do nothing.
        }
    }

    // ADD CONSTRAINT x_u + d^u_p >= 1, for all non-self-loop node u, for all u's parents p(s) where d^u_p exist.
    auto non_self_loops = findNonSelfLoops();
    for (std::string u : non_self_loops) {
        auto u_parents = inEdges[nodes[u]];
        for (auto p : u_parents) {
            if (isReachableMap[u][p->name]){
                model.addConstr(x_w[u] + d_u_v[u][p->name] >= 1);
            }
        }
    }

    // SET OBJECTIVE
    GRBLinExpr obj = 0.0;

    // ADD FIRST PART OF OBJECTIVE: [sum_u (sum_{v s.t. u-v path exists} d^u_v)]
    for (auto u : nodes) {
        for (auto v : nodes) {
            if (isReachableMap[u.first][v.first]){
                obj+=d_u_v[u.first][v.first];
            }
        }
    }

    // Add second part of objective: -(n^2 + 1)(sum_w x_w)
    for (auto x_w_it : x_w) {
        obj -= (n*n + 1) * x_w_it.second;
    }

    // SET MAXIMIZATION
    model.setObjective(obj, GRB_MAXIMIZE);

    // OPTIMIZE MODEL
    model.optimize();

    }
    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }

    // RETRIEVE MINIMUM PATH RECOVERY SET FROM OPTIMAL SOLUTION
    for (auto x_w_it : x_w) {
        auto x_w_name = x_w_it.first;
        auto x_w_var = x_w_it.second;
        auto optimal_x_w_val = x_w_var.get(GRB_DoubleAttr_X);
        if (optimal_x_w_val == 1) {
            minimumPRS.insert(nodes[x_w_it.first]);
        }
    }
    return std::move(minimumPRS);
}

// Yuta's implementation, Yaodan modified it. This returns a minimal path recovery set.
// The second and third arguments are new in edges and out edges.
// V is a user-defined node set to be removed. Default is empty, if so will traverse on all nodes.
void cfg::findMinimalPRS(std::set<node*> V) {
    set<node *> remove_V{};
    std::map<node *, std::set<node *>> newOutEdges = outEdges;
    std::map<node *, std::set<node *>> newInEdges = inEdges;
    if (V.empty()) {
        for (auto it = nodes.begin(); it != nodes.end(); it++) {
            // Directly add ENTRY and EXIT into removal set and EXCLUDE them when vertex traversal.
            if (inEdges[it->second].size()==0 || outEdges[it->second].size()==0) {
                remove_V.insert(it->second);
                continue;
            }
            V.insert(it->second);
        }
    } else {
        // User passes a vertex set V \subsetequal nodes to remove.
        for (auto v : V) {
            if (inEdges[v].size()==0 || outEdges[v].size()==0) {
                remove_V.insert(v);
            }
        }
        for (auto v : remove_V) {
            V.erase(v);
        }
    }
    // Traverse nodes and remove if we can.
    for(node * v : V) {
        bool abort_v = true;
        std::set<std::pair<node *, node *>> newEdges_v = {};
        std::map<std::pair<node *, node *>, std::string> newEdgeAnnotations_v = {};
        for(node *src : newInEdges[v]) {
            for(node *dst : newOutEdges[v]) {
                if(newOutEdges.find(src)!=newOutEdges.end() && newOutEdges[src].find(dst)!=newOutEdges[src].end()) {
                    abort_v = false;
                    break;
                } else {
                    newEdges_v.insert({src,dst});

                    // Add new edge annotations.
                    // Find old annotate(srv, v)
                    std::string annotation_src_v, annotation_v_dst;
                    if (minimal_EdgeAnnotation.find({src,v}) != minimal_EdgeAnnotation.end()) {
                        annotation_src_v = minimal_EdgeAnnotation[{src,v}];
                    } else {
                        annotation_src_v = "->";
                    }
                    // Find old annotation(v, dst)
                    if (minimal_EdgeAnnotation.find({v, dst}) != minimal_EdgeAnnotation.end()) {
                        annotation_v_dst = minimal_EdgeAnnotation[{v,dst}];
                    } else {
                        annotation_v_dst = "->";
                    }
                    // Add new edge annotation
                    newEdgeAnnotations_v[{src,dst}] = annotation_src_v + v->name + annotation_v_dst;
                }
            }

            if(!abort_v) {
                break;
            }
        }
        // v can be removed.
        if(abort_v) {
            // Remove v
            remove_V.insert(v);

            // Remove v's in edges and their annotations
            auto v_srcs = newInEdges[v];
            newInEdges.erase(v);
            for (auto v_src : v_srcs) {
                newOutEdges[v_src].erase(v);
                minimal_EdgeAnnotation.erase({v_src, v});
            }

            // Remove v's out edges and their annotations
            auto v_dsts = newOutEdges[v];
            newOutEdges.erase(v);
            for (auto v_dst : v_dsts) {
                newInEdges[v_dst].erase(v);
                minimal_EdgeAnnotation.erase({v, v_dst});
            }

            // Add new edges
            for (auto newedge : newEdges_v) {
                newInEdges[std::get<1>(newedge)].insert(std::get<0>(newedge));
                newOutEdges[std::get<0>(newedge)].insert(std::get<1>(newedge));
            }

            // Add new edge annotations
            minimal_EdgeAnnotation.insert(newEdgeAnnotations_v.begin(), newEdgeAnnotations_v.end());

        }
    }
    set<node *> result = {};
    std::set_difference(V.begin(), V.end(), remove_V.begin(), remove_V.end(), std::inserter(result, result.begin()));

    // Write to graph's minimal PRS members.
    minimal_PRS = result;
    minimal_inEdges = newInEdges;
    minimal_outEdges = newOutEdges;

    return;
}

static void mergeEdges(map<node *, set<node *>> &tmpOutEdges, map<node *, set<node *>> &outEdges, map<node *, set<node *>> &inEdges) {
    for(auto it = tmpOutEdges.begin(); it != tmpOutEdges.end(); it++) {
        if(outEdges.find(it->first) == outEdges.end()) {
            outEdges[it->first] = set<node *>();
        }
        for(auto it2 = it->second.begin(); it2 != it->second.end(); it2++) {
            outEdges[it->first].insert(*it2);
            if(inEdges.find(*it2) == inEdges.end()) {
                inEdges[*it2] = set<node *>();
            }
            inEdges[*it2].insert(it->first);
        }
    }
    tmpOutEdges.clear();
}

// this returns a set of nodes that can be removed from the graph
set<node *> cfg::findMinimalNodes(PathRecoveryOrder order) {
    set<node *> ret{};
    vector<node *> v{};
    for (auto it = nodes.begin(); it != nodes.end(); it++) {
        v.push_back(it->second);
    }
    srand(unsigned(time(nullptr)));

    switch (order) {
        case PathRecoveryOrder::RANDOM:
            random_shuffle(v.begin(), v.end());
            break;
        case PathRecoveryOrder::INDEGREE:
            std::sort(v.begin(), v.end(), [&](node *a, node *b) {
                return inEdges[a].size() < inEdges[b].size();
            });
            break;
        case PathRecoveryOrder::OUTDEGREE:
            std::sort(v.begin(), v.end(), [&](node *a, node *b) {
                return outEdges[a].size() < outEdges[b].size();
            });
            break;
        case PathRecoveryOrder::BOTH:
            std::sort(v.begin(), v.end(), [&](node *a, node *b) {
                return inEdges[a].size() * outEdges[a].size() < inEdges[b].size() * outEdges[b].size();
            });
            break;
        default:
            random_shuffle(v.begin(), v.end());
            break;
    }

    map<node *, set<node *>> newOutEdges = outEdges, newInEdges = inEdges, tmpOutEdges;
    bool flag = false;
    for(node *n : v) {
        flag = false;
        if(newInEdges.find(n) == newInEdges.end() || newInEdges[n].size() == 0
                || newOutEdges.find(n) == newOutEdges.end() || newOutEdges[n].size() == 0) {
            continue;
        }
        for(node *src : newInEdges[n]) {
            for(node *dst : newOutEdges[n]) {
                if(newOutEdges.find(src) != newOutEdges.end() && newOutEdges[src].find(dst) != newOutEdges[src].end()) {
                    flag = true;
                    break;
                } else {
                    if(tmpOutEdges.find(src) == tmpOutEdges.end()) {
                        tmpOutEdges[src] = set<node *>();
                    }
                    tmpOutEdges[src].insert(dst);
                }
            }
            if(flag) break;
        }
        if(!flag) {
            /* errs() << "Removing node: " << n->name << "\n"; */
            ret.insert(n);
            mergeEdges(tmpOutEdges, newOutEdges, newInEdges);
        }
    }
    return ret;
}



void cfg::storeCFGToFile(std::string cfg_function_name, node *ENTRY, node *EXIT, std::map<std::pair<node *, node *>, std::string> minimal_EdgeAnnotation, std::map<std::string, node *> nodes) {
    std::ofstream cfg_file("cfg.txt", std::ios::app);
    // all the info in cfg
    cfg_file << "FUNCTION: " << cfg_function_name << "\n";
    cfg_file << "ENTRY: " << ENTRY->name << "\n";
    cfg_file << "EXIT: " << EXIT->name << "\n";
    cfg_file << "Edges: \n";
    for (auto it = minimal_EdgeAnnotation.begin(); it != minimal_EdgeAnnotation.end(); it++) {
        cfg_file << it->first.first->name << " -> " << it->first.second->name << " : " << it->second << "\n";
    }
    cfg_file << "Nodes: ";
    for (auto it = nodes.begin(); it != nodes.end(); it++) {
        cfg_file << it->first << ";";
    }
    cfg_file << "\n";
}

/*
input: signature
output: path
 */

std::string cfg::RecoverPath(std::vector<std::string> signature, std::map<std::pair<node *, node *>, std::string> minimal_EdgeAnnotation, std::map<std::string, node *> nodes, node *ENTRY, node *EXIT) {

    // Start at ENTRY
    std::string path = ENTRY->name;
    node * current = ENTRY;

    // Include EXIT into signature for convenience exit purposes
    signature.push_back(EXIT->name);

    // Traverse the minimal CFG and recover path using edge annotations
    for (auto s : signature) {
        node * next = nodes[s];
        std::string edge_annotation = minimal_EdgeAnnotation[{current, next}];
        if (edge_annotation == "") {
            path = path + "->" + next->name;
        } else {
            path = path + edge_annotation + next->name;
        }
        current = next;
    }

    return path;
}

void cfg::printGraph() {
    // Print CFG with colored output for better readability
    errs() << "\nCFG for function: \n";
    errs() << funcname << "\n";

    // Print edges, one per line
    errs() << "Edges: \n";
    for (auto it = outEdges.begin(); it != outEdges.end(); it++) {
        for (auto it2 = it->second.begin(); it2 != it->second.end(); it2++) {
            errs() << it->first->name;
            errs() << " -> ";
            errs() << (*it2)->name;
            errs() << "\n";
        }
    }
    errs().resetColor();
}