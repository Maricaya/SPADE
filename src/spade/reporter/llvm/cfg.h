/* Self-Defined Control Flow Graph Class of a program */
#ifndef CFG_H
#define CFG_H
#include <set>
#include <map>
#include <string>
#include <vector>
#include <ctime>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <chrono>
#include <queue>

#include "llvm/Support/raw_ostream.h"
#include"/opt/gurobi1103/linux64/include/gurobi_c++.h"

using namespace std;

class node;

class node {
public:
    node() = delete;
    node(const std::string& name);
    node(const node&) = default;
    node& operator=(const node&) = default;
    ~node() = default;
    string name;
};

enum class PathRecoveryOrder {
    RANDOM,
    INDEGREE,
    OUTDEGREE,
    BOTH
};

class cfg {
public:
    cfg() = delete;
    cfg(const std::string& funcname);
    cfg(const cfg&) = default;
    cfg& operator=(const cfg&) = default;
    ~cfg();

    void findENTRY();
    void findEXIT();

    void insertEdge(const std::string& from, const std::string& to);
    void insertNode(const std::string& name);

    std::set<std::string> findNonSelfLoops();
    bool isNonSelfLoopU(std::string u_name, std::vector<std::string> &visited, std::vector<std::string> &stack);

    bool isReachable(node* s, node* d);

    std::set<node *> getFullNodes();
    void findMinimalPRS(std::set<node*> V={});
    std::set<node *> findMinimumPRS();
    std::set<node *> findMinimalNodes(PathRecoveryOrder order = PathRecoveryOrder::RANDOM);

    // Recover a Path from runtime signature on the minimal PRS
    std::string RecoverPath(std::vector<std::string> signature, std::map<std::pair<node *, node *>, std::string> minimal_EdgeAnnotation, std::map<std::string, node *> nodes, node *ENTRY, node *EXIT);

    void printGraph();

    void writeDotFile(const std::string& filename, std::map<node *, std::set<node *>>& OutEdges);
    void storeCFGToFile(std::string cfg_function_name, node *ENTRY, node *EXIT, std::map<std::pair<node *, node *>, std::string> minimal_EdgeAnnotation, std::map<std::string, node *> nodes);


    std::string funcname;

    node * ENTRY;
    node * EXIT;
    std::map<std::string, node *> nodes;
    std::map<node *, std::set<node *>> outEdges;
    std::map<node *, std::set<node *>> inEdges;

    std::set<node *> minimal_PRS;
    std::map<node *, std::set<node *>> minimal_outEdges;
    std::map<node *, std::set<node *>> minimal_inEdges;
    std::map<std::pair<node *, node *>, std::string> minimal_EdgeAnnotation;
};
#endif