/* Written against the LLVM 3.0 release.
 * Based on TraceValues pass of LLVM 1.x
 *
 * Usage:
 * gcc llvmReporterLib.c -c -o llvmReporterLib.o
 * g++ LLVMReporter.cpp -shared -o LLVMReporter.so -I$(LLVM_INCLUDE_DIR) -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS
 * (On Ubuntu 12.04 LLVM_INCLUDE_DIR = /usr/lib/llvm-3.0/include)
 *
 * "make" on either Mac or Linux; bug reports to Ian.Mason@SRI.com
 *
 * Here are the current changes (by Ian):
 * 1. Fixed the conflation of FILE* with a 32 bit int, that crashes on 64 bit machines.
 * 2. Solved the "Daemon problem" via adding a "smart close" to the system, then doing
 * some linking magic (Chris Dodd and Bruno helped here).
 * 3. Added some cleaner makefiles.
 */

#include "llvm/IR/Constants.h"
#include "llvm/Pass.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Type.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/ADT/Twine.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/IR/Value.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/CallSite.h"
#include "llvm/IR/InstIterator.h"

#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/CallGraphSCCPass.h"

#include "cfg.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>
#include <memory>

using namespace llvm;
using namespace std;

// Global accumulators for statistics.
unsigned BBNum = 0;
unsigned minimalBBNum = 0;
unsigned minimumBBNum = 0;

unsigned allFunctionNum = 0;
unsigned reducedFunctionNum = 0;

namespace {
    // Value* GetTid; //the syscall argument for getting a Thread ID is different depending on the operating systems.
    std::set<std::string> globalMinimalPRSNodes;

    // Returns the appropriate specifier for printf based on the type of variable being printed
    static std::string getPrintfCodeFor(const Value *V) {
        if (V == 0) return "";
        if (V->getType()->isFloatingPointTy())
            return "%g";
        else if (V->getType()->isLabelTy())
            return "0x%p";
        else if (V->getType()->isPointerTy())
            return "%p";
        else if (V->getType()->isIntegerTy())
            return "%d";
        return "%n";
    }

    static inline bool isInMinimalSet(std::string& functionName) {
        errs().changeColor(raw_ostream::BLUE, true) << "Function Name: " << functionName << "\n";
        for (const auto& node : globalMinimalPRSNodes) {
            errs().changeColor(raw_ostream::YELLOW, true) << "Node Name: " << node << "; ";
            if (node == functionName) {
                errs().changeColor(raw_ostream::GREEN, true) << "isInMinimalSet: true\n";
                return true;
            }
        }
        errs().changeColor(raw_ostream::RED, true) << "isInMinimalSet: false\n";
        errs().resetColor();
        return false;
        // return true;
    }


    // Creating a Global variable for string 'str', and returning the pointer to the Global
    static inline GlobalVariable *getStringRef(Module *M, const std::string &str) {
        Constant *Init = ConstantDataArray::getString(M->getContext(), str);
        Twine twine("trstr");
        GlobalVariable *GV = new GlobalVariable(Init->getType(), true, GlobalVariable::InternalLinkage, Init, twine);
        M->getGlobalList().push_back(GV);
        return GV;
    }

    static void InsertPrintInstruction(
            std::vector<Value*> PrintArgsIn,
            BasicBlock *BB,
            Instruction *InsertBefore,
            std::string Message,
            Function *SPADEThreadIdFunc,
            Function * pidFunction,
	        Function *BufferStrings
        ){

        errs() << "Debug - Message to be sent: " << Message;


        Module *Mod = BB->getParent()->getParent(); //BasicBlock is a child of Function which is a child of Module

        //TODO: Add Single call to Pid and ThreadHandle per function entry/exit
        //TODO: Call to pid function must be conditional on a command line flag
        //Insert Call to getpid function
        CallInst* pid = CallInst::Create((Value*) pidFunction, Twine("pid"), &(*InsertBefore));
        Message += " #Pid = %d";
        PrintArgsIn.push_back(pid);

        //Insert function to get Thread Identifier
        CallInst* threadHandle = CallInst::Create((Value*) SPADEThreadIdFunc, Twine("ThreadHandle"), &(*InsertBefore)); //gets the fd for the getThreadId SPADE library fn
        threadHandle->setTailCall();

        //Message is the string argument to fprintf. GEP is used for getting the handle to Message.
        GlobalVariable *fmtVal;
        fmtVal = getStringRef(Mod, Message);
        Constant *GEP = ConstantExpr::getGetElementPtr((Constant*) fmtVal, ArrayRef<Constant*>(std::vector<Constant*>(2, Constant::getNullValue(Type::getInt64Ty(Mod->getContext())))), 2);

        //Arguments for fprintf. socketHandle, Message and Thread ID followed by arguments
        std::vector<Value*> PrintArgs;

        PrintArgs.push_back(GEP);
        PrintArgs.push_back(threadHandle);

        for (unsigned i = 0; i < PrintArgsIn.size(); i++) {
            PrintArgs.push_back(PrintArgsIn[i]);
        }

        if(BufferStrings != NULL){
            // printf("function was retrieved \n");
        }

	    CallInst::Create((Value*) BufferStrings, ArrayRef<Value*>(PrintArgs), Twine(""), &(*InsertBefore));
    }

    static inline void FunctionEntry(
            Function &F,
            Function *SPADEThreadIdFunc,
            Function * pidFunction,
	        Function *BufferStrings,
            bool useBufferStrings
        ){
        Module *M = F.getParent();
        BasicBlock &BB = F.getEntryBlock();
        Instruction *InsertPos = BB.begin();

        std::string functionName = F.getName().str();

        // 1. create a global variable for the function name
        GlobalVariable *funcNameGV = getStringRef(M, functionName);

        // 2. create a declaration for the pushCallStack function
        PointerType *charPtrTy = Type::getInt8PtrTy(M->getContext());
        std::vector<Type*> pushTypes;
        pushTypes.push_back(charPtrTy);
        FunctionType *pushTy = FunctionType::get(
            Type::getVoidTy(M->getContext()),
            pushTypes,
            false
        );

        // 3. get or create the pushCallStack function
        Function *pushFunc = cast<Function>(
            M->getOrInsertFunction("pushCallStack", pushTy)
        );

        // 4. create a GEP instruction to get the string pointer
        std::vector<Constant*> indices;
        indices.push_back(ConstantInt::get(Type::getInt32Ty(M->getContext()), 0));
        indices.push_back(ConstantInt::get(Type::getInt32Ty(M->getContext()), 0));
        Constant *strPtr = ConstantExpr::getGetElementPtr(funcNameGV, indices);

        // 5. create a call to the pushCallStack function
        std::vector<Value*> pushArgs;
        pushArgs.push_back(strPtr);
        CallInst::Create(pushFunc, ArrayRef<Value*>(pushArgs), "", InsertPos);

        std::string printString;
        std::string argName;
        raw_string_ostream strStream(printString);

        if (isInMinimalSet(functionName)) {
            printString = "%lu E: @" + functionName;
        } else {
            reducedFunctionNum++;
            printString = "%lu E: @***null***";
        }

        unsigned ArgNo = 0;
        std::vector<Value*> PrintArgs;
        for (Function::arg_iterator iterator = F.arg_begin(), E = F.arg_end(); iterator != E; ++iterator, ++ArgNo) {
            if (iterator) {
                argName = "";
                raw_string_ostream strStream2(argName);
                iterator->printAsOperand(strStream2, true, F.getParent());
                argName = strStream2.str();

                //Escaping % in argName
                std::string Tmp;
                std::swap(Tmp, argName);
                std::string::iterator J = std::find(Tmp.begin(), Tmp.end(), '%');
                while (J != Tmp.end()) {
                    argName.append(Tmp.begin(), J);
                    argName += "%%";
                    ++J;
                    Tmp.erase(Tmp.begin(), J);
                    J = std::find(Tmp.begin(), Tmp.end(), '%');
                }
                argName += Tmp;

                PrintArgs.push_back((Value*) iterator);
                printString = printString + " Arg #" + utostr(ArgNo) + ": " + argName + " =" + getPrintfCodeFor(iterator);
            }
        }

        // 3. add call chain information
        printString = printString + " CallChain: %s";

        // get or create the getCallTrace function
        std::vector<Type*> traceTypes;
        FunctionType *traceTy = FunctionType::get(
            Type::getInt8PtrTy(M->getContext()),
            traceTypes,
            false
        );

        Constant *getCallTraceFunc = M->getOrInsertFunction(
            "getCallTrace",
            traceTy
        );

        // create a call to the getCallTrace function
        CallInst *callTraceCall = CallInst::Create(
            getCallTraceFunc,
            ArrayRef<Value*>(),
            "",
            InsertPos
        );

        PrintArgs.push_back(callTraceCall);
        printString = printString + "\n";

        InsertPrintInstruction(PrintArgs, &BB, InsertPos, printString, SPADEThreadIdFunc, pidFunction, BufferStrings);
    }

    static inline void FunctionExit(
            BasicBlock *BB,
            Function *SPADEThreadIdFunc,
            Function * pidFunction,
	        Function *BufferStrings
    ) {
        Module *M = BB->getParent()->getParent();

        // insert a call to the popCallStack function before the return instruction
        std::vector<Type*> popParamTypes;
        FunctionType *popCallStackTy = FunctionType::get(
            Type::getVoidTy(M->getContext()),
            popParamTypes,
            false
        );

        Constant *popCallStackFunc = M->getOrInsertFunction(
            "popCallStack",
            popCallStackTy
        );

        // insert a call to the popCallStack function before the return instruction
        CallInst::Create(
            popCallStackFunc,
            ArrayRef<Value*>(),
            "",
            BB->getTerminator()
        );

        ReturnInst *Ret = (ReturnInst*) (BB->getTerminator());

        std::string printString;
        std::string retName;
        raw_string_ostream strStream(printString);
        raw_string_ostream strStream2(retName);

	    std::string functionName = BB->getParent()->getName().str();
        // printString = "%lu L: @" + BB->getParent()->getName().str(); //WAS %d NOW IS %lu is for Thread ID, L is for Function Leave
        if (isInMinimalSet(functionName)) {
            printString = "%lu L: @" + functionName; //WAS %d NOW IS %lu is for Thread ID, L is for Function Leave
        } else {
            printString = "%lu L: @***null***"; //WAS %d NOW IS %lu is for Thread ID, L is for Function Leave
        }

        std::vector<Value*> PrintArgs;
        if (!BB->getParent()->getReturnType()->isVoidTy()) {

            printString = printString + "  R:  "; //R indicates tInsertPrintInstructionhe return value
	    Ret->getReturnValue()->printAsOperand(strStream2, true, BB->getParent()->getParent());
	    retName = strStream2.str();

            //Escaping % in retName
            std::string Tmp;
            std::swap(Tmp, retName);
            std::string::iterator I = std::find(Tmp.begin(), Tmp.end(), '%');
            //While there are % in Tmp
            while (I != Tmp.end()) {
                retName.append(Tmp.begin(), I);
                retName += "%%";
                ++I; // Skip the % at the current location
                Tmp.erase(Tmp.begin(), I);
                I = std::find(Tmp.begin(), Tmp.end(), '%');
            }

            retName += Tmp;
            printString = printString + retName + " =" + getPrintfCodeFor(Ret->getReturnValue());
            PrintArgs.push_back(Ret->getReturnValue());
        }

        printString = printString + "\n";

        InsertPrintInstruction(PrintArgs, BB, Ret, printString, SPADEThreadIdFunc, pidFunction, BufferStrings);

    }

    static inline bool shouldSkipFunction(const std::string& funcName, bool monitorMethods, const std::map<std::string, int>& methodsToMonitor) {
    // Skip if function is not in monitored list or is a special function
    return (monitorMethods && methodsToMonitor.find(funcName) == methodsToMonitor.end())
           || funcName == "bufferString"
           || funcName == "flushStrings"
           || funcName == "setAtExit";
}

   static cl::opt<std::string> ArgumentsFileName("FunctionNames-input", cl::init(""), cl::Hidden, cl::desc("specifies the input file for the functionNames"));


    class InsertMetadataCode : public ModulePass {
    protected:
        Function* PrintfFunc;
        Function* SPADESocketFunc;
        Function* SPADEThreadIdFunc;
        Function* pidFunction;
        Function* BufferStrings;
        bool monitorMethods;
        bool useBufferStrings;

	    std::map<std::string, int> methodsToMonitor;
    public:
        static char ID; // Pass identification, replacement for typeid

        InsertMetadataCode() : ModulePass(ID) {
        }

        bool doInitialization(Module &M)  {
            FunctionType * ft = FunctionType::get(Type::getInt32Ty(M.getContext()), false);
            pidFunction = Function::Create(ft, GlobalValue::ExternalLinkage, "getpid", &M);


            std::string fileName = ArgumentsFileName == "" ? "arguments" : ArgumentsFileName.getValue().c_str();
            if (strcmp(fileName.c_str(), "-monitor-all") != 0){

                std::ifstream file(fileName);
                cout<<"Invoked Do Initialization"<< fileName << endl;
                std::string str;

                while (std::getline(file, str))
                {
                    methodsToMonitor[str] = 1;
                    cout<< " Function name FROM FILE : " << str <<"\n";
                }
                monitorMethods = true;
            }
            else{
                monitorMethods = false;
            }


            // Setting up argument types for fprintf
            Type *CharTy = Type::getInt8PtrTy(M.getContext());

            //Ian says FILE* can't be considered a 32 but int on a 64 bit machine.
            //This is for FILE*
            Type *GenericPtr = Type::getInt8PtrTy(M.getContext());

            //64 bit rather than 32?
            //Type *IntTy = Type::getInt32Ty(M.getContext());
            Type *IntTy = Type::getInt64Ty(M.getContext());

            std::vector<Type*> args;
            args.push_back(GenericPtr); //IAM was IntTy
            args.push_back(CharTy);

            //Getting handle for fprintf
            FunctionType *MTy = FunctionType::get(IntTy, ArrayRef<Type*>(args), true);
            PrintfFunc = (Function*) M.getOrInsertFunction("fprintf", MTy);

            //Getting handle for SPADEThreadIdFunc
            //This is used for getting a handle to the OS dependent LLVM_getThreadId() function
            MTy = FunctionType::get(IntTy, false);
            SPADEThreadIdFunc = (Function*) M.getOrInsertFunction("LLVMReporter_getThreadId", MTy);

            //Getting handle for SPADESocketFunc
            //This is used for getting a handle to the socket to SPADE
            MTy = FunctionType::get(GenericPtr, false); //IAM was IntTy
            SPADESocketFunc = (Function*) M.getOrInsertFunction("LLVMReporter_getSocket", MTy);

            BufferStrings = M.getFunction("bufferString");
            useBufferStrings = (BufferStrings != NULL);
            if(BufferStrings == NULL){
                printf(" *** Function not found in module **** \n");
                errs() << "Warning: bufferString function not found, falling back to fprintf\n";
            }

            return false;
        }

        bool runOnModule(Module &M)  {
            errs().changeColor(raw_ostream::MAGENTA, true);
            errs() << "\n=== Starting Module Analysis ===\n\n";
            errs().resetColor();
            errs().changeColor(raw_ostream::CYAN, true);
            errs() << "Available functions in module:\n";
            for (auto &F : M) {
                if (!F.isDeclaration()) {  // only count functions with definitions
                    allFunctionNum++;
                    errs() << "  - " << F.getName() << "\n";
                }
            }
            errs().resetColor();

            CallGraph &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();

            Function *MainFunc = M.getFunction("main");
            if (!MainFunc) {
                errs().changeColor(raw_ostream::RED, true) << "Warning: No main function found\n";
                errs().resetColor();
                return false;
            }

            errs().changeColor(raw_ostream::MAGENTA, true) << "\n=== Starting Module Analysis ===\n\n";
            errs().resetColor();

            // for tracking processed functions
            std::set<Function*> processed;

            processCallTree(CG.getOrInsertFunction(MainFunc), processed);

            // process other unprocessed functions
            for (auto &Node : CG) {
                Function *F = Node.second->getFunction();
                if (!F || F->isDeclaration() || processed.count(F)) {
                    continue;
                }
                processFunction(*F);
            }

            // print global statistics after all functions are processed
            errs().changeColor(raw_ostream::YELLOW, true);
            errs() << "\n=== Global Statistics ===\n";
            errs() << "Total functions: " << allFunctionNum << "\n";
            errs() << "Preserved functions: " << reducedFunctionNum << "\n";
            errs() << "Removed functions: " << (allFunctionNum - reducedFunctionNum) << "\n";
            errs() << "Reduction ratio: " << ((allFunctionNum - reducedFunctionNum) * 100.0 / allFunctionNum) << "%\n";
            errs() << "Total basic blocks: " << bbNum << "\n";
            errs() << "======================\n\n";
            errs().resetColor();

            errs().changeColor(raw_ostream::MAGENTA, true);
            errs() << "\n=== Module Analysis Complete ===\n\n";
            errs().resetColor();

            return true;
        }

        void processCallTree(CallGraphNode *Node, std::set<Function*> &processed) {
            Function *F = Node->getFunction();
            if (!F || F->isDeclaration() || processed.count(F)) {
                return;
            }

            // process the current function first
            int depth = 1;
            std::string indent(depth * 2, ' ');
            errs().changeColor(raw_ostream::GREEN, true) << indent << "→ Processing function: ";
            errs().changeColor(raw_ostream::CYAN, true) << F->getName() << "\n";
            errs().resetColor();
            processFunction(*F);
            processed.insert(F);

            // process all called functions recursively
            for (auto &CallRecord : *Node) {
                CallGraphNode *CalleeNode = CallRecord.second;
                processCallTree(CalleeNode, processed);
            }

            errs().changeColor(raw_ostream::BLUE, true) << indent << "✓ Completed: ";
            errs().changeColor(raw_ostream::CYAN, true) << F->getName() << "\n";
            errs().resetColor();
        }

        void processFunction(Function &F) {
            if(strcmp(F.getName().str().c_str(), "main") == 0){
                Function * exitingFunction = F.getParent()->getFunction("setAtExit");
                if(exitingFunction == NULL){
                    errs() << "Warning: setAtExit function not found, falling back to fprintf\n";
                    printf("setAtExit was not retrieved \n");
                } else {
                    BasicBlock &BB = F.getEntryBlock();
                    Instruction *InsertPos = BB.begin();
                    CallInst::Create((Value*) exitingFunction, Twine(""), &(*InsertPos));
                }
            }


	        // If the function is not supposed to be monitored just return true
	        if (shouldSkipFunction(F.getName().str(), monitorMethods, methodsToMonitor)) {
                return;
            }

                        errs().changeColor(raw_ostream::WHITE, true) << "  Analyzing CFG for: " << F.getName() << "\n";
            errs().resetColor();

            // Create a new CFG for the current function.
            constructCFG(F);

            FunctionEntry(F, SPADEThreadIdFunc, pidFunction, BufferStrings, useBufferStrings);
            for (BasicBlock &BB : F) {
                if (isa<ReturnInst>(BB.getTerminator())) {
                    FunctionExit(&BB, SPADEThreadIdFunc, pidFunction, BufferStrings);
                }
            }
        }

        void insertPRSNodes2Global(set<node*> minimalPRSNodes) {
            for (auto *Node : minimalPRSNodes) {
                errs().changeColor(raw_ostream::CYAN, true) << "      - " << Node->name << "\n";
                errs().resetColor();
                globalMinimalPRSNodes.insert(Node->name);
            }
        }

        void constructCFG(Function &F) {
            // Create a new CFG for the current function.
            string funcName = F.getName().str();
            cfg *graph = new cfg(funcName);

            // Data structures for building the basic block mapping.
            unsigned bbCount = 0;
            unsigned funcBBnum = 0;
            map<BasicBlock*, vector<string>> bbFunctionsMap; // Maps a BB to a list of node names.
            map<BasicBlock*, unsigned> bbNumberMap;          // Assigns each BB an index.
            map<string, unsigned> functionCallCount;         // Counts call occurrences.

            // ------------------------------------------------------------------
            // 1. Build the basic block mapping.
            // For each basic block, add a "Head" marker, record each called function,
            // and add a "Tail" marker.
            // ------------------------------------------------------------------
            for (BasicBlock &bb : F) {
            funcBBnum++;
            string headMarker = "BasicBlock_" + to_string(bbCount) + "_Head";
            string tailMarker = "BasicBlock_" + to_string(bbCount) + "_Tail";
            vector<string> nodeList;
            nodeList.push_back(headMarker);

            for (Instruction &I : bb) {
                if (CallInst *callInst = dyn_cast<CallInst>(&I)) {
                if (Function *callee = callInst->getCalledFunction()) {
                    string calleeName = callee->getName().str();
                    // Update call count (if needed for naming or statistics).
                    functionCallCount[calleeName] += 1;
                    // You may append the call count if desired:
                    // string callNode = calleeName + "_" + to_string(functionCallCount[calleeName]);
                    nodeList.push_back(calleeName);
                }
                }
            }

            nodeList.push_back(tailMarker);
            bbFunctionsMap[&bb] = nodeList;
            bbNumberMap[&bb] = bbCount;
            bbCount++;
            }

            // ------------------------------------------------------------------
            // 2. Insert nodes into the CFG.
            // ------------------------------------------------------------------
            for (const auto &entry : bbFunctionsMap) {
            for (const auto &nodeName : entry.second) {
                graph->insertNode(nodeName);
            }
            }

            // ------------------------------------------------------------------
            // 3. Insert inter-basic block edges.
            // For each basic block, link its tail to the head of each successor.
            // For back edges, link the tail to the successor's tail.
            // ------------------------------------------------------------------
            // for (BasicBlock &bb : F) {
            // for (BasicBlock *succ : successors(&bb)) {
            //     if (bbNumberMap[&bb] < bbNumberMap[succ])
            //     graph->insertEdge(bbFunctionsMap[&bb].back(), bbFunctionsMap[succ].front());
            //     else
            //     graph->insertEdge(bbFunctionsMap[&bb].back(), bbFunctionsMap[succ].back());
            // }
            // }

            for (BasicBlock &bb : F) {
                for (succ_iterator SI = succ_begin(&bb), SE = succ_end(&bb); SI != SE; ++SI) {
                    BasicBlock *succ = *SI;
                    if (bbNumberMap[&bb] < bbNumberMap[succ]) {
                        graph->insertEdge(bbFunctionsMap[&bb].back(), bbFunctionsMap[succ].front());
                    } else {
                        graph->insertEdge(bbFunctionsMap[&bb].back(), bbFunctionsMap[succ].back());
                    }
                }
            }

            // ------------------------------------------------------------------
            // 4. Insert intra-basic block edges.
            // For each basic block, link consecutive nodes.
            // ------------------------------------------------------------------
            for (BasicBlock &bb : F) {
            const vector<string> &nodeList = bbFunctionsMap[&bb];
            for (size_t i = 0, n = nodeList.size(); i < n - 1; i++) {
                graph->insertEdge(nodeList[i], nodeList[i+1]);
            }
            }

            // Write out the preliminary (premature) CFG to a DOT file.
            graph->writeDotFile(funcName + "_premature.dot", graph->outEdges);

            // ------------------------------------------------------------------
            // 5. Reduce non-function nodes (i.e. basic block markers).
            // Remove nodes with names beginning with "BasicBlock" and rewire edges.
            // ------------------------------------------------------------------
            map<string, node*> nodesCopy;
            // Copy nodes that are not entry/exit nodes.
            for (auto &kv : graph->nodes) {
            node *n = kv.second;
            if (graph->inEdges[n].empty() || graph->outEdges[n].empty())
                continue;
            nodesCopy.insert(kv);
            }

            for (auto &kv : nodesCopy) {
            const string &nodeName = kv.first;
            node *n = kv.second;
            if (nodeName.substr(0, 10) == "BasicBlock") {
                set<pair<node*, node*>> newEdges;
                // For every predecessor and successor of n, add an edge if not already present.
                for (node *src : graph->inEdges[n]) {
                for (node *dst : graph->outEdges[n]) {
                    if (graph->outEdges[src].find(dst) == graph->outEdges[src].end())
                    newEdges.insert({src, dst});
                }
                }

                // Remove node n from the graph.
                graph->nodes.erase(nodeName);

                // Remove n from inEdges and outEdges of connected nodes.
                auto inNodes = graph->inEdges[n];
                graph->inEdges.erase(n);
                for (node *src : inNodes) {
                graph->outEdges[src].erase(n);
                }
                auto outNodes = graph->outEdges[n];
                graph->outEdges.erase(n);
                for (node *dst : outNodes) {
                graph->inEdges[dst].erase(n);
                }

                // Insert the new direct edges.
                for (auto &edgePair : newEdges) {
                node *src = edgePair.first;
                node *dst = edgePair.second;
                graph->inEdges[dst].insert(src);
                graph->outEdges[src].insert(dst);
                }
            }
            }

            // Reset the graph's ENTRY and EXIT.
            graph->findENTRY();
            graph->findEXIT();

            // ------------------------------------------------------------------
            // 6. Apply PRS on the function-level CFG.
            // ------------------------------------------------------------------
            set<node*> fullNodes = graph->getFullNodes();
            unsigned funcBBNum = fullNodes.size();

            // Optionally, you can choose a heuristic for the minimum PRS.
            graph->findMinimalPRS();

            // Update global statistics.
            BBNum += funcBBNum;
            minimalBBNum += graph->minimal_PRS.size();

            // Store the final CFG to a file.
            graph->storeCFGToFile(funcName, graph->ENTRY, graph->EXIT,
                                graph->minimal_EdgeAnnotation, graph->nodes);

            insertPRSNodes2Global(graph->minimal_PRS);



            delete graph;
        }


        void getAnalysisUsage(AnalysisUsage &AU) const  {
            AU.addRequired<CallGraphWrapperPass>();
        }
    };

    char InsertMetadataCode::ID = 0;
    static RegisterPass<InsertMetadataCode> X("provenance", "insert provenance instrumentation");

}