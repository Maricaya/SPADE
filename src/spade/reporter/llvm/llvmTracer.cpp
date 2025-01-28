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
// #include "llvm/Support/InstIterator.h"
#include "llvm/IR/InstrTypes.h"

#include "cfg.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include <string.h>

using namespace llvm;
using namespace std;

namespace {
    Value* GetTid; //the syscall argument for getting a Thread ID is different depending on the operating systems.
    std::set<node*> globalMinimalPRSNodes;


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

    static inline bool isInMinimalSet(std::string functionName) {
        std::cout << "Function Name: " << functionName << std::endl;
        for (const auto& node : globalMinimalPRSNodes) {
            if (node->name == functionName) {
                std::cout << "isInMinimalSet: true" << std::endl;
                return true;
            }
        }
        std::cout << "isInMinimalSet: false" << std::endl;
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
	    //Get the first instruction of the first Basic Block in the function
        BasicBlock &BB = F.getEntryBlock();
        Instruction *InsertPos = BB.begin();

        std::string printString;
        std::string argName;

        raw_string_ostream strStream(printString);
        //Prints the function name to strStream

	    //F.printAsOperand(strStream, true, F.getParent());
	    std::string functionName;
        functionName = F.getName().str();
        if (isInMinimalSet(functionName)) {
            printString = "%lu E: @" + functionName; //WAS  %d  now is %lu is for Thread ID, E is for Function Entry
        } else {
            printString = "%lu E: @***null***"; //WAS  %d  now is %lu is for Thread ID, E is for Function Entry
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
                //While there are % in Tmp
                while (J != Tmp.end()) {
                    argName.append(Tmp.begin(), J);
                    argName += "%%";
                    ++J; // Skip the % at the current location
                    Tmp.erase(Tmp.begin(), J);
                    J = std::find(Tmp.begin(), Tmp.end(), '%');
                }
                argName += Tmp;

                PrintArgs.push_back((Value*) iterator);
                printString = printString + " Arg #" + utostr(ArgNo) + ": " + argName + " =" + getPrintfCodeFor(iterator);
            }
        }

        printString = printString + "\n";

        InsertPrintInstruction(PrintArgs, &BB, InsertPos, printString, SPADEThreadIdFunc, pidFunction, BufferStrings);
    }

    static inline void FunctionExit(
            BasicBlock *BB,
            Function *SPADEThreadIdFunc,
            Function * pidFunction,
	        Function *BufferStrings
    ) {
        ReturnInst *Ret = (ReturnInst*) (BB->getTerminator());

        std::string printString;
        std::string retName;
        raw_string_ostream strStream(printString);
        raw_string_ostream strStream2(retName);

	    std::string functionName = BB->getParent()->getName().str();
        // printString = "%lu L: @" + BB->getParent()->getName().str(); //WAS %d NOW IS %lu is for Thread ID, L is for Function Leave
        if (isInMinimalSet(functionName)) {
            printString = "%lu L: @null"; //WAS %d NOW IS %lu is for Thread ID, L is for Function Leave
        } else {
            printString = "%lu L: @" + functionName; //WAS %d NOW IS %lu is for Thread ID, L is for Function Leave
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


   static cl::opt<std::string> ArgumentsFileName("FunctionNames-input", cl::init(""), cl::Hidden, cl::desc("specifies the input file for the functionNames"));


    class InsertMetadataCode : public FunctionPass {
    protected:
        Function* PrintfFunc;
        Function* SPADESocketFunc;
        Function* SPADEThreadIdFunc;
        Function* pidFunction;
        Function* BufferStrings;
        bool monitorMethods;
        bool useBufferStrings;

	    std::map<std::string, int> methodsToMonitor;
        std::map<std::string, std::set<node *>> functionToPRSNodes;
    public:
        static char ID; // Pass identification, replacement for typeid

        InsertMetadataCode() : FunctionPass(ID) {
        }

        bool doInitialization(Module &M) {

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


        bool runOnFunction(Function &F) {
            LLVMContext& context = F.getContext();
            Module *m = F.getParent();

            Type *returnType = Type::getVoidTy(m->getContext());
            PointerType *argType = PointerType::get(Type::getInt8Ty(m->getContext()), 0);
            FunctionType *FuncType = FunctionType::get(returnType, argType, false);

            // Extract print functions from printLib.c.
            Function* printFunc = cast<Function>(m->getOrInsertFunction("printFuncName", FuncType));
            Function* printBB = cast<Function>(m->getOrInsertFunction("printBBName", FuncType));

            cfg *graph = new cfg(F.getName().str());

            IRBuilder<> Builder(context);

            unsigned bb_count = 0;

            /* Unroll basic blocks into function-level & basic block level CFG. */

            // Collect functions inside each basic block
            std::map<llvm::BasicBlock*, std::vector<std::string>> bb_functions_map;
            std::map<std::string, unsigned> function_calls;
            std::map<llvm::BasicBlock*, unsigned> bb_number_map;
            for (llvm::BasicBlock &bb : F) {
                bb_functions_map.insert({&bb, {"BasicBlock_"+to_string(bb_count)+"_Head"}});

                for (llvm::Instruction & i: bb) {
                    if (CallInst *Call = dyn_cast<CallInst>(&i)) {
                        if (Function *Callee = Call->getCalledFunction()) {
                            string funcName = Callee->getName().str();
                            function_calls[funcName] +=1 ;
                            bb_functions_map[&bb].push_back(funcName + "_" + to_string(function_calls[funcName]));
                        }
                    } else if (InvokeInst *Call = dyn_cast<InvokeInst>(&i)) {
                        if (Function *Callee = Call->getCalledFunction()) {
                            string funcName = Callee->getName().str();
                            function_calls[funcName] +=1 ;
                            bb_functions_map[&bb].push_back(funcName + "_" + to_string(function_calls[funcName]));
                        }
                    }
                }

                bb_functions_map[&bb].push_back("BasicBlock_"+to_string(bb_count)+"_Tail");
                bb_number_map[&bb] = bb_count;
                bb_count+=1;
            }

            /* Insert nodes including BasicBlock nodes and FuncName nodes */
            for (llvm::BasicBlock &bb : F) {
                for (auto pair : bb_functions_map) {
                    auto function_nodes = pair.second;
                    for (auto function_node : function_nodes) {
                        graph->insertNode(function_node);
                    }
                }
            }

            /* Insert Edges */

            // Insert edges between basic blocks
            for (llvm::BasicBlock &bb : F) {
                for (succ_iterator SI = succ_begin(&bb), SE = succ_end(&bb); SI != SE; ++SI) {
                    BasicBlock *succ = *SI;
                    // Edge is front edge
                    if (bb_number_map[&bb] < bb_number_map[succ]) {
                        graph->insertEdge(bb_functions_map[&bb].back(), bb_functions_map[succ].front());
                    }
                    // Edge is back edge
                    else {
                        graph->insertEdge(bb_functions_map[&bb].back(), bb_functions_map[succ].back());
                    }
                }
            }

            // Insert edges inside basic blocks
            for (llvm::BasicBlock &bb : F) {
                size_t n = bb_functions_map[&bb].size();
                for (size_t i=0; i<n-1; i++) {
                    graph->insertEdge(bb_functions_map[&bb].at(i), bb_functions_map[&bb].at(i+1));
                }
            }

            // Print pre-mature CFG
            // graph->writeDotFile(F.getName().str()+"_premature.dot", graph->outEdges);

            /* Reduce nodes that is not a function node */
            std::map<std::string, node *> nodes_cpy = {};

            // Skip ENTRY and EXIT
            for (auto v: graph->nodes) {
                if (graph->inEdges[v.second].size()==0 || graph->outEdges[v.second].size()==0) {
                    continue;
                }
                nodes_cpy.insert(v);
            }

            for (auto v: nodes_cpy) {

                // Remove BasicBlock node v and its edges, add new edges, ignore duplicate edges when add
                if (v.first.substr(0,10) == "BasicBlock") {

                    std::set<std::pair<node *, node *>> newEdges_v = {};

                    for(node *src : graph->inEdges[v.second]) {
                        for(node *dst : graph->outEdges[v.second]) {
                            if (graph->outEdges[src].find(dst) == graph->outEdges[src].end()){
                                newEdges_v.insert({src,dst});
                            }
                        }
                    }

                    // Remove v from CFG nodes
                    graph->nodes.erase(v.first);

                    // Remove v's in edges
                    auto v_srcs = graph->inEdges[v.second];
                    graph->inEdges.erase(v.second);
                    for (auto v_src : v_srcs) {
                        graph->outEdges[v_src].erase(v.second);
                    }

                    // Remove v's out edges
                    auto v_dsts = graph->outEdges[v.second];
                    graph->outEdges.erase(v.second);
                    for (auto v_dst : v_dsts) {
                        graph->inEdges[v_dst].erase(v.second);
                    }

                    // Add new edges
                    for (auto newedge : newEdges_v) {
                        graph->inEdges[std::get<1>(newedge)].insert(std::get<0>(newedge));
                        graph->outEdges[std::get<0>(newedge)].insert(std::get<1>(newedge));
                    }

                }
            }

            /* Apply PRS on function-level CFG */

            errs() << "Function " << F.getName().str() << " summary:\n";

            set<node *> fullNodes = graph->getFullNodes();

            unsigned funcBBNum = fullNodes.size();

            errs() << "Basic blocks: " << funcBBNum;

            set<node *> minimumPRS;

            // print function call graph
            errs().changeColor(raw_ostream::GREEN, true) << "\nFunction Call Graph:\n";
            errs().resetColor();
            graph->printGraph();

            std::set<node *> V;
            std::tuple<std::set<node *>, std::map<node *, std::set<node *>>, std::map<node *, std::set<node *>>> minimalPRSResult = graph->findMinimalPRS(V);
            std::set<node *> minimalPRSNodes = std::get<0>(minimalPRSResult);

            globalMinimalPRSNodes.insert(minimalPRSNodes.begin(), minimalPRSNodes.end());


            // print minimal PRS
            errs().changeColor(raw_ostream::GREEN, true) << "\n Minimal Path Recovery Set: ";
            errs().resetColor();
            errs().changeColor(raw_ostream::BLUE, true) << "keep: " << "";
            for (std::set<node *>::iterator it = minimalPRSNodes.begin(); it != minimalPRSNodes.end(); ++it) {
                errs().changeColor(raw_ostream::BLUE, true)  << (*it)->name << "; ";
            }
            errs().resetColor();
            errs() << "\n";

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
	        if((monitorMethods  &&  methodsToMonitor.find(F.getName().str()) == methodsToMonitor.end()) || strcmp(F.getName().str().c_str(),"bufferString")==0 || strcmp(F.getName().str().c_str(), "flushStrings")==0 || strcmp(F.getName().str().c_str(), "setAtExit")==0 ) {
	        	return true;
            }

            std::vector<Instruction*> valuesStoredInFunction;
            std::vector<BasicBlock*> exitBlocks;

            //FunctionEntry inserts Provenance instrumentation at the start of every function
            // NOTE: send the function entry message
            FunctionEntry(F, SPADEThreadIdFunc, pidFunction, BufferStrings, useBufferStrings);

            //FunctionExit inserts Provenance instrumentation on the end of every function
            for (Function::iterator BB = F.begin(); BB != F.end(); ++BB) {
                if (isa<ReturnInst > (BB->getTerminator())) {
                    // NOTE: send the function exit message
                    FunctionExit(BB, SPADEThreadIdFunc, pidFunction, BufferStrings);
                }
            }
            return true;
        }
    };

    char InsertMetadataCode::ID = 0;
    static RegisterPass<InsertMetadataCode> X("provenance", "insert provenance instrumentation");

}


