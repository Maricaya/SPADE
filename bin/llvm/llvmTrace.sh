#!/bin/bash
REPLIB_OSFLAG=-D_LLVMREPORTER_LINUX
LLVM_SOURCE=$1
FUNCTION_FILE=$2
LLVM_TARGET=$3
LLVM_Instrumented="${LLVM_SOURCE}_instrumented"

BASE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SRC_PATH=$BASE/../../src
LLC=llc
CC=gcc

echo "base: $BASE"

# 首先编译 llvmReporterLib.c
$CC -fPIC -c ${SRC_PATH}/spade/reporter/llvm/llvmReporterLib.c -o ${SRC_PATH}/spade/reporter/llvm/llvmReporterLib.o

LD_FLAGS=""
if [[ $* == *-instrument-libc* ]]
then
  echo "### Wrapping libc calls"
  LD_FLAGS="$(opt -load $BASE/LibcWrapper.so -wrapper ${LLVM_SOURCE}.bc -o ${LLVM_Instrumented}.bc)"
  echo $LD_FLAGS
else
  cp ${LLVM_SOURCE}.bc ${LLVM_Instrumented}.bc
fi

###
llvm-link ${LLVM_Instrumented}.bc $BASE/flush.bc -o $BASE/linked.bc

if [ "$FUNCTION_FILE" != "-monitor-all" ]; then
	opt -dot-callgraph $BASE/linked.bc -o $BASE/callgraph.bc
	java -cp $BASE/../../build  spade/utility/FunctionMonitor $BASE/callgraph.dot ${FUNCTION_FILE} functionsOut
	opt -load $BASE/LLVMTrace.so -provenance -FunctionNames-input functionsOut $BASE/linked.bc -o ${LLVM_TARGET}.bc
else
	opt -load $BASE/LLVMTrace.so -provenance -FunctionNames-input "-monitor-all" $BASE/linked.bc -o ${LLVM_TARGET}.bc
fi
###


$LLC -relocation-model=pic ${LLVM_TARGET}.bc -o ${LLVM_TARGET}.s
$CC -static -g -ggdb3 ${REPLIB_OSFLAG} ${SRC_PATH}/spade/reporter/llvm/llvmBridge.c -c -o ${SRC_PATH}/spade/reporter/llvm/llvmBridge.o
$CC -fPIC -g -ggdb3 ${SRC_PATH}/spade/reporter/llvm/llvmClose.c -c -o ${SRC_PATH}/spade/reporter/llvm/llvmClose.o
$CC -g -ggdb3 ${LLVM_TARGET}.s -c -o ${LLVM_TARGET}.o
$CC -g -ggdb3 ${LLVM_TARGET}.o ${SRC_PATH}/spade/reporter/llvm/llvmClose.o -shared -o ${LLVM_TARGET}.so $LD_FLAGS

$CC -g -ggdb3 ${LLVM_TARGET}.so ${SRC_PATH}/spade/reporter/llvm/llvmBridge.o ${SRC_PATH}/spade/reporter/llvm/llvmReporterLib.o -o ${LLVM_TARGET} -Wl,-R -Wl,./ -lcrypt -lm

# gcc llvm_test_out.so /home/ubuntu/SPADE/bin/llvm/../../src/spade/reporter/llvm/llvmBridge.o -o llvm_test_out -Wl,-R -Wl,./ -lcrypt -lm
