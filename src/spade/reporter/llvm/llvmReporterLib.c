#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>

#define MAX_CALL_CHAIN_LENGTH 1024
#define MAX_FUNCTIONS 64

// 每个线程的调用栈
typedef struct {
    char* functions[MAX_FUNCTIONS];
    int top;
} CallStack;

// 线程局部存储的调用栈
static __thread CallStack callStack = { .top = -1 };

// 将函数压入调用栈
void pushCallStack(const char* funcName) {
    if (callStack.top < MAX_FUNCTIONS - 1) {
        callStack.top++;
        callStack.functions[callStack.top] = strdup(funcName);
    }
}

// 获取当前完整调用链
const char* getCallTrace() {
    static __thread char callChain[MAX_CALL_CHAIN_LENGTH];
    callChain[0] = '\0';

    for (int i = 0; i <= callStack.top; i++) {
        if (i > 0) {
            strcat(callChain, "->");
        }
        strcat(callChain, callStack.functions[i]);
    }
    return callChain;
}

// 将函数从调用栈弹出
void popCallStack() {
    if (callStack.top >= 0) {
        free(callStack.functions[callStack.top]);
        callStack.top--;
    }
}

// 初始化函数，如果需要的话
__attribute__((constructor))
void initCallStack() {
    callStack.top = -1;
}