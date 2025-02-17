package spade.reporter;

import java.io.*;
import java.util.*;
import java.util.stream.Collectors;

public class Recover {

    // Generic Pair class
    public static class Pair<F, S> {
        public F key;
        public S value;

        public Pair(F first, S second) {
            this.key = first;
            this.value = second;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o)
                return true;
            if (!(o instanceof Pair))
                return false;
            Pair<?, ?> pair = (Pair<?, ?>) o;
            return Objects.equals(key, pair.key) &&
                    Objects.equals(value, pair.value);
        }

        @Override
        public int hashCode() {
            return Objects.hash(key, value);
        }

        @Override
        public String toString() {
            return "(" + key + ", " + value + ")";
        }
    }

    // Node class representing a node in the CFG
    public static class Node {
        String name;

        public Node(String name) {
            this.name = name;
        }
    }

    // CFG class representing the control flow graph of a function
    public static class CFG {
        String funcName;
        Node ENTRY;
        Node EXIT;
        Map<String, Node> nodes;
        Map<Node, Set<Node>> outEdges;
        Map<Node, Set<Node>> inEdges;
        Map<Pair<Node, Node>, List<String>> edgeAnnotations;

        public CFG(String funcName) {
            this.funcName = funcName;
            nodes = new HashMap<>();
            outEdges = new HashMap<>();
            inEdges = new HashMap<>();
            edgeAnnotations = new HashMap<>();
        }

        // Insert a node into the CFG
        public void insertNode(String name) {
            if (!nodes.containsKey(name)) {
                nodes.put(name, new Node(name));
            }
        }

        // Insert an edge into the CFG
        public void insertEdge(String from, String to) {
            Node src = nodes.get(from);
            Node dst = nodes.get(to);
            if (src == null || dst == null)
                return;
            outEdges.computeIfAbsent(src, k -> new HashSet<>()).add(dst);
            inEdges.computeIfAbsent(dst, k -> new HashSet<>()).add(src);
        }

        // Find the entry node (node with no incoming edges)
        public void findENTRY() {
            for (Node n : nodes.values()) {
                Set<Node> ins = inEdges.get(n);
                if (ins == null || ins.isEmpty()) {
                    ENTRY = n;
                    break;
                }
            }
        }

        // Find the exit node (node with no outgoing edges)
        public void findEXIT() {
            for (Node n : nodes.values()) {
                Set<Node> outs = outEdges.get(n);
                if (outs == null || outs.isEmpty()) {
                    EXIT = n;
                    break;
                }
            }
        }

        /**
         * Given a signature (list of node names), recover the call path from ENTRY to
         * EXIT.
         * The recovered path is returned as an arrow-separated string.
         *
         * Note: This method is retained for debugging purposes.
         *
         * @param signature List of node names representing the path signature.
         * @return The recovered call path as a string.
         */
        public List<String> recoverPath(List<String> signature) {
            if (ENTRY == null || EXIT == null) {
                return Arrays.asList("ERROR: ENTRY or EXIT is null.");
            }

            List<String> path = new ArrayList<>();
            path.add(ENTRY.name);
            Node current = ENTRY;

            // Process each node in signature
            System.out.println("\u001B[31m signature: \u001B[0m" + signature);
            if (signature.isEmpty()) {
                // return path;
                // 找到 edgeAnnotations 中，ENTRY 到 EXIT 的 annotation
                List<String> annotation = edgeAnnotations.get(new Pair<>(ENTRY, EXIT));
                System.out.println("\u001B[31m annotation: \u001B[0m" + annotation);
                if (annotation != null) {
                    path.addAll(annotation);
                }
                // return path;
            }

            for (String sig : signature) {
                if (sig.equals("***null***")) {
                    List<String> annotation = edgeAnnotations.get(new Pair<>(ENTRY, EXIT));
                    if (annotation != null) {
                        path.addAll(annotation);
                    }
                    continue;
                }

                Node next = nodes.get(sig);
                if (next == null) {
                    path.add("UnknownNode:" + sig);
                    continue;
                }

                // Instead of direct edge lookup, find matching edge by source and destination
                List<String> annotation = null;
                for (Map.Entry<Pair<Node, Node>, List<String>> entry : edgeAnnotations.entrySet()) {
                    if (entry.getKey().value.name.equals(next.name)) {
                        annotation = entry.getValue();
                        break;
                    }
                }

                if (annotation != null) {
                    path.addAll(annotation);
                }
                path.add(next.name);
                current = next;
            }

            // Add EXIT if not already there
            if (current != EXIT) {
                path.add(EXIT.name);
            }

            // Process BasicBlock markers
            List<String> processedPath = new ArrayList<>();
            for (String nodeName : path) {
                if (nodeName.contains("BasicBlock_")) {
                    int headEnd = nodeName.indexOf("_Head");
                    if (headEnd != -1) {
                        processedPath.add(funcName);
                        continue;
                    }
                }
                processedPath.add(nodeName);
            }

            return processedPath;
        }
    }

    // Class to store parsed CFG information from the file
    public static class ParsedFunctionCFG {
        String funcName;
        String entryName;
        String exitName;
        List<String> nodeNames;
        Map<Pair<String, String>, List<String>> edgeAnnotations;

        public ParsedFunctionCFG() {
            nodeNames = new ArrayList<>();
            edgeAnnotations = new HashMap<>();
        }
    }

    // Build a CFG from the parsed result
    public static CFG buildCFG(ParsedFunctionCFG pfc) {
        CFG g = new CFG(pfc.funcName);

        for (String nm : pfc.nodeNames) {
            g.insertNode(nm);
        }
        for (Map.Entry<Pair<String, String>, List<String>> entry : pfc.edgeAnnotations.entrySet()) {
            String srcName = entry.getKey().key;
            String dstName = entry.getKey().value;
            List<String> anno = entry.getValue();
            g.insertEdge(srcName, dstName);
            Node srcNode = g.nodes.get(srcName);
            Node dstNode = g.nodes.get(dstName);
            if (srcNode != null && dstNode != null) {
                g.edgeAnnotations.put(new Pair<>(srcNode, dstNode), anno);
            }
        }
        if (pfc.entryName != null && !pfc.entryName.isEmpty()) {
            g.ENTRY = g.nodes.get(pfc.entryName);
        } else {
            g.findENTRY();
        }
        if (pfc.exitName != null && !pfc.exitName.isEmpty()) {
            g.EXIT = g.nodes.get(pfc.exitName);
        } else {
            g.findEXIT();
        }
        return g;
    }

    // Parse the CFG file and return a mapping from function name to CFG
    public static Map<String, CFG> parseCFGFile(String filename) {
        Map<String, CFG> result = new HashMap<>();
        try (BufferedReader br = new BufferedReader(new FileReader(filename))) {
            String line;
            ParsedFunctionCFG current = new ParsedFunctionCFG();
            boolean inEdgesSection = false;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty())
                    continue;
                if (line.startsWith("FUNCTION:")) {
                    if (current.funcName != null && !current.funcName.isEmpty()) {
                        CFG cfg = buildCFG(current);
                        result.put(current.funcName, cfg);
                        current = new ParsedFunctionCFG();
                    }
                    current.funcName = line.substring("FUNCTION:".length()).trim();
                    inEdgesSection = false;
                } else if (line.startsWith("ENTRY:")) {
                    current.entryName = line.substring("ENTRY:".length()).trim();
                } else if (line.startsWith("EXIT:")) {
                    current.exitName = line.substring("EXIT:".length()).trim();
                } else if (line.startsWith("Edges:")) {
                    inEdgesSection = true;
                } else if (line.startsWith("Nodes:")) {
                    inEdgesSection = false;
                    String nodesStr = line.substring("Nodes:".length()).trim();
                    String[] parts = nodesStr.split(";");
                    for (String nm : parts) {
                        nm = nm.trim();
                        if (!nm.isEmpty())
                            current.nodeNames.add(nm);
                    }
                } else {
                    if (inEdgesSection) {
                        int pos = line.indexOf(" : ");
                        if (pos != -1) {
                            String left = line.substring(0, pos);
                            String anno = line.substring(pos + 3).trim();
                            // Split the annotation into a List and remove "->" from each element
                            List<String> annoList = Arrays.stream(anno.split("->"))
                                    .filter(s -> !s.isEmpty())
                                    .collect(Collectors.toList());

                            int arrowPos = left.indexOf(" -> ");
                            if (arrowPos != -1) {
                                String srcName = left.substring(0, arrowPos).trim();
                                String dstName = left.substring(arrowPos + 4).trim();
                                current.edgeAnnotations.put(new Pair<>(srcName, dstName), annoList);
                            }
                        }
                    }
                }
            }
            if (current.funcName != null && !current.funcName.isEmpty()) {
                CFG cfg = buildCFG(current);
                result.put(current.funcName, cfg);
            }
        } catch (IOException e) {
            System.err.println("Failed to open file: " + filename);
        }
        return result;
    }

    // Helper function: split string by delimiter
    public static List<String> splitString(String s, String delimiter) {
        List<String> tokens = new ArrayList<>();
        int start = 0, end;
        while ((end = s.indexOf(delimiter, start)) != -1) {
            tokens.add(s.substring(start, end));
            start = end + delimiter.length();
        }
        tokens.add(s.substring(start));
        return tokens;
    }

    // *********************************************************************
    // New classes and methods for building a call tree with explicit
    // function entry and exit events.
    // *********************************************************************

    // Class representing a node in the call tree
    public static class CallNode {
        String name;
        List<CallNode> children;

        public CallNode(String name) {
            this.name = name;
            this.children = new ArrayList<>();
        }
    }

    // Helper function: find a child with the given name in a call tree node
    public static CallNode findChild(CallNode node, String name) {
        for (CallNode child : node.children) {
            if (child.name.equals(name)) {
                return child;
            }
        }
        return null;
    }

    /**
     * Build a call tree from a list of mappings.
     * Each mapping is a Pair where:
     * - The key (a string like "main" or "main->print_even") represents the context
     * (the caller chain).
     * - The signature (a list of function names) represents the callee chain.
     *
     * The tree is built such that each node appears once on entry and its DFS
     * traversal will
     * record the function name on entry and again on exit.
     *
     * @param mappings List of mappings representing call chain segments.
     * @return The root CallNode of the constructed call tree.
     */
    public static CallNode buildCallTree(List<Pair<String, List<String>>> mappings) {
        CallNode root = null;
        for (Pair<String, List<String>> mapping : mappings) {
            // Split the mapping key to get the caller context.
            List<String> context = splitString(mapping.key, "->");
            if (context.isEmpty())
                continue;
            String rootName = context.get(0).trim();
            if (root == null) {
                root = new CallNode(rootName);
            } else if (!root.name.equals(rootName)) {
                // If different, one might choose to create a dummy root.
                // For now, we assume all mappings share the same root.
            }
            CallNode current = root;
            // Traverse (or create) nodes for the context tokens beyond the root.
            for (int i = 1; i < context.size(); i++) {
                String token = context.get(i).trim();
                CallNode child = findChild(current, token);
                if (child == null) {
                    child = new CallNode(token);
                    current.children.add(child);
                }
                current = child;
            }
            // Now, process the signature tokens as a chain of calls.
            for (String token : mapping.value) {
                token = token.trim();
                CallNode child = new CallNode(token);
                current.children.add(child);
                current = child;
            }
        }
        return root;
    }

    /**
     * Depth-first traversal of the call tree.
     * Instead of printing, the function names are added to the provided list.
     * Each function's name is recorded when entering and again when exiting.
     *
     * @param node   The current CallNode.
     * @param result The list to store the trace.
     */
    public static void dfsCollect(CallNode node, List<String> result) {
        // Record entry event
        result.add(node.name);
        for (CallNode child : node.children) {
            dfsCollect(child, result);
        }
        // Record exit event
        result.add(node.name);
    }

    // recover the function name from the lines
    public List<String> recoverFunctions(List<String> lines) {
        Map<String, CFG> allCFGs = parseCFGFile("cfg.txt");
        if (allCFGs.isEmpty()) {
            System.err.println("No CFG parsed or file error!");
        }
        return processAndCombinePaths(lines, allCFGs);
    }

    private List<String> processAndCombinePaths(List<String> lines, Map<String, CFG> allCFGs) {
        List<Pair<String, List<String>>> functionSignatures = extractFunctionSignatures(lines, allCFGs);
        List<Pair<String, List<String>>> recoveredPaths = recoverPaths(functionSignatures, allCFGs);
        return combinePaths(recoveredPaths);
    }

    private List<Pair<String, List<String>>> extractFunctionSignatures(List<String> lines, Map<String, CFG> allCFGs) {
        List<Pair<String, List<String>>> functionSignatures = new ArrayList<>();

        for (String line : lines) {
            if (!line.contains("E:")) {
                continue;
            }

            FunctionInfo info = extractFunctionInfo(line);
            if (info == null) continue;

            if (shouldSkipNullFunction(info, allCFGs)) {
                continue;
            }

            functionSignatures.add(new Pair<>(info.context, info.calleeList));
        }

        debugPrintSignatures(functionSignatures);
        return functionSignatures;
    }

    private static class FunctionInfo {
        String functionName;
        String context;
        List<String> calleeList;
    }

    private FunctionInfo extractFunctionInfo(String line) {
        int callChainIndex = line.indexOf("CallChain:");
        if (callChainIndex == -1) {
            return null;
        }

        FunctionInfo info = new FunctionInfo();
        String callChain = line.substring(callChainIndex + "CallChain:".length()).trim();

        // Extract function name
        int functionNameStart = line.indexOf("@") + 1;
        int functionNameEnd = line.indexOf(" ", functionNameStart);
        info.functionName = line.substring(functionNameStart, functionNameEnd);

        // Extract context
        info.context = extractContext(callChain);

        // Create callee list
        info.calleeList = new ArrayList<>();
        info.calleeList.add(info.functionName);

        return info;
    }

    private String extractContext(String callChain) {
        return callChain.contains("->")
                ? callChain.substring(0, callChain.lastIndexOf("->")).trim()
                : callChain.trim();
    }

    private boolean shouldSkipNullFunction(FunctionInfo info, Map<String, CFG> allCFGs) {
        if (!info.functionName.equals("***null***")) {
            return false;
        }

        String lastFunction = getLastFunctionFromContext(info.context);
        CFG cfg = allCFGs.get(lastFunction);
        return cfg == null || !hasEdgeAnnotation(cfg);
    }

    private String getLastFunctionFromContext(String context) {
        return context.contains("->")
                ? context.substring(context.lastIndexOf("->") + 2)
                : context;
    }

    private void debugPrintSignatures(List<Pair<String, List<String>>> functionSignatures) {
        System.out.println("\u001B[34m function signatures: \u001B[0m");
        for (Pair<String, List<String>> functionSignature : functionSignatures) {
            System.out.println(functionSignature.key + " " + functionSignature.value);
        }
    }

    private List<Pair<String, List<String>>> recoverPaths(
            List<Pair<String, List<String>>> functionSignatures,
            Map<String, CFG> allCFGs) {
        List<Pair<String, List<String>>> recoveredPaths = new ArrayList<>();

        for (Pair<String, List<String>> functionSignature : functionSignatures) {
            String lastFunctionName = functionSignature.key.split("->")[functionSignature.key.split("->").length - 1];

            List<String> recoveredPath = allCFGs.get(lastFunctionName)
                    .recoverPath(functionSignature.value)
                    .stream()
                    .filter(allCFGs::containsKey)
                    .collect(Collectors.toList());
            // color yellow
            System.out.println("\u001B[33m recoveredPath: \u001B[0m" + recoveredPath);

            // System.out.println("\u001B[33m functionSignature: \u001B[0m" +
            // functionSignature.first + " " + functionSignature.second);

            recoveredPaths.add(new Pair<>(functionSignature.key, recoveredPath));
        }

        return recoveredPaths;
    }

    private boolean canMerge(Pair<String, List<String>> p1,
            Pair<String, List<String>> p2) {
        // 解析 p1 的 key，如 "main->print_even" → ["main", "print_even"]
        List<String> p1KeyList = splitKey(p1.key);

        // p1KeyList 的最后一个函数
        if (p1KeyList.isEmpty())
            return false;
        String lastFunc = p1KeyList.get(p1KeyList.size() - 1);

        // 如果 p2 的 recoveredPath 中包含 lastFunc，则满足合并条件
        return p2.value.contains(lastFunc);
    }

    private Pair<String, List<String>> doMerge(
            Pair<String, List<String>> p1,
            Pair<String, List<String>> p2) {
        // 解析 p1.key，找出最后一个函数
        List<String> p1KeyList = splitKey(p1.key);
        String lastFunc = p1KeyList.get(p1KeyList.size() - 1);

        // 在 p2.value 中找到 lastFunc 的位置
        List<String> p2Path = new ArrayList<>(p2.value);
        int idx = p2Path.indexOf(lastFunc);
        if (idx < 0) {
            // 理论上不会出现，因为 canMerge 保证了包含
            return p2;
        }

        // 把 p1.value 中的剩余部分插入到 p2Path 中 lastFunc 之后
        // 例如 p1.value = ["print_even", "add"], lastFunc = "print_even"
        // p2.value = ["main", "print_even", "empty"]
        // 那么要把 "add" 插到 "print_even" 之后
        List<String> p1Path = p1.value;
        // 找到 p1Path 中 lastFunc 的位置
        int posInP1 = p1Path.indexOf(lastFunc);
        // 若 posInP1 >= 0，则 posInP1 之后的所有函数都插入 p2
        if (posInP1 >= 0 && posInP1 + 1 < p1Path.size()) {
            // 需要插入的部分
            List<String> leftover = p1Path.subList(posInP1 + 1, p1Path.size());
            // 在 idx+1 位置插入 leftover
            p2Path.addAll(idx + 1, leftover);
        }

        // 合并后用 p2 的 key 作为新的 key（通常更外层）
        return new Pair<>(p2.key, p2Path);
    }

    private List<String> combinePaths(List<Pair<String, List<String>>> recoveredPaths) {
        // 反复尝试合并，直到无法再合并
        boolean merged = true;
        while (merged) {
            merged = false;
            for (int i = 0; i < recoveredPaths.size() - 1; i++) {
                Pair<String, List<String>> p1 = recoveredPaths.get(i);
                Pair<String, List<String>> p2 = recoveredPaths.get(i + 1);

                if (canMerge(p1, p2)) {
                    // 执行合并
                    Pair<String, List<String>> mergedPair = doMerge(p1, p2);
                    // 用合并结果替换 p2
                    recoveredPaths.set(i + 1, mergedPair);
                    // 移除 p1
                    recoveredPaths.remove(i);
                    merged = true;
                    break; // 重新开始扫描
                }
            }
        }

        // 经过上面多轮合并后，recoveredPaths 中的若干映射可能已经合并成更大粒度
        // 最后可以把所有剩余映射都拼接起来（或根据需要处理）
        List<String> combinedLog = new ArrayList<>();
        for (Pair<String, List<String>> p : recoveredPaths) {
            combinedLog.addAll(p.value);
        }

        // 可选：去除相邻重复
        combinedLog = removeAdjacentDuplicates(combinedLog);

        // 输出调试
        System.out.println("Combined path (preserving order): " + combinedLog);
        return combinedLog;
    }

    // 工具函数：去除相邻重复
    private List<String> removeAdjacentDuplicates(List<String> input) {
        List<String> result = new ArrayList<>();
        for (String s : input) {
            if (result.isEmpty() || !result.get(result.size() - 1).equals(s)) {
                result.add(s);
            }
        }
        return result;
    }

    // 工具函数：解析 key，如 "main->print_even" → ["main", "print_even"]
    private List<String> splitKey(String key) {
        if (key == null || key.isEmpty())
            return new ArrayList<>();
        // 你可以用正则或更简单的 split
        return Arrays.asList(key.split("->"));
    }

    // 新增辅助方法：检查是否存在边的注释
    private boolean hasEdgeAnnotation(CFG cfg) {
        if (cfg.ENTRY == null || cfg.EXIT == null) {
            return false;
        }

        // 检查 edgeAnnotations 中是否存在从入口到出口的边
        Pair<Node, Node> edge = new Pair<>(cfg.ENTRY, cfg.EXIT);
        List<String> annotations = cfg.edgeAnnotations.get(edge);
        return annotations != null && !annotations.isEmpty();
    }

    // *********************************************************************
    // Main function
    // *********************************************************************
    // recover the line form **null** to **function name**
    // todo
    public static void main(String[] args) {
        // Recover recover = new Recover();
        // recover.recoverFunctions(lines);
    }

    public static void printLines(List<String> lines) {
        for (String line : lines) {
            System.out.println(line);
        }
    }
}
