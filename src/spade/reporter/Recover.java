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
                // 如果 sig 是 ***null***，则直接使用 ENTRY 到 EXIT 的 annotation
                if (sig.equals("***null***")) {
                    List<String> annotation = edgeAnnotations.get(new Pair<>(ENTRY, EXIT));
                    // print entry and exit
                    System.out.println("\u001B[31m ENTRY: \u001B[0m" + ENTRY.name);
                    System.out.println("\u001B[31m EXIT: \u001B[0m" + EXIT.name);
                    if (annotation != null) {
                        // print annotation green
                        System.out.println("\u001B[32m annotation: \u001B[0m" + annotation);
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

            // print recoveredPath
            System.out.println("\u001B[33m recoveredPath before : \u001B[0m" + lastFunctionName);
            List<String> recoveredPath = allCFGs.get(lastFunctionName)
                    .recoverPath(functionSignature.value)
                    .stream()
                    .filter(allCFGs::containsKey)
                    .collect(Collectors.toList());
            // color yellow
            System.out.println("\u001B[33m recoveredPath after: \u001B[0m" + recoveredPath);

            // print contain keys
            // System.out.println("\u001B[33m contain keys: \u001B[0m" + allCFGs.keySet());

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

        /**
     * 根据映射 key 的深度（如 "main->print_even" 深度为 2）先合并更深的映射，再合并更浅的映射。
     * 最终返回合并后的一条完整 trace。
     */
    private List<String> combinePathsDeeperFirst(List<Pair<String, List<String>>> recoveredPaths) {
        for (Pair<String, List<String>> p : recoveredPaths) {
            System.out.println("\u001B[32m p: \u001B[0m" + p.key + " " + p.value);
        }
        // 1. 将映射按 key 的深度分组并降序排列
        //    深度 = splitKey(key).size()
        Map<Integer, List<Pair<String, List<String>>>> depthMap = new TreeMap<>(Collections.reverseOrder());
        for (Pair<String, List<String>> p : recoveredPaths) {
            int depth = splitKey(p.key).size();
            depthMap.computeIfAbsent(depth, k -> new ArrayList<>()).add(p);
        }

        // 合并结果容器
        List<Pair<String, List<String>>> mergedAll = new ArrayList<>();

        // 2. 依次处理从大到小的深度
        for (Map.Entry<Integer, List<Pair<String, List<String>>>> entry : depthMap.entrySet()) {
            int currentDepth = entry.getKey();
            List<Pair<String, List<String>>> sameDepthList = entry.getValue();

            // 将同一深度的映射按原始顺序插入到 mergedAll 中，以便和相邻映射尝试合并
            // 也可以根据需要，把它们先插入到 mergedAll 的“合适位置”。
            mergedAll.addAll(sameDepthList);

            // 不断尝试在 mergedAll 中合并“相邻”且可合并的映射
            boolean changed = true;
            while (changed) {
                changed = false;
                for (int i = 0; i < mergedAll.size() - 1; i++) {
                    Pair<String, List<String>> p1 = mergedAll.get(i);
                    Pair<String, List<String>> p2 = mergedAll.get(i + 1);

                    if (canMerge(p1, p2)) {
                        // doMerge, 用 p2 替换
                        Pair<String, List<String>> newPair = doMerge(p1, p2);
                        mergedAll.set(i + 1, newPair);
                        mergedAll.remove(i);
                        changed = true;
                        break;
                    }
                    // 如果也可能 p2 合并进 p1，就加一个分支:
                    else if (canMerge(p2, p1)) {
                        Pair<String, List<String>> newPair = doMerge(p2, p1);
                        mergedAll.set(i, newPair);
                        mergedAll.remove(i + 1);
                        changed = true;
                        break;
                    }
                }
            }
        }

        // 3. 现在 mergedAll 中包含了所有深度映射合并后的列表，但顺序不一定完美。
        //    可能还可以做一次“相邻合并”，以确保剩余的相邻映射也能互相合并。
        //    这里可以与原先的 combinePaths 的逻辑类似：
        boolean merged = true;
        while (merged && mergedAll.size() > 1) {
            merged = false;
            for (int i = 0; i < mergedAll.size() - 1; i++) {
                Pair<String, List<String>> p1 = mergedAll.get(i);
                Pair<String, List<String>> p2 = mergedAll.get(i + 1);
                if (canMerge(p1, p2)) {
                    Pair<String, List<String>> mergedPair = doMerge(p1, p2);
                    mergedAll.set(i + 1, mergedPair);
                    mergedAll.remove(i);
                    merged = true;
                    break;
                }
                else if (canMerge(p2, p1)) {
                    Pair<String, List<String>> mergedPair = doMerge(p2, p1);
                    mergedAll.set(i, mergedPair);
                    mergedAll.remove(i + 1);
                    merged = true;
                    break;
                }
            }
        }

        // 4. 最后将 mergedAll 中的所有 recoveredPath 顺序拼接
        List<String> combinedLog = new ArrayList<>();
        for (Pair<String, List<String>> p : mergedAll) {
            combinedLog.addAll(p.value);
        }
        combinedLog = removeAdjacentDuplicates(combinedLog);

        System.out.println("\u001B[35m[DeeperFirst] Combined path:\u001B[0m " + combinedLog);
        return combinedLog;
    }

    /**
 * 如果segment中有多个token，并且segment的首token已经在combinedChain中出现，
 * 则移除segment的首token；重复该过程直到segment只剩下一个token或首token不在combinedChain中。
 * 如果经过移除后segment为空，则返回原segment的最后一个token（以保留至少一个信息）。
 */
private List<String> trimSegment(List<String> segment, List<String> combinedChain) {
    if (segment == null || segment.isEmpty()) {
        return segment;
    }
    // 保留原始拷贝以便在必要时还原
    List<String> original = new ArrayList<>(segment);
    while (segment.size() > 1 && combinedChain.contains(segment.get(0))) {
        // 移除第一个token
        segment.remove(0);
    }
    if (segment.isEmpty() && !original.isEmpty()) {
        // 如果全部移除导致空，则至少返回最后一个token
        segment.add(original.get(original.size() - 1));
    }
    return segment;
}

/**
 * 合并多个映射恢复的调用链。
 *
 * 输入 recoveredPaths 为 List<Pair<String, List<String>>>
 *   - 每个 Pair 的 key 表示映射的上下文（例如 "main" 或 "main->print_even"），
 *     recovered path 则是通过 CFG 恢复的调用链（以 List<String> 表示）。
 *
 * 合并规则（启发式）：
 * 1. 对于第一条映射，直接保留其 recovered path；
 * 2. 对于后续映射，先对它们的 recovered path 进行预处理（调用 trimSegment），
 *    如果该映射的 key较浅（比如只包含一个 token），则移除 recovered path 中已经在前面组合中出现过的前导部分，
 *    保留剩余部分后追加到最终组合中。
 *
 * 最终将所有处理后的段按顺序拼接，并对相邻重复的 token 做一次去重，得到最终调用链。
 */

/**
 * 针对单个映射的 recovered path 进行裁剪
 * 如果映射 key 只有一个 token（例如 "main"），则从该 recovered path 中
 * 移除前导部分（即那些已经在当前组合链中出现过的 token），
 * 直到只剩下一个 token或首 token不再重复。
 * 如果映射 key 包含 "->"（深层映射），则不做裁剪。
 */
private List<String> trimSegment(List<String> segment, String key, List<String> combinedChain) {
    List<String> keyTokens = splitString(key, "->");
    if (keyTokens.size() > 1) {
        // 深层映射：不裁剪，直接返回
        return segment;
    }
    // 浅层映射：如果首 token 已经在组合链中，则移除
    List<String> original = new ArrayList<>(segment);
    while (segment.size() > 1 && combinedChain.contains(segment.get(0))) {
        segment.remove(0);
    }
    if (segment.isEmpty() && !original.isEmpty()) {
        segment.add(original.get(original.size() - 1));
    }
    return segment;
}

private List<String> combinePaths(List<Pair<String, List<String>>> recoveredPaths) {
    List<List<String>> segments = new ArrayList<>();
    // 把每个映射的 recovered path 拷贝一份作为独立段
    for (Pair<String, List<String>> p : recoveredPaths) {
        List<String> seg = new ArrayList<>(p.value);
        segments.add(seg);
    }

    // 初始化组合链为第一个映射的 recovered path（去除相邻重复）
    List<String> combined = new ArrayList<>();
    if (!segments.isEmpty()) {
        combined.addAll(removeAdjacentDuplicates(segments.get(0)));
    }

    // 对后续映射依次处理：
    // 如果映射 key 为浅层（即 key 中不包含 "->"），则对该段进行裁剪；
    // 否则直接追加。
    for (int i = 1; i < segments.size(); i++) {
        Pair<String, List<String>> p = recoveredPaths.get(i);
        List<String> seg = new ArrayList<>(segments.get(i));
        List<String> keyTokens = splitString(p.key, "->");
        if (keyTokens.size() == 1) {
            seg = trimSegment(seg, p.key, combined);
        }
        combined.addAll(seg);
    }

    combined = removeAdjacentDuplicates(combined);
    System.out.println("Combined call stack: " + combined);
    return combined;
}

/**
 * 将字符串按照 delimiter 分割为 token 列表
 */
private static List<String> splitString(String s, String delimiter) {
    List<String> tokens = new ArrayList<>();
    int start = 0, end;
    while ((end = s.indexOf(delimiter, start)) != -1) {
        tokens.add(s.substring(start, end));
        start = end + delimiter.length();
    }
    tokens.add(s.substring(start));
    return tokens;
}

/**
 * 去除列表中相邻重复的 token
 */
private List<String> removeAdjacentDuplicates(List<String> input) {
    List<String> result = new ArrayList<>();
    for (String s : input) {
        if (result.isEmpty() || !result.get(result.size() - 1).equals(s)) {
            result.add(s);
        }
    }
    return result;
}



    // private List<String> combinePaths(List<Pair<String, List<String>>> recoveredPaths) {
        // 打印 recoveredPaths
        // for (Pair<String, List<String>> p : recoveredPaths) {
            // System.out.println("\u001B[32m p: \u001B[0m" + p.key + " " + p.value);
        // }

        // 反复尝试合并，直到无法再合并
        // boolean merged = true;
        // while (merged) {
            // merged = false;
            // for (int i = 0; i < recoveredPaths.size() - 1; i++) {
                // Pair<String, List<String>> p1 = recoveredPaths.get(i);
                // Pair<String, List<String>> p2 = recoveredPaths.get(i + 1);

                // if (canMerge(p1, p2)) {
                    // 执行合并
                    // Pair<String, List<String>> mergedPair = doMerge(p1, p2);
                    // 用合并结果替换 p2
                    // recoveredPaths.set(i + 1, mergedPair);
                    // 移除 p1
                    // recoveredPaths.remove(i);
                    // merged = true;
                    // break; // 重新开始扫描
                // }
            // }
        // }

        // 经过上面多轮合并后，recoveredPaths 中的若干映射可能已经合并成更大粒度
        // 最后可以把所有剩余映射都拼接起来（或根据需要处理）
        // List<String> combinedLog = new ArrayList<>();
        // for (Pair<String, List<String>> p : recoveredPaths) {
            // combinedLog.addAll(p.value);
        // }

        // 可选：去除相邻重复
        // combinedLog = removeAdjacentDuplicates(combinedLog);

        // 输出调试
        // System.out.println("Combined path (preserving order): " + combinedLog);
        // return combinedLog;
    // }

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
