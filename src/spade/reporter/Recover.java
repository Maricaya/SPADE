package spade.reporter;

import java.io.*;
import java.util.*;
import java.util.stream.Collectors;

public class Recover {

    private String cfgFile;

    public Recover(String cfgFile) {
        this.cfgFile = cfgFile;
    }

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
        Map<String, CFG> allCFGs = parseCFGFile(cfgFile);
        if (allCFGs.isEmpty()) {
            System.err.println("No CFG parsed or file error!");
        }
        return processAndCombinePaths(lines, allCFGs);
    }

    private List<String> processAndCombinePaths(List<String> lines, Map<String, CFG> allCFGs) {
        List<Pair<String, List<String>>> functionSignatures = extractFunctionSignatures(lines, allCFGs);
        List<Pair<List<String>, List<String>>> recoveredPaths = recoverPaths(functionSignatures, allCFGs);
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

    private List<Pair<List<String>, List<String>>> recoverPaths(
            List<Pair<String, List<String>>> functionSignatures,
            Map<String, CFG> allCFGs) {
        List<Pair<List<String>, List<String>>> recoveredPaths = new ArrayList<>();

        for (Pair<String, List<String>> functionSignature : functionSignatures) {
            String lastFunctionName = functionSignature.key.split("->")[functionSignature.key.split("->").length - 1];

            List<String> recoveredPath = allCFGs.get(lastFunctionName)
                    .recoverPath(functionSignature.value)
                    .stream()
                    .filter(allCFGs::containsKey)
                    .collect(Collectors.toList());
            // color yellow
            System.out.println("\u001B[33m recoveredPath after: \u001B[0m" + recoveredPath);

            List<String> keys = Arrays.asList(functionSignature.key.split("->"));

            recoveredPaths.add(new Pair<>(keys, recoveredPath));
        }

        return recoveredPaths;
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
private List<String> trimSegment(List<String> segment, List<String> key, List<String> combinedChain) {
    System.out.println("\u001B[33mtrimSegment: segment: " + segment + "\u001B[0m\n");
    System.out.println("\u001B[33m trimSegment: key: " + key + "\u001B[0m\n");
    System.out.println("\u001B[33m trimSegment: combinedChain: " + combinedChain + "\u001B[0m\n");
    if (key.size() > 1) {
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

private List<String> combinePaths(List<Pair<List<String>, List<String>>> recoveredPaths) {
    List<List<String>> segments = new ArrayList<>();
    // print yellow
    System.out.println("\u001B[32m recoveredPaths: " + recoveredPaths + "\u001B[0m\n");
    // 把每个映射的 recovered path 拷贝一份作为独立段
    for (Pair<List<String>, List<String>> p : recoveredPaths) {
        // p.key 也要被放进 seg 里， 放进的规则是
        // 如果 p.key 的第一个 token 已经在 segments 中，则不放进
        // 否则，放进

        List<String> seg = new ArrayList<>(p.value);

        // 检查p.key的第一个元素是否已在segments中的任何列表中
        boolean firstTokenExists = false;
        if (!p.key.isEmpty()) {
            String firstToken = p.key.get(0);
            for (List<String> existingSeg : segments) {
                if (!existingSeg.isEmpty() && existingSeg.contains(firstToken)) {
                    firstTokenExists = true;
                    break;
                }
            }

            // 如果第一个token不存在于segments中，则将p.key添加到seg
            if (!firstTokenExists) {
                seg.addAll(0, p.key); // 在seg开头添加p.key
            }
        }

        segments.add(seg);
    }

    System.out.println("\u001B[33m segments: " + segments + "\u001B[0m\n");

    // 初始化组合链为第一个映射的 recovered path（去除相邻重复）
    List<String> combined = new ArrayList<>();
    if (!segments.isEmpty()) {
        combined.addAll(removeAdjacentDuplicates(segments.get(0)));
    }

    // 对后续映射依次处理：
    // 如果映射 key 为浅层（即 key 中不包含 "->"），则对该段进行裁剪；
    // 但对于特殊函数如 close_stdout，保留不裁剪
    for (int i = 1; i < segments.size(); i++) {
        Pair<List<String>, List<String>> p = recoveredPaths.get(i);
        List<String> seg = new ArrayList<>(segments.get(i));
        List<String> keyTokens = p.key;

        seg = trimSegment(seg, keyTokens, combined);
        combined.addAll(seg);
    }

    System.out.println("\u001B[33m combined: " + combined + "\u001B[0m\n");

    combined = removeAdjacentDuplicates(combined);
    // color green
    System.out.println("\u001B[32m Combined call stack: \u001B[0m" + combined);
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
}
