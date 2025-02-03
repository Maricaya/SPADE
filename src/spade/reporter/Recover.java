package spade.reporter;

import java.io.*;
import java.util.*;

public class Recover {

    // Generic Pair class
    public static class Pair<F, S> {
        public F first;
        public S second;

        public Pair(F first, S second) {
            this.first = first;
            this.second = second;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o)
                return true;
            if (!(o instanceof Pair))
                return false;
            Pair<?, ?> pair = (Pair<?, ?>) o;
            return Objects.equals(first, pair.first) &&
                   Objects.equals(second, pair.second);
        }

        @Override
        public int hashCode() {
            return Objects.hash(first, second);
        }

        @Override
        public String toString() {
            return "(" + first + ", " + second + ")";
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
        Map<Pair<Node, Node>, String> edgeAnnotations;

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
         * Given a signature (list of node names), recover the call path from ENTRY to EXIT.
         * The recovered path is returned as an arrow-separated string.
         *
         * Note: This method is retained for debugging purposes.
         *
         * @param signature List of node names representing the path signature.
         * @return The recovered call path as a string.
         */
        public String recoverPath(List<String> signature) {
            if (ENTRY == null || EXIT == null) {
                return "ERROR: ENTRY or EXIT is null.";
            }

            String path = ENTRY.name;
            Node current = ENTRY;

            for (String sig : signature) {
                Node next = nodes.get(sig);
                if (next == null) {
                    path += "->(UnknownNode:" + sig + ")";
                    continue;
                }
                Pair<Node, Node> edge = new Pair<>(current, next);
                String anno = edgeAnnotations.get(edge);
                if (anno != null) {
                    path += anno + next.name;
                } else {
                    path += "->" + next.name;
                }
                current = next;
            }
            if (current != EXIT) {
                path += "->" + EXIT.name;
            }

            // Extra processing on the path (example processing; adjust as needed)
            int headPos = path.indexOf("BasicBlock_");
            if (headPos != -1) {
                int headEnd = path.indexOf("_Head", headPos);
                if (headEnd != -1) {
                    int lastTailPos = path.lastIndexOf("BasicBlock_");
                    if (lastTailPos != -1) {
                        int tailEnd = path.indexOf("_Tail", lastTailPos);
                        if (tailEnd != -1) {
                            System.out.println("function name: " + funcName + "\n");
                            if (lastTailPos > headEnd + 5) {
                                path = funcName + path.substring(headEnd + 5, lastTailPos) + funcName;
                            }
                        }
                    }
                }
            }
            return path;
        }
    }

    // Class to store parsed CFG information from the file
    public static class ParsedFunctionCFG {
        String funcName;
        String entryName;
        String exitName;
        List<String> nodeNames;
        Map<Pair<String, String>, String> edgeAnnotations;

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
        for (Map.Entry<Pair<String, String>, String> entry : pfc.edgeAnnotations.entrySet()) {
            String srcName = entry.getKey().first;
            String dstName = entry.getKey().second;
            String anno = entry.getValue();
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
                            int arrowPos = left.indexOf(" -> ");
                            if (arrowPos != -1) {
                                String srcName = left.substring(0, arrowPos).trim();
                                String dstName = left.substring(arrowPos + 4).trim();
                                current.edgeAnnotations.put(new Pair<>(srcName, dstName), anno);
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
     *   - The key (a string like "main" or "main->print_even") represents the context (the caller chain).
     *   - The signature (a list of function names) represents the callee chain.
     *
     * The tree is built such that each node appears once on entry and its DFS traversal will
     * record the function name on entry and again on exit.
     *
     * @param mappings List of mappings representing call chain segments.
     * @return The root CallNode of the constructed call tree.
     */
    public static CallNode buildCallTree(List<Pair<String, List<String>>> mappings) {
        CallNode root = null;
        for (Pair<String, List<String>> mapping : mappings) {
            // Split the mapping key to get the caller context.
            List<String> context = splitString(mapping.first, "->");
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
            for (String token : mapping.second) {
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

    // *********************************************************************
    // Main function
    // *********************************************************************
    // recover the line form **null** to **function name**
    public static List<String> main(List<String> lines) {
        // print the lines
        for (String line : lines) {
            System.out.println(line);
        }

        // Parse the CFG file to get a mapping from function name to CFG
        Map<String, CFG> allCFGs = parseCFGFile("cfg_1.txt");
        if (allCFGs.isEmpty()) {
            System.err.println("No CFG parsed or file error!");
        }

        // lines to functionSignatures
        List<Pair<String, List<String>>> functionSignatures = new ArrayList<>();

        for (String line : lines) {
            // Only process lines that contain "E:" and don't contain "***null***"
            if (line.contains("E:") && !line.contains("***null***")) {
                // Extract the function name and call chain
                int callChainIndex = line.indexOf("CallChain:");
                if (callChainIndex != -1) {
                    String callChain = line.substring(callChainIndex + "CallChain:".length()).trim();

                    // Extract the function name
                    int functionNameStart = line.indexOf("@") + 1;
                    int functionNameEnd = line.indexOf(" ", functionNameStart);
                    String functionName = line.substring(functionNameStart, functionNameEnd);

                    // Process call chain
                    String context;
                    List<String> calleeList = new ArrayList<>();
                    calleeList.add(functionName);

                    // If the call chain contains multiple functions (connected by ->)
                    if (callChain.contains("->")) {
                        // Get all but the last function as context
                        int lastArrowIndex = callChain.lastIndexOf("->");
                        context = callChain.substring(0, lastArrowIndex).trim();
                    } else {
                        // If there's only one function in the call chain
                        context = callChain.trim();
                    }

                    functionSignatures.add(new Pair<>(context, calleeList));
                }
            }
        }

        // Build the call tree and get the trace
        CallNode root = buildCallTree(functionSignatures);
        List<String> callChainTrace = new ArrayList<>();
        dfsCollect(root, callChainTrace);

        // Create new list for reconstructed lines
        List<String> reconstructedLines = new ArrayList<>();
        int nullIndex = 0;

        // Process each line
        for (String line : lines) {
            if (line.contains("***null***")) {
                // Replace ***null*** with the current function from callChainTrace
                String newLine = line.replace("***null***", callChainTrace.get(nullIndex));
                // Remove CallChain part if it exists
                int callChainIndex = newLine.indexOf("CallChain:");
                if (callChainIndex != -1) {
                    newLine = newLine.substring(0, callChainIndex).trim();
                }
                reconstructedLines.add(newLine.trim());
                nullIndex++;
            } else {
                // For non-null lines, just remove the CallChain part
                int callChainIndex = line.indexOf("CallChain:");
                if (callChainIndex != -1) {
                    line = line.substring(0, callChainIndex).trim();
                }
                reconstructedLines.add(line.trim());
            }
        }

        return reconstructedLines;
    }
}
