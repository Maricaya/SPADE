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

    // Node class representing a node in the graph
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

    /**
     * Combine call chains (supports a custom expected set)
     * Explanation:
     * - For each mapping key (e.g., "main" or "main->print_even"), split the key to get prefix tokens.
     * - The corresponding CFG is obtained using the last token in the prefix; call recoverPath to get the call chain string,
     *   then split the string by "->" into tokens.
     * - Combination:
     *   1. For the first mapping, use its call chain tokens as the initial combined chain.
     *   2. For subsequent mappings, find the common prefix between the combined chain and the mapping prefix,
     *      remove the duplicate token at the tail of the combined chain, then append the rest of the prefix,
     *      and finally append the recovered chain (skipping the first token as it is already included).
     * - Finally, filter out unwanted nodes using the expected set and merge adjacent duplicate nodes.
     */
    public static String combineCallChains(List<Pair<String, List<String>>> functionSignatures,
                                           Map<String, CFG> allCFGs,
                                           Set<String> expected) {
        List<List<String>> recoveredChains = new ArrayList<>();
        List<List<String>> prefixes = new ArrayList<>();

        for (Pair<String, List<String>> fs : functionSignatures) {
            List<String> mappingPrefix = splitString(fs.first, "->");
            String effectiveFuncName = mappingPrefix.get(mappingPrefix.size() - 1).trim();
            CFG currCfg = allCFGs.get(effectiveFuncName);
            if (currCfg == null) {
                System.err.println("No function '" + effectiveFuncName + "' found in CFGs.");
                continue;
            }
            String recovered = currCfg.recoverPath(fs.second);
            List<String> tokens = splitString(recovered, "->");
            recoveredChains.add(tokens);
            prefixes.add(mappingPrefix);
        }

        if (recoveredChains.isEmpty())
            return "";

        List<String> combinedChain = new ArrayList<>(recoveredChains.get(0));

        for (int i = 1; i < recoveredChains.size(); i++) {
            List<String> curPrefix = prefixes.get(i);
            int common = 0;
            while (common < combinedChain.size() && common < curPrefix.size() &&
                   combinedChain.get(common).trim().equals(curPrefix.get(common).trim())) {
                common++;
            }
            if (common > 0 && !combinedChain.isEmpty() &&
                combinedChain.get(combinedChain.size() - 1).trim().equals(curPrefix.get(common - 1).trim())) {
                combinedChain.remove(combinedChain.size() - 1);
            }
            for (int j = common; j < curPrefix.size(); j++) {
                combinedChain.add(curPrefix.get(j).trim());
            }
            List<String> tokens = recoveredChains.get(i);
            for (int j = 1; j < tokens.size(); j++) {
                combinedChain.add(tokens.get(j).trim());
            }
        }
        if (combinedChain.isEmpty() || !combinedChain.get(combinedChain.size() - 1).trim().equals(prefixes.get(0).get(0).trim())) {
            combinedChain.add(prefixes.get(0).get(0).trim());
        }

        List<String> filtered = new ArrayList<>();
        for (String token : combinedChain) {
            if (expected.contains(token))
                filtered.add(token);
        }
        List<String> dedup = new ArrayList<>();
        for (String token : filtered) {
            if (dedup.isEmpty() || !dedup.get(dedup.size() - 1).equals(token))
                dedup.add(token);
        }

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < dedup.size(); i++) {
            if (i > 0)
                sb.append("->");
            sb.append(dedup.get(i));
        }
        return sb.toString();
    }


    // cd /home/ubuntu/SPADE ; /usr/bin/env /usr/lib/jvm/java-11-openjdk-amd64/bin/java @/tmp/cp_6hthejvvuesmmrkxt5y716be.argfile spade.reporter.Recover

    public static void main() {
        // Assume the CFG file is located at "hello/cfg.txt"
        String filename = "cfg.txt";

        // Parse the CFG file to get a mapping from function name to CFG
        Map<String, CFG> allCFGs = parseCFGFile(filename);
        if (allCFGs.isEmpty()) {
            System.err.println("No CFG parsed or file error!");
            return;
        }

        // Define multiple call chain signatures
        // For example:
        //   1st mapping: { "main", {"print_odd"} }
        //   2nd mapping: { "main->print_even", {"add"} }
        //   3rd mapping: { "main", {"empty"} }
        List<Pair<String, List<String>>> functionSignatures = new ArrayList<>();
        functionSignatures.add(new Pair<>("main", Arrays.asList("print_odd")));
        functionSignatures.add(new Pair<>("main->print_even", Arrays.asList("add")));
        functionSignatures.add(new Pair<>("main", Arrays.asList("empty")));

        // Recover and print the call chain for each mapping (for debugging)
        for (Pair<String, List<String>> fs : functionSignatures) {
            List<String> mappingPrefix = splitString(fs.first, "->");
            String effectiveFuncName = mappingPrefix.get(mappingPrefix.size() - 1).trim();
            CFG curCFG = allCFGs.get(effectiveFuncName);
            if (curCFG == null) {
                System.err.println("No function '" + effectiveFuncName + "' found in CFGs.");
            } else {
                String recoveredPath = curCFG.recoverPath(fs.second);
                System.out.println("[<< " + fs.first + " >>] Recovered path: " + recoveredPath);
            }
        }

        // Define a custom expected set (adjust as needed)
        Set<String> expected = new HashSet<>(Arrays.asList("main", "print_odd", "print_even", "add", "empty"));

        // Combine multiple call chains with the custom expected set
        String combinedPath = combineCallChains(functionSignatures, allCFGs, expected);
        System.out.println("Combined call chain: " + combinedPath);
    }
}
