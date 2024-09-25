/*
 --------------------------------------------------------------------------------
 SPADE - Support for Provenance Auditing in Distributed Environments.
 Copyright (C) 2022 SRI International

 This program is free software: you can redistribute it and/or  
 modify it under the terms of the GNU General Public License as  
 published by the Free Software Foundation, either version 3 of the  
 License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of  
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU  
 General Public License for more details.

 You should have received a copy of the GNU General Public License  
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 --------------------------------------------------------------------------------
 */

 /*
 About the filter:
	The VersionProcess filter assumes that the input graph contains 
	edges representing a read ("Used") operation involving process 
	vertices. It is also assumed that the state of a process vertex 
	can change as a result of a read operation, which is reflected 
	by versioning the process vertex.
	This is done by incrementing the version number of the process 
	vertex each time it reads data (i.e., each time a "Used" edge is 
	encountered). The filter also creates new edges to connect the 
	versioned process vertices to their respective artifacts.
	The transformed graph will have versioned process vertices, with 
	each version representing a different state of the process.
 */
package spade.filter;

import java.util.HashMap;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

import spade.core.AbstractEdge;
import spade.core.AbstractFilter;
import spade.core.AbstractVertex;
import spade.core.Edge;
import spade.core.Settings;
import spade.utility.ArgumentFunctions;
import spade.utility.HelperFunctions;

public class VersionProcess extends AbstractFilter{

	private static final Logger logger = Logger.getLogger(VersionProcess.class.getName());

	private static final String keyVersionAnnotationName = "versionAnnotationName",
			keyInitialVersion = "initialVersion", keyEdgeAnnoKey = "edgeAnnoKey", keyEdgeAnnoValue = "edgeAnnoValue";

	private String versionAnnotationName;
	private long initialVersion;
	private String edgeAnnoKey, edgeAnnoValue;

	private Map<String, VertexState> verticesState = new HashMap<String, VertexState>();

	@Override
	public boolean initialize(final String arguments){
		try{
			final Map<String, String> map = HelperFunctions.parseKeyValuePairsFrom(arguments,
					new String[]{Settings.getDefaultConfigFilePath(this.getClass())});
			final String versionAnnotationName = ArgumentFunctions.mustParseNonEmptyString(keyVersionAnnotationName,
					map);
			final long initialVersion = ArgumentFunctions.mustParseLong(keyInitialVersion, map);
			final String edgeAnnoKey = ArgumentFunctions.mustParseNonEmptyString(keyEdgeAnnoKey, map);
			final String edgeAnnoValue = ArgumentFunctions.mustParseNonEmptyString(keyEdgeAnnoValue, map);
			return initialize(versionAnnotationName, initialVersion, edgeAnnoKey, edgeAnnoValue);
		}catch(Exception e){
			logger.log(Level.SEVERE, "Failed to initialize filter", e);
			return false;
		}
	}

	public boolean initialize(final String versionAnnotationName, final long initialVersion, final String edgeAnnoKey,
			final String edgeAnnoValue){
		this.versionAnnotationName = versionAnnotationName;
		this.initialVersion = initialVersion;
		this.edgeAnnoKey = edgeAnnoKey;
		this.edgeAnnoValue = edgeAnnoValue;
		logger.log(Level.INFO,
				"Arguments {" + keyVersionAnnotationName + "=" + this.versionAnnotationName + ", " + keyInitialVersion
						+ "=" + this.initialVersion + ", " + keyEdgeAnnoKey + "=" + this.edgeAnnoKey + ", "
						+ keyEdgeAnnoValue + "=" + this.edgeAnnoValue 
				+ "}");
		return true;
	}

	// Only store state for process, rest will be passed to putInNextFilter
	@Override
	public void putVertex(final AbstractVertex vertex){
		if(vertex == null){
			return;
		}
		final String hash = vertex.bigHashCode();
		if(verticesState.get(hash) == null && vertex.type() == "Process"){
			verticesState.put(hash, new VertexState());
		}
		else {
			if(vertex.type() == "Process"){
				verticesState.get(hash).putInNextFilterIfHasNotBeenPut(vertex);
			}
			else {
				putInNextFilter(vertex);
			}
		}
	}

	// Add annotation for versioned edge
	private void addReadInfoToEdge(final AbstractEdge edge){
		edge.addAnnotation(edgeAnnoKey, edgeAnnoValue);
	}

	@Override
	public void putEdge(final AbstractEdge edge){
		if(edge == null){
			return;
		}
		final AbstractVertex childVertex = edge.getChildVertex();
		final AbstractVertex parentVertex = edge.getParentVertex();
		if(childVertex == null || parentVertex == null){
			return;
		}
		putVertex(childVertex);
		putVertex(parentVertex);

		// Versioning process vertex for read operation 
		if(childVertex.type() == "Process" && parentVertex.type() == "Artifact" && edge.type() == "Used"){
			final VertexState childState = verticesState.get(childVertex.bigHashCode());
			final AbstractVertex childVertexCurrentState = childState.putInNextFilterIfHasNotBeenPut(childVertex);
			childState.incrementVORVersion();
			final AbstractVertex childVertexNewState = childState.putInNextFilterIfHasNotBeenPut(childVertex);
			
			final AbstractVertex parentCurrentVertex = new spade.core.Vertex();
			parentCurrentVertex.addAnnotations(parentVertex.getCopyOfAnnotations());
			putInNextFilter(parentCurrentVertex);

			// Edge for new versioned process and artifact
			final AbstractEdge edgeCopy = new Edge(childVertexNewState, parentCurrentVertex);
			edgeCopy.addAnnotations(edge.getCopyOfAnnotations());
			putInNextFilter(edgeCopy);
			
			// Edge for new versioned process and old versioned process
			final AbstractEdge childStateChangeEdge = new Edge(childVertexNewState, childVertexCurrentState);
			addReadInfoToEdge(childStateChangeEdge);
			putInNextFilter(childStateChangeEdge);
			
		}
	}

	private class VertexState{
		private long vorVersion = initialVersion;
		private boolean hasBeenPut = false;

		private long getVORVersion(){
			return vorVersion;
		}

		private void incrementVORVersion(){
			if(!hasBeenPut){
				// don't double-increment
			}else{
				vorVersion++;
				hasBeenPut = false;
			}
		}

		private AbstractVertex createVertexWithCurrentState(final AbstractVertex vertex){
			final AbstractVertex vertexCopy = new spade.core.Vertex();
			vertexCopy.addAnnotations(vertex.getCopyOfAnnotations());
			vertexCopy.addAnnotation(versionAnnotationName, String.valueOf(getVORVersion()));
			return vertexCopy;
		}

		private AbstractVertex putInNextFilterIfHasNotBeenPut(final AbstractVertex vertex){
			final AbstractVertex vertexCopy = createVertexWithCurrentState(vertex);
			if(!this.hasBeenPut){
				putInNextFilter(vertexCopy);
				this.hasBeenPut = true;
			}
			return vertexCopy;
		}
	}
}
