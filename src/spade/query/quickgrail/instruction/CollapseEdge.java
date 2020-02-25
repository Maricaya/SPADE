/*
 --------------------------------------------------------------------------------
 SPADE - Support for Provenance Auditing in Distributed Environments.
 Copyright (C) 2018 SRI International

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
package spade.query.quickgrail.instruction;

import java.util.ArrayList;

import spade.query.quickgrail.entities.Graph;
import spade.query.quickgrail.utility.TreeStringSerializable;

/**
 * Collapse all edges whose specified fields are the same.
 */
public class CollapseEdge extends Instruction{
	// Input graph.
	public final Graph targetGraph;
	// Output graph.
	public final Graph sourceGraph;
	// Fields to check.
	private final ArrayList<String> fields;

	public CollapseEdge(Graph targetGraph, Graph sourceGraph, ArrayList<String> fields){
		this.targetGraph = targetGraph;
		this.sourceGraph = sourceGraph;
		this.fields = fields;
	}

	public final ArrayList<String> getFields(){
		return fields == null ? null : new ArrayList<String>(fields);
	}

	@Override
	public String getLabel(){
		return "CollapseEdge";
	}

	@Override
	protected void getFieldStringItems(ArrayList<String> inline_field_names, ArrayList<String> inline_field_values,
			ArrayList<String> non_container_child_field_names,
			ArrayList<TreeStringSerializable> non_container_child_fields, ArrayList<String> container_child_field_names,
			ArrayList<ArrayList<? extends TreeStringSerializable>> container_child_fields){
		inline_field_names.add("targetGraph");
		inline_field_values.add(targetGraph.name);
		inline_field_names.add("sourceGraph");
		inline_field_values.add(sourceGraph.name);
		inline_field_names.add("fields");
		inline_field_values.add(String.join(",", fields == null ? new ArrayList<String>() : fields));
	}
}
