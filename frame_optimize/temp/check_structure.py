#!/usr/bin/env python3
"""Check structural connectivity of the frame"""
import json
import numpy as np

# Read input JSON
with open('/Users/sota/projects/FomulaTMU/sim_overall/frame_optimize/temp/input_7400385809466126.json', 'r') as f:
    data = json.load(f)

n_nodes = len(data['nodes'])
elements = data['elements']
bcs = data['bcs']

print(f"Total nodes: {n_nodes}")
print(f"Total elements: {len(elements)}")
print(f"Total boundary conditions: {len(bcs)}")

# Check which nodes are connected to elements
connected_nodes = set()
for elem in elements:
    node_i = int(elem[0]) - 1  # Convert to 0-indexed
    node_j = int(elem[1]) - 1
    connected_nodes.add(node_i)
    connected_nodes.add(node_j)

# Find floating nodes
all_nodes = set(range(n_nodes))
floating_nodes = all_nodes - connected_nodes
if floating_nodes:
    print(f"\n⚠️  WARNING: Floating nodes (not connected to any element):")
    for node in sorted(floating_nodes):
        print(f"  Node {node+1}: {data['nodes'][node]}")
else:
    print("\n✓ All nodes are connected to at least one element")

# Check constrained DOFs
constrained_dofs = {}
for bc in bcs:
    node = int(bc[0])
    dof = int(bc[1])
    if node not in constrained_dofs:
        constrained_dofs[node] = []
    constrained_dofs[node].append(dof)

print(f"\n✓ Boundary conditions:")
for node in sorted(constrained_dofs.keys()):
    print(f"  Node {node}: DOFs {sorted(constrained_dofs[node])}")

# Count total constrained DOFs
total_constrained = sum(len(dofs) for dofs in constrained_dofs.values())
print(f"\nTotal constrained DOFs: {total_constrained}")
print(f"Total DOFs in system: {n_nodes * 6}")

# Check for minimum constraints (need at least 6 to remove rigid body modes)
if total_constrained < 6:
    print(f"\n❌ ERROR: Need at least 6 constrained DOFs to remove rigid body modes")
else:
    print(f"\n✓ Sufficient constraints to remove rigid body modes")

# Check element connectivity matrix
connectivity = {}
for i in range(n_nodes):
    connectivity[i] = set()

for elem in elements:
    node_i = int(elem[0]) - 1
    node_j = int(elem[1]) - 1
    connectivity[node_i].add(node_j)
    connectivity[node_j].add(node_i)

# Find isolated components using BFS
def find_components():
    visited = set()
    components = []

    for start in range(n_nodes):
        if start not in visited and start in connected_nodes:
            # BFS
            component = set()
            queue = [start]
            while queue:
                node = queue.pop(0)
                if node in visited:
                    continue
                visited.add(node)
                component.add(node)
                for neighbor in connectivity[node]:
                    if neighbor not in visited:
                        queue.append(neighbor)
            components.append(component)

    return components

components = find_components()
print(f"\n✓ Structural connectivity analysis:")
print(f"  Number of separate components: {len(components)}")
for i, comp in enumerate(components):
    print(f"  Component {i+1}: {len(comp)} nodes")
    if len(comp) <= 5:
        print(f"    Nodes: {sorted([n+1 for n in comp])}")

if len(components) > 1:
    print(f"\n⚠️  WARNING: Structure has {len(components)} separate components!")
    print("   This may cause numerical issues in the solver.")
