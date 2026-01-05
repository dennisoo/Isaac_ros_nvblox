#!/usr/bin/env python3
# ist noch unbenutzt, aber vielleicht spaeter hilfreich
import argparse
import numpy as np
import yaml
from plyfile import PlyData, PlyElement

def load_semantic_config(config_path):
    """Load semantic class configuration."""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Build ID to color mapping
    id_to_color = {}
    for name, info in config['semantic_classes'].items():
        id_to_color[info['id']] = info['color']
    
    return id_to_color

def colorize_mesh(input_ply, output_ply, config_path):
    """
    Colorize mesh vertices based on semantic labels.
    Assumes mesh has a 'label' or 'class_id' property per vertex.
    """
    print(f"Loading mesh: {input_ply}")
    ply_data = PlyData.read(input_ply)
    
    # Get vertex data
    vertices = ply_data['vertex']
    n_vertices = len(vertices)
    print(f"Vertices: {n_vertices}")
    
    # Load semantic config
    id_to_color = load_semantic_config(config_path)
    print(f"Loaded {len(id_to_color)} semantic classes")
    
    # Check if mesh has semantic labels
    has_labels = False
    label_field = None
    
    for field in ['label', 'class_id', 'semantic_id', 'semantic_label']:
        if field in vertices.dtype.names:
            label_field = field
            has_labels = True
            print(f"Found semantic labels in field: {label_field}")
            break
    
    if not has_labels:
        print("WARNING: No semantic labels found in mesh!")
        print("Available fields:", vertices.dtype.names)
        print("Mesh will be colored with default color.")
        label_field = None
    
    # Prepare new vertex data with colors
    new_vertices = []
    
    for i, vertex in enumerate(vertices):
        x, y, z = vertex['x'], vertex['y'], vertex['z']
        
        if has_labels:
            # Get semantic label
            class_id = int(vertex[label_field])
            
            # Get color for this class
            color = id_to_color.get(class_id, [200, 200, 200])  # Default gray
            r, g, b = color
        else:
            # Default white color
            r, g, b = 255, 255, 255
        
        new_vertices.append((x, y, z, r, g, b))
        
        if i % 10000 == 0:
            print(f"Processed {i}/{n_vertices} vertices...")
    
    # Create new PLY element with colors
    vertex_dtype = [
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
    ]
    
    vertex_array = np.array(new_vertices, dtype=vertex_dtype)
    vertex_element = PlyElement.describe(vertex_array, 'vertex')
    
    # Copy faces if they exist
    elements = [vertex_element]
    if 'face' in ply_data:
        elements.append(ply_data['face'])
        print(f"Faces: {len(ply_data['face'])}")
    
    # Save colored mesh
    print(f"Saving colored mesh: {output_ply}")
    PlyData(elements).write(output_ply)
    print("Done!")
    
    # Print statistics
    if has_labels:
        unique_labels = np.unique([v[label_field] for v in vertices])
        print(f"\nFound {len(unique_labels)} unique semantic classes:")
        for label in sorted(unique_labels):
            count = np.sum([v[label_field] == label for v in vertices])
            percentage = (count / n_vertices) * 100
            print(f"  Class {label}: {count} vertices ({percentage:.1f}%)")

def main():
    parser = argparse.ArgumentParser(
        description='Colorize PLY mesh based on semantic labels'
    )
    parser.add_argument('input', help='Input PLY file')
    parser.add_argument('output', help='Output PLY file')
    parser.add_argument(
        '--config',
        default='/workspaces/isaac_ros-dev/config/semantic_classes.yaml',
        help='Semantic classes YAML config'
    )
    
    args = parser.parse_args()
    
    colorize_mesh(args.input, args.output, args.config)

if __name__ == '__main__':
    main()
