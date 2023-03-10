# Grid Configuration File Format

## Template
```
#num_vertices #num_cells #num_anchors #num_paths #length_path
nv nc na np lp

#vertices
x y

#anchors
n n

#cells [type s=shear r=rigid a=active]\
t v v v v

#input path
v1
x y
x y


v2
x y
x y
```

## Initial Information

Information about the input must be in the following order, separated by a space:

Number of vertices, Number of cells, Number of anchors, Number of paths to trace, Number of points in each path

## Initial Vertex Locations

For each vertex, specify its coordinates when every cell in the grid is at 90 degrees. The first coordinate listed corresponds to vertex 0, and so on.

## Anchor Vertices
Specify the anchor vertices by their index, separated by a space, on a single line.

## Cells
For each cell, specify the type by a single letter, and give the 4 vertices comprising its corners, starting from the lower left going counter-clockwise. The first cell specified is cell 0, and so on. When giving an input to the simulation tool, do not specify any cells as active. It does not matter whether the cells are rigid or shear.

## Input Path
Specify the vertex the path is at, then give the x and y coordinate the vertex should try to reach at each step. If there are multiple paths to trace, give 2 blank lines after the end of the previous path, then repeat.
