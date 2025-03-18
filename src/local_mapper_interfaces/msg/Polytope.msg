# A single polytope represented by extreme points and half-space (Ax <= b)

# List of extreme points (N x 2) representation
float64[] vertices  # Flattened (N*2) list

# Half-space representation Ax <= b
float64[] normals   # Flattened (M*2) list
float64[] b   # Flattened (M*1) list
