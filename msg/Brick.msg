string id
string type
string color
int32 layer

# --- Grid Coordinates (Target) ---
GridCell center_cell
GridCell[] occupied_cells

# --- Orientation ---
int32 orientation

# --- Source Location (From Camera) ---
# This is the new field we are adding:
geometry_msgs/Pose pickup_pose
geometry_msgs/Pose place_pose
int32 location
