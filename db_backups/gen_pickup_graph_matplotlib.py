import psycopg2
import math
import networkx as nx
import matplotlib.pyplot as plt

# ==========================================
# Parameter Definitions
# ==========================================
# Robot & Arm settings
ARM_BASE_POSE = (-0.20, -0.15, 3.14) 
OBJ_POS_ARMBASE = (0.0, 0.3)       

# Distances
ENTER_OFFSET_DIST = 0.8
EXIT_OFFSET_DIST = 0.7
VIA_NODE_DIST = 1.0
VIA_MARGIN = 1.5
STAGING_DISTANCE = 1.5


# Edge Weights
PICKUP_EDGE_WEIGHT = 3.0
INOUT_EDGE_WEIGHT = 4.0
VIA_EDGE_WEIGHT = 1.0
STAGING_EDGE_WEIGHT = 2.0

# Mock Docking Station Poses (x, y, theta_rad)
DOCK_POSES = [
    (-5.0, 1.0, math.pi/2),
    (-5.3, 9.0, math.pi/3)
]

# Database Credentials
DB_CONFIG = {
    'dbname': 'ros2',
    'user': 'admin',
    'password': 'Robotlab2019',
    'host': 'localhost',
    'port': '5432'
}

# --- Helper Functions ---
def transform_pose(pose_parent, pose_child):
    """Transforms a child pose (x, y, theta) to global frame given parent pose."""
    px, py, pt = pose_parent
    cx, cy, ct = (pose_child[0], pose_child[1], pose_child[2] if len(pose_child)>2 else 0)
    gx = px + cx * math.cos(pt) - cy * math.sin(pt)
    gy = py + cx * math.sin(pt) + cy * math.cos(pt)
    gt = (pt + ct) % (2 * math.pi)
    return (gx, gy, gt)

def line_intersects_bbox(p1, p2, bbox):
    """Checks if a line segment crosses a bounding box [xmin, ymin, xmax, ymax]"""
    xmin, ymin, xmax, ymax = bbox
    # Simple bounding box intersection using parametric line equations
    t_min, t_max = 0.0, 1.0
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    
    for p, q in [(-dx, p1[0] - xmin), (dx, xmax - p1[0]), (-dy, p1[1] - ymin), (dy, ymax - p1[1])]:
        if p == 0:
            if q < 0: return True # parallel and outside
        else:
            t = q / p
            if p < 0 and t > t_min: t_min = t
            elif p > 0 and t < t_max: t_max = t
    return t_min <= t_max

def fetch_object_data():
    """Connects to PostgreSQL and fetches object poses."""
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cursor = conn.cursor()
        query = "SELECT object_id, row_id, x_coord, y_coord, orientation_deg FROM object_data ORDER BY row_id, object_id;"
        cursor.execute(query)
        rows = cursor.fetchall()
        cursor.close()
        conn.close()
        return rows
    except Exception as e:
        print(f"Database connection failed: {e}")
        print("Using fallback dummy data for demonstration purposes...\n")
        # Dummy data: (object_id, row_id, x, y, theta_deg)
        return [
            (1, 1, 2.0, 2.0, 0.0),
            (2, 1, 4.0, 2.0, 0.0),
            (3, 1, 6.0, 2.0, 0.0),
            (4, 2, 2.0, 5.0, 45.0),
            (5, 2, 5.0, 8.0, 45.0)
        ]

def get_relative_object_to_robot():
    """Calculates where the object needs to be relative to the robot's base_link."""
    x_ar, y_ar, theta_ar = ARM_BASE_POSE
    x_oa, y_oa = OBJ_POS_ARMBASE
    
    # Transformation: object position in base_link frame
    x_or = x_ar + x_oa * math.cos(theta_ar) - y_oa * math.sin(theta_ar)
    y_or = y_ar + x_oa * math.sin(theta_ar) + y_oa * math.cos(theta_ar)
    
    return x_or, y_or

def calc_robot_pose(obj_x, obj_y, robot_theta_rad):
    """Calculates the required base_link pose (x,y) to pick up the object."""
    x_or, y_or = get_relative_object_to_robot()
    
    # Inverse transformation to find world coordinates of the robot base_link
    x_r = obj_x - (x_or * math.cos(robot_theta_rad) - y_or * math.sin(robot_theta_rad))
    y_r = obj_y - (x_or * math.sin(robot_theta_rad) + y_or * math.cos(robot_theta_rad))
    
    return x_r, y_r

def get_arm_base_world_pose(x_r, y_r, theta_r):
    """Calculates the world pose of the arm_base_link for plotting."""
    x_ar, y_ar, theta_ar = ARM_BASE_POSE
    x_aw = x_r + x_ar * math.cos(theta_r) - y_ar * math.sin(theta_r)
    y_aw = y_r + x_ar * math.sin(theta_r) + y_ar * math.cos(theta_r)
    return x_aw, y_aw, theta_r + theta_ar

def calc_dist(n1, n2):
    return math.hypot(n2['x'] - n1['x'], n2['y'] - n1['y'])

def build_navigation_graph(object_data):
    G = nx.DiGraph()
    objects = [] 
    node_id_counter = 1
    
    entering_nodes_ids = []
    exiting_nodes_ids = []

    # 1. Process Objects -> Pickup, Entering, Exiting Nodes
    rows = {}
    for obj in object_data:
        _, row_id, _, _, _ = obj
        if row_id not in rows: rows[row_id] = []
        rows[row_id].append(obj)
        objects.append(obj)

    for row_id, objs in rows.items():
        base_theta_deg = objs[0][4]
        theta1_rad = math.radians(base_theta_deg)
        theta2_rad = (math.radians(base_theta_deg + 180.0) + math.pi) % (2 * math.pi) - math.pi
        
        # Sort objects sequentially along line of travel
        def sort_objs(objs_list, dir_rad):
            return sorted(objs_list, key=lambda o: o[2]*math.cos(dir_rad) + o[3]*math.sin(dir_rad))

        seq1_objs = sort_objs(objs, theta1_rad)
        seq2_objs = sort_objs(objs, theta2_rad)

        for seq, t_rad in [(seq1_objs, theta1_rad), (seq2_objs, theta2_rad)]:
            if not seq: continue
            
            # Temporary storage for this row/direction's pickup nodes
            pickup_nodes = []
            all_pickup_nodes = []
            for obj in seq:
                obj_id, _, obj_x, obj_y, _ = obj
                r_x, r_y = calc_robot_pose(obj_x, obj_y, t_rad)
                pickup_nodes.append({'id': node_id_counter, 'x': r_x, 'y': r_y, 'theta': t_rad, 'obj_id': obj_id})
                G.add_node(node_id_counter, type='Pickup', x=r_x, y=r_y, theta=t_rad, obj_id=obj_id)
                all_pickup_nodes.append((r_x, r_y))
                node_id_counter += 1
                
            # Entering Node
            ent_x = pickup_nodes[0]['x'] - ENTER_OFFSET_DIST * math.cos(t_rad)
            ent_y = pickup_nodes[0]['y'] - ENTER_OFFSET_DIST * math.sin(t_rad)
            ent_id = node_id_counter
            G.add_node(ent_id, type='Entering', x=ent_x, y=ent_y, theta=t_rad)
            entering_nodes_ids.append(ent_id)
            node_id_counter += 1
            
            # Exiting Node
            ext_x = pickup_nodes[-1]['x'] + EXIT_OFFSET_DIST * math.cos(t_rad)
            ext_y = pickup_nodes[-1]['y'] + EXIT_OFFSET_DIST * math.sin(t_rad)
            ext_id = node_id_counter
            G.add_node(ext_id, type='Exiting', x=ext_x, y=ext_y, theta=t_rad)
            exiting_nodes_ids.append(ext_id)
            node_id_counter += 1

            # Edges: Entering -> First Pickup
            dist = calc_dist(G.nodes[ent_id], pickup_nodes[0])
            G.add_edge(ent_id, pickup_nodes[0]['id'], weight=dist*INOUT_EDGE_WEIGHT, type='Entering Edge')
            
            # Edges: Pickup -> Pickup
            for i in range(len(pickup_nodes) - 1):
                n1, n2 = pickup_nodes[i], pickup_nodes[i+1]
                dist = calc_dist(n1, n2)
                G.add_edge(n1['id'], n2['id'], weight=dist*PICKUP_EDGE_WEIGHT, type='Pickup Edge')
                
            # Edges: Last Pickup -> Exiting
            dist = calc_dist(pickup_nodes[-1], G.nodes[ext_id])
            G.add_edge(pickup_nodes[-1]['id'], ext_id, weight=dist*INOUT_EDGE_WEIGHT, type='Exiting Edge')

    # 2. Build Via Nodes (Bounding Box Perimeter)
    min_x = min(data['x'] for _, data in G.nodes(data=True)) - VIA_MARGIN
    max_x = max(data['x'] for _, data in G.nodes(data=True)) + VIA_MARGIN
    min_y = min(data['y'] for _, data in G.nodes(data=True)) - VIA_MARGIN
    max_y = max(data['y'] for _, data in G.nodes(data=True)) + VIA_MARGIN

    w, h = max_x - min_x, max_y - min_y
    nx_count, ny_count = max(1, int(round(w / VIA_NODE_DIST))), max(1, int(round(h / VIA_NODE_DIST)))

    # Perimeter definition CCW: (x, y, theta_rad)
    perimeter_points = []
    for i in range(nx_count): perimeter_points.append((min_x + i*(w/nx_count), min_y, 0.0))
    for i in range(ny_count): perimeter_points.append((max_x, min_y + i*(h/ny_count), math.pi/2))
    for i in range(nx_count): perimeter_points.append((max_x - i*(w/nx_count), max_y, math.pi))
    for i in range(ny_count): perimeter_points.append((min_x, max_y - i*(h/ny_count), -math.pi/2))

    via_ccw_ids, via_cw_ids = [], []
    for (px, py, p_theta) in perimeter_points:
        # CCW Node
        ccw_id = node_id_counter
        G.add_node(ccw_id, type='Via_CCW', x=px, y=py, theta=p_theta)
        via_ccw_ids.append(ccw_id)
        node_id_counter += 1
        
        # CW Node (Opposite orientation at same location)
        cw_id = node_id_counter
        cw_theta = (p_theta + math.pi + math.pi) % (2 * math.pi) - math.pi
        G.add_node(cw_id, type='Via_CW', x=px, y=py, theta=cw_theta)
        via_cw_ids.append(cw_id)
        node_id_counter += 1

        # Allow turning around by adding bidirectional zero-weight edges at the exact same location
        G.add_edge(ccw_id, cw_id, weight=0.0, type='Via Swap')
        G.add_edge(cw_id, ccw_id, weight=0.0, type='Via Swap')

    # Link rings
    for i in range(len(via_ccw_ids)):
        next_i = (i + 1) % len(via_ccw_ids)
        # CCW Edge (Forward in array)
        d_ccw = calc_dist(G.nodes[via_ccw_ids[i]], G.nodes[via_ccw_ids[next_i]])
        G.add_edge(via_ccw_ids[i], via_ccw_ids[next_i], weight=d_ccw*VIA_EDGE_WEIGHT, type='Via Edge CCW')
        # CW Edge (Backward in array)
        d_cw = calc_dist(G.nodes[via_cw_ids[next_i]], G.nodes[via_cw_ids[i]])
        G.add_edge(via_cw_ids[next_i], via_cw_ids[i], weight=d_cw*VIA_EDGE_WEIGHT, type='Via Edge CW')

    # 4. Generate Staging Nodes
    min_xs = min(data['x'] for _, data in G.nodes(data=True)) + VIA_MARGIN
    max_xs = max(data['x'] for _, data in G.nodes(data=True)) - VIA_MARGIN
    min_ys = min(data['y'] for _, data in G.nodes(data=True)) + VIA_MARGIN
    max_ys = max(data['y'] for _, data in G.nodes(data=True)) - VIA_MARGIN
    pickup_bbox = [min_xs, min_ys, max_xs, max_ys]
    staging_nodes = []
    for dx, dy, dt in DOCK_POSES:
        # Transform local (0, STAGING_DISTANCE) to global
        sx, sy, _ = transform_pose((dx, dy, dt), (0, STAGING_DISTANCE))
        # Pointing to origin of dock means looking back at (dx, dy)
        st = math.atan2(dy - sy, dx - sx)
        
        st_id = node_id_counter; node_id_counter += 1
        G.add_node(st_id, type='Staging', x=sx, y=sy, theta=st)
        staging_nodes.append(st_id)
        
        # Connect to visible Via nodes
        for v_id in via_ccw_ids + via_cw_ids:
            vx = G.nodes[v_id]['x']
            vy = G.nodes[v_id]['y']
            if not line_intersects_bbox((sx, sy), (vx, vy), pickup_bbox):
                dist = math.hypot(vx-sx, vy-sy)
                G.add_edge(st_id, v_id, weight=dist * STAGING_EDGE_WEIGHT, type='Staging')
                G.add_edge(v_id, st_id, weight=dist * STAGING_EDGE_WEIGHT, type='Staging')

    # 3. Connect Via Nodes to Entering and Exiting
    def get_two_closest(target_node_id, candidates_ids):
        sorted_candidates = sorted(candidates_ids, key=lambda c: calc_dist(G.nodes[target_node_id], G.nodes[c]))
        return sorted_candidates[:2]

    for ent_id in entering_nodes_ids:
        for via_id in get_two_closest(ent_id, via_ccw_ids) + get_two_closest(ent_id, via_cw_ids):
            dist = calc_dist(G.nodes[via_id], G.nodes[ent_id])
            G.add_edge(via_id, ent_id, weight=dist*INOUT_EDGE_WEIGHT, type='Transit Edge')

    for ext_id in exiting_nodes_ids:
        for via_id in get_two_closest(ext_id, via_ccw_ids) + get_two_closest(ext_id, via_cw_ids):
            dist = calc_dist(G.nodes[ext_id], G.nodes[via_id])
            G.add_edge(ext_id, via_id, weight=dist*INOUT_EDGE_WEIGHT, type='Transit Edge')

    return G, objects

def plot_arrow_head_only(ax, x, y, theta_rad, color, size, label=""):
    """Uses a rotated triangle marker to plot ONLY the arrowhead."""
    angle_deg = math.degrees(theta_rad) - 90
    ax.plot(x, y, marker=(3, 0, angle_deg), markersize=size, color=color,label=label, linestyle='none', zorder=3)

def visualize_graph(G, objects):
    plt.figure(figsize=(14, 12))
    ax = plt.gca()
    
    # 1. Objects (Black Arrowheads)
    for i, obj in enumerate(objects):
        _, _, x, y, theta_deg = obj
        dx, dy = math.cos(math.radians(theta_deg))*0.2, math.sin(math.radians(theta_deg))*0.2
        plt.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1, fc='black', ec='grey') # Arrow shaft
        # plot_arrow_head_only(ax, x, y, math.radians(theta_deg), 'black', size=15, 
        #                      label='Object Pose' if i == 0 else "")
        ax.text(x + 0.15, y, f" O{obj[0]}", color='black', fontsize=10, fontweight='bold')

    # 2. Nodes & Arm poses
    drawn_node, drawn_arm_x, drawn_arm_y = False, False, False

    for node_id, data in G.nodes(data=True):
        nx_val, ny_val, ntheta = data['x'], data['y'], data['theta']
        
        # Node (Orange Arrowhead)
        lbl = 'Robot Node' if not drawn_node else ""

        dx, dy = math.cos(ntheta)*0.05, math.sin(ntheta)*0.05
        plt.arrow(nx_val, ny_val, dx, dy, head_width=0.05, head_length=0.05, fc='orange', ec='red', label=lbl) # Arrow shaft
        
        # plot_arrow_head_only(ax, nx_val, ny_val, ntheta, 'orange', size=10, label=lbl)
        drawn_node = True
        
        # Keep map uncluttered: only draw Node text for non-Via nodes
        # if 'Via' not in data['type']:
        #     ax.text(nx_val, ny_val - 0.2, f" {data['type'][0]}{node_id}", color='darkorange', fontsize=8)
        ax.text(nx_val, ny_val - 0.2, f" {data['type'][0]}{node_id}", color='darkorange', fontsize=8)

        # 3. Arm Base Frame
        ax_w, ay_w, atheta_w = get_arm_base_world_pose(nx_val, ny_val, ntheta)
        
        # Arm X axis (Red)
        ax.quiver(ax_w, ay_w, math.cos(atheta_w), math.sin(atheta_w),
                  color='red', scale=50, width=0.003, headwidth=1,
                  label='Arm Base X' if not drawn_arm_x else "")
        drawn_arm_x = True
        
        # Arm Y axis (Green)
        ax.quiver(ax_w, ay_w, math.cos(atheta_w + math.pi/2), math.sin(atheta_w + math.pi/2),
                  color='green', scale=50, width=0.003, headwidth=2, 
                  label='Arm Base Y' if not drawn_arm_y else "")
        drawn_arm_y = True

    # 4. Edges (Blue Arrows)
    drawn_edge = False
    for u, v, data in G.edges(data=True):
        # Skip 'Via Swap' 0-weight edges for visual clarity
        # if data['type'] == 'Via Swap': continue 
        
        x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(edgecolor='blue', facecolor='blue', arrowstyle='-|>', 
                                    lw=1.0, shrinkA=10, shrinkB=10, alpha=0.4))
        if not drawn_edge:
            ax.plot([], [], color='blue', marker='>', linestyle='-', label='Directed Edge')
            drawn_edge = True

    # Draw Docking Stations
    for dx, dy, dt in DOCK_POSES:
        plt.arrow(dx, dy, math.cos(dt)*0.2, math.sin(dt)*0.2, head_width=0.1, fc='orange', ec='orange', label='Docking Station X-axis' if dx == DOCK_POSES[0][0] and dy == DOCK_POSES[0][1] else "") # X-axis
        plt.arrow(dx, dy, math.cos(dt+math.pi/2)*0.2, math.sin(dt+math.pi/2)*0.2, head_width=0.1, fc='cyan', ec='cyan', label='Docking Station Y-axis' if dx == DOCK_POSES[0][0] and dy == DOCK_POSES[0][1] else "") # Y-axis
        plt.plot(dx, dy, 'o', color='gray', markersize=4, label='Docking Station' if dx == DOCK_POSES[0][0] and dy == DOCK_POSES[0][1] else "")

    ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1.0))
    plt.title("Husky + Kinova Nav2 Complex Waypoint Graph")
    plt.xlabel("X Coordinate (m)")
    plt.ylabel("Y Coordinate (m)")
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

def main():
    object_data = fetch_object_data()
    if not object_data:
        print("No object data found in the database.")
        return

    # Build Graph
    G, objects = build_navigation_graph(object_data)
    
    # Terminal Output
    print(f"=== Directed Graph Nodes (Total: {G.number_of_nodes()}) ===")
    for node_id, data in sorted(G.nodes(data=True), key=lambda x: x[0]):
        print(f"Node {node_id:<3} | Type: {data['type']:<10} | x: {data['x']:>6.2f}, y: {data['y']:>6.2f}, theta: {data['theta']:>5.2f} rad")
        
    print(f"\n=== Directed Graph Edges (Total: {G.number_of_edges()}) ===")
    for u, v, data in sorted(G.edges(data=True), key=lambda e: (e[0], e[1])):
        print(f"Edge N{u:<3} -> N{v:<3} | Type: {data['type']:<15} | Weight: {data['weight']:.2f}")
        
    # Visual Output
    visualize_graph(G, objects)

if __name__ == '__main__':
    main()