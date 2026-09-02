import psycopg2
import math
import networkx as nx
import plotly.graph_objects as go

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
    
    x_or = x_ar + x_oa * math.cos(theta_ar) - y_oa * math.sin(theta_ar)
    y_or = y_ar + x_oa * math.sin(theta_ar) + y_oa * math.cos(theta_ar)
    return x_or, y_or

def calc_robot_pose(obj_x, obj_y, robot_theta_rad):
    """Calculates the required base_link pose (x,y) to pick up the object."""
    x_or, y_or = get_relative_object_to_robot()
    
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
        
        def sort_objs(objs_list, dir_rad):
            return sorted(objs_list, key=lambda o: o[2]*math.cos(dir_rad) + o[3]*math.sin(dir_rad))

        seq1_objs = sort_objs(objs, theta1_rad)
        seq2_objs = sort_objs(objs, theta2_rad)

        for seq, t_rad in [(seq1_objs, theta1_rad), (seq2_objs, theta2_rad)]:
            if not seq: continue
            
            pickup_nodes = []
            for obj in seq:
                obj_id, _, obj_x, obj_y, _ = obj
                r_x, r_y = calc_robot_pose(obj_x, obj_y, t_rad)
                pickup_nodes.append({'id': node_id_counter, 'x': r_x, 'y': r_y, 'theta': t_rad, 'obj_id': obj_id})
                G.add_node(node_id_counter, type='Pickup', x=r_x, y=r_y, theta=t_rad, obj_id=obj_id)
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

            # Edges
            dist = calc_dist(G.nodes[ent_id], pickup_nodes[0])
            G.add_edge(ent_id, pickup_nodes[0]['id'], weight=dist*INOUT_EDGE_WEIGHT, type='Entering Edge')
            
            for i in range(len(pickup_nodes) - 1):
                n1, n2 = pickup_nodes[i], pickup_nodes[i+1]
                dist = calc_dist(n1, n2)
                G.add_edge(n1['id'], n2['id'], weight=dist*PICKUP_EDGE_WEIGHT, type='Pickup Edge')
                
            dist = calc_dist(pickup_nodes[-1], G.nodes[ext_id])
            G.add_edge(pickup_nodes[-1]['id'], ext_id, weight=dist*INOUT_EDGE_WEIGHT, type='Exiting Edge')

    # 2. Build Via Nodes (Bounding Box Perimeter)
    min_x = min(data['x'] for _, data in G.nodes(data=True)) - VIA_MARGIN
    max_x = max(data['x'] for _, data in G.nodes(data=True)) + VIA_MARGIN
    min_y = min(data['y'] for _, data in G.nodes(data=True)) - VIA_MARGIN
    max_y = max(data['y'] for _, data in G.nodes(data=True)) + VIA_MARGIN

    w, h = max_x - min_x, max_y - min_y
    nx_count, ny_count = max(1, int(round(w / VIA_NODE_DIST))), max(1, int(round(h / VIA_NODE_DIST)))

    perimeter_points = []
    for i in range(nx_count): perimeter_points.append((min_x + i*(w/nx_count), min_y, 0.0))
    for i in range(ny_count): perimeter_points.append((max_x, min_y + i*(h/ny_count), math.pi/2))
    for i in range(nx_count): perimeter_points.append((max_x - i*(w/nx_count), max_y, math.pi))
    for i in range(ny_count): perimeter_points.append((min_x, max_y - i*(h/ny_count), -math.pi/2))

    via_ccw_ids, via_cw_ids = [], []
    for (px, py, p_theta) in perimeter_points:
        ccw_id = node_id_counter
        G.add_node(ccw_id, type='Via_CCW', x=px, y=py, theta=p_theta)
        via_ccw_ids.append(ccw_id)
        node_id_counter += 1
        
        cw_id = node_id_counter
        cw_theta = (p_theta + math.pi + math.pi) % (2 * math.pi) - math.pi
        G.add_node(cw_id, type='Via_CW', x=px, y=py, theta=cw_theta)
        via_cw_ids.append(cw_id)
        node_id_counter += 1

        G.add_edge(ccw_id, cw_id, weight=0.0, type='Via Swap')
        G.add_edge(cw_id, ccw_id, weight=0.0, type='Via Swap')

    for i in range(len(via_ccw_ids)):
        next_i = (i + 1) % len(via_ccw_ids)
        d_ccw = calc_dist(G.nodes[via_ccw_ids[i]], G.nodes[via_ccw_ids[next_i]])
        G.add_edge(via_ccw_ids[i], via_ccw_ids[next_i], weight=d_ccw*VIA_EDGE_WEIGHT, type='Via Edge CCW')
        d_cw = calc_dist(G.nodes[via_cw_ids[next_i]], G.nodes[via_cw_ids[i]])
        G.add_edge(via_cw_ids[next_i], via_cw_ids[i], weight=d_cw*VIA_EDGE_WEIGHT, type='Via Edge CW')

    # 3. Generate Staging Nodes
    min_xs = min(data['x'] for _, data in G.nodes(data=True)) + VIA_MARGIN
    max_xs = max(data['x'] for _, data in G.nodes(data=True)) - VIA_MARGIN
    min_ys = min(data['y'] for _, data in G.nodes(data=True)) + VIA_MARGIN
    max_ys = max(data['y'] for _, data in G.nodes(data=True)) - VIA_MARGIN
    pickup_bbox = [min_xs, min_ys, max_xs, max_ys]
    
    staging_nodes = []
    for dx, dy, dt in DOCK_POSES:
        sx, sy, _ = transform_pose((dx, dy, dt), (0, STAGING_DISTANCE))
        st = math.atan2(dy - sy, dx - sx)
        
        st_id = node_id_counter; node_id_counter += 1
        G.add_node(st_id, type='Staging', x=sx, y=sy, theta=st)
        staging_nodes.append(st_id)
        
        for v_id in via_ccw_ids + via_cw_ids:
            vx = G.nodes[v_id]['x']
            vy = G.nodes[v_id]['y']
            if not line_intersects_bbox((sx, sy), (vx, vy), pickup_bbox):
                dist = math.hypot(vx-sx, vy-sy)
                G.add_edge(st_id, v_id, weight=dist * STAGING_EDGE_WEIGHT, type='Staging')
                G.add_edge(v_id, st_id, weight=dist * STAGING_EDGE_WEIGHT, type='Staging')

    # 4. Connect Via Nodes to Entering and Exiting
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

# ==========================================
# Plotly Visualization Helpers
# ==========================================
def add_vectorized_arrows(fig, coords_list, color, width=2, name="", showlegend=False):
    """Draws batched lines with arrowheads for high performance in Plotly."""
    ax, ay = [], []
    for (x, y, theta, length) in coords_list:
        end_x = x + length * math.cos(theta)
        end_y = y + length * math.sin(theta)
        
        # Arrowhead wings (V shape)
        hl = length * 0.3
        h1x = end_x - hl * math.cos(theta - math.pi/6)
        h1y = end_y - hl * math.sin(theta - math.pi/6)
        h2x = end_x - hl * math.cos(theta + math.pi/6)
        h2y = end_y - hl * math.sin(theta + math.pi/6)
        
        ax.extend([x, end_x, h1x, end_x, h2x, None])
        ay.extend([y, end_y, h1y, end_y, h2y, None])
        
    fig.add_trace(go.Scatter(
        x=ax, y=ay, mode='lines', 
        line=dict(color=color, width=width), 
        name=name, showlegend=showlegend, hoverinfo='none'
    ))

def visualize_graph(G, objects):
    fig = go.Figure()

    # ----------------------------------------
    # 1. Base Geometry Drawing
    # ----------------------------------------
    # Edges (Blue Lines + Arrows)
    edge_ax, edge_ay = [], []
    for u, v, data in G.edges(data=True):
        if data['type'] == 'Via Swap': continue 
        
        x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        theta = math.atan2(y2 - y1, x2 - x1)
        
        hl = 0.05 # Arrowhead size for edges
        h1x = x2 - hl * math.cos(theta - math.pi/6)
        h1y = y2 - hl * math.sin(theta - math.pi/6)
        h2x = x2 - hl * math.cos(theta + math.pi/6)
        h2y = y2 - hl * math.sin(theta + math.pi/6)

        edge_ax.extend([x1, x2, h1x, x2, h2x, None])
        edge_ay.extend([y1, y2, h1y, y2, h2y, None])

    fig.add_trace(go.Scatter(
        x=edge_ax, y=edge_ay, mode='lines',
        line=dict(color='rgba(0, 0, 255, 0.4)', width=1),
        name='Directed Edge', showlegend=True, hoverinfo='none'
    ))

    # Objects (Black Arrows & Text)
    obj_arrows, obj_text_x, obj_text_y, obj_text = [], [], [], []
    for obj in objects:
        obj_id, _, x, y, theta_deg = obj
        obj_arrows.append((x, y, math.radians(theta_deg), 0.2))
        obj_text_x.append(x + math.cos(math.radians(theta_deg)+math.pi/4) * 0.15)
        obj_text_y.append(y + math.sin(math.radians(theta_deg)+math.pi/4) * 0.15)
        obj_text.append(f"O{obj_id}")

    add_vectorized_arrows(fig, obj_arrows, 'black', width=3, name="Object Pose", showlegend=True)
    fig.add_trace(go.Scatter(
        x=obj_text_x, y=obj_text_y, mode='text', text=obj_text,
        textfont=dict(color='black', size=12, family="Arial Black"), 
        textposition='middle right', showlegend=False, hoverinfo='none'
    ))

    # Nodes and Arm Poses
    node_arrows = []
    node_text_x, node_text_y, node_text = [], [], []
    arm_x_arrows, arm_y_arrows = [], []

    for node_id, data in G.nodes(data=True):
        nx_val, ny_val, ntheta = data['x'], data['y'], data['theta']
        
        # Node geometry
        node_arrows.append((nx_val, ny_val, ntheta, 0.1))
        node_text_x.append(nx_val + math.cos(ntheta+math.pi/4) * 0.15)
        node_text_y.append(ny_val + math.sin(ntheta+math.pi/4) * 0.15)
        node_text.append(f"{data['type'][0]}{node_id}")

        # Arm geometry
        ax_w, ay_w, atheta_w = get_arm_base_world_pose(nx_val, ny_val, ntheta)
        arm_x_arrows.append((ax_w, ay_w, atheta_w, 0.15))
        arm_y_arrows.append((ax_w, ay_w, atheta_w + math.pi/2, 0.15))

    add_vectorized_arrows(fig, node_arrows, 'orange', width=2, name="Robot Node", showlegend=True)
    fig.add_trace(go.Scatter(
        x=node_text_x, y=node_text_y, mode='text', text=node_text,
        textfont=dict(color='darkorange', size=9), textposition='bottom center', 
        showlegend=False, hoverinfo='none'
    ))
    
    add_vectorized_arrows(fig, arm_x_arrows, 'red', width=1.5, name="Arm Base X", showlegend=True)
    add_vectorized_arrows(fig, arm_y_arrows, 'green', width=1.5, name="Arm Base Y", showlegend=True)

    # Docking Stations
    dock_x_arrows, dock_y_arrows = [], []
    dock_center_x, dock_center_y = [], []
    for dx, dy, dt in DOCK_POSES:
        dock_x_arrows.append((dx, dy, dt, 0.2))
        dock_y_arrows.append((dx, dy, dt + math.pi/2, 0.2))
        dock_center_x.append(dx)
        dock_center_y.append(dy)

    add_vectorized_arrows(fig, dock_x_arrows, 'orange', width=3, name="Docking X-axis", showlegend=True)
    add_vectorized_arrows(fig, dock_y_arrows, 'cyan', width=3, name="Docking Y-axis", showlegend=True)
    fig.add_trace(go.Scatter(
        x=dock_center_x, y=dock_center_y, mode='markers',
        marker=dict(color='gray', size=8), name="Docking Station", 
        showlegend=True, hoverinfo='none'
    ))

    # ----------------------------------------
    # 2. Hover Interaction Layers (Invisible Markers)
    # ----------------------------------------
    
    # Hover Layer for Nodes
    node_hover_x, node_hover_y, node_hover_text = [], [], []
    for node_id, data in G.nodes(data=True):
        nx_val, ny_val, ntheta = data['x'], data['y'], data['theta']
        theta_deg = math.degrees(ntheta) % 360
        node_hover_x.append(nx_val + math.cos(ntheta) * 0.05)  # Slight offset for better hover detection
        node_hover_y.append(ny_val + math.sin(ntheta) * 0.05)
        node_hover_text.append(
            f"<b>Node {node_id}</b><br>"
            f"Type: {data['type']}<br>"
            f"X: {nx_val:.2f}, Y: {ny_val:.2f}<br>"
            f"Orientation: {theta_deg:.1f}°"
        )

    fig.add_trace(go.Scatter(
        x=node_hover_x, y=node_hover_y, mode='markers',
        marker=dict(size=14, color='rgba(0,0,0,0)'), # Invisible marker sized to catch mouse
        hoverinfo='text', hovertext=node_hover_text,
        showlegend=False, name="Node Info"
    ))

    # Hover Layer for Edges (Placed at the midpoints of the lines)
    edge_hover_x, edge_hover_y, edge_hover_text = [], [], []
    for u, v, data in G.edges(data=True):
        if data['type'] == 'Via Swap': continue 
        
        x1, y1 = G.nodes[u]['x'], G.nodes[u]['y']
        x2, y2 = G.nodes[v]['x'], G.nodes[v]['y']
        
        # Calculate midpoint
        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0
        
        edge_hover_x.append(mid_x)
        edge_hover_y.append(mid_y)
        edge_hover_text.append(
            f"<b>Edge {u} → {v}</b><br>"
            f"Type: {data['type']}<br>"
            f"Weight: {data['weight']:.2f}"
        )

    fig.add_trace(go.Scatter(
        x=edge_hover_x, y=edge_hover_y, mode='markers',
        marker=dict(size=10, color='rgba(0,0,0,0)'), # Invisible marker on the line
        hoverinfo='text', hovertext=edge_hover_text,
        showlegend=False, name="Edge Info"
    ))

    # ----------------------------------------
    # 3. Layout Formatting
    # ----------------------------------------
    fig.update_layout(
        title="Husky + Kinova Nav2 Complex Waypoint Graph (Plotly)",
        xaxis=dict(scaleanchor="y", scaleratio=1, title="X Coordinate (m)", showgrid=True, gridcolor='rgba(0,0,0,0.1)'),
        yaxis=dict(title="Y Coordinate (m)", showgrid=True, gridcolor='rgba(0,0,0,0.1)'),
        plot_bgcolor='white',
        legend=dict(x=1.02, y=1, bordercolor='LightGray', borderwidth=1),
        width=1200, height=900,
        margin=dict(l=50, r=150, t=80, b=50), # Extra right margin for legend
        hoverlabel=dict(bgcolor="white", font_size=14, font_family="Arial")
    )
    
    fig.show()

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