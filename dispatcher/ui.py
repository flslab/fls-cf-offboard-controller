import tkinter as tk
import math
import logging

logger = logging.getLogger(__name__)

class DispatcherUI(tk.Tk):
    def __init__(self, assignments, outliers, mission):
        super().__init__()
        self.title("Swarm Dispatcher")
        self.geometry("1100x800")
        self.configure(bg="#1E1E1E")
        self.confirmed = False
        
        self.assignments = assignments
        self.outliers = outliers or []
        self.mission = mission
        
        self.viewpoint_var = tk.IntVar(value=-1)
        self.anchor_var = tk.IntVar(value=-1)
        
        # Header label
        header = tk.Label(self, text="Automated Swarm Dispatcher\nClick a drone, then another to swap ID assignments.",
                          bg="#1E1E1E", fg="#FFFFFF", font=("Arial", 14, "bold"))
        header.pack(pady=10)

        main_frame = tk.Frame(self, bg="#1E1E1E")
        main_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(main_frame, bg="#1E1E1E", highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(20, 10), pady=10)

        if self.outliers:
            sidebar = tk.Frame(main_frame, bg="#2A2A2A", width=300)
            sidebar.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 20), pady=10)
            
            lbl_outliers = tk.Label(sidebar, text="Unassigned Markers\n(Viewpoint / Anchor)", 
                                    bg="#2A2A2A", fg="#FFFFFF", font=("Arial", 12, "bold"))
            lbl_outliers.pack(pady=(10, 5))

            hdr_frame = tk.Frame(sidebar, bg="#2A2A2A")
            hdr_frame.pack(fill=tk.X, padx=5, pady=5)
            tk.Label(hdr_frame, text="Coordinates", bg="#2A2A2A", fg="#AAAAAA", width=15).pack(side=tk.LEFT)
            tk.Label(hdr_frame, text="View ", bg="#2A2A2A", fg="#AAAAAA", width=5).pack(side=tk.LEFT)
            tk.Label(hdr_frame, text="Anchor", bg="#2A2A2A", fg="#AAAAAA", width=5).pack(side=tk.LEFT)

            def on_vp_select():
                v = self.viewpoint_var.get()
                if v != -1 and self.anchor_var.get() == v:
                    self.anchor_var.set(-1)

            def on_an_select():
                a = self.anchor_var.get()
                if a != -1 and self.viewpoint_var.get() == a:
                    self.viewpoint_var.set(-1)

            for i, p in enumerate(self.outliers):
                row_frame = tk.Frame(sidebar, bg="#2A2A2A")
                row_frame.pack(fill=tk.X, padx=5, pady=2)
                
                coords = f"[{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}]"
                tk.Label(row_frame, text=coords, bg="#2A2A2A", fg="#FFFFFF", width=15).pack(side=tk.LEFT)
                
                # using a label context instead of radio string so it aligns with header columns
                vp_radio = tk.Radiobutton(row_frame, variable=self.viewpoint_var, value=i, bg="#2A2A2A", activebackground="#2A2A2A", command=on_vp_select)
                vp_radio.pack(side=tk.LEFT, padx=(5, 10))

                an_radio = tk.Radiobutton(row_frame, variable=self.anchor_var, value=i, bg="#2A2A2A", activebackground="#2A2A2A", command=on_an_select)
                an_radio.pack(side=tk.LEFT, padx=(5, 10))
                
            clear_btn = tk.Button(sidebar, text="Clear Selections", 
                                  command=lambda: (self.viewpoint_var.set(-1), self.anchor_var.set(-1)),
                                  bg="#555555", fg="white")
            clear_btn.pack(pady=10)
        
        # Bottom frame for controls
        self.control_frame = tk.Frame(self, bg="#1E1E1E")
        self.control_frame.pack(fill=tk.X, pady=10)
        
        self.confirm_btn = tk.Button(self.control_frame, text="Confirm Assignments", 
                                     command=self.confirm, bg="#4CAF50", fg="black",
                                     font=("Arial", 14, "bold"), padx=20, pady=10)
        self.confirm_btn.pack(pady=10)
        
        self.selected_idx = None
        self.nodes = []
        
        self.canvas.bind("<Button-1>", self.on_click)
        
        # Initial draw delayed slightly to allow window to maximize
        self.after(50, self.draw_map)

    def confirm(self):
        self.confirmed = True
        self.extra_params = {}
        if self.outliers:
            v_idx = self.viewpoint_var.get()
            a_idx = self.anchor_var.get()
            if v_idx != -1:
                self.extra_params['viewpoint'] = self.outliers[v_idx]
            if a_idx != -1:
                self.extra_params['anchor'] = self.outliers[a_idx]
        self.quit()
        self.destroy()

    def _world_to_canvas(self, x, y, min_x, max_x, min_y, max_y, width, height):
        pad = 0.2 # 20% padding around the field
        w_range = max_x - min_x
        h_range = max_y - min_y
        
        if w_range == 0: w_range = 1
        if h_range == 0: h_range = 1
        
        scale = min(width / (w_range * (1 + pad*2)), height / (h_range * (1 + pad*2)))
        
        cx_world = (min_x + max_x) / 2
        cy_world = (min_y + max_y) / 2
        
        cx_canvas = width / 2
        cy_canvas = height / 2
        
        # in canvas, y goes down, so flip y
        cx = cx_canvas + (x - cx_world) * scale
        cy = cy_canvas - (y - cy_world) * scale
        return cx, cy

    def draw_map(self):
        self.canvas.delete("all")
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        
        if width <= 1 or height <= 1:
            self.after(50, self.draw_map)
            return

        points_x = []
        points_y = []
        for a in self.assignments:
            points_x.append(a['vicon_pos'][0])
            points_y.append(a['vicon_pos'][1])
            d_id = a['id']
            if self.mission and 'drones' in self.mission and d_id in self.mission['drones']:
                target = self.mission['drones'][d_id].get('target')
                if target:
                    points_x.append(target[0])
                    points_y.append(target[1])
                    
        if not points_x:
            min_x, max_x = -1.0, 1.0
            min_y, max_y = -1.0, 1.0
        else:
            min_x, max_x = min(points_x), max(points_x)
            min_y, max_y = min(points_y), max(points_y)

        self.nodes = []
        
        for idx, a in enumerate(self.assignments):
            d_id = a['id']
            vx, vy = a['vicon_pos'][0], a['vicon_pos'][1]
            cx, cy = self._world_to_canvas(vx, vy, min_x, max_x, min_y, max_y, width, height)
            
            tx, ty = None, None
            if self.mission and 'drones' in self.mission and d_id in self.mission['drones']:
                target = self.mission['drones'][d_id].get('target')
                if target:
                    tx, ty = self._world_to_canvas(target[0], target[1], min_x, max_x, min_y, max_y, width, height)
            
            if tx is not None and ty is not None:
                # Line
                self.canvas.create_line(cx, cy, tx, ty, fill="#555555", dash=(4, 4), width=2)
                # Cross for target
                L = 8
                self.canvas.create_line(tx-L, ty, tx+L, ty, fill="#FFA500", width=2)
                self.canvas.create_line(tx, ty-L, tx, ty+L, fill="#FFA500", width=2)
                self.canvas.create_text(tx, ty-15, text=f"{d_id}_Target", fill="#FFA500", font=("Arial", 10))

            r = 15
            color = "#00BFFF" if idx != self.selected_idx else "#FF1493"
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=color, outline="#FFFFFF", width=2)
            self.canvas.create_text(cx, cy+r+15, text=d_id, fill="#FFFFFF", font=("Arial", 12, "bold"))
            
            # Also show actual position as text below
            self.canvas.create_text(cx, cy+r+30, text=f"({vx:.2f}, {vy:.2f})", fill="#888888", font=("Arial", 9))
            
            self.nodes.append({"idx": idx, "cx": cx, "cy": cy, "r": r})

    def on_click(self, event):
        x, y = event.x, event.y
        clicked_idx = None
        for node in self.nodes:
            if math.hypot(node["cx"] - x, node["cy"] - y) <= node["r"] + 5:  # slight grace area
                clicked_idx = node["idx"]
                break
                
        if clicked_idx is not None:
            if self.selected_idx is None:
                self.selected_idx = clicked_idx
            else:
                if self.selected_idx != clicked_idx:
                    # Swap
                    id1 = self.assignments[self.selected_idx]['id']
                    id2 = self.assignments[clicked_idx]['id']
                    self.assignments[self.selected_idx]['id'] = id2
                    self.assignments[clicked_idx]['id'] = id1
                self.selected_idx = None
            self.draw_map()
        else:
            if self.selected_idx is not None:
                self.selected_idx = None
                self.draw_map()

def show_ui(assignments, outliers, mission):
    """
    Renders the Dispatcher UI and waits for user confirmation.
    Returns (edited assignments, extra_params) if confirmed, (None, None) otherwise.
    """
    app = DispatcherUI(assignments, outliers, mission)
    # Make sure window takes focus
    app.lift()
    app.attributes('-topmost', True)
    app.after_idle(app.attributes, '-topmost', False)
    app.focus_force()
    
    import os
    # Bring the specific python process to the front on macOS
    os.system(f"osascript -e 'tell application \"System Events\" to set frontmost of the first process whose unix id is {os.getpid()} to true'")
    
    app.mainloop()
    return (app.assignments, app.extra_params) if app.confirmed else (None, None)
