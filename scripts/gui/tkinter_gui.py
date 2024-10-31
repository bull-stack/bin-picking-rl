import tkinter as tk
from ttkthemes import ThemedTk

class TkinterGui(ThemedTk):
    def __init__(self, spawn_objects, start_sim, reset_sim, pause_sim=None, generate_image=None):
        super().__init__()
        self.title("UR10 Bin-Picking Simulator")
        
        self.spawn_objects = spawn_objects
        self.start_sim = start_sim
        # self.pause_sim = pause_sim  
        self.reset_sim = reset_sim
        # self.generate_image = generate_image
        
        self.resizable(False, False)
        self.geometry("215x170")
        self.configure(background="gray")
        self.dark_style()
            
    def create_frame(self):
        self.frame = tk.Frame(self, height=150)
        self.frame.pack(fill='both', padx=5, pady=5)

        count_label = tk.Label(self.frame, text="Quantity", width=10)
        count_label.grid(row=0, column=0, padx=10, pady=(5, 0))

        count_entry = tk.Entry(self.frame, width=12)
        count_entry.grid(row=0, column=1, padx=10, pady=(5, 0))

        spawn_button = tk.Button(self.frame, text="Generate", command=lambda: self.spawn_objects(count=int(count_entry.get())))
        spawn_button.grid(row=1, column=0, columnspan=2, pady=5, sticky='we')  # Set columnspan to 2 and use sticky='we'

        # Center the button within the frame
        self.frame.grid_rowconfigure(1, weight=1)  # Allow row 1 to expand vertically
        self.frame.grid_columnconfigure(0, weight=1)  # Allow column 0 to expand horizontally

    
    def create_sim_frame(self):
        self.sim_frame = tk.Frame(self)
        self.sim_frame.pack(fill='both', padx=5, pady=5)

        start_button = tk.Button(self.sim_frame, text="Start", command=lambda: self.start_sim())
        start_button.pack(fill='x', pady=5)
    
        
        reset_button = tk.Button(self.sim_frame, text="Reset", command=lambda: self.reset_sim())
        reset_button.pack(fill='x', pady=5)
    
    # def create_image_frame(self):
    #     self.image_frame = tk.Frame(self)
    #     self.image_frame.pack(fill='both', padx=5, pady=5)

    #     gen_image = tk.Button(self.image_frame, text="Generate Image", command=lambda: self.generate_image())
    #     gen_image.pack(padx=10, pady=5)
        
        # pause_button = tk.Button(self.sim_frame, text="Pause", command=lambda: self.pause_sim())
        # pause_button.pack(padx=10, pady=5)
        
        # reset_button = tk.Button(self.sim_frame, text="Reset", command=lambda: self.reset_sim())
        # reset_button.pack(padx=10, pady=5)

    def dark_style(self):
        style = tk.ttk.Style()
        style.theme_use("equilux")
