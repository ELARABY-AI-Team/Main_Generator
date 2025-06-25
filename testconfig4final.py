import google.generativeai as genai
import os
import re
import datetime
import threading
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText
import webbrowser
import json
import sys
import requests
import base64
from tkinter import simpledialog
import random
import fitz  # PyMuPDF
import time

# Support for PyInstaller .exe path
BASE_PATH = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
MCU_DATA_FILE = os.path.join(BASE_PATH, "mcu_data.json")
TOKEN_FILE = os.path.join(BASE_PATH, "github_token.txt")
SETTINGS_FILE = os.path.join(BASE_PATH, "app_settings.json")

# Load GitHub token
try:
    with open(TOKEN_FILE, "r") as f:
        GITHUB_TOKEN = f.read().strip()
except FileNotFoundError:
    GITHUB_TOKEN = ""

GITHUB_USERNAME = "ELARABY-AI-Team"
REPO_NAME = "Main_Generator"
HEADERS_FOLDER = "Main_files"
CONFIG_FOLDER_REGISTERS = "Configs_Registers"
CONFIG_FOLDER_OPTION_BITE = "Configs_OptionBite"


genai.configure(api_key=os.getenv('GEMINI_API_KEY', 'AIzaSyARSUt--3_I1NWk7blW9qCuNfs3cmTULnc'))
model = genai.GenerativeModel("gemini-2.5-flash-preview-04-17")

class AdvancedSettingsDialog(tk.Toplevel):
    def __init__(self, parent, title, initial_values=None):
        super().__init__(parent)
        self.title(title)
        self.parent = parent
        self.initial_values = initial_values or {}
        self.result = None
        
        # Window styling
        self.configure(bg='#2c3e50')
        self.geometry("450x400")
        self.resizable(False, False)
        
        # Main container
        self.main_container = tk.Frame(self, bg='#3498db', padx=1, pady=1)
        self.main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Content frame
        content_frame = tk.Frame(self.main_container, bg='#ecf0f1')
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # Custom title bar
        self.title_bar = tk.Frame(content_frame, bg='#3498db', height=40)
        self.title_bar.pack(fill=tk.X)
        
        # Title label
        self.title_label = tk.Label(
            self.title_bar, 
            text="⚙️ ADVANCED SETTINGS", 
            font=("Segoe UI", 12, "bold"), 
            bg='#3498db', 
            fg='white'
        )
        self.title_label.pack(side=tk.LEFT, padx=10)
        
        # Close button
        close_btn = tk.Label(
            self.title_bar, 
            text="×", 
            font=("Segoe UI", 16), 
            bg='#3498db', 
            fg='white',
            cursor="hand2"
        )
        close_btn.pack(side=tk.RIGHT, padx=10)
        close_btn.bind("<Button-1>", lambda e: self.on_cancel())
        
        # Settings container
        settings_container = tk.Frame(content_frame, bg='#ecf0f1', padx=20, pady=15)
        settings_container.pack(fill=tk.BOTH, expand=True)
        
        # --- VOLTAGE SETTING (Default: 3.3V) ---
        self.voltage = tk.StringVar(value=self.initial_values.get('voltage', '3.3'))  # Default: 3.3V
        voltage_frame = tk.LabelFrame(
            settings_container,
            text=" Operating Voltage ",
            font=("Segoe UI", 10, "bold"),
            bg='#ecf0f1',
            fg='#2c3e50',
            bd=2,
            relief=tk.GROOVE
        )
        voltage_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Radiobutton(
            voltage_frame,
            text="3.3V",
            variable=self.voltage,
            value="3.3",
            font=("Segoe UI", 10),
            bg='#ecf0f1',
            fg='#2c3e50',
            selectcolor='#bdc3c7',
            activebackground='#ecf0f1',
            activeforeground='#e74c3c'
        ).pack(side=tk.LEFT, padx=10, pady=5)
        
        tk.Radiobutton(
            voltage_frame,
            text="5V",
            variable=self.voltage,
            value="5",
            font=("Segoe UI", 10),
            bg='#ecf0f1',
            fg='#2c3e50',
            selectcolor='#bdc3c7',
            activebackground='#ecf0f1',
            activeforeground='#e74c3c'
        ).pack(side=tk.LEFT, padx=10, pady=5)
        
        # --- CLOCK SOURCE (Default: Internal) ---
        self.clock_source = tk.StringVar(value=self.initial_values.get('clock_source', 'Internal'))  # Default: Internal
        clock_frame = tk.LabelFrame(
            settings_container,
            text=" Clock Source ",
            font=("Segoe UI", 10, "bold"),
            bg='#ecf0f1',
            fg='#2c3e50',
            bd=2,
            relief=tk.GROOVE
        )
        clock_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Radiobutton(
            clock_frame,
            text="Internal",
            variable=self.clock_source,
            value="Internal",
            font=("Segoe UI", 10),
            bg='#ecf0f1',
            fg='#2c3e50',
            selectcolor='#bdc3c7',
            activebackground='#ecf0f1',
            activeforeground='#e74c3c'
        ).pack(side=tk.LEFT, padx=10, pady=5)
        
        tk.Radiobutton(
            clock_frame,
            text="External",
            variable=self.clock_source,
            value="External",
            font=("Segoe UI", 10),
            bg='#ecf0f1',
            fg='#2c3e50',
            selectcolor='#bdc3c7',
            activebackground='#ecf0f1',
            activeforeground='#e74c3c'
        ).pack(side=tk.LEFT, padx=10, pady=5)
        
        # --- REGISTER MODE (Default: Registers) ---
        self.register_mode = tk.StringVar(value=self.initial_values.get('register_mode', 'Registers'))  # Default: Registers
        register_frame = tk.LabelFrame(
            settings_container,
            text=" Register Mode ",
            font=("Segoe UI", 10, "bold"),
            bg='#ecf0f1',
            fg='#2c3e50',
            bd=2,
            relief=tk.GROOVE
        )
        register_frame.pack(fill=tk.X, pady=(0, 15))
        
        tk.Radiobutton(
            register_frame,
            text="Registers",
            variable=self.register_mode,
            value="Registers",
            font=("Segoe UI", 10),
            bg='#ecf0f1',
            fg='#2c3e50',
            selectcolor='#bdc3c7',
            activebackground='#ecf0f1',
            activeforeground='#e74c3c'
        ).pack(side=tk.LEFT, padx=10, pady=5)
        
        tk.Radiobutton(
            register_frame,
            text="Option Bite",
            variable=self.register_mode,
            value="Option Bite",
            font=("Segoe UI", 10),
            bg='#ecf0f1',
            fg='#2c3e50',
            selectcolor='#bdc3c7',
            activebackground='#ecf0f1',
            activeforeground='#e74c3c'
        ).pack(side=tk.LEFT, padx=10, pady=5)
        
        # Button frame
        button_frame = tk.Frame(settings_container, bg='#ecf0f1')
        button_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Save button
        save_btn = tk.Button(
            button_frame,
            text="💾 SAVE SETTINGS",
            command=self.on_save,
            bg='#2ecc71',
            fg='white',
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=20,
            pady=8,
            relief=tk.FLAT,
            activebackground='#27ae60',
            activeforeground='white'
        )
        save_btn.pack(side=tk.RIGHT, padx=(10, 0))
        
        # Cancel button
        cancel_btn = tk.Button(
            button_frame,
            text="🚫 CANCEL",
            command=self.on_cancel,
            bg='#e74c3c',
            fg='white',
            font=("Segoe UI", 10),
            bd=0,
            padx=20,
            pady=8,
            relief=tk.FLAT,
            activebackground='#c0392b',
            activeforeground='white'
        )
        cancel_btn.pack(side=tk.RIGHT)
        
        # Make draggable
        self.title_bar.bind("<ButtonPress-1>", self.start_move)
        self.title_bar.bind("<ButtonRelease-1>", self.stop_move)
        self.title_bar.bind("<B1-Motion>", self.on_move)
        
        # Center the dialog
        self.update_idletasks()
        x = parent.winfo_x() + (parent.winfo_width() // 2) - (self.winfo_width() // 2)
        y = parent.winfo_y() + (parent.winfo_height() // 2) - (self.winfo_height() // 2)
        self.geometry(f"+{x}+{y}")
        
        # Focus on window
        self.focus_force()
    
    def start_move(self, event):
        self.x = event.x
        self.y = event.y
    
    def stop_move(self, event):
        self.x = None
        self.y = None
    
    def on_move(self, event):
        deltax = event.x - self.x
        deltay = event.y - self.y
        x = self.winfo_x() + deltax
        y = self.winfo_y() + deltay
        self.geometry(f"+{x}+{y}")
    
    def on_save(self):
        self.result = {
            'voltage': self.voltage.get(),
            'clock_source': self.clock_source.get(),
            'register_mode': self.register_mode.get()
        }
        self.destroy()
    
    def on_cancel(self):
        self.result = None
        self.destroy()
    

def check_github_for_header(mcu_name):
    if not GITHUB_TOKEN:
        return None
        
    file_path = f"{HEADERS_FOLDER}/{mcu_name}_MAIN.h"
    url = f"https://api.github.com/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{file_path}"
    headers = {"Authorization": f"token {GITHUB_TOKEN}"}
    response = requests.get(url, headers=headers)

    if response.status_code == 200:
        data = response.json()
        return data.get("download_url")
    else:
        print(f"❌ File not found on GitHub: {file_path} — {response.status_code}")
    return None

def download_header_from_github(url):
    response = requests.get(url)
    if response.status_code == 200:
        return response.text
    return None

def upload_header_to_github(mcu_name, code):
    if not GITHUB_TOKEN:
        return False
        
    file_path = f"{HEADERS_FOLDER}/{mcu_name}_MAIN.h"
    url = f"https://api.github.com/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{file_path}"
    headers = {"Authorization": f"token {GITHUB_TOKEN}"}
    content_encoded = base64.b64encode(code.encode()).decode()

    # Check if file already exists (to get SHA)
    sha = None
    check_resp = requests.get(url, headers=headers)
    if check_resp.status_code == 200:
        sha = check_resp.json()['sha']

    data = {
        "message": f"Add {mcu_name}_MAIN.h",
        "content": content_encoded,
        "sha": sha  # This is None for new files
    }

    response = requests.put(url, headers=headers, json=data)
    return response.status_code in [200, 201]

def upload_config_to_github(mcu_name, h_content, c_content, folder_name):
    """Upload both config.h and config.c to GitHub in the specified folder"""
    if not GITHUB_TOKEN:
        return False

    h_file_path = f"{folder_name}/{mcu_name}_config.h"
    c_file_path = f"{folder_name}/{mcu_name}_config.c"

    h_url = f"https://api.github.com/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{h_file_path}"
    c_url = f"https://api.github.com/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{c_file_path}"

    headers = {"Authorization": f"token {GITHUB_TOKEN}"}
    h_encoded = base64.b64encode(h_content.encode()).decode()
    c_encoded = base64.b64encode(c_content.encode()).decode()

    # Check if files already exist
    sha_info = {}
    for url, path, content in [(h_url, h_file_path, h_encoded), (c_url, c_file_path, c_encoded)]:
        resp = requests.get(url, headers=headers)
        sha = resp.json().get("sha") if resp.status_code == 200 else None
        sha_info[path] = sha

    h_data = {
        "message": f"Add {mcu_name}_config.h",
        "content": h_encoded,
        "sha": sha_info[h_file_path]
    }

    c_data = {
        "message": f"Add {mcu_name}_config.c",
        "content": c_encoded,
        "sha": sha_info[c_file_path]
    }

    h_resp = requests.put(h_url, headers=headers, json=h_data)
    c_resp = requests.put(c_url, headers=headers, json=c_data)

    return h_resp.status_code in [200, 201] and c_resp.status_code in [200, 201]

def check_github_for_config(mcu_name, folder_name):
    """Check if config files exist on GitHub in the given folder"""
    if not GITHUB_TOKEN:
        return None, None

    h_file_path = f"{folder_name}/{mcu_name}_config.h"
    c_file_path = f"{folder_name}/{mcu_name}_config.c"

    h_url = f"https://api.github.com/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{h_file_path}"
    c_url = f"https://api.github.com/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{c_file_path}"

    headers = {"Authorization": f"token {GITHUB_TOKEN}"}

    h_response = requests.get(h_url, headers=headers)
    c_response = requests.get(c_url, headers=headers)

    h_download_url = h_response.json().get("download_url") if h_response.status_code == 200 else None
    c_download_url = c_response.json().get("download_url") if c_response.status_code == 200 else None

    return h_download_url, c_download_url


def load_settings(use_defaults=False):
    if use_defaults:
        return {
            'voltage': '3.3',
            'clock_source': 'Internal',
            'register_mode': 'Registers'
        }
        
    try:
        if os.path.exists(SETTINGS_FILE):
            with open(SETTINGS_FILE, 'r') as f:
                return json.load(f)
    except Exception as e:
        print(f"Error loading settings: {e}")
        
    return {
        'voltage': '3.3',
        'clock_source': 'Internal',
        'register_mode': 'Registers'
    }

def save_settings(settings):
    try:
        with open(SETTINGS_FILE, 'w') as f:
            json.dump(settings, f, indent=4)
    except Exception as e:
        print(f"Error saving settings: {e}")

class MCUManager:
    def __init__(self):
        self.mcu_list = self.load_mcu_list()
        
    def load_mcu_list(self):
        default_list = [
            "RENESAS_R5F11BBC"
        ]
        try:
            if os.path.exists(MCU_DATA_FILE):
                with open(MCU_DATA_FILE, 'r') as f:
                    data = json.load(f)
                    return data.get('mcu_list', default_list)
            return default_list
        except Exception as e:
            print(f"Error loading MCU list: {e}")
            return default_list
            
    def save_mcu_list(self):
        try:
            with open(MCU_DATA_FILE, 'w') as f:
                json.dump({'mcu_list': self.mcu_list}, f, indent=4)
        except Exception as e:
            print(f"Error saving MCU list: {e}")
            
    def add_mcu(self, mcu_name):
        if mcu_name and mcu_name.strip() and mcu_name not in self.mcu_list:
            self.mcu_list.append(mcu_name.strip())
            self.save_mcu_list()
            return True
        return False

class MainApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Ragura")
        self.root.geometry("700x600")
        self.root.configure(bg="#2c3e50")
        self.root.resizable(True, True)
        self.last_saved_path = None
        self.last_generated_mcu = None
        self.settings = load_settings()
        self.extracted_pdf_text = ""
        self.running = False  # Flag to control process execution
        self.current_thread = None  # Reference to current running thread

        self.mcu_manager = MCUManager()
        self.setup_ui()

    def setup_ui(self):
        self.root.state('zoomed')
        self.root.resizable(True, True)
        
        # Main container
        self.main_frame = tk.Frame(self.root, bg="#2c3e50")
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.header_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.header_frame.pack(fill=tk.X, pady=(0, 10))

        self.logo_label = tk.Label(self.header_frame, text="⚙️", font=("Segoe UI", 24), bg="#2c3e50", fg="#3498db")
        self.logo_label.pack(side=tk.LEFT, padx=(0, 10))

        self.title_label = tk.Label(self.header_frame, text="Ragura", font=("Segoe UI", 18, "bold"), bg="#2c3e50", fg="#ecf0f1")
        self.title_label.pack(side=tk.LEFT)
        
        self.subtitle_label = tk.Label(
            self.header_frame,
            text="AI-Driven Code Generator for Embedded Microcontrollers",
            font=("Segoe UI", 10, "italic"),
            bg="#2c3e50",
            fg="#bdc3c7"
        )
        self.subtitle_label.pack(side=tk.LEFT, padx=(10, 0))

        self.settings_frame = tk.LabelFrame(self.main_frame, text=" Settings ", font=("Segoe UI", 10, "bold"), bg="#34495e", fg="#ecf0f1", relief=tk.GROOVE, bd=2)
        self.settings_frame.pack(fill=tk.X, pady=(0, 10))

        self.mcu_label = tk.Label(self.settings_frame, text="Select Microcontroller:", bg="#34495e", fg="#ecf0f1", font=("Segoe UI", 10))
        self.mcu_label.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        self.mcu_combobox = ttk.Combobox(self.settings_frame, values=self.mcu_manager.mcu_list, font=("Segoe UI", 10), width=30)
        self.mcu_combobox.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        if self.mcu_manager.mcu_list:
            self.mcu_combobox.current(0)
        self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")

        self.custom_mcu_frame = tk.Frame(self.settings_frame, bg="#34495e")
        self.custom_mcu_label = tk.Label(self.custom_mcu_frame, text="Enter New MCU (Example: HOLTEK_HT46R24):", bg="#34495e", fg="#ecf0f1", font=("Segoe UI", 10))
        self.custom_mcu_label.pack(side=tk.LEFT, padx=(0, 5))
        self.custom_mcu_entry = tk.Entry(self.custom_mcu_frame, bg="#2c3e50", fg="#ecf0f1", insertbackground="white", font=("Segoe UI", 10), width=28)
        self.custom_mcu_entry.pack(side=tk.LEFT)
        self.custom_mcu_frame.grid(row=1, column=0, columnspan=2, pady=(5, 0), sticky=tk.W)
        self.custom_mcu_frame.grid_remove()
        self.mcu_combobox.bind("<<ComboboxSelected>>", self.on_mcu_selected)

        self.buttons_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.buttons_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Left-aligned buttons
        left_buttons_frame = tk.Frame(self.buttons_frame, bg="#2c3e50")
        left_buttons_frame.pack(side=tk.LEFT)
        
        self.advanced_btn = tk.Button(
            left_buttons_frame, 
            text="🎛️ Advanced Settings", 
            command=self.show_advanced_settings, 
            bg="#9b59b6", 
            fg="#7f8c8d",
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT,
            state=tk.DISABLED
        )
        self.advanced_btn.pack(side=tk.LEFT)

        self.generate_config_btn = tk.Button(
            left_buttons_frame,
            text="⚙️ Generate config.h/.c",
            command=self.on_generate_config_click,
            bg="#ADEBB3",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=15,
            pady=8,
            relief=tk.FLAT,
            state=tk.DISABLED
        )
        self.generate_config_btn.pack(side=tk.LEFT, padx=(10, 0))

        self.generate_btn = tk.Button(
            left_buttons_frame, 
            text="✨ Generate main.h", 
            command=self.on_generate_click, 
            bg="#3498db", 
            fg="white", 
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT
        )
        self.generate_btn.pack(side=tk.LEFT)

        # Right-aligned cancel button
        right_buttons_frame = tk.Frame(self.buttons_frame, bg="#2c3e50")
        right_buttons_frame.pack(side=tk.RIGHT)
        self.reset_btn = tk.Button(
            right_buttons_frame,
            text="🔄 Reset",
            command=self.reset_system,
            bg="#f39c12",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=15,
            pady=8,
            relief=tk.FLAT,
            activebackground='#d35400',
            activeforeground='white'
        )
        self.reset_btn.pack(side=tk.RIGHT, padx=(0, 10))
        
        self.cancel_btn = tk.Button(
            right_buttons_frame,
            text="⛔ Cancel",
            command=self.cancel_process,
            bg="#e74c3c",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=15,
            pady=8,
            relief=tk.FLAT,
            state=tk.DISABLED
        )
        self.cancel_btn.pack(side=tk.RIGHT)

        self.progress_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.progress_frame.pack(fill=tk.X, pady=(0, 10))
        self.progress_label = tk.Label(self.progress_frame, text="Ready", bg="#2c3e50", fg="#bdc3c7", font=("Segoe UI", 9))
        self.progress_label.pack(side=tk.LEFT)
        self.progress_bar = ttk.Progressbar(self.progress_frame, orient=tk.HORIZONTAL, length=100, mode='determinate')
        self.progress_bar.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        # Create notebook for tabbed preview
        self.notebook = ttk.Notebook(self.main_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Tab for main.h preview
        self.main_tab = tk.Frame(self.notebook, bg="#34495e")
        self.notebook.add(self.main_tab, text="main.h")
        self.main_preview = ScrolledText(self.main_tab, bg="#1e272e", fg="#ecf0f1", insertbackground="white", 
                                       font=("Consolas", 10), wrap=tk.NONE, padx=10, pady=10)
        self.main_preview.pack(fill=tk.BOTH, expand=True)
        
        # Tab for config files preview
        self.config_tab = tk.Frame(self.notebook, bg="#34495e")
        self.notebook.add(self.config_tab, text="config.h/.c")
        
        # Paned window for split view of config files
        self.config_paned = tk.PanedWindow(self.config_tab, orient=tk.HORIZONTAL, bg="#34495e")
        self.config_paned.pack(fill=tk.BOTH, expand=True)
        
        # config.h panel
        self.config_h_frame = tk.Frame(self.config_paned, bg="#34495e")
        self.config_h_label = tk.Label(self.config_h_frame, text="config.h", bg="#34495e", fg="#ecf0f1", 
                                      font=("Segoe UI", 10, "bold"))
        self.config_h_label.pack()
        self.config_h_preview = ScrolledText(self.config_h_frame, bg="#1e272e", fg="#ecf0f1", 
                                           insertbackground="white", font=("Consolas", 10), wrap=tk.NONE, padx=10, pady=10)
        self.config_h_preview.pack(fill=tk.BOTH, expand=True)
        
        # config.c panel
        self.config_c_frame = tk.Frame(self.config_paned, bg="#34495e")
        self.config_c_label = tk.Label(self.config_c_frame, text="config.c", bg="#34495e", fg="#ecf0f1", 
                                      font=("Segoe UI", 10, "bold"))
        self.config_c_label.pack()
        self.config_c_preview = ScrolledText(self.config_c_frame, bg="#1e272e", fg="#ecf0f1", 
                                           insertbackground="white", font=("Consolas", 10), wrap=tk.NONE, padx=10, pady=10)
        self.config_c_preview.pack(fill=tk.BOTH, expand=True)
        
        # Add both frames to paned window with equal weight
        self.config_paned.add(self.config_h_frame)
        self.config_paned.add(self.config_c_frame)
        self.config_paned.paneconfigure(self.config_h_frame, minsize=200)
        self.config_paned.paneconfigure(self.config_c_frame, minsize=200)

        # Bottom frame with status bar and copyright
        self.bottom_frame = tk.Frame(self.root, bg="#1a1a1a")
        self.bottom_frame.pack(side=tk.BOTTOM, fill=tk.X)

        self.status_bar = tk.Label(
            self.bottom_frame,
            text="Ready",
            bd=1,
            relief=tk.SUNKEN,
            anchor=tk.W,
            bg="#34495e",
            fg="#bdc3c7",
            font=("Segoe UI", 9)
        )
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        copyright_text = "© 2026 R&D Group - Technology and Innovation Center | Software Development Team"
        self.copyright_label = tk.Label(
            self.bottom_frame,
            text=copyright_text,
            font=("Segoe UI", 9, "italic"),
            bg="#1a1a1a",
            fg="#f1f1f1",
            anchor="center"
        )
        self.copyright_label.pack(side=tk.TOP, fill=tk.X, pady=2)

        self.mcu_combobox.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        self.browse_pdf_btn = tk.Button(
            self.settings_frame,
            text="📄 Browse PDF",
            command=self.browse_pdf_and_extract_text,
            bg="#16a085",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            bd=0,
            padx=10,
            pady=4,
            relief=tk.FLAT
        )
        self.browse_pdf_btn.grid(row=0, column=2, padx=5, pady=5)



    def show_advanced_settings(self):
        dialog = AdvancedSettingsDialog(self.root, "Advanced Settings", load_settings(use_defaults=True))

        self.root.wait_window(dialog)  # Wait for the dialog to close
        
        if dialog.result:  # Only update if Save was clicked
            self.settings = dialog.result
            save_settings(self.settings)  # Save to file
            messagebox.showinfo("Settings Saved", "Advanced settings have been updated.")

    def reset_ui(self):
        self.generate_btn.config(state=tk.NORMAL)
        self.progress_label.config(text="Ready")
        self.progress_bar["value"] = 0
        self.status_bar.config(text="Ready")

    def on_mcu_selected(self, event):
        selected = self.mcu_combobox.get()
        if selected == "Custom...":
            self.custom_mcu_frame.grid()
            self.custom_mcu_entry.focus()
        else:
            self.custom_mcu_frame.grid_remove()

    def reset_system(self):
        """Reset the entire system to initial state"""
        # Clear all text previews
        self.main_preview.delete(1.0, tk.END)
        self.config_h_preview.delete(1.0, tk.END)
        self.config_c_preview.delete(1.0, tk.END)
        
        # Reset MCU selection
        self.mcu_combobox.set('')
        if self.mcu_manager.mcu_list:
            self.mcu_combobox.current(0)
        self.custom_mcu_frame.grid_remove()
        self.custom_mcu_entry.delete(0, tk.END)
        
        # Clear generated file references
        self.last_saved_path = None
        self.last_generated_mcu = None
        
        # Reset UI state
        self.generate_btn.config(state=tk.NORMAL)
        self.generate_config_btn.config(state=tk.DISABLED, bg="#ADEBB3", fg="white")
        self.advanced_btn.config(state=tk.DISABLED, fg="#7f8c8d")
        self.cancel_btn.config(state=tk.DISABLED)
        
        # Reset progress
        self.progress_bar["value"] = 0
        self.progress_label.config(text="Ready")
        self.status_bar.config(text="System reset - Ready for new generation")
        
        # Show main tab
        self.notebook.select(self.main_tab)
        
        messagebox.showinfo("System Reset", "The system has been reset and is ready for a new generation.")

    def cancel_process(self):
        """Terminate any ongoing generation processes"""
        self.running = False
        if self.current_thread and self.current_thread.is_alive():
            # You can't actually kill a thread in Python, but we can set the flag
            pass
            
        self.status_bar.config(text="Process canceled by user")
        self.progress_label.config(text="Canceled")
        self.progress_bar["value"] = 0
        
        # Re-enable buttons
        self.generate_btn.config(state=tk.NORMAL)
        self.generate_config_btn.config(state=tk.NORMAL)
        self.advanced_btn.config(state=tk.NORMAL)
        self.cancel_btn.config(state=tk.DISABLED)
        
        messagebox.showinfo("Canceled", "The current operation was canceled")

    def on_generate_click(self):
        selected = self.mcu_combobox.get()
        mcu_name = self.custom_mcu_entry.get().strip() if selected == "Custom..." else selected
        
        if not mcu_name:
            messagebox.showerror("Input Error", "Please enter or select a microcontroller.")
            return
            
        if selected == "Custom..." and self.mcu_manager.add_mcu(mcu_name):
            self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")
            self.mcu_combobox.set(mcu_name)

        self.running = True
        self.generate_btn.config(state=tk.DISABLED)
        self.advanced_btn.config(state=tk.DISABLED)
        self.generate_config_btn.config(state=tk.DISABLED)
        self.cancel_btn.config(state=tk.NORMAL)
        self.main_preview.delete(1.0, tk.END)

        def callback(result):
            success, msg, code = result
            self.status_bar.config(text=msg)
            self.generate_btn.config(state=tk.NORMAL)
            self.cancel_btn.config(state=tk.DISABLED)
            if success:
                self.last_generated_mcu = mcu_name
                self.advanced_btn.config(state=tk.NORMAL, fg="white")
                self.generate_config_btn.config(state=tk.NORMAL, bg="#1520A6", fg="white")
                self.display_code(code, "main")
            else:
                messagebox.showerror("Error", msg)

        self.current_thread = threading.Thread(
            target=self.run_generation, 
            args=(mcu_name, callback),
            daemon=True
        )
        self.current_thread.start()

    def browse_pdf_and_extract_text(self):
        file_path = filedialog.askopenfilename(filetypes=[("PDF files", "*.pdf")])
        if file_path:
            try:
                doc = fitz.open(file_path)
                full_text = ""
                for page in doc:
                    full_text += page.get_text()
                doc.close()
                self.extracted_pdf_text = full_text.strip()
                messagebox.showinfo("PDF Loaded", "✅ PDF text extracted and stored successfully.")
                # Enable generate button since we now have PDF content
                self.generate_btn.config(state=tk.NORMAL)
            except Exception as e:
                messagebox.showerror("PDF Error", f"❌ Failed to extract text from PDF:\n{e}")


    def run_generation(self, mcu_name, callback):
        def progress(msg, val):
            self.progress_label.config(text=msg)
            self.progress_bar["value"] = val
            self.root.update_idletasks()

        try:
            # Check GitHub first
            progress("Checking GitHub...", 10)
            url = check_github_for_header(mcu_name)
            if url:
                code = download_header_from_github(url)
                if code:
                    default_filename = f"{mcu_name}_MAIN.h"
                    file_path = filedialog.asksaveasfilename(
                        defaultextension=".h",
                        initialfile=default_filename,
                        filetypes=[("Header Files", "*.h"), ("All Files", "*.*")]
                    )

                    if file_path:
                        with open(file_path, "w", encoding="utf-8") as f:
                            f.write(code)
                        self.last_saved_path = file_path
                        messagebox.showinfo("Downloaded", f"✅ File saved at:\n{file_path}")
                        progress("Done! File loaded from GitHub.", 100)
                        self.root.after(0, callback, (True, f"{mcu_name}_MAIN.h loaded from GitHub.", code))
                        return
                    else:
                        messagebox.showinfo("Canceled", "✅ File loaded but not saved.")
                        return

            # Generate with Gemini if not found on GitHub
            current_date = datetime.datetime.now().strftime("%Y-%m-%d")
            prompt = f"""You are an embedded C engineer YOU RNAME IS : AI. Given only the name of a microcontroller, generate a minimal and clean `main.h` file that includes only the most important and commonly used elements in embedded projects for that chip and also you MISRA C stander.

The header file should include:

1. A clean professional comment block at the top with:
   - File name
   - Short description
   - Author 
   - Device name
   - Creation date: {current_date}  <!-- This will use today's date -->
   - Standared: {"MISRA C"}
   - Copyright: {"ELARABY GROUP-TECHNOLOGY & INOVATION CENTER-EMBEDED SYSTEM GROUP"}

2. Only the **necessary standard and MCU-specific includes, donot Include intrinsic functions header**.

3. Include the following standard C headers exactly as shown below, in this order:
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <iso646.h>
#include <limits.h>
#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

4. Useful typedefs:
   - `tbyte` for `uint8_t`
   - `tword` for `uint16_t`
   - `tlong` for `uint32_t`

5. Core macros:
   - `SET_BIT(reg, bit)`
   - `CLR_BIT(reg, bit)`
   - `GET_BIT(reg, bit)`
   - `TOG_BIT(reg, bit)`
   - If available: macros like DI, EI or Global_Int_Disable, Global_Int_Enable (depending on the microcontroller's naming convention for global interrupt control), `HALT`, `NOP`, etc uut () after the name bec its function dont forget that.

6. SAFEGUARD MACROS — IMPORTANT:
   - You must **generate fully implemented versions** of the following two macros:
     - `GPIO_SAFEGUARD_Init()` → Should configure the actual GPIO (Data or value or logic)= 0 for all ports then Should configure the actual GPIO (Control or mode or Direction) = input for all ports, Disable all (pull high or pull up resistors) for all ports if exists, Disable all (woke up) registers for all ports if exists
     - `Registers_SAFEGUARD_Init()` → should Disable global intrupt register if exists, should Disable all Timers if exists,should Disable pulse width modulation (pwm) if exists, should Disable watchdog timer (wdt) if exists, should Disable input capture unit (ICU) if exists, should Disable Analog to digital converter (ADC) if exists, should Disable UART if exists, should Disable I2C if exists, should Disable SPI communication if exists, configure all GPIOS as an input/output pins (I/O)not special function registers like(ADC or UART)
   - DO NOT leave the implementation to the user.
   - Use **real registers and values** based on the given microcontroller. Avoid writing comments like "User: add implementation".
   - Format macros using `do {{ ... }} while(0)` style.

7.📘 Notes:

    - Add short comments to explain what each part does and if you didnot find or assume a value or register please right a comment beside it says "that you assum that please change it"

Microcontroller: **{mcu_name}**

Output: A clean, production-ready `main.h` file."""
            if self.extracted_pdf_text:
                prompt += f"""

---

### 📄 PDF CONTEXT:

Use the following microcontroller manual content to determine:
- GPIO registers
- Watchdog Timer configuration
- ADC, UART, Timers, etc.
- Bitfields and control logic

🛠️ **IMPORTANT INSTRUCTIONS:**
- If you use any value (register name, bit flag, timing value, etc.) from the PDF, add `/* PDF Reference */` beside it.
- If you make an assumption due to missing data, write a comment like: `/* Assumed value – please verify */`
- Keep these comments **on the same line** as the register or logic.
- Format everything clearly and professionally.

👇 Here’s the extracted PDF content for you to use:
{self.extracted_pdf_text}  # Truncate for size
"""

            
            progress("Generating header file...", 25)
            response = model.generate_content(prompt)
            raw_output = response.text

            progress("Processing response...", 50)
            clean_output = re.sub(r"<think>.*?</think>", "", raw_output, flags=re.DOTALL)
            code_blocks = re.findall(r"```(?:c|C)?\n(.*?)```", clean_output, flags=re.DOTALL)

            if code_blocks:
                final_code = code_blocks[0].strip()
                guard = f"{mcu_name.upper()}_MAIN_H_"
                final_code = re.sub(r'#ifndef\s+\w+\s*[\r\n]+#define\s+\w+', f'#ifndef {guard}\n#define {guard}', final_code, count=1, flags=re.IGNORECASE)
            else:
                lines = clean_output.splitlines()
                code_lines = [line for line in lines if line.strip()]
                final_code = "\n".join(code_lines).strip()

            progress("Saving file...", 75)
            default_filename = f"{mcu_name}_MAIN.h"
            file_path = filedialog.asksaveasfilename(defaultextension=".h", initialfile=default_filename, filetypes=[("Header Files", "*.h"), ("All Files", "*.*")])

            if not file_path:
                self.root.after(0, callback, (False, "File save canceled.", None))
                return

            with open(file_path, "w", encoding="utf-8") as file:
                file.write(final_code)
            self.last_saved_path = file_path

            # Attempt to upload to GitHub if token exists
            if GITHUB_TOKEN:
                progress("Uploading to GitHub...", 90)
                upload_success = upload_header_to_github(mcu_name, final_code)
                if upload_success:
                    progress("✅ Done! Uploaded to GitHub.", 100)
                    messagebox.showinfo("Upload Successful", f"✅ {mcu_name}_MAIN.h uploaded to GitHub.")
                else:
                    progress("⚠️ Done (upload failed)", 100)
                    messagebox.showwarning("Upload Failed", f"⚠️ Upload to GitHub failed for {mcu_name}_MAIN.h.")
            else:
                progress("✅ Done!", 100)

            self.root.after(0, callback, (True, f"'{os.path.basename(file_path)}' saved.", final_code))

        except Exception as e:
            self.root.after(0, callback, (False, f"Something went wrong:\n{str(e)}", None))

    def on_generate_config_click(self):
        if not self.last_generated_mcu:
            return
            
        self.running = True
        self.generate_config_btn.config(state=tk.DISABLED)
        self.cancel_btn.config(state=tk.NORMAL)
        self.config_h_preview.delete(1.0, tk.END)
        self.config_c_preview.delete(1.0, tk.END)
        
        def callback(result):
            success, msg, h_content, c_content = result
            self.status_bar.config(text=msg)
            self.generate_config_btn.config(state=tk.NORMAL if success else tk.DISABLED)
            self.cancel_btn.config(state=tk.DISABLED)
            if success:
                self.display_code(h_content, "config_h")
                self.display_code(c_content, "config_c")
            else:
                messagebox.showerror("Error", msg)

        self.current_thread = threading.Thread(
            target=self.run_config_generation,
            args=(self.last_generated_mcu, callback),
            daemon=True
        )
        self.current_thread.start()

    def run_config_generation(self, mcu_name, callback):
        def progress(msg, val):
            self.progress_label.config(text=msg)
            self.progress_bar["value"] = val
            self.root.update_idletasks()

        try:
            # Get settings
            current_date = datetime.datetime.now().strftime("%Y-%m-%d")
            voltage = self.settings['voltage']
            clock_source = self.settings['clock_source']
            register_mode = self.settings.get('register_mode', 'Registers').strip().lower()

            # Choose GitHub folder
            if register_mode == "registers":
                config_folder = "Configs_Registers"
            else:
                config_folder = "Configs_OptionBite"

            # Check GitHub
            progress("Checking GitHub for existing config files...", 10)
            h_url, c_url = check_github_for_config(mcu_name, config_folder)
            if h_url and c_url:
                progress("Downloading config files from GitHub...", 20)
                h_content = download_header_from_github(h_url)
                c_content = download_header_from_github(c_url)

                if h_content and c_content:
                    progress("Saving config files...", 80)
                    base_path = filedialog.askdirectory(title="Select folder to save config files")
                    if not base_path:
                        self.root.after(0, callback, (False, "Save canceled", None, None))
                        return

                    with open(os.path.join(base_path, f"{mcu_name}_config.h"), "w", encoding="utf-8") as f:
                        f.write(h_content)
                    with open(os.path.join(base_path, f"{mcu_name}_config.c"), "w", encoding="utf-8") as f:
                        f.write(c_content)

                    progress("Done! Files loaded from GitHub.", 100)
                    self.root.after(0, callback, (True, f"Config files loaded from GitHub and saved to {base_path}", h_content, c_content))
                    return

            # Generate config.h
            progress("Creating config.h...", 30)
            config_h_prompt = f"""Create a clean, production-ready **config.h** header file for the {mcu_name} microcontroller.  
    This file must define only what is needed for `config.c`, and reuse all typedefs and macros from **main.h**.

    ---

    ### 🔗 Linking to main.h:

    - Add `#include "{mcu_name}_MAIN.h"` at the top (assumes main.h was already generated)
    - DO NOT redefine `tbyte`, `tword`, `tlong`, or bit macros like `SET_BIT()` — they already exist in main.h
    - Reuse GPIO and register safety macros such as `GPIO_SAFEGUARD_Init()` and `Registers_SAFEGUARD_Init()`

    ---

    ### 🔧 Contents:

    1. Add standard include guard: `#ifndef CONFIG_H_`
    2. Wrap with `extern "C"` for C++ compatibility if needed
    3. Declare:
    - `void mcu_config_Init(void);`
    - `void WDI_Reset(void);`
    4. If needed:
    - Declare `extern` config state/status variables
    - Add `#define` for optional features (e.g., `ENABLE_WATCHDOG`)
    5. Define symbolic return/error codes (`CONFIG_OK`, `CONFIG_FAIL`)

    ---

    ### 📘 Notes:

    - Keep it minimal and clean — no redundant includes or macro definitions
    - Match naming with `config.c`
    - Add short comments to explain what each part does and if you didnot find or assume a value or register please right a comment beside it says "that you assum that please change it"

    Output must be real code, ready to use, no placeholders or TODOs."""
            response = model.generate_content(config_h_prompt)
            config_h_content = self.clean_code_output(response.text)

            # Create PDF note for config.c
            pdf_note = ""
            if self.extracted_pdf_text:
                pdf_note = f"""/*
    * Configuration generated with PDF reference:
    * {self.extracted_pdf_text[:100]}... (truncated)
    */
    """

            # Generate config.c
            progress("Creating config.c...", 60)
            if register_mode == "registers":
                config_c_prompt = f"""Create a clean and production-ready **config.c** file for the {mcu_name} microcontroller.  
    This file must implement the functions declared in `config.h`, using the macros and typedefs from `{mcu_name}_MAIN.h`.

    ---

    ### 🔗 File Linkage:

    - Add `#include "{mcu_name}_config.h"` and `#include "{mcu_name}_MAIN.h"` at the top
    - Do not redefine macros like `SET_BIT`, `GPIO_SAFEGUARD_Init()`, or typedefs like `tword` — just use them

    ---

    ### 🔧 Main Initialization Function:

    Implement `void mcu_config_Init(void)` using the following sequence:

    1. `safe_guards()` → should call `GPIO_SAFEGUARD_Init()` and `Registers_SAFEGUARD_Init()`
    2. `LVR_init()` → initialize Low Voltage Reset (LVR) threshold for operation at {voltage}V
    3. `LVR_Enable()` → enable LVR at correct voltage threshold
    4. `WDT_INIT()` → setup watchdog timer (≥ 8ms)
    5. `WDT_Enable()` → enable WDT with correct window config
    6. `CLC_Init()` → configure clock source using `{clock_source}`

    Implement `void WDI_Reset(void);` to refresh/reset the watchdog timer.

    ---

    ### 🔒 Visibility Rules:
    Only the following functions must be marked static:

    - safe_guards()
    - LVR_init()
    - LVR_Enable()
    - WDT_INIT()

    The following functions must not be static:

    - WDT_Enable()
    - CLC_Init()

    ### ⚠️ Safety Guidelines:

    - Use error codes (`return 0` for success, `<0` for failure)
    - Add timeout checks when polling hardware
    - No magic numbers — use `#define` or values from main.h
    - Short inline comments where needed (e.g. for register access or safety logic)

    ---

    ### 📘 Comments:

    - Add short function headers
    - Keep it clean and readable — no unnecessary comments or placeholders
    - Only comment what's important (e.g. timing-critical code, register logic) if you didnot find or assume a value or register please right a comment beside it says "that you assum that please change it"

    Output must be a complete, working C file — not pseudocode or partial implementation."""
                if self.extracted_pdf_text:
                    config_c_prompt += f"""

    ### PDF CONTEXT FOR FUNCTION IMPLEMENTATIONS:
    Use ONLY the following PDF content to implement the function bodies:
    {self.extracted_pdf_text}

    IMPORTANT:
    - Keep the exact function signatures and structure
    - Only modify the implementations inside {{}} blocks
    - Preserve all static/non-static modifiers
    - Only get the specific values (registers etc)
    - Add /* PDF Reference */ comments where used
    
    🛠️ **IMPORTANT INSTRUCTIONS:**
- If you use any value (register name, bit flag, timing value, etc.) from the PDF, add `/* PDF Reference */` beside it.
- If you make an assumption due to missing data, write a comment like: `/* Assumed value – please verify */`
- Keep these comments **on the same line** as the register or logic.
- Format everything clearly and professionally.
"""
            else:
                config_c_prompt = f"""Create a clean and production-ready **config.c** file for the {mcu_name} microcontroller in Option Bite mode.  
    This file must implement the functions declared in `config.h`, using the macros and typedefs from `{mcu_name}_MAIN.h`.

    ---

    ### 🔗 File Linkage:

    - Add `#include "{mcu_name}_config.h"` and `#include "{mcu_name}_MAIN.h"` at the top
    - Do not redefine macros like `SET_BIT`, `GPIO_SAFEGUARD_Init()`, or typedefs like `tword` — just use them

    ---

    ### 🔧 Main Initialization Function:

    Implement `void mcu_config_Init(void)` using the following sequence:

    1. `safe_guards()` → should call `GPIO_SAFEGUARD_Init()` and `Registers_SAFEGUARD_Init()`

    Implement `void WDI_Reset(void);` to refresh/reset the watchdog timer.

    ---

    ### 🔒 Visibility Rules:
    Only the following function must be marked static:

    - safe_guards()

    ### ⚠️ Safety Guidelines:

    - Use error codes (`return 0` for success, `<0` for failure)
    - Add timeout checks when polling hardware
    - No magic numbers — use `#define` or values from main.h
    - Short inline comments where needed (e.g. for register access or safety logic)

    ---

    ### 📘 Comments:

    - Add short function headers
    - Keep it clean and readable — no unnecessary comments or placeholders
    - Only comment what's important (e.g. timing-critical code, register logic) if you didnot find or assume a value or register please right a comment beside it says "that you assum that please change it"

    Output must be a complete, working C file — not pseudocode or partial implementation."""
                if self.extracted_pdf_text:
                    config_c_prompt += f"""

    ### PDF CONTEXT FOR FUNCTION IMPLEMENTATIONS:
    Use ONLY the following PDF content to implement the function bodies:
    {self.extracted_pdf_text}

    IMPORTANT:
    - Keep the exact function signatures and structure
    - Only modify the implementations inside {{}} blocks
    - Preserve all static/non-static modifiers
    - Only get the specific values (registers etc)
    - Add /* PDF Reference */ comments where used

    🛠️ **IMPORTANT INSTRUCTIONS:**
- If you use any value (register name, bit flag, timing value, etc.) from the PDF, add `/* PDF Reference */` beside it.
- If you make an assumption due to missing data, write a comment like: `/* Assumed value – please verify */`
- Keep these comments **on the same line** as the register or logic.
- Format everything clearly and professionally.
"""

            response = model.generate_content(config_c_prompt)
            config_c_content = pdf_note + self.clean_code_output(response.text)

            # Save files
            progress("Saving files...", 80)
            base_path = filedialog.askdirectory(title="Select folder to save config files")
            if not base_path:
                self.root.after(0, callback, (False, "Save canceled", None, None))
                return

            config_h_path = os.path.join(base_path, f"{mcu_name}_config.h")
            config_c_path = os.path.join(base_path, f"{mcu_name}_config.c")
            with open(config_h_path, "w", encoding="utf-8") as f:
                f.write(config_h_content)
            with open(config_c_path, "w", encoding="utf-8") as f:
                f.write(config_c_content)

            # Upload to GitHub
            if GITHUB_TOKEN:
                progress("Uploading to GitHub...", 90)
                upload_success = upload_config_to_github(mcu_name, config_h_content, config_c_content, config_folder)
                if upload_success:
                    progress("✅ Done! Uploaded to GitHub.", 100)
                    messagebox.showinfo("Upload Successful", f"✅ {mcu_name}_config.h/.c uploaded to GitHub.")
                else:
                    progress("⚠️ Done (upload failed)", 100)
                    messagebox.showwarning("Upload Failed", f"⚠️ Upload to GitHub failed for {mcu_name}_config.h/.c.")
            else:
                progress("✅ Done!", 100)

            self.root.after(0, callback, (True, f"Config files saved to {base_path}", config_h_content, config_c_content))

        except Exception as e:
            self.root.after(0, callback, (False, f"Config generation failed: {str(e)}", None, None))

    def clean_code_output(self, raw_output):
        clean_output = re.sub(r"<think>.*?</think>", "", raw_output, flags=re.DOTALL)
        code_blocks = re.findall(r"```(?:c|C)?\n(.*?)```", clean_output, flags=re.DOTALL)
        return code_blocks[0].strip() if code_blocks else clean_output.strip()

    def display_code(self, code, target):
        if target == "main":
            self.main_preview.delete(1.0, tk.END)
            self.main_preview.insert(tk.END, code)
            self.notebook.select(self.main_tab)
        elif target == "config_h":
            self.config_h_preview.delete(1.0, tk.END)
            self.config_h_preview.insert(tk.END, code)
            self.notebook.select(self.config_tab)
        elif target == "config_c":
            self.config_c_preview.delete(1.0, tk.END)
            self.config_c_preview.insert(tk.END, code)
            self.notebook.select(self.config_tab)

if __name__ == "__main__":
    root = tk.Tk()
    app = MainApp(root)
    root.mainloop()