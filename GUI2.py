import os
import json
import threading
import tkinter as tk
import traceback
import re
import fitz  # PyMuPDF
from tkinter import ttk, messagebox, filedialog
from tkinter.scrolledtext import ScrolledText
from datetime import datetime
import google.generativeai as genai
import base64
import requests
from requests.exceptions import RequestException
import sys
import copy

# Configuration paths
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
MCU_DATA_FILE = os.path.join(BASE_PATH, "mcu_data.json")
SETTINGS_FILE = os.path.join(BASE_PATH, "app_settings.json")

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

# GitHub API Configuration
GITHUB_API_URL = "https://api.github.com"
GITHUB_USERNAME = "ELARABY-AI-Team"
REPO_NAME = "Main_Generator"
REGISTERS_FOLDER = "Extracted_Registers"
MCAL_FOLDER = "MCAL_Files"
HALL_FOLDER = "HALL_Files"
TT_FOLDER="TT_Files"
RTOS_FOLDER="RTOS_Files"


# === Gemini API Config ===
genai.configure(api_key=os.getenv('GEMINI_API_KEY', 'AIzaSyCPfR-skCxo9GGlynnc0ODvMgDf3OX3Qhk'))
model = genai.GenerativeModel("gemini-2.5-flash")

# === deepseek API Config ===

def call_deepseek_api(prompt):
    url = "http://10.11.3.181:11434/api/generate"
    headers = {"Content-Type": "application/json"}

    data = {
        "model": "deepseek-r1:70b",  # or whatever your model is
        "prompt": prompt,
        "stream": False
    }

    try:
        response = requests.post(url, headers=headers, json=data)
        response.raise_for_status()

        result = response.json()
        return result.get("response", "").strip()

    except requests.exceptions.RequestException as e:
        raise Exception(f"Connection error: {e}")
    except Exception as e:
        raise Exception(f"Unexpected error: {e}")


# Register extraction prompt template
REGISTER_EXTRACTION_PROMPT = """
Extract all microcontroller registers from the documentation and format them as a structured hierarchy where:

Each **register name** is the parent key.  
Each register must include these child properties:

- `address`: (hex string, e.g., "0xA3")
- `description`: (brief summary of its purpose)
- `assigned_pin`: (an array of pin names like ["PA0", "PA1", "PB3"] if the register directly controls or is associated with specific GPIO pins, such as in GPIOx_MODER, GPIOx_AFRL, GPIOx_ODR, etc.)

🔍 Validation Requirements:

✅ For **register names and addresses**:  
- Use the information provided in the PDF to extract all registers.  
- If a peripheral is mentioned in the datasheet or block diagram but some registers are missing from the provided documents, use the official reference manual or datasheet to complete them.
- If the name or address are missing, incomplete, or unclear, **look them up using official {{mcu_name}} sources**, such as the reference manual, datasheets, or the manufacturer’s official website.  
- Ensure the address is in **hexadecimal format**, logically matches the register name, and is part of the valid memory map for {{mcu_name}}.
-Please make sure to extract all registers and Ensure you extract all control/status/data registers within the following peripheral modules:
    - GPIO
    - RCC
    - TIM
    - ADC
    - USART
    - I2C
    - SPI


✅ For **assigned_pin**:  
- If a register controls an entire I/O port (e.g., data direction or mode registers), assign all associated pins (like "P1.0" to "P1.7" or "RA0" to "RA7").  
- For peripheral-related registers (e.g., UART, ADC, Timer), assign only pins that are directly tied to the peripheral's function, based on {{mcu_name}}'s pin mapping or alternate function table.  
- If no pins are involved or no mapping is available, set `"assigned_pin": null`.

✅ For timer capture/compare registers (like TIMx_CCR1–CCR4):
- Look up the alternate function mappings for the specific timer and channel in the {{mcu_name}} reference manual or datasheet.
- Populate `"assigned_pin"` with all possible GPIO pins that map to that channel.
- If there is no channel-to-pin mapping, set to `null`.


✅ Do not guess or invent register names, addresses, or pins.  

✅ Add Function Tags

🧩 Normalize register names using STM32 standard format:
- Format should follow MODULEx_REGNAME, e.g., GPIOA_MODER, TIM2_CNT, USART1_SR
- Avoid aliases, lowercase, or shorthand like “GPIO_MOD” or “tim counter”
---

**Output Format Example:**

```json
{{
  "GPIOA_MODER": {{
    "address": "0x40020000",
    "description": "GPIO port mode register for Port A.",
    "assigned_pin": ["PA0", "PA1", "PA2", "PA3", "PA4", "PA5", "PA6", "PA7", "PA8", "PA9", "PA10", "PA11", "PA12", "PA13", "PA14", "PA15"],
    "function": "gpio_config"
  }},
  "TIM1_CNT": {{
    "address": "0x40010024",
    "description": "Counter register for TIM1.",
    "assigned_pin": null,
    "function": "timer_counter"
  }}
}}

- ⚠️ If a commonly used peripheral (GPIO, RCC, ADC, etc.) has missing or incomplete registers, retrieve them from the official reference manual
+ ⚠️ If a commonly used peripheral (GPIO, RCC, UART, I2C, SPI, ADC, etc.) has missing or incomplete registers, retrieve them from the official reference manual

"""



MCAL_GENERATION_PROMPT = """
You are an embedded systems expert. Generate two files — `MCAL.c` and `MCAL.h` — for a Microcontroller Abstraction Layer (MCAL) using only the inputs below.

--- MCU NAME ---
{{mcu_name}}

--- REGISTER JSON ---
MCU register definitions: address, description, function.
{{register_json}}

--- DOCUMENTS ---
1. API.json — allowed API function names per category.
2. Rules.json — coding rules and constraints.
{{documents}}

--- INSTRUCTIONS ---

✅ Generate:
- `MCAL.h`
- `MCAL.c`

✅ Use only defined inputs:
- API functions from `API.json`
- Registers from `register_json`
- Follow all rules from `Rules.json`

🚫 Do NOT:
- Invent registers, peripherals, or APIs
- Access peripherals directly (e.g., `USART1->CR1`)
- Implement APIs without required registers (write a comment instead)

✅ For each function:
- Map relevant registers from `register_json` by description or keyword.
- Example: `UART_Init` → UART registers; `MCU_Config_Init` → RCC, FLASH
- If WDT-related registers exist in `register_json`, implement using them.
+   - If missing, implement a generic watchdog refresh sequence based on {{mcu_name}}.
    - Must follow `API_implementation_sequence`.

✅ Rules.json:
- Insert `before_each_function` snippets in all functions.
- Add `global_includes` at the top of `.c` or `.h`.
- Follow other style/structural rules.

✅ Register normalization:
- Internally normalize register names by removing instance numbers and grouping by peripheral type 
  (GPIO, UART/USART, TIMER, SPI, I2C, ADC, CAN, etc.).
- Always keep the original register names in the generated code.


✅ Output format:
```c filename=MCAL.h
// content here

```c filename=MCAL.c
// content here

Check that both files are generated per these requirements.
"""









MCAL2_GENERATION_PROMPT = """
You are an embedded systems expert. Your task is to generate two C files — `MCAL.c` and `MCAL.h` — for a Microcontroller Abstraction Layer (MCAL) based on the following inputs.

Only use what is given in the inputs below. Do not invent API functions, registers, or rules.

--- MCU NAME ---
{{mcu_name}}

--- REGISTER JSON ---
This contains the MCU register definitions, including address, description, and function.

{{register_json}}

--- DOCUMENTS ---
This contains two JSONs:
1. API.json — describes available API categories and their allowed function names.
2. Rules.json — contains coding rules and constraints to be applied globally or per function.

{{documents}}

--- INSTRUCTIONS ---

✅ Generate two code files:
- `MCAL.h`
- `MCAL.c`

✅ USE ONLY what is defined in the inputs:
- Use only API function names from `API.json`
- Use only register names and addresses from `register_json`
- Apply all structural and coding rules from `Rules.json`

🚫 DO NOT:
- Invent registers or peripherals that aren't in `register_json`
- Use direct peripheral access like `USART1->CR1` or `PWR->CR`
- Write code for any API if required registers are not defined (write a comment instead)

✅ For each function:
- Map relevant registers from register_json based on the function's purpose and the register description.
- Example: `UART_Init` → use UART-related registers like `USARTx_CR1`, `USARTx_BRR`, etc., based on the descriptions.
- You may map by keyword (e.g., `MCU_Config_Init` → RCC, FLASH).
-implement `WDT_Reset()` function body must be real code, not a comment, as per `API_implementation_sequence`.


✅ Apply rules from Rules.json:
- Insert any code snippets from `before_each_function` at the start of every function.
- Add any `global_includes` at the top of the `.c` or `.h` files as appropriate.
-`WDT_Reset()` must be real code, not a comment, as per `API_implementation_sequence`.
- Follow other structural or style rules if present.

✅ REGISTER NORMALIZATION:
- Normalize register names to general peripheral categories using both string patterns and, if needed, external online reference (e.g., STM32, NXP datasheets, or MCU vendor docs).
- Examples:
    - `GPIOA_MODER`, `GPIOB_MODER` → `GPIO_MODER`
    - `USART1_CR1`, `USART2_CR1` → `UART_CR1`
    - `TIM2_CR1`, `TIM3_CR1` → `TIMER_CR1`
    - `ADC1_DR`, `ADC2_DR` → `ADC_DR`
- Always retain original register names in generated code. Use normalized names only for internal mapping.


✅ Format your output in two separate code blocks with filenames like:

```c filename=MCAL.h
// content here

```c filename=MCAL.c
// content here
please check if both files are generated based on the specific requirment written.

"""





TT_GENERATION_PROMPT = """
You are an embedded systems expert. Generate two files — `TT.c` and `TT.h` — for a Timing & Task (TT) module using only the inputs below.

--- MCU NAME ---
{{mcu_name}}

--- REGISTER JSON ---
MCU register definitions: address, description, function.
{{register_json}}

--- DOCUMENTS ---
1. API.json — allowed API function names per category.
2. Rules.json — coding rules and constraints.
{{documents}}

--- INSTRUCTIONS ---

✅ Generate:
- `TT.h`
- `TT.c`

✅ Use only defined inputs:
- API functions from `API.json`
- Registers from `register_json`
- Follow all rules from `Rules.json`

🚫 Do NOT:
- Invent registers, peripherals, or APIs
- Access peripherals directly (e.g., `TIM1->CR1`)
- Implement APIs without required registers (write a comment instead)

✅ For each function:
- Map relevant registers from `register_json` by description or keyword.
- Example: `TT_Scheduler_Init` → TIMER registers; `TT_Task_Add` → internal task table structures
- If WDT-related registers exist in `register_json`, implement using them.
+   - If missing, implement a generic watchdog refresh sequence based on {{mcu_name}}.
    - Must follow `API_implementation_sequence`.

✅ Rules.json:
- Insert `before_each_function` snippets in all functions.
- Add `global_includes` at the top of `.c` or `.h`.
- Follow other style/structural rules.

✅ Register normalization:
- Internally normalize register names by removing instance numbers and grouping by peripheral type 
  (GPIO, UART/USART, TIMER, SPI, I2C, ADC, CAN, etc.).
- Always keep the original register names in the generated code.

✅ Output format:
```c filename=TT.h
// content here

```c filename=TT.c
// content here

Check that both files are generated per these requirements.
"""


# Add this prompt template near the other prompts (around line 200)
HALL_GENERATION_PROMPT = """
You are an embedded systems expert. Your task is to generate two C files — `HALL.c` and `HALL.h` — for Hall Effect sensor drivers based on the following inputs.

Only use what is given in the inputs below. Do not invent API functions, registers, or rules.

--- MCU NAME ---
{{mcu_name}}

--- REGISTER JSON ---
This contains the MCU register definitions, including address, description, and function.

{{register_json}}

--- DOCUMENTS ---
This contains two JSONs:
1. API.json — describes available API categories and their allowed function names.
2. Rules.json — contains coding rules and constraints to be applied globally or per function.

{{hall_documents}}

--- INSTRUCTIONS ---

✅ Generate two code files:
- `HALL.h` - Header file with declarations and macros
- `HALL.c` - Implementation file with driver functions

✅ Focus on:
1. GPIO configuration for Hall sensor inputs
2. Timer/counter configuration for speed measurement
3. Interrupt handling for edge detection
4. Filtering/debouncing logic
5. Speed/direction calculation functions

✅ USE ONLY what is defined in the inputs:
- Use only API function names from `API.json`
- Use only register names and addresses from `register_json`
- Apply all structural and coding rules from `Rules.json`

✅ Output format:
```c filename=HALL.h
// content here

```c filename=HALL.c
// content here
"""


class GitHubManager:
    def __init__(self, token):
        self.token = token
        self.headers = {
            "Authorization": f"token {token}",
            "Accept": "application/vnd.github.v3+json"
        }
    
    def check_file_exists(self, folder, filename):
        """Check if a file exists in a specific GitHub folder"""
        try:
            url = f"{GITHUB_API_URL}/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{folder}/{filename}"
            response = requests.get(url, headers=self.headers)
            if response.status_code == 200:
                return True
            return False
        except RequestException as e:
            print(f"Error checking file existence: {e}")
            return False
    
    def download_file(self, folder, filename):
        """Download a file from GitHub"""
        try:
            url = f"{GITHUB_API_URL}/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{folder}/{filename}"
            response = requests.get(url, headers=self.headers)
            if response.status_code == 200:
                content = base64.b64decode(response.json()['content']).decode('utf-8')
                return content
            return None
        except RequestException as e:
            print(f"Error downloading file: {e}")
            return None
    
    def upload_file(self, folder, filename, content, message="Upload file"):
        """Upload a file to GitHub"""
        try:
            url = f"{GITHUB_API_URL}/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{folder}/{filename}"
            
            # Check if file exists to get SHA for update
            sha = None
            check_response = requests.get(url, headers=self.headers)
            if check_response.status_code == 200:
                sha = check_response.json().get('sha')
            
            data = {
                "message": message,
                "content": base64.b64encode(content.encode('utf-8')).decode('utf-8'),
                "branch": "main"
            }
            
            if sha:
                data["sha"] = sha
            
            response = requests.put(url, headers=self.headers, json=data)
            return response.status_code == 200 or response.status_code == 201
        except RequestException as e:
            print(f"Error uploading file: {e}")
            return False

class MCUManager:
    def __init__(self):
        self.mcu_list = self.load_mcu_list()
        self.github = GitHubManager(GITHUB_TOKEN) if GITHUB_TOKEN else None
        
    def load_mcu_list(self):
        default_list = ["RENESAS_R5F11BBC"]
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

    def save_selected_mcu(self, mcu_name):
        """Save the selected MCU to config file"""
        config_file = os.path.join(BASE_PATH, "mcu_config.json")
        try:
            with open(config_file, 'w') as f:
                json.dump({'mcu_name': mcu_name}, f, indent=4)
            return True
        except Exception as e:
            print(f"Error saving selected MCU: {e}")
            return False

def clean_extracted_registers(raw_output):
    # First try to extract JSON part if it exists
    json_match = re.search(r'```json\n(.*?)\n```', raw_output, re.DOTALL)
    if json_match:
        return json_match.group(1).strip()
    
    # Fallback cleaning for non-JSON output
    raw_output = re.sub(r"<think>.*?</think>", "", raw_output, flags=re.DOTALL)
    raw_output = re.sub(r"```.*?```", "", raw_output, flags=re.DOTALL)
    
    banned_phrases = [
        "here are the registers", "based on the documentation", 
        "I found the following", "the documentation shows",
        "this is a summary", "please note that", "typically",
        "would be", "usually", "example", "important", "note that"
    ]
    
    lines = raw_output.splitlines()
    clean_lines = []
    for line in lines:
        if not any(phrase.lower() in line.lower() for phrase in banned_phrases):
            clean_lines.append(line)
    
    return "\n".join(clean_lines).strip()

class MCUGeneratorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Ragura Software")
        self.root.geometry("800x700")
        self.root.configure(bg="#2c3e50")
        self.root.resizable(True, True)
        
        self.mcu_manager = MCUManager()
        self.settings = self.load_settings(use_defaults=True)
        self.save_settings()
        

        self.extracted_pdf_text = ""
        self.extracted_files = []  # Store multiple files separately
        self.extracted_hall_files = [] 

        self.running = False
        self.current_thread = None
        self.last_generated_mcu = None
        
        self.setup_ui()
    
    def load_settings(self, use_defaults=False):
        if use_defaults:
            return {
                'voltage': '3.3',
                'clock_source': 'Internal',
                'register_mode': 'Registers',
                'scheduler_type': 'Time-Trigger',
            }

        try:
            if os.path.exists(SETTINGS_FILE):
                with open(SETTINGS_FILE, 'r') as f:
                    settings = json.load(f)
                    return settings
        except Exception as e:
            print(f"Error loading settings: {e}")

        return {
            'voltage': '3.3',
            'clock_source': 'Internal',
            'register_mode': 'Registers',
            'scheduler_type': 'Time-Trigger',
        }

    def save_settings(self):
        try:
            with open(SETTINGS_FILE, 'w') as f:
                json.dump(self.settings, f, indent=4)
        except Exception as e:
            print(f"Error saving settings: {e}")
            
    def setup_ui(self):
        self.root.state('zoomed')
        self.root.resizable(True, True)
        # Main container
        self.main_frame = tk.Frame(self.root, bg="#2c3e50")
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Header frame
        self.header_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.header_frame.pack(fill=tk.X, pady=(0, 10))

        # Logo and title
        self.logo_label = tk.Label(self.header_frame, text="⚙️", font=("Segoe UI", 24), bg="#2c3e50", fg="#3498db")
        self.logo_label.pack(side=tk.LEFT, padx=(0, 10))

        self.title_label = tk.Label(self.header_frame, text="Ragura Software", font=("Segoe UI", 18, "bold"), bg="#2c3e50", fg="#ecf0f1")
        self.title_label.pack(side=tk.LEFT)

        # Settings frame
        self.settings_frame = tk.LabelFrame(self.main_frame, text=" Settings ", font=("Segoe UI", 10, "bold"), 
                                      bg="#34495e", fg="#ecf0f1", relief=tk.GROOVE, bd=2)
        self.settings_frame.pack(fill=tk.X, pady=(0, 10))
        
        # MCU selection row (grid row 0)
        self.mcu_row = tk.Frame(self.settings_frame, bg="#34495e")
        self.mcu_row.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        
        self.mcu_label = tk.Label(self.mcu_row, text="Select Microcontroller:", bg="#34495e", 
                                fg="#ecf0f1", font=("Segoe UI", 10))
        self.mcu_label.pack(side=tk.LEFT)

        self.mcu_combobox = ttk.Combobox(self.mcu_row, values=self.mcu_manager.mcu_list, 
                                    font=("Segoe UI", 10), width=30)
        self.mcu_combobox.pack(side=tk.LEFT, padx=5)
        if self.mcu_manager.mcu_list:
            self.mcu_combobox.current(0)
        self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")


        self.browse_pdf_btn = tk.Button(
            self.mcu_row,
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
        self.browse_pdf_btn.pack(side=tk.LEFT, padx=5)

        # Action buttons frame (grid row 0, column 1)
        self.action_buttons_frame = tk.Frame(self.settings_frame, bg="#34495e")
        self.action_buttons_frame.grid(row=0, column=1, sticky="e", padx=5, pady=5)
        
        # Advanced Settings button
        self.advanced_btn = tk.Button(
            self.action_buttons_frame, 
            text="🎛️ Advanced Settings", 
            command=self.show_advanced_settings, 
            bg="#9b59b6", 
            fg="white",
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT
        )
        self.advanced_btn.pack(side=tk.LEFT, padx=2)


        # Cancel button
        self.cancel_btn = tk.Button(
            self.action_buttons_frame,
            text="⛔ Cancel",
            command=self.cancel_process,
            bg="#e74c3c",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=15,
            pady=5,
            relief=tk.FLAT,
            state=tk.DISABLED
        )
        self.cancel_btn.pack(side=tk.LEFT, padx=2)

        # Reset button
        self.reset_btn = tk.Button(
            self.action_buttons_frame,
            text="🔄 Reset",
            command=self.reset_system,
            bg="#f39c12",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=15,
            pady=5,
            relief=tk.FLAT
        )
        self.reset_btn.pack(side=tk.LEFT, padx=2)

        # Custom MCU entry (grid row 1)
        self.custom_mcu_frame = tk.Frame(self.settings_frame, bg="#34495e")
        self.custom_mcu_label = tk.Label(self.custom_mcu_frame, text="Enter Custom MCU:", 
                                    bg="#34495e", fg="#ecf0f1", font=("Segoe UI", 10))
        self.custom_mcu_label.pack(side=tk.LEFT, padx=(0, 5))
        self.custom_mcu_entry = tk.Entry(self.custom_mcu_frame, bg="#2c3e50", fg="#ecf0f1", 
                                    insertbackground="white", font=("Segoe UI", 10), width=30)
        self.custom_mcu_entry.pack(side=tk.LEFT)
        
        # Configure grid weights
        self.settings_frame.grid_columnconfigure(0, weight=1)
        self.settings_frame.grid_columnconfigure(1, weight=0)
        
        # Bind combobox selection event
        self.mcu_combobox.bind("<<ComboboxSelected>>", self.on_mcu_selected)

        # Content area
        self.content_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.content_frame.pack(fill=tk.BOTH, expand=True)

        # Left panel for buttons
        self.left_panel = tk.Frame(self.content_frame, bg="#2c3e50", width=200)
        self.left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        # Extract Registers button
        self.extract_registers_btn = tk.Button(
            self.left_panel, 
            text="✨ Extract Registers", 
            command=self.on_generate_click, 
            bg="#3498db", 
            fg="white", 
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT,
            width=20
        )
        self.extract_registers_btn.pack(fill=tk.X, pady=5)

        self.generate_mcal_btn = tk.Button(
            self.left_panel, 
            text="⚡ Generate MCAL", 
            command=self.on_generate_mcal_click, 
            bg="#e67e22", 
            fg="white", 
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT,
            width=20
        )
        self.generate_mcal_btn.pack(fill=tk.X, pady=5)


        self.generate_TT_btn = tk.Button(
            self.left_panel, 
            text="⚡ Generate TT", 
            command=self.on_generate_tt_click, 
            bg="#9b59b6", 
            fg="white", 
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT,
            width=20
        ) 
        self.generate_TT_btn.pack(fill=tk.X, pady=5)




        self.generate_hall_btn = tk.Button(
            self.left_panel, 
            text="⚡ Generate HALL", 
            command=self.on_generate_hall_click, 
            bg="#9b59b6", 
            fg="white", 
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT,
            width=20
        )
        self.generate_hall_btn.pack(fill=tk.X, pady=5)

        # Notebook for code preview
        self.notebook = ttk.Notebook(self.content_frame)
        self.notebook.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Create tab for registers
        self.create_tab("extracted_registers")

        # Progress bar
        self.progress_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.progress_frame.pack(fill=tk.X, pady=(10, 0))
        
        self.progress_bar = ttk.Progressbar(
            self.progress_frame, 
            orient=tk.HORIZONTAL, 
            length=120, 
            mode='determinate'
        )
        self.progress_bar.pack(side=tk.RIGHT, fill=tk.X, padx=5)
        
        self.progress_label = tk.Label(
            self.progress_frame,
            text="Ready",
            bg="#2c3e50",
            fg="#bdc3c7",
            font=("Segoe UI", 9)
        )
        self.progress_label.pack(side=tk.RIGHT, padx=5)

        # Status bar
        self.status_bar = tk.Label(
            self.main_frame,
            text="Ready",
            bd=1,
            relief=tk.SUNKEN,
            anchor=tk.W,
            bg="#34495e",
            fg="#bdc3c7",
            font=("Segoe UI", 9)
        )
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        # Copyright frame
        self.copyright_frame = tk.Frame(self.main_frame, bg="#1a1a1a")
        self.copyright_frame.pack(side=tk.BOTTOM, fill=tk.X)

        copyright_text = "© 2025 ELARABY GROUP - Technology & Innovation Center"
        self.copyright_label = tk.Label(
            self.copyright_frame,
            text=copyright_text,
            font=("Segoe UI", 9, "italic"),
            bg="#1a1a1a",
            fg="#f1f1f1",
            anchor="center"
        )
        self.copyright_label.pack(fill=tk.X, pady=2)

        self.browse_multiple_category_files()

    def create_tab(self, name):
        tab = tk.Frame(self.notebook, bg="#34495e")
        self.notebook.add(tab, text=name)
        
        preview = ScrolledText(
            tab,
            bg="#1e272e",
            fg="#ecf0f1",
            insertbackground="white",
            font=("Consolas", 10),
            wrap=tk.NONE,
            padx=10,
            pady=10
        )
        preview.pack(fill=tk.BOTH, expand=True)
        
        # Store reference to the preview widget
        setattr(self, f"{name.replace('.', '_')}_preview", preview)

    def on_mcu_selected(self, event):
        selected = self.mcu_combobox.get()
        if selected == "Custom...":
            # Place the custom MCU frame below the selection row
            self.custom_mcu_frame.grid(row=1, column=0, columnspan=2, sticky="w", padx=5, pady=(0, 5))
            self.custom_mcu_entry.focus()
        else:
            self.custom_mcu_frame.grid_remove()
            # Save the selected MCU
            self.mcu_manager.save_selected_mcu(selected)

    def show_advanced_settings(self):
        dialog = AdvancedSettingsDialog(self.root, "Advanced Settings", self.settings)
        self.root.wait_window(dialog)
        
        if dialog.result:
            self.settings = dialog.result
            self.save_settings()
            messagebox.showinfo("Settings Saved", "Advanced settings have been updated.")

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
                self.extract_registers_btn.config(state=tk.NORMAL)
            except Exception as e:
                messagebox.showerror("PDF Error", f"❌ Failed to extract text from PDF:\n{e}")


    def resource_path(self, relative_path):
        """ Get absolute path to resource for dev and PyInstaller exe """
        try:
            base_path = sys._MEIPASS  # folder created by PyInstaller when running exe
        except AttributeError:
            base_path = os.path.abspath(".")  # dev mode
        return os.path.join(base_path, relative_path)

    def browse_multiple_category_files(self):
        """
        Browse and load multiple files for all categories.
        Supports: JSON, .c, .h files
        Categories: 'mcal', 'hall', 'Time-Trigger'
        """

        print("📁 browse_multiple_category_files() called (all categories)")

        # Define file mappings by category
        category_files = {
            "mcal": ["API.json", "Rules.json"],
            "hall": ["hall_API.json", "hall_Rules.json"],
            "Time-Trigger": ["tt_scheduler.c", "tt_scheduler.h"],
            "RTOS": ["rtos_scheduler.c", "rtos_scheduler.h"],

        }

        for category, target_filenames in category_files.items():
            print(f"\n🔎 Loading category: {category}")

            # Create dynamic attribute for extracted files per category
            extracted_attr = f"extracted_{category}_files"
            setattr(self, extracted_attr, [])

            success_count = 0
            failed_files = []
            missing_files = []

            for filename in target_filenames:
                file_path = self.resource_path(filename)

                if not os.path.exists(file_path):
                    print(f"❌ Missing: {filename}")
                    missing_files.append(filename)
                    continue

                try:
                    file_ext = os.path.splitext(filename)[1].lower()

                    if file_ext == ".json":
                        with open(file_path, 'r', encoding='utf-8') as f:
                            json_obj = json.load(f)
                        content = json.dumps(json_obj, indent=2)
                        file_type = "JSON"

                    elif file_ext in [".c", ".h"]:
                        with open(file_path, 'r', encoding='utf-8') as f:
                            content = f.read()
                        file_type = "C/H Source"

                    else:
                        raise ValueError(f"Unsupported file type: {file_ext}")

                    file_info = {
                        'path': file_path,
                        'name': filename,
                        'type': file_type,
                        'content': content
                    }

                    getattr(self, extracted_attr).append(file_info)
                    print(f"✅ Loaded: {filename}")
                    success_count += 1

                except Exception as e:
                    failed_files.append(f"{filename} (error: {str(e)})")
                    print(f"❌ Error loading {filename}: {e}")

            # Build upload summary for this category
            upload_message = f"📁 Upload Summary for {category.upper()}:\n\n"
            upload_message += f"✅ Successfully loaded: {success_count} file(s)\n"
            if missing_files:
                upload_message += f"⚠️ Missing files: {', '.join(missing_files)}\n"
            if failed_files:
                upload_message += f"❌ Failed to load:\n" + "\n".join(failed_files) + "\n"

            upload_message += "\n📋 File Details:\n"
            for file in getattr(self, extracted_attr):
                upload_message += f"\n• {file['name']} ({file['type']}) - {len(file['content'])} chars"

            print("🧾 Final Report:")
            print(upload_message)

            # Show upload summary
            self.root.after(1000, lambda msg=upload_message: self.show_upload_report(msg))

            # Enable button if files loaded
            if getattr(self, extracted_attr):
                self.extract_registers_btn.config(state=tk.NORMAL)






    def show_upload_report(self, message):
        """Show detailed upload report in a scrollable dialog"""
        report_window = tk.Toplevel(self.root)
        report_window.title("Upload Report")
        report_window.geometry("600x400")
        report_window.resizable(True, True)
        
        # Center the window
        window_width = 600
        window_height = 400
        screen_width = report_window.winfo_screenwidth()
        screen_height = report_window.winfo_screenheight()
        position_x = int((screen_width/2) - (window_width/2))
        position_y = int((screen_height/2) - (window_height/2))
        report_window.geometry(f"{window_width}x{window_height}+{position_x}+{position_y}")
        
        # Create text widget with scrollbar
        text_frame = tk.Frame(report_window)
        text_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        text_area = ScrolledText(
            text_frame,
            wrap=tk.WORD,
            font=("Consolas", 10),
            bg="#f0f0f0",
            padx=10,
            pady=10
        )
        text_area.pack(fill=tk.BOTH, expand=True)
        
        # Insert message and configure tags for styling
        text_area.insert(tk.END, message)
        
        # Apply formatting
        text_area.tag_configure("success", foreground="green")
        text_area.tag_configure("error", foreground="red")
        text_area.tag_configure("header", font=("Segoe UI", 11, "bold"))
        
        # Apply tags to specific parts
        text_area.tag_add("header", "1.0", "1.18")
        
        # Add close button
        button_frame = tk.Frame(report_window)
        button_frame.pack(fill=tk.X, pady=(0, 10))
        
        close_btn = tk.Button(
            button_frame,
            text="Close",
            command=report_window.destroy,
            bg="#3498db",
            fg="white",
            font=("Segoe UI", 10),
            padx=20
        )
        close_btn.pack()
        
        # Disable text widget
        text_area.config(state=tk.DISABLED)



    def cancel_process(self):
        self.running = False
        if self.current_thread and self.current_thread.is_alive():
            pass  # Can't kill threads in Python, just set the flag
            
        self.status_bar.config(text="Process canceled by user")
        self.progress_label.config(text="Canceled")
        self.progress_bar["value"] = 0
        
        # Re-enable buttons
        self.extract_registers_btn.config(state=tk.NORMAL)
        self.advanced_btn.config(state=tk.NORMAL)
        self.cancel_btn.config(state=tk.DISABLED)
        
        messagebox.showinfo("Canceled", "The current operation was canceled")

    def reset_system(self):
        """Reset the entire system to initial state"""
        # Clear text preview
        self.extracted_registers_preview.delete(1.0, tk.END)
        
        # Reset MCU selection
        self.mcu_combobox.set('')
        if self.mcu_manager.mcu_list:
            self.mcu_combobox.current(0)
        self.custom_mcu_frame.grid_remove()
        self.custom_mcu_entry.delete(0, tk.END)
        self.mcu_manager.save_selected_mcu("")
        self.last_generated_mcu = None

        # Clear generated file references
        self.last_generated_mcu = None
        self.extracted_pdf_text = ""
        
        # Reset UI state
        self.extract_registers_btn.config(state=tk.NORMAL)
        self.generate_mcal_btn.config(state=tk.NORMAL)
        self.generate_hall_btn.config(state=tk.NORMAL)
        self.generate_TT_btn.config(state=tk.NORMAL)
        self.advanced_btn.config(state=tk.NORMAL)
        self.cancel_btn.config(state=tk.DISABLED)

        # Reset advanced settings to default
        self.settings = self.load_settings(use_defaults=True)
        self.save_settings()
        messagebox.showinfo(
            "Settings Reset", 
            "Advanced settings have been reset to default:\n• Voltage: 3.3V\n• Clock: Internal\n• Mode: Registers\n• Scheduler: Time-Trigger"
        )

        # Reset progress
        self.progress_bar["value"] = 0
        self.progress_label.config(text="Ready")
        self.status_bar.config(text="System reset - Ready for new generation")

        # Show main tab
        self.notebook.select(0)
        
        messagebox.showinfo("System Reset", "The system has been reset and is ready for a new generation.")

    def update_progress(self, msg, val):
        self.progress_label.config(text=msg)
        self.progress_bar["value"] = val
        self.root.update_idletasks()

    def on_generate_click(self):
        selected = self.mcu_combobox.get()
        
        if selected == "Custom...":
            mcu_name = self.custom_mcu_entry.get().strip()
            if not mcu_name:
                messagebox.showerror("Input Error", "Please enter a custom microcontroller name.")
                return
            if not self.mcu_manager.add_mcu(mcu_name):
                messagebox.showerror("Error", "Failed to add custom MCU.")
                return
            # Update combobox values and set new MCU name
            self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")
            self.mcu_combobox.set(mcu_name)  # important!
            self.custom_mcu_frame.grid_remove()
        else:
            mcu_name = selected.strip()

        # 🔐 PROTECTION: if mcu_name still ends up blank
        if not mcu_name or mcu_name == "Custom...":
            messagebox.showerror("Error", "MCU name is invalid.")
            return

        # ✅ Save and store the selected mcu_name
        self.mcu_manager.save_selected_mcu(mcu_name)
        self.last_generated_mcu = mcu_name

        # Disable buttons
        self.running = True
        self.extract_registers_btn.config(state=tk.DISABLED)
        self.advanced_btn.config(state=tk.DISABLED)
        self.cancel_btn.config(state=tk.NORMAL)
        self.extracted_registers_preview.delete(1.0, tk.END)

        def callback(result):
            success, msg, registers = result
            self.status_bar.config(text=msg)
            self.extract_registers_btn.config(state=tk.NORMAL)
            self.cancel_btn.config(state=tk.DISABLED)
            if success:
                self.advanced_btn.config(state=tk.NORMAL)
                self.display_code(registers, "extracted_registers")
            else:
                messagebox.showerror("Error", msg)

        # 🧠 START generation using correct mcu_name
        self.current_thread = threading.Thread(
            target=self.run_register_extraction,
            args=(mcu_name, callback),
            daemon=True
        )
        self.current_thread.start()

    def run_register_extraction(self, mcu_name, callback):
        try:
            # Check GitHub first if we have a token
            github_file = f"{mcu_name}_REGISTERS.json"
            if self.mcu_manager.github and self.mcu_manager.github.check_file_exists(REGISTERS_FOLDER, github_file):
                self.update_progress("Checking GitHub...", 10)
                content = self.mcu_manager.github.download_file(REGISTERS_FOLDER, github_file)
                if content:
                    # First display in GUI
                    self.display_code(content, "extracted_registers")
                    
                    # Then ask for save location
                    file_path = filedialog.asksaveasfilename(
                        initialfile=github_file,
                        defaultextension=".json",
                        filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                        title="Save Register File As"
                    )
                    if not file_path:
                        self.update_progress("Save cancelled", 0)
                        self.root.after(0, callback, (False, "File save cancelled", None))
                        return
                    
                    try:
                        with open(file_path, "w", encoding="utf-8") as f:
                            f.write(content)
                        self.update_progress("Downloaded from GitHub!", 100)
                        self.root.after(0, callback, (True, f"Saved to: {os.path.basename(file_path)}", content))
                    except Exception as e:
                        self.update_progress("Save failed", 0)
                        self.root.after(0, callback, (False, f"Failed to save file: {str(e)}", None))
                    return

            # Prepare the base prompt
            prompt_template = REGISTER_EXTRACTION_PROMPT.replace("{mcu_name}", mcu_name)
            if mcu_name:
                prompt_template = f"For {mcu_name} microcontroller:\n\n" + prompt_template

            final_raw_output = ""
            if not self.extracted_pdf_text:
                # ❗ No PDF uploaded — run prompt without PDF chunks
                self.update_progress("No PDF provided, running with prompt only...", 30)
                try:
                    response = model.generate_content(prompt_template)
                    final_raw_output = response.text
                except Exception as e:
                    self.update_progress("Gemini error", 0)
                    self.root.after(0, callback, (False, f"Failed to generate: {str(e)}", None))
                    return
            else:
                self.update_progress("Splitting PDF for analysis...", 20)
                
                # Improved chunking logic
                def split_text(text, max_tokens_per_chunk=10000, max_total_tokens=250000):
                    max_chars_per_chunk = max_tokens_per_chunk * 4
                    max_total_chars = max_total_tokens * 4

                    paragraphs = text.split('\n')
                    chunks = []
                    current_chunk = ""
                    current_total_chars = 0

                    for para in paragraphs:
                        if not para.strip():
                            continue  # skip empty lines

                        if len(current_chunk) + len(para) > max_chars_per_chunk:
                            if current_chunk:
                                if current_total_chars + len(current_chunk) > max_total_chars:
                                    break
                                chunks.append(current_chunk)
                                current_total_chars += len(current_chunk)
                                current_chunk = ""

                        current_chunk += para + '\n'

                    if current_chunk.strip() and current_total_chars + len(current_chunk) <= max_total_chars:
                        chunks.append(current_chunk)

                    return chunks

                chunks = split_text(self.extracted_pdf_text)
                total_chunks = len(chunks)
                
                if not chunks:
                    raise ValueError("Failed to split PDF text into usable chunks")
                
                self.update_progress(f"Processing {total_chunks} chunks...", 30)
                
                for i, chunk in enumerate(chunks):
                    if not self.running:
                        return
                    
                    chunk_progress = 30 + int((i / total_chunks) * 60)
                    self.update_progress(f"Processing chunk {i+1}/{total_chunks}...", chunk_progress)
                    
                    chunk_prompt = f"{prompt_template}\nNow analyze this documentation (part {i+1} of {total_chunks}):\n{chunk}"
                    
                    try:
                        response = model.generate_content(chunk_prompt)
                        final_raw_output += "\n" + response.text
                    except Exception as e:
                        print(f"[ERROR] Failed to process chunk {i+1}: {str(e)}")
                        continue

            self.update_progress("Processing registers...", 90)

            # Clean the output but preserve the JSON structure
            registers = clean_extracted_registers(final_raw_output)

            # Find the JSON part in the output
            json_match = re.search(r'```json\n(.*?)\n```', final_raw_output, re.DOTALL)
            if json_match:
                registers = json_match.group(1).strip()
            else:
                registers = final_raw_output  # Fallback to raw output if no JSON markers

            # Validate and format the JSON
            try:
                json_data = json.loads(registers)
                formatted_json = json.dumps(json_data, indent=4)
            except json.JSONDecodeError as e:
                raise ValueError(f"Failed to parse AI output as JSON: {e}\n\nRaw output:\n{final_raw_output}")

            # Display in GUI before saving
            self.display_code(formatted_json, "extracted_registers")

            # Ask user to save the extracted registers
            file_path = filedialog.asksaveasfilename(
                initialfile=f"{mcu_name}_REGISTERS.json",
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Save Register File As"
            )
            if not file_path:
                self.update_progress("Save cancelled", 0)
                self.root.after(0, callback, (False, "File save cancelled", None))
                return
            
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(formatted_json)

            # Upload to GitHub if we have a token
            upload_success = False
            if self.mcu_manager.github:
                self.update_progress("Uploading to GitHub...", 95)
                upload_success = self.mcu_manager.github.upload_file(
                    REGISTERS_FOLDER, 
                    github_file, 
                    formatted_json, 
                    f"Add register file for {mcu_name}"
                )
            
            if upload_success:
                self.update_progress("Uploaded to GitHub!", 100)
            else:
                self.update_progress("GitHub upload skipped/failed", 100)

            self.root.after(0, callback, (True, f"Saved to: {os.path.basename(file_path)}", formatted_json))

        except Exception as e:
            error_msg = f"Exception occurred: {str(e)}"
            if hasattr(e, 'response') and e.response:
                error_msg += f"\nAPI Response: {e.response.text}"
            self.update_progress("Extraction failed", 0)
            self.root.after(0, callback, (False, error_msg, None))


    def display_code(self, code, target):
            preview = getattr(self, f"{target}_preview")
            preview.delete(1.0, tk.END)
            preview.insert(tk.END, code)
            
            # Select the registers tab
            self.notebook.select(0)

    def on_generate_mcal_click(self):
        selected = self.mcu_combobox.get()

        if selected == "Custom...":
            mcu_name = self.custom_mcu_entry.get().strip()
            if not mcu_name:
                messagebox.showerror("Input Error", "Please enter a custom microcontroller name.")
                return
            if not self.mcu_manager.add_mcu(mcu_name):
                messagebox.showerror("Error", "Failed to add custom MCU.")
                return
            self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")
            self.mcu_combobox.set(mcu_name)
            self.custom_mcu_frame.grid_remove()
        else:
            mcu_name = selected.strip()

        if not mcu_name or mcu_name == "Custom...":
            messagebox.showerror("Error", "MCU name is invalid.")
            return

        self.mcu_manager.save_selected_mcu(mcu_name)
        self.last_generated_mcu = mcu_name

        # Disable buttons
        self.running = True
        self.generate_mcal_btn.config(state=tk.DISABLED)
        self.cancel_btn.config(state=tk.NORMAL)

        def callback(result):  # Change this to accept a single argument
            success, msg = result  # Unpack the tuple
            self.status_bar.config(text=msg)
            self.generate_mcal_btn.config(state=tk.NORMAL)
            self.cancel_btn.config(state=tk.DISABLED)
            if success:
                messagebox.showinfo("Success", msg)
            else:
                messagebox.showerror("Error", msg)

        # Run generation in a thread
        self.current_thread = threading.Thread(
            target=self.run_mcal_generation,
            args=(mcu_name, callback),
            daemon=True
        )
        self.current_thread.start()




    def run_mcal_generation(self, mcu_name, callback):
        try:
            category = "mcal"
            h_filename = f"{mcu_name}_MCAL.h"
            c_filename = f"{mcu_name}_MCAL.c"

            # --- STEP 1: Check GitHub for existing files ---
            if (
                self.mcu_manager.github
                and self.mcu_manager.github.check_file_exists(MCAL_FOLDER, h_filename)
                and self.mcu_manager.github.check_file_exists(MCAL_FOLDER, c_filename)
            ):
                self.update_progress("Downloading MCAL files from GitHub...", 10)

                output_dir = filedialog.askdirectory(title="Select Output Directory for MCAL Files")
                if not output_dir:
                    self.update_progress("Save cancelled", 0)
                    self.root.after(0, callback, (False, "File save cancelled"))
                    return

                for filename in [h_filename, c_filename]:
                    content = self.mcu_manager.github.download_file(MCAL_FOLDER, filename)
                    if not content:
                        self.update_progress("Download failed", 0)
                        self.root.after(0, callback, (False, f"Failed to download {filename} from GitHub"))
                        return

                    with open(os.path.join(output_dir, filename), "w", encoding="utf-8") as f:
                        f.write(content)

                self.update_progress("Downloaded from GitHub!", 100)
                self.root.after(0, callback, (True, f"MCAL files downloaded to: {output_dir}"))
                return

            # --- STEP 2: Generate files if not found on GitHub ---
            self.update_progress("Preparing register data...", 20)
            register_json = self.extracted_registers_preview.get("1.0", tk.END).strip()
            if not register_json:
                self.update_progress("No JSON available", 0)
                self.root.after(0, callback, (False, "Register data is empty. Run extraction first."))
                return

            try:
                json_data = json.loads(register_json)
                formatted_json = json.dumps(json_data, indent=4)
                print(formatted_json)
            except json.JSONDecodeError as e:
                self.update_progress("Invalid JSON", 0)
                self.root.after(0, callback, (False, f"Failed to parse JSON:\n{str(e)}"))
                return

            doc_sections = ""
            extracted_attr = f"extracted_{category}_files"
            for f in getattr(self, extracted_attr, []):
                doc_sections += f"\n--- File: {f['name']} ({f['type']}) ---\n{f['content']}\n"
                print(f['content'])
            prompt = MCAL_GENERATION_PROMPT.replace("{{mcu_name}}", mcu_name)\
                                        .replace("{{register_json}}", formatted_json)\
                                        .replace("{{documents}}", doc_sections)

            self.update_progress("Generating MCAL files with AI...", 50)
            try:
                response = model.generate_content(prompt)
                ai_output = response.text
            except Exception as e:
                self.update_progress("AI generation failed", 0)
                self.root.after(0, callback, (False, f"Generation failed:\n{str(e)}"))
                return

            file_blocks = re.findall(r"```c filename=(.*?)\n(.*?)```", ai_output, re.DOTALL)
            if not file_blocks:
                self.update_progress("No files found", 0)
                self.root.after(0, callback, (False, "No code blocks found in AI output."))
                return

            output_dir = filedialog.askdirectory(title="Select Output Directory for MCAL Files")
            if not output_dir:
                self.update_progress("Save cancelled", 0)
                self.root.after(0, callback, (False, "File save cancelled"))
                return

            generated_files = {}
            for filename, content in file_blocks:
                if filename.lower().endswith(".h"):
                    filename = h_filename
                elif filename.lower().endswith(".c"):
                    filename = c_filename
                else:
                    continue  # Ignore unexpected files

                filepath = os.path.join(output_dir, filename)
                with open(filepath, "w", encoding="utf-8") as f:
                    f.write(content.strip())

                generated_files[filename] = content.strip()

            # --- STEP 3: Upload to GitHub ---
            if self.mcu_manager.github:
                self.update_progress("Uploading MCAL files to GitHub...", 95)
                for filename, content in generated_files.items():
                    self.mcu_manager.github.upload_file(
                        MCAL_FOLDER,
                        filename,
                        content,
                        f"Add MCAL file {filename} for {mcu_name}"
                    )

            self.update_progress("MCAL files generated and uploaded!", 100)
            self.root.after(0, callback, (True, f"MCAL files saved to: {output_dir}"))

        except Exception as e:
            self.update_progress("MCAL generation failed", 0)
            self.root.after(0, callback, (False, f"Exception occurred:\n{str(e)}"))









    def on_generate_hall_click(self):
            selected = self.mcu_combobox.get()

            if selected == "Custom...":
                mcu_name = self.custom_mcu_entry.get().strip()
                if not mcu_name:
                    messagebox.showerror("Input Error", "Please enter a custom microcontroller name.")
                    return
                if not self.mcu_manager.add_mcu(mcu_name):
                    messagebox.showerror("Error", "Failed to add custom MCU.")
                    return
                self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")
                self.mcu_combobox.set(mcu_name)
                self.custom_mcu_frame.grid_remove()
            else:
                mcu_name = selected.strip()

            if not mcu_name or mcu_name == "Custom...":
                messagebox.showerror("Error", "MCU name is invalid.")
                return

            self.mcu_manager.save_selected_mcu(mcu_name)
            self.last_generated_mcu = mcu_name

            # Disable buttons
            self.running = True
            self.generate_hall_btn.config(state=tk.DISABLED)
            self.cancel_btn.config(state=tk.NORMAL)

            def callback(result):
                success, msg = result
                self.status_bar.config(text=msg)
                self.generate_hall_btn.config(state=tk.NORMAL)
                self.cancel_btn.config(state=tk.DISABLED)
                if success:
                    messagebox.showinfo("Success", msg)
                else:
                    messagebox.showerror("Error", msg)

            # Run generation in a thread
            self.current_thread = threading.Thread(
                target=self.run_hall_generation,
                args=(mcu_name, callback),
                daemon=True
            )
            self.current_thread.start()


    def run_hall_generation(self, mcu_name, callback):
        try:
            category = "hall"
            h_filename = f"{mcu_name}_HALL.h"
            c_filename = f"{mcu_name}_HALL.c"

            # --- STEP 1: Check GitHub for existing files ---
            if (
                self.mcu_manager.github
                and self.mcu_manager.github.check_file_exists(HALL_FOLDER, h_filename)
                and self.mcu_manager.github.check_file_exists(HALL_FOLDER, c_filename)
            ):
                self.update_progress("Downloading HALL files from GitHub...", 10)

                output_dir = filedialog.askdirectory(title="Select Output Directory for HALL Files")
                if not output_dir:
                    self.update_progress("Save cancelled", 0)
                    self.root.after(0, callback, (False, "File save cancelled"))
                    return

                for filename in [h_filename, c_filename]:
                    content = self.mcu_manager.github.download_file(HALL_FOLDER, filename)
                    if not content:
                        self.update_progress("Download failed", 0)
                        self.root.after(0, callback, (False, f"Failed to download {filename} from GitHub"))
                        return

                    with open(os.path.join(output_dir, filename), "w", encoding="utf-8") as f:
                        f.write(content)

                self.update_progress("Downloaded from GitHub!", 100)
                self.root.after(0, callback, (True, f"HALL files downloaded to: {output_dir}"))
                return

            # --- STEP 2: Generate files if not found on GitHub ---
            self.update_progress("Preparing register data...", 20)
            register_json = self.extracted_registers_preview.get("1.0", tk.END).strip()
            if not register_json:
                self.update_progress("No JSON available", 0)
                self.root.after(0, callback, (False, "Register data is empty. Run extraction first."))
                return

            try:
                json_data = json.loads(register_json)
                formatted_json = json.dumps(json_data, indent=4)
                print(formatted_json)
            except json.JSONDecodeError as e:
                self.update_progress("Invalid JSON", 0)
                self.root.after(0, callback, (False, f"Failed to parse JSON:\n{str(e)}"))
                return

            # --- Build documents from extracted hall files ---
            doc_sections = ""
            extracted_attr = f"extracted_{category}_files"
            for f in getattr(self, extracted_attr, []):
                doc_sections += f"\n--- File: {f['name']} ({f['type']}) ---\n{f['content']}\n"
                print(f['content'])

            prompt = HALL_GENERATION_PROMPT.replace("{{mcu_name}}", mcu_name)\
                                        .replace("{{register_json}}", formatted_json)\
                                        .replace("{{documents}}", doc_sections)

            self.update_progress("Generating HALL files with AI...", 50)
            try:
                response = model.generate_content(prompt)
                ai_output = response.text
            except Exception as e:
                self.update_progress("AI generation failed", 0)
                self.root.after(0, callback, (False, f"Generation failed:\n{str(e)}"))
                return

            file_blocks = re.findall(r"```c filename=(.*?)\n(.*?)```", ai_output, re.DOTALL)
            if not file_blocks:
                self.update_progress("No files found", 0)
                self.root.after(0, callback, (False, "No code blocks found in AI output."))
                return

            output_dir = filedialog.askdirectory(title="Select Output Directory for HALL Files")
            if not output_dir:
                self.update_progress("Save cancelled", 0)
                self.root.after(0, callback, (False, "File save cancelled"))
                return

            generated_files = {}
            for filename, content in file_blocks:
                if filename.lower().endswith(".h"):
                    filename = h_filename
                elif filename.lower().endswith(".c"):
                    filename = c_filename
                else:
                    continue  # Ignore unexpected files

                filepath = os.path.join(output_dir, filename)
                with open(filepath, "w", encoding="utf-8") as f:
                    f.write(content.strip())

                generated_files[filename] = content.strip()

            # --- STEP 3: Upload to GitHub ---
            if self.mcu_manager.github:
                self.update_progress("Uploading HALL files to GitHub...", 95)
                for filename, content in generated_files.items():
                    self.mcu_manager.github.upload_file(
                        HALL_FOLDER,
                        filename,
                        content,
                        f"Add HALL file {filename} for {mcu_name}"
                    )

            self.update_progress("HALL files generated and uploaded!", 100)
            self.root.after(0, callback, (True, f"HALL files saved to: {output_dir}"))

        except Exception as e:
            self.update_progress("HALL generation failed", 0)
            self.root.after(0, callback, (False, f"Exception occurred:\n{str(e)}"))






    def on_generate_tt_click(self):
        selected = self.mcu_combobox.get()

        if selected == "Custom...":
            mcu_name = self.custom_mcu_entry.get().strip()
            if not mcu_name:
                messagebox.showerror("Input Error", "Please enter a custom microcontroller name.")
                return
            if not self.mcu_manager.add_mcu(mcu_name):
                messagebox.showerror("Error", "Failed to add custom MCU.")
                return
            self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")
            self.mcu_combobox.set(mcu_name)
            self.custom_mcu_frame.grid_remove()
        else:
            mcu_name = selected.strip()

        if not mcu_name or mcu_name == "Custom...":
            messagebox.showerror("Error", "MCU name is invalid.")
            return

        self.mcu_manager.save_selected_mcu(mcu_name)
        self.last_generated_mcu = mcu_name

        # Disable buttons
        self.running = True
        self.generate_TT_btn.config(state=tk.DISABLED)
        self.cancel_btn.config(state=tk.NORMAL)

        # ✅ Get category directly (saved when dialog was used earlier)
        settings = self.load_settings()
        category = settings.get("scheduler_type", "Time-Trigger")  # fallback = Time-Trigger

        def callback(result):
            success, msg = result
            self.status_bar.config(text=msg)
            self.generate_TT_btn.config(state=tk.NORMAL)
            self.cancel_btn.config(state=tk.DISABLED)
            if success:
                messagebox.showinfo("Success", msg)
            else:
                messagebox.showerror("Error", msg)

        # Run generation in a thread with category
        self.current_thread = threading.Thread(
            target=self.run_tt_generation,
            args=(mcu_name, callback, category),
            daemon=True
        )
        self.current_thread.start()




    def run_tt_generation(self, mcu_name, callback, category):
        try:
            # --- STEP 1: Build filenames based on category ---
            h_filename = f"{mcu_name}_{category}.h"
            c_filename = f"{mcu_name}_{category}.c"

            # Decide GitHub folder (adjust names if you want different folders)
            folder = TT_FOLDER if category == "Time-Trigger" else RTOS_FOLDER

            # --- STEP 2: Try fetching files from GitHub ---
            if (
                self.mcu_manager.github
                and self.mcu_manager.github.check_file_exists(folder, h_filename)
                and self.mcu_manager.github.check_file_exists(folder, c_filename)
            ):
                self.update_progress(f"Downloading {category} files from GitHub...", 10)

                output_dir = filedialog.askdirectory(title=f"Select Output Directory for {category} Files")
                if not output_dir:
                    self.update_progress("Save cancelled", 0)
                    self.root.after(0, callback, (False, "File save cancelled"))
                    return

                for filename in [h_filename, c_filename]:
                    content = self.mcu_manager.github.download_file(folder, filename)
                    if not content:
                        self.update_progress("Download failed", 0)
                        self.root.after(0, callback, (False, f"Failed to download {filename} from GitHub"))
                        return

                    with open(os.path.join(output_dir, filename), "w", encoding="utf-8") as f:
                        f.write(content)

                self.update_progress(f"{category} files downloaded from GitHub!", 100)
                self.root.after(0, callback, (True, f"{category} files downloaded to: {output_dir}"))
                return

            # --- STEP 3: Use local extracted files ---
            self.update_progress(f"Preparing {category} source files...", 20)

            extracted_attr = f"extracted_{category}_files"
            if not hasattr(self, extracted_attr) or not getattr(self, extracted_attr):
                self.update_progress(f"No {category} source files loaded", 0)
                self.root.after(0, callback, (False, f"No {category} scheduler files loaded. Please browse first."))
                return

            doc_sections = ""
            for f in getattr(self, extracted_attr, []):
                doc_sections += f"\n--- File: {f['name']} ({f['type']}) ---\n{f['content']}\n"

            # Ask user where to save
            output_dir = filedialog.askdirectory(title=f"Select Output Directory for {category} Files")
            if not output_dir:
                self.update_progress("Save cancelled", 0)
                self.root.after(0, callback, (False, "File save cancelled"))
                return

            # Save the files locally
            saved_files = {}
            for f in getattr(self, extracted_attr, []):
                target_name = h_filename if f['name'].endswith(".h") else c_filename
                filepath = os.path.join(output_dir, target_name)
                with open(filepath, "w", encoding="utf-8") as out_file:
                    out_file.write(f['content'])
                saved_files[target_name] = f['content']

            # --- STEP 4: Upload to GitHub ---
            if self.mcu_manager.github:
                self.update_progress(f"Uploading {category} files to GitHub...", 95)
                for filename, content in saved_files.items():
                    self.mcu_manager.github.upload_file(
                        folder,
                        filename,
                        content,
                        f"Add {category} file {filename} for {mcu_name}"
                    )

            self.update_progress(f"{category} files processed successfully!", 100)
            self.root.after(0, callback, (True, f"{category} files saved to: {output_dir}"))

        except Exception as e:
            self.update_progress(f"{category} generation failed", 0)
            self.root.after(0, callback, (False, f"Exception occurred:\n{str(e)}"))










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
        
        # Voltage setting
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
        
        # Clock source setting
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
        
        # Register mode setting
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




        # RTOS/Time-Trigger setting
        self.scheduler_type = tk.StringVar(value=self.initial_values.get('scheduler_type', 'Time-Trigger'))  # Default: Time-Trigger
        scheduler_frame = tk.LabelFrame(
            settings_container,
            text=" Scheduler Type ",
            font=("Segoe UI", 10, "bold"),
            bg='#ecf0f1',
            fg='#2c3e50',
            bd=2,
            relief=tk.GROOVE
        )
        scheduler_frame.pack(fill=tk.X, pady=(0, 15))

        tk.Radiobutton(
            scheduler_frame,
            text="Time-Trigger",
            variable=self.scheduler_type,
            value="Time-Trigger",
            font=("Segoe UI", 10),
            bg='#ecf0f1',
            fg='#2c3e50',
            selectcolor='#bdc3c7',
            activebackground='#ecf0f1',
            activeforeground='#e74c3c'
        ).pack(side=tk.LEFT, padx=10, pady=5)

        tk.Radiobutton(
            scheduler_frame,
            text="RTOS",
            variable=self.scheduler_type,
            value="RTOS",
            font=("Segoe UI", 10),
            bg='#ecf0f1',
            fg='#2c3e50',
            selectcolor='#bdc3c7',
            activebackground='#ecf0f1',
            activeforeground='#e74c3c'
        ).pack(side=tk.LEFT, padx=10, pady=5)

        # Button frame - MODIFIED TO CENTER BUTTONS
        button_frame = tk.Frame(settings_container, bg='#ecf0f1')
        button_frame.pack(fill=tk.X, pady=(10, 0))

        # Create a sub-frame to center the buttons
        center_frame = tk.Frame(button_frame, bg='#ecf0f1')
        center_frame.pack(expand=True)  # This will center the buttons

        # Save button - MODIFIED PACKING
        save_btn = tk.Button(
            center_frame,  # Changed from button_frame to center_frame
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
        save_btn.pack(side=tk.LEFT, padx=(0, 10))  # Changed from RIGHT to LEFT

        # Cancel button - MODIFIED PACKING
        cancel_btn = tk.Button(
            center_frame,  # Changed from button_frame to center_frame
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
        cancel_btn.pack(side=tk.LEFT)  # Changed from RIGHT to LEFT
        
        # Center the dialog
        self.update_idletasks()
        x = parent.winfo_x() + (parent.winfo_width() // 2) - (self.winfo_width() // 2)
        y = parent.winfo_y() + (parent.winfo_height() // 2) - (self.winfo_height() // 2)
        self.geometry(f"+{x}+{y}")
        
        # Focus on window
        self.focus_force()
    
    def on_save(self):
        self.result = {
            'voltage': self.voltage.get(),
            'clock_source': self.clock_source.get(),
            'register_mode': self.register_mode.get(),
            'scheduler_type': self.scheduler_type.get(),  # Add this line

        }
        self.parent.selected_category = self.scheduler_type.get()
        self.destroy()
    
    def on_cancel(self):
        self.result = None
        self.destroy()

def run_gui():
    root = tk.Tk()
    app = MCUGeneratorApp(root)
    root.mainloop()

if __name__ == "__main__":
    run_gui()