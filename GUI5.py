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
from PIL import Image, ImageTk 
from tkinter import simpledialog


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
MCAL_H_FOLDER = "MCAL_H_Files"
MCAL_C_FOLDER = "MCAL_C_Files"

Rules_API= "Rules&API"
HAL_FOLDER = "HAL_Files"
TT_FOLDER="TT_Files"
RTOS_FOLDER="RTOS_Files"
APP_FOLDER="APP_Files"

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
    - BUZZER
    - DAC
    - I2S
    - MQTT
    - HTTP
    - WiFi
    - DTC
    


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



MCAL_H_GENERATION_PROMPT = """
You are an embedded systems expert. Generate `MCAL.h` file — for a Microcontroller Abstraction Layer (MCAL) using only the inputs below.

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
  - If missing, implement a generic watchdog refresh sequence based on {{mcu_name}}.
  - Must follow `API_implementation_sequence`.

✅ Rules.json:
- Insert `before_each_function` snippets in all functions.
- Add `global_includes` at the top of `.h`.
- Follow other style/structural rules.

✅ Register normalization:
- Internally normalize register names by removing instance numbers and grouping by peripheral type 
  (GPIO, UART/USART, TIMER, SPI, I2C, ADC, CAN, etc.).
- Always keep the original register names in the generated code.

✅ Output format:
```c filename=MCAL.h
// content here
"""








MCAL_C_GENERATION_PROMPT = """
You are an embedded systems expert. Generate `MCAL.c` file — for a Microcontroller Abstraction Layer (MCAL) using only the inputs below.

--- MCU NAME ---
{{mcu_name}}

--- REGISTER JSON ---
MCU register definitions: address, description, function.
{{register_json}}

--- DOCUMENTS ---
1. API.json — allowed API function names per category.
2. Rules.json — coding rules and constraints.
3. MCAL.h
{{documents}}

--- INSTRUCTIONS ---

✅ Generate:
- `MCAL.c`

✅ Use only defined inputs:
- API functions from `API.json`
- Registers from `register_json`
- Follow all rules from `Rules.json`

🚫 Do NOT:
- Invent registers, peripherals, or APIs
- Access peripherals directly (e.g., `USART1->CR1`)
- Redefine function prototypes (they already exist in `MCAL.h`)
- Implement APIs without required registers (write a comment instead)

✅ For each function:
- Map relevant registers from `register_json` by description or keyword.
- Example: `UART_Init` → UART registers; `MCU_Config_Init` → RCC, FLASH
- If WDT-related registers exist in `register_json`, implement using them.
  - If missing, implement a generic watchdog refresh sequence based on {{mcu_name}}.
  - Must follow `API_implementation_sequence`.

✅ Apply Rules.json:
- Insert `before_each_function` snippets in all functions.
- Add `global_includes` at the top of `.c`.
- Follow all style/structural rules.

✅ Register normalization:
- Internally normalize register names by removing instance numbers and grouping by peripheral type 
  (GPIO, UART/USART, TIMER, SPI, I2C, ADC, CAN, etc.).
- Always keep the original register names in the generated code.

✅ Output format:
```c filename=MCAL.c
// content here
"""










APP_GENERATION_PROMPT = """
You are an expert embedded systems developer.  
Your task is to generate the middleware application code for the microcontroller project.

### Requirements:
1. The application name is **{{app_name}}**.  
2. The microcontroller is **{{mcu_name}}**.  
3. Use the provided documents (MCAL, HAL, tt_scheduler, Interval_Timer, uploaded files (Flowchart) to design the code.  
   - The **Time_Trigger_Folder** contains:
     - `tt_scheduler.c` / `tt_scheduler.h` → The global time-triggered task scheduler.  
     - `Interval_Timer.c` / `Interval_Timer.h` → The specific hardware timer driver that must be used by the scheduler.  
     -  when #define SCHEDULER_TICK_MS you must get it from typdef which inside `Interval_Timer.h`.
   - You must fully integrate the application with both the scheduler and the interval timer if found.  
   - Do not implement a new scheduler or timer — always reuse these provided files.  
4. Always generate exactly 4 files:
   - **{{app_name}}.c** → Main source file containing initialization, middleware logic, and application entry points.
   - **{{app_name}}.h** → Header file with function prototypes, macros, and global structures for the middleware.
   - **{{app_name}}_user.h** → User configuration header with constants, feature flags, and editable parameters (so end-users can adjust behavior without modifying the core code).  
        ⚠ Remove anything related to hardware like pin configuration or hardware parameter values (those belong to HAL layer), PIN assignment must be included in that file.  
   - **{{app_name}}_main.c** → Application entry point.  
        Must contain these elements only inside it :
        1. `main()` function must be void.  
        2. Call to `App_Init()` at startup. 
        3. Enable Global Interrupt
        4. Task scheduler setup using `tt_add_task()`.  
        5. Call to `tt_start()` ensuring it runs on the **Interval Timer driver**.
        6. Infinite loop with `WDT_Reset()` and `tt_dispatch_task()`. 

### File Format:
- Each file must be clearly separated with this exact marker:

FILE: {{app_name}}.c
// code here

FILE: {{app_name}}.h
// code here

FILE: {{app_name}}_user.h
// code here

FILE: {{app_name}}_main.c
// code here

- Do **not** wrap files in Markdown fences (no ``` markers). Only use the `FILE:` prefix.

### Coding Guidelines:
- Use clean, modular, and portable C code.
- Include `#include` guards in all header files.
- Place user-editable macros (like buffer sizes, feature toggles, debug flags) only inside `{{app_name}}_user.h`.
- In `{{app_name}}.c`, include both `{{app_name}}.h` and `{{app_name}}_user.h`.
- Document functions with brief Doxygen-style comments (`/** ... */`).
- Ensure full integration with MCAL, HAL, Flowchart and the **Time Trigger system (tt_scheduler + Interval_Timer)**.
- Scheduler must always run based on the **Interval Timer** driver.

### Provided Documents:
{{documents}}
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

    def list_files(self, folder):
        """Recursively list all files in a GitHub folder"""
        url = f"{GITHUB_API_URL}/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{folder}"
        response = requests.get(url, headers=self.headers)

        if response.status_code != 200:
            return []

        data = response.json()
        all_files = []

        for item in data:
            if item["type"] == "file":
                all_files.append(item["path"])  # full path in repo
            elif item["type"] == "dir":
                # Recurse into subdir
                all_files.extend(self.list_files(item["path"]))

        return all_files

    def download_file(self, path, save_as=None, binary_mode=True):
        """Download a file from GitHub and optionally save it locally."""
        try:
            url = f"{GITHUB_API_URL}/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{path}"
            response = requests.get(url, headers=self.headers)

            if response.status_code == 200:
                data = response.json()
                content = data.get("content", "")
                encoding = data.get("encoding", "")

                if content and encoding == "base64":
                    file_bytes = base64.b64decode(content)

                    # Save to file if requested
                    if save_as:
                        # Ensure the directory exists
                        os.makedirs(os.path.dirname(save_as), exist_ok=True)
                        
                        mode = "wb" if binary_mode else "w"
                        encoding = None if binary_mode else "utf-8"
                        
                        with open(save_as, mode, encoding=encoding) as f:
                            if binary_mode:
                                f.write(file_bytes)
                            else:
                                # Try to decode bytes to string
                                try:
                                    text_content = file_bytes.decode('utf-8')
                                    f.write(text_content)
                                except UnicodeDecodeError:
                                    # Fallback to writing bytes if it's binary data
                                    f.write(file_bytes)
                        print(f"File saved successfully: {save_as}")

                    # Return appropriate content type
                    if binary_mode:
                        return file_bytes
                    else:
                        try:
                            return file_bytes.decode('utf-8')
                        except UnicodeDecodeError:
                            return file_bytes  # Return bytes if it's binary data
                else:
                    print(f"File content is empty or wrong encoding: {path}")
            else:
                print(f"Failed to download {path}, status: {response.status_code}")
            return None

        except requests.RequestException as e:
            print(f"Error downloading file: {e}")
            return None
        except Exception as e:
            print(f"Unexpected error: {e}")
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
        self.uploaded_files = {}  # Dictionary to store uploaded files

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

        # New button for uploading multiple files
        self.upload_files_btn = tk.Button(
            self.mcu_row,
            text="📂 Upload Files",
            command=self.upload_files,
            bg="#9b59b6",
            fg="white",
            font=("Segoe UI", 9, "bold"),
            bd=0,
            padx=10,
            pady=4,
            relief=tk.FLAT
        )
        self.upload_files_btn.pack(side=tk.LEFT, padx=5)


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
            text="⚡ Generate MCAL.h", 
            command=self.on_generate_mcal_h_click, 
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

        self.generate_mcal_c_btn = tk.Button(
            self.left_panel, 
            text="⚡ Generate MCAL.c", 
            command=self.on_generate_mcal_c_click, 
            bg="#e67e22", 
            fg="white", 
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT,
            width=20
        )
        self.generate_mcal_c_btn.pack(fill=tk.X, pady=5)


        self.generate_general_mcal_btn = tk.Button(
            self.left_panel,
            text="📄 Generate General MCAL",
            command=self.on_generate_general_mcal_click,
            bg="#2ecc71",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=15,
            pady=8,
            relief=tk.FLAT,
            width=20
        )
        self.generate_general_mcal_btn.pack(fill=tk.X, pady=5)


        self.generate_TT_btn = tk.Button(
            self.left_panel, 
            text="⚡ Generate OS", 
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




        self.generate_hal_btn = tk.Button(
            self.left_panel, 
            text="⚡ Generate HAL", 
            command=self.on_generate_hal_click, 
            bg="#9b59b6", 
            fg="white", 
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT,
            width=20
        )
        self.generate_hal_btn.pack(fill=tk.X, pady=5)




        self.generate_app_btn = tk.Button(
            self.left_panel, 
            text="⚡ Generate Application", 
            command=self.on_generate_app_click, 
            bg="#9b59b6", 
            fg="white", 
            font=("Segoe UI", 10, "bold"), 
            bd=0, 
            padx=15, 
            pady=8, 
            relief=tk.FLAT,
            width=20
        )
        self.generate_app_btn.pack(fill=tk.X, pady=5)

        # Notebook for code preview
        self.notebook = ttk.Notebook(self.content_frame)
        self.notebook.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # Create tab for registers
        self.create_tab("extracted_registers")

        # ✅ Make sure uploaded files tab exists
        self.create_uploaded_files_tab()

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

        self.about_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.about_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(0, 1))

        self.about_btn = tk.Button(
            self.about_frame,
            text="ℹ️ About",
            command=self.show_about,
            bg="#95a5a6",
            fg="white",
            font=("Segoe UI", 9),
            bd=0,
            padx=10,
            pady=3,
            relief=tk.FLAT
        )
        self.about_btn.pack(side=tk.LEFT)

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


    def create_uploaded_files_tab(self):
            """Create a tab to display uploaded files"""
            self.uploaded_files_tab = tk.Frame(self.notebook, bg="#34495e")
            self.notebook.add(self.uploaded_files_tab, text="Uploaded Files")
            
            # Frame for file list
            file_list_frame = tk.Frame(self.uploaded_files_tab, bg="#34495e")
            file_list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            
            # Label
            label = tk.Label(
                file_list_frame, 
                text="Uploaded Files:", 
                bg="#34495e", 
                fg="#ecf0f1",
                font=("Segoe UI", 10, "bold")
            )
            label.pack(anchor=tk.W, pady=(0, 5))
            
            # Listbox to display uploaded files
            self.uploaded_files_listbox = tk.Listbox(
                file_list_frame,
                bg="#1e272e",
                fg="#ecf0f1",
                selectbackground="#3498db",
                font=("Consolas", 10),
                height=10
            )
            self.uploaded_files_listbox.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
            
            # Bind double-click event to view file content
            self.uploaded_files_listbox.bind("<Double-Button-1>", self.view_uploaded_file)
            
            # Frame for buttons
            button_frame = tk.Frame(file_list_frame, bg="#34495e")
            button_frame.pack(fill=tk.X, pady=(5, 0))
            
            # View button
            view_btn = tk.Button(
                button_frame,
                text="View File",
                command=self.view_selected_file,
                bg="#3498db",
                fg="white",
                font=("Segoe UI", 9),
                bd=0,
                padx=10,
                pady=3,
                relief=tk.FLAT
            )
            view_btn.pack(side=tk.LEFT, padx=(0, 5))
            
            # Remove button
            remove_btn = tk.Button(
                button_frame,
                text="Remove File",
                command=self.remove_selected_file,
                bg="#e74c3c",
                fg="white",
                font=("Segoe UI", 9),
                bd=0,
                padx=10,
                pady=3,
                relief=tk.FLAT
            )
            remove_btn.pack(side=tk.LEFT)

    def on_mcu_selected(self, event):
        selected = self.mcu_combobox.get()
        if selected == "Custom...":
            # Place the custom MCU frame below the selection row
            self.custom_mcu_frame.grid(row=1, column=0, columnspan=2, sticky="w", padx=5, pady=(0, 5))
            self.custom_mcu_entry.focus()
        else:
            # Hide custom MCU frame if not needed
            self.custom_mcu_frame.grid_forget()

    def upload_files(self):
        """Browse and select multiple files (flow charts or JSON files)"""
        filetypes = [
            ("All supported files", "*.pdf *.json *.txt *.c *.h *.jpg *.png *.bmp"),
            ("PDF files", "*.pdf"),
            ("JSON files", "*.json"),
            ("Text files", "*.txt"),
            ("C source files", "*.c"),
            ("Header files", "*.h"),
            ("Image files", "*.jpg *.png *.bmp")
        ]
        
        files = filedialog.askopenfilenames(
            title="Select Files (Flow Charts, JSON, etc.)",
            filetypes=filetypes
        )
        
        if files:
            for file_path in files:
                filename = os.path.basename(file_path)
                try:
                    # Read file content
                    if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
                        # For images, store the file path
                        self.uploaded_files[filename] = {
                            'path': file_path,
                            'type': 'image',
                            'content': None
                        }
                    else:
                        # For text-based files, read the content
                        with open(file_path, 'r', encoding='utf-8') as f:
                            content = f.read()
                        self.uploaded_files[filename] = {
                            'path': file_path,
                            'type': 'text',
                            'content': content
                        }
                    
                    # Update the listbox
                    self.uploaded_files_listbox.insert(tk.END, filename)
                    
                except Exception as e:
                    messagebox.showerror("Error", f"Could not read file {filename}: {str(e)}")
            
            self.update_status(f"Uploaded {len(files)} file(s)")
            self.notebook.select(self.uploaded_files_tab)


    def view_selected_file(self):
        """View the currently selected file in the listbox"""
        selection = self.uploaded_files_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a file to view")
            return
        
        filename = self.uploaded_files_listbox.get(selection[0])
        self.view_uploaded_file_content(filename)

    def view_uploaded_file(self, event=None):
        """Handle double-click event to view file"""
        selection = self.uploaded_files_listbox.curselection()
        if selection:
            filename = self.uploaded_files_listbox.get(selection[0])
            self.view_uploaded_file_content(filename)

    def view_uploaded_file_content(self, filename):
        """View content of an uploaded file in a tab with editing + saving"""
        if filename not in self.uploaded_files:
            messagebox.showerror("Error", "File not found")
            return
        
        file_info = self.uploaded_files[filename]

        if file_info['type'] == 'image':
            # Images still open in popup
            img_window = tk.Toplevel(self.root)
            img_window.title(filename)

            img = Image.open(file_info['path'])
            img.thumbnail((800, 600))
            img_tk = ImageTk.PhotoImage(img)

            label = tk.Label(img_window, image=img_tk, bg="black")
            label.image = img_tk
            label.pack(fill=tk.BOTH, expand=True)

        else:
            # --- Create tab ---
            frame = ttk.Frame(self.notebook)
            self.notebook.add(frame, text=filename)
            self.notebook.select(frame)

            if not hasattr(self, "view_tabs"):
                self.view_tabs = []
            self.view_tabs.append(frame)

            # --- Text area (black theme) ---
            text = tk.Text(
                frame, wrap="word",
                bg="black", fg="white",
                insertbackground="white",  # cursor color
                selectbackground="gray"
            )
            text.pack(expand=True, fill="both")
            text.insert("1.0", file_info['content'])

            # --- Save changes ---
            def save_changes():
                new_content = text.get("1.0", tk.END).rstrip()
                file_info['content'] = new_content  # update memory
                try:
                    with open(file_info['path'], "w", encoding="utf-8") as f:
                        f.write(new_content)
                    self.update_status(f"Saved changes to {filename}")
                    messagebox.showinfo("Success", f"Changes saved to {filename}")
                except Exception as e:
                    messagebox.showerror("Error", f"Could not save file:\n{e}")

            # --- Buttons (Save + Close) ---
            btn_frame = tk.Frame(frame, bg="black")
            btn_frame.pack(fill="x")

            save_btn = tk.Button(
                btn_frame, text="💾 Save",
                command=save_changes,
                bg="white", fg="green",
                activebackground="black", activeforeground="white"
            )
            save_btn.pack(side="left", padx=5, pady=5)

            close_btn = tk.Button(
                btn_frame, text="❌ Close",
                command=lambda tab=frame: self.close_tab(tab),
                bg="white", fg="red",
                activebackground="black", activeforeground="white"
            )
            close_btn.pack(side="right", padx=5, pady=5)



    def remove_selected_file(self):
        """Remove the selected file from the uploaded files"""
        selection = self.uploaded_files_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a file to remove")
            return
        
        filename = self.uploaded_files_listbox.get(selection[0])
        
        # Remove from dictionary
        if filename in self.uploaded_files:
            del self.uploaded_files[filename]
        
        # Remove from listbox
        self.uploaded_files_listbox.delete(selection[0])
        
        self.update_status(f"Removed file: {filename}")     

    def update_status(self, message):
        """Update the status bar with a message"""
        if hasattr(self, "status_bar"):
            self.status_bar.config(text=message)
        else:
            print(f"STATUS: {message}")  # fallback if status_bar doesn't exist yet
       
    def close_tab(self, tab):
        """Close a specific file tab"""
        if hasattr(self, "view_tabs") and tab in self.view_tabs:
            self.view_tabs.remove(tab)
        self.notebook.forget(tab)



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
        self.generate_mcal_c_btn.config(state=tk.NORMAL)
        self.generate_hal_btn.config(state=tk.NORMAL)
        self.generate_TT_btn.config(state=tk.NORMAL)
        self.generate_app_btn.config(state=tk.NORMAL)
        self.upload_files_btn.config(state=tk.NORMAL)
        self.advanced_btn.config(state=tk.NORMAL)
        self.cancel_btn.config(state=tk.DISABLED)
        self.uploaded_files.clear()
        self.uploaded_files_listbox.delete(0, tk.END)
        self.update_status("System reset successfully")

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

        # ✅ Close all "view file" tabs
        if hasattr(self, "view_tabs"):
            for tab in self.view_tabs:
                self.notebook.forget(tab)
            self.view_tabs = []

        messagebox.showinfo("System Reset", "The system has been reset and is ready for a new generation.")

    def update_progress(self, msg, val):
        self.progress_label.config(text=msg)
        self.progress_bar["value"] = val
        self.root.update_idletasks()


    def show_about(self):
        """Display the About dialog with version information"""
        about_text = """
Ragura Software Developer Kit - Release Notes

Version: 1.4.0

Core Features

- PDF Register Extraction: Automated parsing of microcontroller documentation
- MCAL Generation: Complete header and source file creation
- HAL Implementation: Hardware abstraction layer generation
- Scheduler Support: Time-Triggered and RTOS scheduler code generation

Version History

Version 1.4.0 (Current)
- New Functionality:
  - Separate generation controls for Mcal.h and Mcal.c
  - Enhanced PDF text extraction algorithms
  - Improved user interface with dedicated action buttons

Version 1.3.0
- New Features:
  - HAL file generation capabilities
  - Enhanced GitHub integration for version control
- Improvements:
  - Optimized file management workflows
  - Extended documentation support

Version 1.2.0
- Scheduler Implementation:
  - Time-Triggered scheduler architecture
  - RTOS scheduler integration
  - Configurable scheduling parameters

Version 1.1.0
- MCAL Generation:
  - Unified MCAL.h and MCAL.c generation
  - Enhanced register mapping algorithms
  - Improved code structure and formatting

Version 1.0.0
- Initial Release:
  - Basic PDF register extraction
  - Advanced configuration settings:
    - Voltage parameters
    - Clock source selection
    - Register mode configuration
  - MCU reference manual processing

---

Copyright © 2025 ELARABY GROUP - Technology & Innovation Center 
All rights reserved.
"""
        
        # Create a simple dialog
        about_dialog = tk.Toplevel(self.root)
        about_dialog.title("About Ragura Software")
        about_dialog.geometry("600x600")
        about_dialog.configure(bg="#2c3e50")
        about_dialog.resizable(True, True)
        
        # Center the dialog
        about_dialog.transient(self.root)
        about_dialog.grab_set()
        
        x = self.root.winfo_x() + (self.root.winfo_width() // 2) - (300)  # Half of 500
        y = self.root.winfo_y() + (self.root.winfo_height() // 2) - (300)  # Half of 400
        about_dialog.geometry(f"+{x}+{y}")
        
        # Create a text widget with scrollbar
        text_frame = tk.Frame(about_dialog, bg="#2c3e50")
        text_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        text_widget = ScrolledText(
            text_frame,
            bg="#1e272e",
            fg="#ecf0f1",
            insertbackground="white",
            font=("Consolas", 10),
            wrap=tk.WORD,
            padx=10,
            pady=10
        )
        text_widget.pack(fill=tk.BOTH, expand=True)
        text_widget.insert(tk.END, about_text)
        text_widget.config(state=tk.DISABLED)  # Make it read-only
        
        # OK button
        button_frame = tk.Frame(about_dialog, bg="#2c3e50")
        button_frame.pack(fill=tk.X, pady=(0, 10))
        
        ok_btn = tk.Button(
            button_frame,
            text="OK",
            command=about_dialog.destroy,
            bg="#3498db",
            fg="white",
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=20,
            pady=5,
            relief=tk.FLAT
        )
        ok_btn.pack()

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
                content = self.mcu_manager.github.download_file(f"{REGISTERS_FOLDER}/{github_file}", binary_mode=False)
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
                        print(response)
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

    def on_generate_mcal_h_click(self):
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
            target=self.run_mcal_h_generation,
            args=(mcu_name, callback),
            daemon=True
        )
        self.current_thread.start()




    def run_mcal_h_generation(self, mcu_name, callback):
        try:
            h_filename = f"{mcu_name}_MCAL.h"
            reg_filename = f"{mcu_name}_REGISTERS.json"

            # STEP 1: Check GitHub for already existing MCAL files
            if self.mcu_manager.github \
                and self.mcu_manager.github.check_file_exists(MCAL_H_FOLDER, h_filename):

                self.update_progress("Downloading MCAL.h file from GitHub...", 20)

                output_dir = filedialog.askdirectory(title="Select Output Directory for MCAL Files")
                if not output_dir:
                    self.update_progress("Save cancelled", 0)
                    self.root.after(0, callback, (False, "File save cancelled"))
                    return

                # Create MCAL subfolder
                mcal_dir = os.path.join(output_dir, "MCAL_H")
                os.makedirs(mcal_dir, exist_ok=True)

                # Download header file
                h_content = self.mcu_manager.github.download_file(f"{MCAL_H_FOLDER}/{h_filename}", binary_mode=False)
                if not h_content:
                    self.update_progress("Download failed", 0)
                    self.root.after(0, callback, (False, f"Failed to download {h_filename} from GitHub"))
                    return

                

                # Save files in MCAL folder
                with open(os.path.join(mcal_dir, h_filename), "w", encoding="utf-8") as f:
                    f.write(h_content)

                self.update_progress("MCAL files downloaded successfully!", 100)
                self.root.after(0, callback, (True, f"MCAL files downloaded to: {mcal_dir}"))
                return

            # STEP 2: If not found, prepare to generate
            self.update_progress("Preparing generation input files...", 30)

            # Required input JSONs
            input_files = {
                "registers": (REGISTERS_FOLDER, reg_filename),
                "rule1": (Rules_API, "API.json"),
                "rule2": (Rules_API, "Rules.json")
            }
            json_contents = {}

            # Download required JSONs from GitHub
            for key, (folder, jfile) in input_files.items():
                if self.mcu_manager.github.check_file_exists(folder, jfile):
                    content = self.mcu_manager.github.download_file(f"{folder}/{jfile}", binary_mode=False)
                    if not content:
                        self.update_progress("Missing input JSON", 0)
                        self.root.after(0, callback, (False, f"Failed to download {jfile} from GitHub"))
                        return
                    json_contents[jfile] = content
                else:
                    self.update_progress("Missing input JSON", 0)
                    self.root.after(0, callback, (False, f"{jfile} not found in {folder}/ on GitHub"))
                    return

            # STEP 3: Build AI prompt
            self.update_progress("Building AI prompt...", 50)
            try:
                formatted_registers = json.dumps(json.loads(json_contents[reg_filename]), indent=4)
                print(formatted_registers)
            except Exception as e:
                self.update_progress("Invalid registers.json", 0)
                self.root.after(0, callback, (False, f"Invalid {reg_filename}:\n{str(e)}"))
                return

            doc_sections = ""
            for name, content in json_contents.items():
                if name != reg_filename:  # add rules JSONs to documents section
                    doc_sections += f"\n--- File: {name} ---\n{content}\n"
                    print(f"{name}: {content}")

            prompt = MCAL_H_GENERATION_PROMPT.replace("{{mcu_name}}", mcu_name) \
                                        .replace("{{register_json}}", formatted_registers) \
                                        .replace("{{documents}}", doc_sections)

            # STEP 4: Generate files with AI
            self.update_progress("Generating MCAL.h file with AI...", 70)
            try:
                response = model.generate_content(prompt)
                ai_output = response.text
                print(response)
            except Exception as e:
                self.update_progress("AI generation failed", 0)
                self.root.after(0, callback, (False, f"AI generation failed:\n{str(e)}"))
                return

            file_blocks = re.findall(r"```c filename=(.*?)\n(.*?)```", ai_output, re.DOTALL)
            if not file_blocks:
                self.update_progress("No files found in AI output", 0)
                self.root.after(0, callback, (False, "No code blocks found in AI output."))
                return

            output_dir = filedialog.askdirectory(title="Select Output Directory for Generated MCAL.h File")
            if not output_dir:
                self.update_progress("Save cancelled", 0)
                self.root.after(0, callback, (False, "File save cancelled"))
                return

            # Create MCAL subfolder
            mcal_dir = os.path.join(output_dir, "MCAL_H")
            os.makedirs(mcal_dir, exist_ok=True)

            generated_files = {}
            for filename, content in file_blocks:
                if filename.lower().endswith(".h"):
                    filename = h_filename
                
                else:
                    continue  # ignore extra files

                filepath = os.path.join(mcal_dir, filename)
                with open(filepath, "w", encoding="utf-8") as f:
                    f.write(content.strip())

                generated_files[filename] = content.strip()

            # STEP 5: Upload to GitHub
            self.update_progress("Uploading generated files to GitHub...", 90)
            for filename, content in generated_files.items():
                self.mcu_manager.github.upload_file(
                    MCAL_H_FOLDER,
                    filename,
                    content,
                    f"Add generated {filename} for {mcu_name}"
                )

            self.update_progress("MCAL.h file generated and uploaded!", 100)
            self.root.after(0, callback, (True, f"MCAL.h file saved to: {mcal_dir}"))

        except Exception as e:
            self.update_progress("MCAL.h generation failed", 0)
            self.root.after(0, callback, (False, f"Exception occurred:\n{str(e)}"))






    def on_generate_mcal_c_click(self):
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
            self.generate_mcal_c_btn.config(state=tk.DISABLED)
            self.cancel_btn.config(state=tk.NORMAL)

            def callback(result):  # Change this to accept a single argument
                success, msg = result  # Unpack the tuple
                self.status_bar.config(text=msg)
                self.generate_mcal_c_btn.config(state=tk.NORMAL)
                self.cancel_btn.config(state=tk.DISABLED)
                if success:
                    messagebox.showinfo("Success", msg)
                else:
                    messagebox.showerror("Error", msg)

            # Run generation in a thread
            self.current_thread = threading.Thread(
                target=self.run_mcal_c_generation,
                args=(mcu_name, callback),
                daemon=True
            )
            self.current_thread.start()





    def run_mcal_c_generation(self, mcu_name, callback):
        try:
            c_filename = f"{mcu_name}_MCAL.c"
            h_filename = f"{mcu_name}_MCAL.h"   # <-- add reference to .h file
            reg_filename = f"{mcu_name}_REGISTERS.json"

            # STEP 1: Check GitHub for already existing MCAL files
            if self.mcu_manager.github \
                and self.mcu_manager.github.check_file_exists(MCAL_C_FOLDER, c_filename):

                self.update_progress("Downloading MCAL files from GitHub...", 20)

                output_dir = filedialog.askdirectory(title="Select Output Directory for MCAL Files")
                if not output_dir:
                    self.update_progress("Save cancelled", 0)
                    self.root.after(0, callback, (False, "File save cancelled"))
                    return

                # Create MCAL subfolder
                mcal_dir = os.path.join(output_dir, "MCAL_C")
                os.makedirs(mcal_dir, exist_ok=True)

                # Download C file
                h_content = self.mcu_manager.github.download_file(f"{MCAL_C_FOLDER}/{c_filename}", binary_mode=False)
                if not h_content:
                    self.update_progress("Download failed", 0)
                    self.root.after(0, callback, (False, f"Failed to download {c_filename} from GitHub"))
                    return

                with open(os.path.join(mcal_dir, c_filename), "w", encoding="utf-8") as f:
                    f.write(h_content)

                self.update_progress("MCAL files downloaded successfully!", 100)
                self.root.after(0, callback, (True, f"MCAL files downloaded to: {mcal_dir}"))
                return

            # STEP 2: Prepare input files
            self.update_progress("Preparing generation input files...", 30)

            input_files = {
                "registers": (REGISTERS_FOLDER, reg_filename),
                "rule1": (Rules_API, "API.json"),
                "rule2": (Rules_API, "Rules.json"),
            }
            json_contents = {}

            for key, (folder, jfile) in input_files.items():
                if self.mcu_manager.github.check_file_exists(folder, jfile):
                    content = self.mcu_manager.github.download_file(f"{folder}/{jfile}", binary_mode=False)
                    if not content:
                        self.update_progress("Missing input JSON", 0)
                        self.root.after(0, callback, (False, f"Failed to download {jfile} from GitHub"))
                        return
                    json_contents[jfile] = content
                else:
                    self.update_progress("Missing input JSON", 0)
                    self.root.after(0, callback, (False, f"{jfile} not found in {folder}/ on GitHub"))
                    return

            # ✅ NEW STEP: download the MCAL.h file (generated earlier)
            h_content = None
            if self.mcu_manager.github.check_file_exists(MCAL_H_FOLDER, h_filename):
                h_content = self.mcu_manager.github.download_file(f"{MCAL_H_FOLDER}/{h_filename}", binary_mode=False)
                if not h_content:
                    self.update_progress("Failed to fetch MCAL.h", 0)
                    self.root.after(0, callback, (False, f"Failed to download {h_filename} from GitHub"))
                    return

            # STEP 3: Build AI prompt
            self.update_progress("Building AI prompt...", 50)
            try:
                formatted_registers = json.dumps(json.loads(json_contents[reg_filename]), indent=4)
                print(formatted_registers)
            except Exception as e:
                self.update_progress("Invalid registers.json", 0)
                self.root.after(0, callback, (False, f"Invalid {reg_filename}:\n{str(e)}"))
                return

            doc_sections = ""
            for name, content in json_contents.items():
                if name != reg_filename:
                    doc_sections += f"\n--- File: {name} ---\n{content}\n"
                    #print(f"{name}: {content}")

            # ✅ Add MCAL.h as input to the documents
            if h_content:
                doc_sections += f"\n--- File: {h_filename} ---\n{h_content}\n"
                print(doc_sections)

            prompt = MCAL_C_GENERATION_PROMPT.replace("{{mcu_name}}", mcu_name) \
                                            .replace("{{register_json}}", formatted_registers) \
                                            .replace("{{documents}}", doc_sections)

            # STEP 4: Generate files with AI
            self.update_progress("Generating MCAL.c file with AI...", 70)
            try:
                response = model.generate_content(prompt)
                ai_output = response.text
                print(response)
            except Exception as e:
                self.update_progress("AI generation failed", 0)
                self.root.after(0, callback, (False, f"AI generation failed:\n{str(e)}"))
                return

            file_blocks = re.findall(r"```c filename=(.*?)\n(.*?)```", ai_output, re.DOTALL)
            if not file_blocks:
                self.update_progress("No files found in AI output", 0)
                self.root.after(0, callback, (False, "No code blocks found in AI output."))
                return

            output_dir = filedialog.askdirectory(title="Select Output Directory for Generated MCAL.c File")
            if not output_dir:
                self.update_progress("Save cancelled", 0)
                self.root.after(0, callback, (False, "File save cancelled"))
                return

            # Create MCAL subfolder
            mcal_dir = os.path.join(output_dir, "MCAL_C")
            os.makedirs(mcal_dir, exist_ok=True)

            generated_files = {}
            for filename, content in file_blocks:
                if filename.lower().endswith(".c"):
                    filename = c_filename
                else:
                    continue  # ignore extra files

                filepath = os.path.join(mcal_dir, filename)
                with open(filepath, "w", encoding="utf-8") as f:
                    f.write(content.strip())

                generated_files[filename] = content.strip()

            # STEP 5: Upload to GitHub
            self.update_progress("Uploading generated files to GitHub...", 90)
            for filename, content in generated_files.items():
                self.mcu_manager.github.upload_file(
                    MCAL_C_FOLDER,
                    filename,
                    content,
                    f"Add generated {filename} for {mcu_name}"
                )

            self.update_progress("MCAL.c file generated and uploaded!", 100)
            self.root.after(0, callback, (True, f"MCAL.c file saved to: {mcal_dir}"))

        except Exception as e:
            self.update_progress("MCAL.c generation failed", 0)
            self.root.after(0, callback, (False, f"Exception occurred:\n{str(e)}"))








    def on_generate_general_mcal_click(self):
        selected = self.mcu_combobox.get()
        if selected == "Custom...":
            mcu_name = self.custom_mcu_entry.get().strip()
        else:
            mcu_name = selected.strip()

        if not mcu_name or mcu_name == "Custom...":
            messagebox.showerror("Error", "Please select or enter a valid MCU name.")
            return

        # Ask where to save
        output_dir = filedialog.askdirectory(title="Select Output Directory for GENERATED_MCAL")
        if not output_dir:
            return  # user cancelled

        filename = os.path.join(output_dir, "GENERATED_MCAL.h")
        try:
            with open(filename, "w", encoding="utf-8") as f:
                f.write(f'#include "{mcu_name}_MCAL.h"\n')
            messagebox.showinfo("Success", f"GENERATED_MCAL.c created at:\n{filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to create file:\n{e}")






    def on_generate_hal_click(self):
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
            self.generate_hal_btn.config(state=tk.DISABLED)
            self.cancel_btn.config(state=tk.NORMAL)

            def callback(result):
                success, msg = result
                self.status_bar.config(text=msg)
                self.generate_hal_btn.config(state=tk.NORMAL)
                self.cancel_btn.config(state=tk.DISABLED)
                if success:
                    messagebox.showinfo("Success", msg)
                else:
                    messagebox.showerror("Error", msg)

            # Run generation in a thread
            self.current_thread = threading.Thread(
                target=self.run_hal_generation,
                args=(mcu_name, callback),
                daemon=True
            )
            self.current_thread.start()


    
    def run_hal_generation(self, mcu_name, callback):
        try:
            # --- STEP 1: Download entire HAL Files folder from GitHub ---
            if not self.mcu_manager.github:
                self.update_progress("GitHub not configured", 0)
                self.root.after(0, callback, (False, "GitHub connection not available"))
                return

            self.update_progress("Fetching HAL files list from GitHub...", 10)

            # Use the actual folder name with space
            github_folder_name = "HAL_Files"
            
            # Get all files in HAL Files folder (recursively)
            file_list = self.mcu_manager.github.list_files(github_folder_name)
            if not file_list:
                self.update_progress("No HAL files found on GitHub", 0)
                self.root.after(0, callback, (False, "No HAL files found in GitHub HAL Files folder"))
                return

            # Ask user where to save
            output_dir = filedialog.askdirectory(title="Select Output Directory for HAL Files")
            if not output_dir:
                self.update_progress("Save cancelled", 0)
                self.root.after(0, callback, (False, "File save cancelled"))
                return

            downloaded_files = {}
            total_files = len(file_list)

            for i, filepath in enumerate(file_list, start=1):
                if not self.running:
                    return
                    
                # Determine if file is binary based on extension
                is_binary = not filepath.lower().endswith(('.c', '.h', '.txt', '.json', '.md', '.cpp', '.hpp'))
                
                # Download file in binary mode
                content = self.mcu_manager.github.download_file(filepath, binary_mode=True)
                
                if content is None:
                    self.update_progress(f"Download failed for: {os.path.basename(filepath)}", 0)
                    self.root.after(0, callback, (False, f"Failed to download {filepath} from GitHub"))
                    return

                # Preserve folder structure relative to the GitHub folder
                relative_path = os.path.relpath(filepath, github_folder_name)
                local_path = os.path.join(output_dir, "HAL_Files", relative_path)
                os.makedirs(os.path.dirname(local_path), exist_ok=True)

                # Write file - always write as binary
                with open(local_path, "wb") as f:
                    f.write(content)

                downloaded_files[filepath] = content

                # Update progress dynamically
                progress = 10 + int((i / total_files) * 80)
                self.update_progress(f"Downloading HAL files... ({i}/{total_files})", progress)

            self.update_progress("HAL files downloaded successfully!", 100)
            self.root.after(0, callback, (True, f"HAL files saved to: {output_dir}"))

        except Exception as e:
            self.update_progress("HAL generation failed", 0)
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
            # Pick folder based on category (force forward slashes for GitHub)
            if category == "Time-Trigger":
                folder = os.path.join("TT_Files", f"{mcu_name}_TimeTrigger").replace("\\", "/")
            elif category == "RTOS":
                folder = os.path.join("RTOS_Files", f"{mcu_name}_RTOS").replace("\\", "/")
            else:
                self.update_progress("Invalid category", 0)
                self.root.after(0, callback, (False, f"Unknown category: {category}"))
                return

            if not self.mcu_manager.github:
                self.update_progress("GitHub not configured", 0)
                self.root.after(0, callback, (False, "GitHub connection not available"))
                return

            self.update_progress(f"Fetching {category} files list from GitHub...", 10)

            # Get only the files for the target folder
            file_list = self.mcu_manager.github.list_files(folder)

            if not file_list:
                self.update_progress(f"No {category} files found on GitHub", 0)
                self.root.after(0, callback, (False, f"No {category} files found in {folder}"))
                return

            # Ask user where to save
            output_dir = filedialog.askdirectory(title=f"Select Output Directory for {category} Files")
            if not output_dir:
                self.update_progress("Save cancelled", 0)
                self.root.after(0, callback, (False, "File save cancelled"))
                return

            downloaded_files = {}
            for filepath in file_list:
                # Determine if file is binary based on extension
                is_binary = not filepath.lower().endswith(('.c', '.h', '.txt', '.json', '.md'))
                content = self.mcu_manager.github.download_file(filepath, binary_mode=is_binary)

                if content is None:
                    self.update_progress("Download failed", 0)
                    self.root.after(0, callback, (False, f"Failed to download {filepath} from GitHub"))
                    return

                # Keep original repo substructure
                relative_path = os.path.relpath(filepath, folder)
                dest_path = os.path.join(output_dir, category, relative_path)
                os.makedirs(os.path.dirname(dest_path), exist_ok=True)

                # Write file with appropriate mode
                if is_binary:
                    with open(dest_path, "wb") as f:
                        f.write(content)
                else:
                    if isinstance(content, bytes):
                        try:
                            content = content.decode("utf-8")
                        except UnicodeDecodeError:
                            with open(dest_path, "wb") as f:
                                f.write(content)
                            continue

                    with open(dest_path, "w", encoding="utf-8") as f:
                        f.write(content)

                downloaded_files[filepath] = content

            self.update_progress(f"{category} files downloaded successfully!", 100)
            self.root.after(0, callback, (True, f"{category} files saved to: {output_dir}"))

        except Exception as e:
            self.update_progress(f"{category} generation failed", 0)
            self.root.after(0, callback, (False, f"Exception occurred:\n{str(e)}"))





    def on_generate_app_click(self):
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

        # ✅ Ask user for application name
        app_name = simpledialog.askstring("Application Name", "Enter a name for your application:")
        if not app_name:
            messagebox.showwarning("Cancelled", "Application generation cancelled. No application name entered.")
            return

        self.mcu_manager.save_selected_mcu(mcu_name)
        self.last_generated_mcu = mcu_name

        # Disable buttons
        self.running = True
        self.generate_app_btn.config(state=tk.DISABLED)
        self.cancel_btn.config(state=tk.NORMAL)

        def callback(result):
            success, msg = result
            self.status_bar.config(text=msg)
            self.generate_app_btn.config(state=tk.NORMAL)
            self.cancel_btn.config(state=tk.DISABLED)
            if success:
                messagebox.showinfo("Success", msg)
            else:
                messagebox.showerror("Error", msg)

        # Run generation in a thread
        self.current_thread = threading.Thread(
            target=self.run_app_generation,
            args=(mcu_name, app_name, callback),  # ✅ pass app_name too
            daemon=True
        )
        self.current_thread.start()


    def run_app_generation(self, mcu_name, app_name, callback=None):
        """Collect all required sources (uploaded + generated) and generate final application with AI."""
        try:
            app_files = [
                f"{app_name}.c",
                f"{app_name}.h",
                f"{app_name}_user.h",
                f"{app_name}_main.c"
            ]

            # STEP 1: Check GitHub for existing files
            if self.mcu_manager.github:
                all_exist = all(
                    self.mcu_manager.github.check_file_exists(APP_FOLDER, fname)
                    for fname in app_files
                )
                if all_exist:
                    self.update_progress("Downloading application files from GitHub...", 20)

                    output_dir = filedialog.askdirectory(title="Select Output Directory for Application")
                    if not output_dir:
                        self.update_progress("Save cancelled", 0)
                        self.root.after(0, callback, (False, "File save cancelled"))
                        return

                    app_dir = os.path.join(output_dir, "APPLICATION")
                    os.makedirs(app_dir, exist_ok=True)

                    for fname in app_files:
                        content = self.mcu_manager.github.download_file(f"{APP_FOLDER}/{fname}", binary_mode=False)
                        if not content:
                            self.update_progress("Download failed", 0)
                            self.root.after(0, callback, (False, f"Failed to download {fname} from GitHub"))
                            return

                        with open(os.path.join(app_dir, fname), "w", encoding="utf-8") as f:
                            f.write(content)

                    self.update_progress("Application downloaded successfully!", 100)
                    self.root.after(0, callback, (True, f"Application files downloaded to: {app_dir}"))
                    return

            # STEP 2: Collect uploaded files (text + images for Gemini)
            self.update_progress("Collecting uploaded files...", 30)
            uploaded_files = {}
            for filename, file_info in self.uploaded_files.items():
                if file_info['type'] == 'image':
                    with open(file_info['path'], "rb") as f:
                        image_bytes = f.read()
                        uploaded_files[filename] = {
                            "type": "image",
                            "mime_type": "image/png",  # or "image/jpeg"
                            "data": image_bytes
                        }
                else:
                    uploaded_files[filename] = {
                        "type": "text",
                        "content": file_info['content']
                    }

            # STEP 3: Collect generated sources from GitHub
            self.update_progress("Collecting generated sources...", 40)
            sources = {}
            
            # Get MCAL.h
            mcal_h_filename = f"{mcu_name}_MCAL.h"
            if self.mcu_manager.github and self.mcu_manager.github.check_file_exists(MCAL_H_FOLDER, mcal_h_filename):
                mcal_h_content = self.mcu_manager.github.download_file(f"{MCAL_H_FOLDER}/{mcal_h_filename}", binary_mode=False)
                if mcal_h_content:
                    sources["MCAL.h"] = mcal_h_content
            
            # Get MCAL.c
            mcal_c_filename = f"{mcu_name}_MCAL.c"
            if self.mcu_manager.github and self.mcu_manager.github.check_file_exists(MCAL_C_FOLDER, mcal_c_filename):
                mcal_c_content = self.mcu_manager.github.download_file(f"{MCAL_C_FOLDER}/{mcal_c_filename}", binary_mode=False)
                if mcal_c_content:
                    sources["MCAL.c"] = mcal_c_content
            
            # Get HAL files
            hal_files = {}
            if self.mcu_manager.github:
                hal_file_list = self.mcu_manager.github.list_files(HAL_FOLDER)
                for filepath in hal_file_list:
                    if filepath.lower().endswith(('.c', '.h')):
                        content = self.mcu_manager.github.download_file(filepath, binary_mode=False)
                        if content:
                            filename = os.path.basename(filepath)
                            hal_files[filename] = content
                if hal_files:
                    sources["HAL"] = hal_files
            
            # Get Scheduler files (TT or RTOS based on settings)
            scheduler_files = {}
            settings = self.load_settings()
            scheduler_type = settings.get("scheduler_type", "Time-Trigger")
            scheduler_folder = TT_FOLDER if scheduler_type == "Time-Trigger" else RTOS_FOLDER
            
            if self.mcu_manager.github:
                scheduler_file_list = self.mcu_manager.github.list_files(scheduler_folder)
                for filepath in scheduler_file_list:
                    if filepath.lower().endswith(('.c', '.h')):
                        content = self.mcu_manager.github.download_file(filepath, binary_mode=False)
                        if content:
                            filename = os.path.basename(filepath)
                            scheduler_files[filename] = content
                if scheduler_files:
                    sources["Scheduler"] = scheduler_files

            # STEP 4: Build AI prompt
            self.update_progress("Building AI prompt...", 60)
            doc_sections = ""
            # Uploaded files
            for fname, content in uploaded_files.items():
                if content["type"] == "text":
                    doc_sections += f"\n--- Uploaded File: {fname} ---\n{content['content']}\n"
                elif content["type"] == "image":
                    doc_sections += f"\n--- Uploaded Image: {fname} ---\n<IMAGE ATTACHED>\n"

            # Generated sources
            for name, content in sources.items():
                if isinstance(content, dict):  # Folders like HAL, Scheduler
                    for subname, subcontent in content.items():
                        doc_sections += f"\n--- {name} File: {subname} ---\n{subcontent}\n"
                else:  # Single files like MCAL.h, MCAL.c
                    doc_sections += f"\n--- {name} ---\n{content}\n"
            print(doc_sections)
            prompt = APP_GENERATION_PROMPT \
                .replace("{{app_name}}", app_name) \
                .replace("{{mcu_name}}", mcu_name) \
                .replace("{{documents}}", doc_sections)

            # STEP 5: Generate application with Gemini AI
            self.update_progress("Generating application with AI...", 80)
            try:
                # Gemini requires parts for images
                parts = [{"text": prompt}]
                for fname, content in uploaded_files.items():
                    if content["type"] == "image":
                        parts.append({
                            "inline_data": {
                                "mime_type": content["mime_type"],
                                "data": content["data"]
                            }
                        })

                response = model.generate_content([{"role": "user", "parts": parts}])
                ai_output = response.text
            except Exception as e:
                self.update_progress("AI generation failed", 0)
                self.root.after(0, callback, (False, f"AI generation failed:\n{str(e)}"))
                return

            # STEP 6: Split AI output into files
            generated_files = {}
            current_file = None
            for line in ai_output.splitlines():
                if line.strip().startswith("FILE:"):
                    current_file = line.strip().replace("FILE:", "").strip()
                    generated_files[current_file] = []
                elif current_file:
                    generated_files[current_file].append(line)

            # STEP 7: Save generated files locally
            self.update_progress("Saving application files...", 90)
            output_dir = filedialog.askdirectory(title="Select Output Directory for Application")
            if not output_dir:
                self.update_progress("Save cancelled", 0)
                self.root.after(0, callback, (False, "File save cancelled"))
                return

            app_dir = os.path.join(output_dir, "APPLICATION")
            os.makedirs(app_dir, exist_ok=True)

            for fname in app_files:
                content = "\n".join(generated_files.get(fname, ["// Empty file generated"]))
                with open(os.path.join(app_dir, fname), "w", encoding="utf-8") as f:
                    f.write(content)

            # STEP 8: Upload generated files to GitHub
            if self.mcu_manager.github:
                try:
                    for fname in app_files:
                        content = "\n".join(generated_files.get(fname, ["// Empty file generated"]))
                        self.mcu_manager.github.upload_file(
                            APP_FOLDER, fname, content,
                            message= f"Add generated {fname} for {app_name}"
                        )
                    self.update_progress("Application uploaded to GitHub!", 100)
                except Exception as e:
                    self.update_progress("GitHub upload failed", 95)
                    self.root.after(0, callback, (False, f"Files saved locally but GitHub upload failed:\n{str(e)}"))
                    return

            # Final success
            self.update_progress("Application generated successfully!", 100)
            self.root.after(0, callback, (True, f"Application files saved to: {app_dir}"))

        except Exception as e:
            self.update_progress("Application generation failed", 0)
            self.root.after(0, callback, (False, f"Application generation failed:\n{str(e)}"))



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