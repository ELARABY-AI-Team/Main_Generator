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

# Support for PyInstaller .exe path
BASE_PATH = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
MCU_DATA_FILE = os.path.join(BASE_PATH, "mcu_data.json")

# Configure Gemini API
genai.configure(api_key=os.getenv('GEMINI_API_KEY', 'AIzaSyB4kW-XsrZSJOkekbkasW9M2QJ_VG0Iuzo'))

# Create the model instance
model = genai.GenerativeModel("gemini-2.5-flash-preview-04-17")


class MCUManager:
    def __init__(self):
        self.mcu_list = self.load_mcu_list()
        
    def load_mcu_list(self):
        """Load MCU list from file or use default if file doesn't exist"""
        default_list = [
            "R5F11BBC", "STM32F103C8T6", "ATmega328P", "PIC16F877A", "ESP32",
            "MSP430G2553", "NXP LPC1768", "ATSAMD21G18A", "STM32F401RE", "TM4C123GH6PM"
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
        """Save the current MCU list to file"""
        try:
            with open(MCU_DATA_FILE, 'w') as f:
                json.dump({'mcu_list': self.mcu_list}, f, indent=4)
        except Exception as e:
            print(f"Error saving MCU list: {e}")
            
    def add_mcu(self, mcu_name):
        """Add a new MCU to the list if it doesn't already exist"""
        if mcu_name and mcu_name.strip() and mcu_name not in self.mcu_list:
            self.mcu_list.append(mcu_name.strip())
            self.save_mcu_list()
            return True
        return False

class MainApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Main.h Generator Pro")
        self.root.geometry("700x600")
        self.root.configure(bg="#2c3e50")
        self.root.resizable(True, True)
        self.last_saved_path = None

        
        self.mcu_manager = MCUManager()
        self.setup_ui()
        
    def setup_ui(self):
        # Main container
        self.main_frame = tk.Frame(self.root, bg="#2c3e50")
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Header
        self.header_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.header_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.logo_label = tk.Label(
            self.header_frame, 
            text="⚙️", 
            font=("Segoe UI", 24), 
            bg="#2c3e50", 
            fg="#3498db"
        )
        self.logo_label.pack(side=tk.LEFT, padx=(0, 10))
        
        self.title_label = tk.Label(
            self.header_frame, 
            text="Main.h Generator Pro", 
            font=("Segoe UI", 18, "bold"), 
            bg="#2c3e50", 
            fg="#ecf0f1"
        )
        self.title_label.pack(side=tk.LEFT)
        
        # Settings frame
        self.settings_frame = tk.LabelFrame(
            self.main_frame, 
            text=" Settings ", 
            font=("Segoe UI", 10, "bold"), 
            bg="#34495e", 
            fg="#ecf0f1",
            relief=tk.GROOVE,
            bd=2
        )
        self.settings_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Microcontroller selection
        self.mcu_label = tk.Label(
            self.settings_frame, 
            text="Select Microcontroller:", 
            bg="#34495e", 
            fg="#ecf0f1",
            font=("Segoe UI", 10)
        )
        self.mcu_label.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        
        self.mcu_combobox = ttk.Combobox(
            self.settings_frame, 
            values=self.mcu_manager.mcu_list, 
            font=("Segoe UI", 10), 
            width=30
        )
        self.mcu_combobox.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        if self.mcu_manager.mcu_list:
            self.mcu_combobox.current(0)
        
        # Add "Custom..." option to combobox
        self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")
        
        # Entry for custom MCU
        self.custom_mcu_frame = tk.Frame(self.settings_frame, bg="#34495e")
        self.custom_mcu_label = tk.Label(
            self.custom_mcu_frame, 
            text="Enter New MCU:", 
            bg="#34495e", 
            fg="#ecf0f1",
            font=("Segoe UI", 10)
        )
        self.custom_mcu_label.pack(side=tk.LEFT, padx=(0, 5))
        
        self.custom_mcu_entry = tk.Entry(
            self.custom_mcu_frame, 
            bg="#2c3e50", 
            fg="#ecf0f1",
            insertbackground="white",
            font=("Segoe UI", 10),
            width=28
        )
        self.custom_mcu_entry.pack(side=tk.LEFT)
        self.custom_mcu_frame.grid(row=1, column=0, columnspan=2, pady=(5, 0), sticky=tk.W)
        self.custom_mcu_frame.grid_remove()  # Hidden by default
        
        # Bind combobox selection event
        self.mcu_combobox.bind("<<ComboboxSelected>>", self.on_mcu_selected)
        
        
        # Buttons frame
        self.buttons_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.buttons_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.generate_btn = tk.Button(
            self.buttons_frame, 
            text="Generate main.h", 
            command=self.on_generate_click,
            bg="#3498db",
            fg="white",
            activebackground="#2980b9",
            activeforeground="white",
            font=("Segoe UI", 10, "bold"),
            bd=0,
            padx=15,
            pady=8,
            relief=tk.FLAT
        )
        self.generate_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.open_btn = tk.Button(
            self.buttons_frame, 
            text="Open Generated File", 
            command=self.open_generated_file,
            bg="#2ecc71",
            fg="white",
            activebackground="#27ae60",
            activeforeground="white",
            font=("Segoe UI", 10),
            bd=0,
            padx=15,
            pady=8,
            relief=tk.FLAT,
            state=tk.DISABLED
        )
        self.open_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        self.copy_btn = tk.Button(
            self.buttons_frame, 
            text="Copy to Clipboard", 
            command=self.copy_to_clipboard,
            bg="#9b59b6",
            fg="white",
            activebackground="#8e44ad",
            activeforeground="white",
            font=("Segoe UI", 10),
            bd=0,
            padx=15,
            pady=8,
            relief=tk.FLAT,
            state=tk.DISABLED
        )
        self.copy_btn.pack(side=tk.LEFT)
        
        # Progress bar
        self.progress_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.progress_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.progress_label = tk.Label(
            self.progress_frame, 
            text="Ready", 
            bg="#2c3e50", 
            fg="#bdc3c7",
            font=("Segoe UI", 9)
        )
        self.progress_label.pack(side=tk.LEFT)
        
        self.progress_bar = ttk.Progressbar(
            self.progress_frame, 
            orient=tk.HORIZONTAL, 
            length=100, 
            mode='determinate'
        )
        self.progress_bar.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        
        # Preview frame
        self.preview_frame = tk.LabelFrame(
            self.main_frame, 
            text=" Code Preview ", 
            font=("Segoe UI", 10, "bold"), 
            bg="#34495e", 
            fg="#ecf0f1",
            relief=tk.GROOVE,
            bd=2
        )
        self.preview_frame.pack(fill=tk.BOTH, expand=True)
        
        self.preview_text = ScrolledText(
            self.preview_frame, 
            bg="#1e272e", 
            fg="#ecf0f1", 
            insertbackground="white",
            font=("Consolas", 10),
            wrap=tk.NONE,
            padx=10,
            pady=10
        )
        self.preview_text.pack(fill=tk.BOTH, expand=True)
        
        # Status bar
        self.status_bar = tk.Label(
            self.root, 
            text="Ready", 
            bd=1, 
            relief=tk.SUNKEN, 
            anchor=tk.W,
            bg="#34495e",
            fg="#bdc3c7",
            font=("Segoe UI", 9)
        )
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Configure tags for syntax highlighting
        self.preview_text.tag_configure("comment", foreground="#7f8c8d")
        self.preview_text.tag_configure("directive", foreground="#3498db")
        self.preview_text.tag_configure("keyword", foreground="#e74c3c")
        self.preview_text.tag_configure("type", foreground="#2ecc71")
        
        # Set theme for ttk widgets
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure("TCombobox", fieldbackground="#34495e", background="#34495e", foreground="white")
        self.style.configure("Horizontal.TProgressbar", troughcolor="#34495e", background="#3498db")
        
    def on_mcu_selected(self, event):
        """Handle MCU selection from combobox"""
        selected = self.mcu_combobox.get()
        if selected == "Custom...":
            self.custom_mcu_frame.grid()
            self.custom_mcu_entry.focus()
        else:
            self.custom_mcu_frame.grid_remove()
            
    def on_generate_click(self):
        """Handle generate button click with custom MCU support"""
        selected = self.mcu_combobox.get()
        
        if selected == "Custom...":
            mcu_name = self.custom_mcu_entry.get().strip()
            if not mcu_name:
                messagebox.showerror("Input Error", "Please enter a microcontroller name.")
                return
                
            # Add the new MCU to the list
            if self.mcu_manager.add_mcu(mcu_name):
                messagebox.showinfo("Success", f"Added new MCU: {mcu_name}")
                # Update combobox values
                self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")
                self.mcu_combobox.set(mcu_name)
        else:
            mcu_name = selected
            
        if not mcu_name:
            messagebox.showerror("Input Error", "Please select a microcontroller.")
            return
            
        # Disable buttons during generation
        self.generate_btn.config(state=tk.DISABLED)
        self.open_btn.config(state=tk.DISABLED)
        self.copy_btn.config(state=tk.DISABLED)
        self.preview_text.delete(1.0, tk.END)
        
        def generation_complete(result):
            success, message, code = result
            self.generate_btn.config(state=tk.NORMAL)
            
            if success:
                messagebox.showinfo("Success", message)
                self.open_btn.config(state=tk.NORMAL)
                self.copy_btn.config(state=tk.NORMAL)
                self.display_code(code)
                self.status_bar.config(text="Generation complete")
            else:
                messagebox.showerror("Error", message)
                self.status_bar.config(text="Error during generation")
            
        threading.Thread(
            target=self.run_generation, 
            args=(mcu_name, generation_complete), 
            daemon=True
        ).start()
        
    def run_generation(self, mcu_name, callback):
        def progress_callback(message, value):
            self.root.after(0, self.update_progress, message, value)
            
        try:
            current_date = datetime.datetime.now().strftime("%Y-%m-%d")
            prompt = f"""You are an embedded C engineer. Given only the name of a microcontroller, generate a minimal and clean `main.h` file that includes only the most important and commonly used elements in embedded projects for that chip.

The header file should include:

1. A clean professional comment block at the top with:
   - File name
   - Short description
   - Author
   - Device name
   - Creation date: {current_date}  <!-- This will use today's date -->
   - Copyright

2. Only the **necessary standard and MCU-specific includes**.

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
   - If available: macros like DI, EI or Global_Int_Disable, Global_Int_Enable (depending on the microcontroller's naming convention for global interrupt control), `HALT`, `NOP`, etc.

6. SAFEGUARD MACROS — IMPORTANT:
   - You must **generate fully implemented versions** of the following two macros:
     - `GPIO_SAFEGUARD_Init()` → Should configure the actual GPIO direction registers (e.g., PM0 to PM14 or equivalent) and set them to input or other safe defaults.
     - `Registers_SAFEGUARD_Init()` → Should disable unused peripherals (e.g., ADC, I2C, timers, watchdogs) by clearing registers like PMCx, PERx, etc.
   - DO NOT leave the implementation to the user.
   - Use **real registers and values** based on the given microcontroller. Avoid writing comments like "User: add implementation".
   - If unsure, assume safe defaults: input mode for all GPIOs, disable analog options, stop unused peripheral clocks.
   - Format macros using `do {{ ... }} while(0)` style.

Microcontroller: **{mcu_name}**

Output: A clean, production-ready `main.h` file."""

            progress_callback("Generating header file...", 25)
            
            response = model.generate_content(prompt)
            raw_output = response.text

            progress_callback("Processing response...", 50)
            
            # Clean output
            clean_output = re.sub(r"<think>.*?</think>", "", raw_output, flags=re.DOTALL)
            code_blocks = re.findall(r"```(?:c|C)?\n(.*?)```", clean_output, flags=re.DOTALL)

            if code_blocks:
                final_code = code_blocks[0].strip()
                guard = f"{mcu_name.upper()}_MAIN_H_"

                # Replace existing guard if present
                final_code = re.sub(
                    r'#ifndef\s+\w+\s*[\r\n]+#define\s+\w+',
                    f'#ifndef {guard}\n#define {guard}',
                    final_code,
                    count=1,
                    flags=re.IGNORECASE
                )
            else:
                lines = clean_output.splitlines()
                code_lines = [line for line in lines if line.strip()]
                final_code = "\n".join(code_lines).strip()

            progress_callback("Saving file...", 75)
            
            # Save file with MCU-specific name
            output_file = f"{mcu_name}_MAIN.h"
            progress_callback("Saving file...", 75)

            # 🧠 Ask user where to save the file
            default_filename = f"{mcu_name}_MAIN.h"
            file_path = filedialog.asksaveasfilename(
                defaultextension=".h",
                initialfile=default_filename,
                filetypes=[("Header Files", "*.h"), ("All Files", "*.*")]
            )

            if not file_path:
                self.root.after(0, callback, (False, "File save canceled.", None))
                return

            with open(file_path, "w", encoding="utf-8") as file:
                file.write(final_code)
            self.last_saved_path = file_path 
            progress_callback("Done!", 100)

            self.root.after(0, callback, (True, f"'{os.path.basename(file_path)}' saved successfully at:\n{file_path}", final_code))
        except Exception as e:
            self.root.after(0, callback, (False, f"Something went wrong:\n{str(e)}", None))
    
    def update_progress(self, message, value):
        self.progress_label.config(text=message)
        self.progress_bar['value'] = value
        self.root.update_idletasks()
        
    def display_code(self, code):
        self.preview_text.delete(1.0, tk.END)
        self.preview_text.insert(tk.END, code)
        self.apply_syntax_highlighting()
        
    def apply_syntax_highlighting(self):
        text = self.preview_text.get("1.0", tk.END)
        
        for tag in self.preview_text.tag_names():
            self.preview_text.tag_remove(tag, "1.0", tk.END)
        
        lines = text.split('\n')
        for i, line in enumerate(lines, 1):
            if line.strip().startswith('//') or line.strip().startswith('/*'):
                self.preview_text.tag_add("comment", f"{i}.0", f"{i}.end")
            elif line.strip().startswith('#'):
                self.preview_text.tag_add("directive", f"{i}.0", f"{i}.end")
            elif any(word in line for word in ['void', 'return', 'if', 'else', 'while', 'for', 'switch', 'case', 'break']):
                self.preview_text.tag_add("keyword", f"{i}.0", f"{i}.end")
            elif any(word in line for word in ['int', 'char', 'float', 'double', 'uint8_t', 'uint16_t', 'uint32_t']):
                self.preview_text.tag_add("type", f"{i}.0", f"{i}.end")
                
    def open_generated_file(self):
        if self.last_saved_path and os.path.exists(self.last_saved_path):
            webbrowser.open(self.last_saved_path)
        else:
            messagebox.showerror("Error", "No generated file to open. Please generate a header file first.")

            
    def copy_to_clipboard(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.preview_text.get(1.0, tk.END))
        self.status_bar.config(text="Code copied to clipboard!")

if __name__ == "__main__":
    root = tk.Tk()
    app = MainApp(root)
    root.mainloop()