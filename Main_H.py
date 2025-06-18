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

# Support for PyInstaller .exe path
BASE_PATH = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
MCU_DATA_FILE = os.path.join(BASE_PATH, "mcu_data.json")
TOKEN_FILE = os.path.join(BASE_PATH, "github_token.txt")

# Load GitHub token
with open(TOKEN_FILE, "r") as f:
    GITHUB_TOKEN = f.read().strip()

GITHUB_USERNAME = "ELARABY-AI-Team"
REPO_NAME = "Main_Generator"
HEADERS_FOLDER = "Main_files"

genai.configure(api_key=os.getenv('GEMINI_API_KEY', 'AIzaSyB4kW-XsrZSJOkekbkasW9M2QJ_VG0Iuzo'))
model = genai.GenerativeModel("gemini-2.5-flash-preview-04-17")

def check_github_for_header(mcu_name):
    file_path = f"{HEADERS_FOLDER}/{mcu_name}_MAIN.h"
    url = f"https://api.github.com/repos/{GITHUB_USERNAME}/{REPO_NAME}/contents/{file_path}"
    headers = {"Authorization": f"token {GITHUB_TOKEN}"}
    response = requests.get(url, headers=headers)

    if response.status_code == 200:
        data = response.json()
        return data.get("download_url")  # ✅ This is what you use to download
    else:
        print(f"❌ File not found on GitHub: {file_path} — {response.status_code}")
    return None


def download_header_from_github(url):
    response = requests.get(url)
    if response.status_code == 200:
        return response.text
    return None

def upload_header_to_github(mcu_name, code):
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

class MCUManager:
    def __init__(self):
        self.mcu_list = self.load_mcu_list()
        
    def load_mcu_list(self):
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
        self.root.title("Main.h Generator Pro")
        self.root.geometry("700x600")
        self.root.configure(bg="#2c3e50")
        self.root.resizable(True, True)
        self.last_saved_path = None

        self.mcu_manager = MCUManager()
        self.setup_ui()
        
    def setup_ui(self):
        # Configure main window for proper fullscreen
        self.root.state('zoomed')  # Maximized but with window controls
        # Alternative: self.root.attributes('-fullscreen', True) for true fullscreen
        
        # Make the window resizable
        self.root.resizable(True, True)
        
        # Main container
        self.main_frame = tk.Frame(self.root, bg="#2c3e50")
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.header_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.header_frame.pack(fill=tk.X, pady=(0, 10))

        self.logo_label = tk.Label(self.header_frame, text="⚙️", font=("Segoe UI", 24), bg="#2c3e50", fg="#3498db")
        self.logo_label.pack(side=tk.LEFT, padx=(0, 10))

        self.title_label = tk.Label(self.header_frame, text="Main.h Generator Pro", font=("Segoe UI", 18, "bold"), bg="#2c3e50", fg="#ecf0f1")
        self.title_label.pack(side=tk.LEFT)

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
        self.custom_mcu_label = tk.Label(self.custom_mcu_frame, text="Enter New MCU:", bg="#34495e", fg="#ecf0f1", font=("Segoe UI", 10))
        self.custom_mcu_label.pack(side=tk.LEFT, padx=(0, 5))
        self.custom_mcu_entry = tk.Entry(self.custom_mcu_frame, bg="#2c3e50", fg="#ecf0f1", insertbackground="white", font=("Segoe UI", 10), width=28)
        self.custom_mcu_entry.pack(side=tk.LEFT)
        self.custom_mcu_frame.grid(row=1, column=0, columnspan=2, pady=(5, 0), sticky=tk.W)
        self.custom_mcu_frame.grid_remove()
        self.mcu_combobox.bind("<<ComboboxSelected>>", self.on_mcu_selected)

        self.buttons_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.buttons_frame.pack(fill=tk.X, pady=(0, 10))
        self.generate_btn = tk.Button(self.buttons_frame, text="Generate main.h", command=self.on_generate_click, bg="#3498db", fg="white", font=("Segoe UI", 10, "bold"), bd=0, padx=15, pady=8, relief=tk.FLAT)
        self.generate_btn.pack(side=tk.LEFT, padx=(0, 10))
        self.open_btn = tk.Button(self.buttons_frame, text="Open Generated File", command=self.open_generated_file, bg="#2ecc71", fg="white", font=("Segoe UI", 10), bd=0, padx=15, pady=8, relief=tk.FLAT, state=tk.DISABLED)
        self.open_btn.pack(side=tk.LEFT, padx=(0, 10))
        self.copy_btn = tk.Button(self.buttons_frame, text="Copy to Clipboard", command=self.copy_to_clipboard, bg="#9b59b6", fg="white", font=("Segoe UI", 10), bd=0, padx=15, pady=8, relief=tk.FLAT, state=tk.DISABLED)
        self.copy_btn.pack(side=tk.LEFT)

        self.progress_frame = tk.Frame(self.main_frame, bg="#2c3e50")
        self.progress_frame.pack(fill=tk.X, pady=(0, 10))
        self.progress_label = tk.Label(self.progress_frame, text="Ready", bg="#2c3e50", fg="#bdc3c7", font=("Segoe UI", 9))
        self.progress_label.pack(side=tk.LEFT)
        self.progress_bar = ttk.Progressbar(self.progress_frame, orient=tk.HORIZONTAL, length=100, mode='determinate')
        self.progress_bar.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        self.preview_frame = tk.LabelFrame(self.main_frame, text=" Code Preview ", font=("Segoe UI", 10, "bold"), bg="#34495e", fg="#ecf0f1", relief=tk.GROOVE, bd=2)
        self.preview_frame.pack(fill=tk.BOTH, expand=True)
        self.preview_text = ScrolledText(self.preview_frame, bg="#1e272e", fg="#ecf0f1", insertbackground="white", font=("Consolas", 10), wrap=tk.NONE, padx=10, pady=10)
        self.preview_text.pack(fill=tk.BOTH, expand=True)

        self.status_bar = tk.Label(self.root, text="Ready", bd=1, relief=tk.SUNKEN, anchor=tk.W, bg="#34495e", fg="#bdc3c7", font=("Segoe UI", 9))
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
         # Footer frame for copyright
        # ✅ Container to hold both status and footer
        self.bottom_frame = tk.Frame(self.root, bg="#1a1a1a")
        self.bottom_frame.pack(side=tk.BOTTOM, fill=tk.X)

        # ✅ Status bar
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
       

        # ✅ Copyright label
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
        self.bottom_frame.pack(side=tk.BOTTOM, fill=tk.X)

        print("✅ Copyright label created.")

        
    def reset_ui(self):
        self.generate_btn.config(state=tk.NORMAL)
        self.open_btn.config(state=tk.DISABLED)
        self.copy_btn.config(state=tk.DISABLED)
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

    def on_generate_click(self):
        selected = self.mcu_combobox.get()
        mcu_name = self.custom_mcu_entry.get().strip() if selected == "Custom..." else selected
        if not mcu_name:
            messagebox.showerror("Input Error", "Please enter or select a microcontroller.")
            return
        if selected == "Custom..." and self.mcu_manager.add_mcu(mcu_name):
            self.mcu_combobox['values'] = (*self.mcu_manager.mcu_list, "Custom...")
            self.mcu_combobox.set(mcu_name)

        self.generate_btn.config(state=tk.DISABLED)
        self.open_btn.config(state=tk.DISABLED)
        self.copy_btn.config(state=tk.DISABLED)
        self.preview_text.delete(1.0, tk.END)

        def callback(result):
            success, msg, code = result
            self.status_bar.config(text=msg)
            self.generate_btn.config(state=tk.NORMAL)
            if success:
                self.open_btn.config(state=tk.NORMAL)
                self.copy_btn.config(state=tk.NORMAL)
                self.display_code(code)
            else:
                messagebox.showerror("Error", msg)

        threading.Thread(target=self.run_generation, args=(mcu_name, callback), daemon=True).start()

    def run_generation(self, mcu_name, callback):
        def progress(msg, val):
            self.progress_label.config(text=msg)
            self.progress_bar["value"] = val
            self.root.update_idletasks()

        try:
            # ✅ Step 1: Check GitHub first
            progress("Checking GitHub...", 10)
            url = check_github_for_header(mcu_name)
            if url:
                code = download_header_from_github(url)
                if code:
                    self.last_saved_path = None
                    self.preview_text.delete(1.0, tk.END)
                    self.preview_text.insert(tk.END, code)
                    self.status_bar.config(text=f"✅ Loaded existing {mcu_name}_MAIN.h from GitHub")

                    # Prompt user to save the file
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
                    else:
                        messagebox.showinfo("Canceled", "✅ File loaded but not saved.")
                    # ✅ Set progress to 100%
                    progress("Done! File loaded from GitHub.", 100)
                    self.root.after(0, callback, (True, f"{mcu_name}_MAIN.h loaded from GitHub.", code))
                    self.root.after(0, self.reset_ui)
                    return

            # ✅ Step 2: Not found — generate with Gemini
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

            progress("Uploading to GitHub...", 90)
            upload_success = upload_header_to_github(mcu_name, final_code)

            if upload_success:
                progress("✅ Done! Uploaded to GitHub.", 100)
                messagebox.showinfo("Upload Successful", f"✅ {mcu_name}_MAIN.h uploaded to GitHub.")
            else:
                progress("⚠️ Done (upload failed)", 100)
                messagebox.showwarning("Upload Failed", f"⚠️ Upload to GitHub failed for {mcu_name}_MAIN.h.")

            self.root.after(0, callback, (True, f"'{os.path.basename(file_path)}' saved and upload attempted.", final_code))
            self.root.after(0, self.reset_ui)


        except Exception as e:
            self.root.after(0, callback, (False, f"Something went wrong:\n{str(e)}", None))


    def display_code(self, code):
        self.preview_text.delete(1.0, tk.END)
        self.preview_text.insert(tk.END, code)

    def open_generated_file(self):
        if self.last_saved_path and os.path.exists(self.last_saved_path):
            webbrowser.open(self.last_saved_path)

    def copy_to_clipboard(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.preview_text.get(1.0, tk.END))
        self.status_bar.config(text="Copied!")

if __name__ == "__main__":
    root = tk.Tk()
    app = MainApp(root)
    root.mainloop()
