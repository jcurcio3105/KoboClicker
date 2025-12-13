import os
import sys
import csv
import uuid
import io
import time
from datetime import datetime, timedelta
from zoneinfo import ZoneInfo
import xml.etree.ElementTree as ET
from urllib.parse import urlparse
import requests
import re


import customtkinter as ctk
from tkinter import filedialog, messagebox
from PIL import Image, ImageTk


# -----------------------------
# Millisecond → ISO8601 conversion
# -----------------------------
LOCAL_TZ = ZoneInfo("America/New_York")


def convert_ms_to_iso8601_local(ms_val: int, base_ms: int, start_datetime_str: str) -> str:
   anchor_dt = datetime.strptime(start_datetime_str, "%m/%d/%Y %H:%M:%S")
   anchor_dt = anchor_dt.replace(tzinfo=LOCAL_TZ)
   offset_ms = ms_val - base_ms
   final_dt = anchor_dt + timedelta(milliseconds=offset_ms)
   return final_dt.isoformat(timespec="seconds")


# -----------------------
# CONSTANTS
# -----------------------
NSMAP = {
   "h": "http://www.w3.org/1999/xhtml",
   "xsd": "http://www.w3.org/2001/XMLSchema",
   "jr": "http://openrosa.org/javarosa",
   "ev": "http://www.w3.org/2001/xml-events",
   "orx": "http://openrosa.org/xforms",
   "odk": "http://www.opendatakit.org/xforms",
}


CSV_COL_TIME = "time"
CSV_BUTTONS = ["button1", "button2", "button3", "button4", "button5"]


EXCLUDED_NAMES = {
   "start", "end", "__version__", "meta/instanceid", "meta",
   "_submission_time", "_submitted_by", "_id", "_uuid", "validation",
   "version", "id", "uuid"
}


# -----------------------
# HELPERS
# -----------------------
def normalize_header(h):
   if not h:
       return ""
   s = str(h).strip().lower()
   s = re.sub(r'[^a-z0-9]+', '', s)
   return s


def derive_kf_base_from_link(link: str) -> str:
   host = urlparse(link).netloc.lower()
   if "kf-eu." in host:
       return "https://kf-eu.kobotoolbox.org"
   return "https://kf.kobotoolbox.org"


def parse_form_id_from_link(link: str) -> str:
   needle = "/forms/"
   idx = link.find(needle)
   if idx == -1:
       raise ValueError("Could not find '/forms/' in link.")
   tail = link[idx + len(needle):]
   for sep in ["/", "?", "#"]:
       cut = tail.find(sep)
       if cut != -1:
           tail = tail[:cut]
   if not tail:
       raise ValueError("Form ID segment was empty.")
   return tail


def get_field_name(item: dict) -> str:
   name = (item.get("name") or "").strip()
   if not name:
       name = (item.get("$autoname") or "").strip()
   if not name:
       xpath = (item.get("$xpath") or "").strip()
       if xpath:
           name = xpath.split("/")[-1].strip()
   return name


def get_field_label(item: dict) -> str:
   label = item.get("label", "")
   if isinstance(label, list) and label:
       first = label[0]
       if isinstance(first, dict):
           return (first.get("text") or "").strip()
       return str(first).strip()
   if isinstance(label, dict):
       return str(next(iter(label.values()), "")).strip()
   return str(label).strip()


def kf_get_custom_fields(kf_base: str, form_uid: str, username: str, password: str):
   url = f"{kf_base}/api/v2/assets/{form_uid}/?format=json"
   r = requests.get(url, auth=(username, password), timeout=60)
   if r.status_code != 200:
       raise RuntimeError(f"Failed to fetch form definition ({r.status_code}): {r.text}")
   data = r.json()
   survey = (data.get("content") or {}).get("survey") or data.get("survey")
   if not isinstance(survey, list):
       raise RuntimeError("Form definition did not include a 'survey' array.")


   items = []
   for q in survey:
       if not isinstance(q, dict):
           continue
       qtype = (q.get("type") or "").strip().lower()
       if qtype in {"begin_group", "end_group", "begin_repeat", "end_repeat", "note"}:
           continue
       name = get_field_name(q)
       if not name or name.lower() in EXCLUDED_NAMES:
           continue
       label_text = get_field_label(q)
       items.append({"name": name, "label": label_text, "type": qtype})
   if len(items) > 5:
       raise RuntimeError(f"Form has {len(items)} custom fields (max 5).")
   return items


def build_instance_xml_dynamic(root_tag, form_id_attr, start_iso, end_iso,
                              mapped_values: dict, instance_uuid_str: str):
   for prefix, uri in NSMAP.items():
       ET.register_namespace(prefix, uri)
   root = ET.Element(
       root_tag,
       {
           "id": form_id_attr,
           "version": "1",
           "xmlns:h": NSMAP["h"],
           "xmlns:xsd": NSMAP["xsd"],
           "xmlns:jr": NSMAP["jr"],
           "xmlns:ev": NSMAP["ev"],
           "xmlns:orx": NSMAP["orx"],
           "xmlns:odk": NSMAP["odk"],
       },
   )
   formhub = ET.SubElement(root, "formhub")
   ET.SubElement(formhub, "uuid").text = form_id_attr
   ET.SubElement(root, "start").text = start_iso
   ET.SubElement(root, "end").text = end_iso
   for field_name, val in mapped_values.items():
       if field_name:
           ET.SubElement(root, field_name).text = "" if val is None else str(val)
   ET.SubElement(root, "__version__").text = "1"
   meta = ET.SubElement(root, "meta")
   ET.SubElement(meta, "instanceID").text = f"uuid:{instance_uuid_str}"
   return ET.tostring(root, encoding="UTF-8", xml_declaration=True, method="xml")


def submit_xml(session, submit_url, xml_bytes, display_name="submission.xml"):
   files = {"xml_submission_file": (display_name, io.BytesIO(xml_bytes), "text/xml")}
   return session.post(submit_url, files=files, timeout=60)


# -----------------------
# UPLOAD LOGIC
# -----------------------
def run_upload_dynamic(username, password, survey_link, csv_path, output_root, mapping, start_datetime_str):
   form_id = parse_form_id_from_link(survey_link)
   kc_base = derive_kf_base_from_link(survey_link)
   submit_url = f"{kc_base}/submission"
   os.makedirs(output_root, exist_ok=True)


   session = requests.Session()
   session.auth = (username, password)
   session.headers.update({"User-Agent": "kobo-bulk-uploader/2.0", "X-OpenRosa-Version": "1.0"})


   with open(csv_path, "r", newline="", encoding="utf-8-sig") as f:
       reader = csv.DictReader(f)
       if not reader.fieldnames:
           raise ValueError("CSV has no header row.")
       rows = [{normalize_header(k): (v or "").strip() for k, v in row.items()} for row in reader]


   base_ms = None
   results = []


   for idx, row in enumerate(rows, start=1):
       # Handle timestamp
       time_str = row.get(CSV_COL_TIME, "")
       if not time_str or not time_str.isdigit():
           results.append(f"[SKIP] Row {idx}: invalid or missing ms timestamp")
           continue
       ms_val = int(time_str)
       if base_ms is None:
           base_ms = ms_val
       iso_time = convert_ms_to_iso8601_local(ms_val, base_ms, start_datetime_str)


       # Map fields
       mapped_values = {}
       for field_name, choice in mapping.items():
           if choice == "(Use CSV Time Column)":
               mapped_values[field_name] = iso_time
           else:
               mapped_values[field_name] = row.get(normalize_header(choice), "")


       # Build XML
       inst_uuid = str(uuid.uuid4())
       xml_bytes = build_instance_xml_dynamic(
           root_tag=form_id,
           form_id_attr=form_id,
           start_iso=iso_time,
           end_iso=iso_time,
           mapped_values=mapped_values,
           instance_uuid_str=inst_uuid
       )


       # Save locally
       folder = os.path.join(output_root, f"instance{idx}")
       os.makedirs(folder, exist_ok=True)
       file_path = os.path.join(folder, f"instance{idx}.xml")
       with open(file_path, "wb") as xf:
           xf.write(xml_bytes)


       # Submit
       try:
           r = submit_xml(session, submit_url, xml_bytes, display_name=os.path.basename(file_path))
           results.append(f"{file_path}: {r.status_code}\n{r.text.strip()}\n{'-'*60}")
       except Exception as e:
           results.append(f"{file_path}: ERROR submitting: {e}\n{'-'*60}")


       time.sleep(0.2)


   return "\n".join(results)


# -----------------------
# GUI
# -----------------------
ctk.set_appearance_mode("light")
ctk.set_default_color_theme("blue")


def popup_credentials(on_success):
    app = ctk.CTk()
    app.title("Upload to KoboToolbox")
    app.geometry("500x520")
    app.resizable(False, False)
    app.configure(fg_color="white")

    frame = ctk.CTkFrame(app, corner_radius=14, fg_color="white", border_width=1, border_color="#E0E0E0")
    frame.pack(pady=20, padx=20, fill="both", expand=True)

    # Logo
    BASE_DIR = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    logo_path = os.path.join(BASE_DIR, "KoboToolbox_log.jpg")
    logo_image = Image.open(logo_path)
    logo_image.thumbnail((260, 55), Image.Resampling.LANCZOS)
    logo_photo = ImageTk.PhotoImage(logo_image)
    ctk.CTkLabel(frame, image=logo_photo, text="").pack(pady=(10, 15))

    ctk.CTkLabel(frame, text="Upload to KoboToolbox", text_color="black",
                 font=ctk.CTkFont(size=18, weight="bold")).pack(pady=(5, 18))

    # Helper to create labeled entry
    def create_labeled_entry(parent, label_text, text_var=None, show=None):
        row = ctk.CTkFrame(parent, fg_color="white")
        row.pack(fill="x", pady=6, padx=10)
        ctk.CTkLabel(row, text=label_text, width=90, anchor="w").pack(side="left")
        entry = ctk.CTkEntry(row, width=280, textvariable=text_var, show=show)
        entry.pack(side="left", padx=5)
        return entry

    # Variables
    username_var = ctk.StringVar()
    password_var = ctk.StringVar()
    link_var = ctk.StringVar()
    csv_var = ctk.StringVar()
    output_var = ctk.StringVar(value="instances")
    startdate_var = ctk.StringVar()

    # Input rows
    create_labeled_entry(frame, "Username:", username_var)
    create_labeled_entry(frame, "Password:", password_var, show="*")
    create_labeled_entry(frame, "Survey Link:", link_var)

    # CSV selector row
    csv_row = ctk.CTkFrame(frame, fg_color="white")
    csv_row.pack(pady=10, fill="x", padx=10)
    ctk.CTkLabel(csv_row, text="CSV File:", width=90, anchor="w").pack(side="left")
    ctk.CTkEntry(csv_row, width=200, textvariable=csv_var).pack(side="left", padx=(5, 8))

    def choose_csv_local():
        path = filedialog.askopenfilename(title="Select CSV file", filetypes=[("CSV files", "*.csv")])
        if path:
            csv_var.set(path)

    ctk.CTkButton(csv_row, text="Browse…", width=70, command=choose_csv_local).pack(side="left")

    # Output folder row
    output_row = ctk.CTkFrame(frame, fg_color="white")
    output_row.pack(pady=6, fill="x", padx=10)
    ctk.CTkLabel(output_row, text="Output Folder:", width=90, anchor="w").pack(side="left")
    ctk.CTkEntry(output_row, width=280, textvariable=output_var).pack(side="left", padx=5)

    # Start Date row
    start_row = ctk.CTkFrame(frame, fg_color="white")
    start_row.pack(fill="x", pady=6, padx=10)
    ctk.CTkLabel(start_row, text="Start Date:", width=90, anchor="w").pack(side="left")
    start_entry = ctk.CTkEntry(start_row, width=280, textvariable=startdate_var, fg_color="white", text_color="black")
    start_entry.pack(side="left", padx=(5,8))

    # Placeholder text
    placeholder = "MM/DD/YYYY HH:MM:SS"
    start_entry.insert(0, placeholder)
    start_entry.configure(text_color="grey")

    # Remove placeholder on focus
    def on_focus_in(event):
        if start_entry.get() == placeholder:
            start_entry.delete(0, "end")
            start_entry.configure(text_color="black")

    # Restore placeholder if empty on focus out
    def on_focus_out(event):
        if not start_entry.get():
            start_entry.insert(0, placeholder)
            start_entry.configure(text_color="grey")

    start_entry.bind("<FocusIn>", on_focus_in)
    start_entry.bind("<FocusOut>", on_focus_out)

    # NEXT button
    def start_next():
        username = username_var.get().strip()
        password = password_var.get().strip()
        link = link_var.get().strip()
        csv_path = csv_var.get().strip()
        output_root = output_var.get().strip()
        start_dt = startdate_var.get().strip()

        if not all([username, password, link, csv_path, output_root, start_dt]):
            messagebox.showerror("Missing Information", "All fields are required.")
            return

        # Validate start date format
        try:
            datetime.strptime(start_dt, "%m/%d/%Y %H:%M:%S")
        except ValueError:
            messagebox.showerror("Invalid Date", "Start Date must be in format MM/DD/YYYY HH:MM:SS")
            return

        try:
            form_uid = parse_form_id_from_link(link)
            kf_base = derive_kf_base_from_link(link)
            fields = kf_get_custom_fields(kf_base, form_uid, username, password)
        except Exception as e:
            messagebox.showerror("Error", str(e))
            return

        app.destroy()
        on_success(username, password, link, csv_path, output_root, fields, start_dt)

    ctk.CTkButton(frame, text="Next", width=200, command=start_next).pack(pady=(20, 10))

    app.mainloop()


# -----------------------
# Mapping GUI
# -----------------------
def popup_mapping(username, password, link, csv_path, output_root, custom_fields, start_datetime_str):
   win = ctk.CTk()
   win.title("Map CSV Columns to Form Fields")
   win.geometry("540x650")
   win.configure(fg_color="white")


   scroll = ctk.CTkScrollableFrame(win, fg_color="white", width=680, height=250)
   scroll.pack(padx=20, pady=15)


   ctk.CTkLabel(scroll, text="Assign CSV columns to form fields",
                text_color="black", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(0, 10))


   var_by_field = {}


   # Include "(Use CSV Time Column)" as an option
   choices = CSV_BUTTONS + [ "(Use CSV Time Column)"]


   for fld in custom_fields:
       container = ctk.CTkFrame(scroll, fg_color="white")
       container.pack(fill="x", pady=4, padx=5)


       label_text = fld.get("label") or fld["name"]
       ctk.CTkLabel(container, text=label_text, text_color="black",
                    width=200, anchor="w").pack(side="left", padx=6)


       var = ctk.StringVar(value=choices[0])
       combo = ctk.CTkComboBox(container, values=choices, variable=var, width=200)
       combo.pack(side="left", padx=6)
       var_by_field[fld["name"]] = var


   output_box = ctk.CTkTextbox(win, width=680, height=230)
   output_box.pack(padx=20, pady=10)


   def do_upload():
       mapping = {}
       selected = []
       for field, var in var_by_field.items():
           val = var.get()
           if val not in ["(Unused)"]:
               mapping[field] = val
               selected.append(val)


       if len(set(selected)) != len(selected):
           messagebox.showerror("Invalid Mapping", "Each CSV column may only be used once.")
           return


       output_box.delete("0.0", "end")
       output_box.insert("0.0", "Starting upload...\n\n")
       win.update_idletasks()


       try:
           results = run_upload_dynamic(
               username, password, link, csv_path, output_root, mapping, start_datetime_str
           )
           output_box.insert("end", results + "\n")
           messagebox.showinfo("Done", "Upload completed.")
       except Exception as e:
           output_box.insert("end", f"ERROR: {e}\n")
           messagebox.showerror("Upload Failed", str(e))


   ctk.CTkButton(win, text="Upload", width=200, command=do_upload).pack(pady=10)
   win.mainloop()


# -----------------------
# Entry point
# -----------------------
if __name__ == "__main__":
   popup_credentials(on_success=popup_mapping)



