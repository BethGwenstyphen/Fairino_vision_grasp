#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox
import xml.etree.ElementTree as ET
import os

# 锁定战术图纸绝对路径
XML_PATH = os.path.expanduser("~/frc_ws/src/bt_grasp_brain/behavior_trees/main_task.xml")

class FairinoXMLViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("Fairino 战术图纸全量编辑器 (XML Editor)")
        self.root.geometry("1000x650") # 加宽窗口，容纳 6 位长数组
        self.root.resizable(False, False)
        
        self.entries = {}
        
        self.config = {
            "1. 绝对关节定点 (Joint Targets)": {
                "attrs": ["target", "speed", "tol"],
                "nodes": [
                    {"name": "MoveToObserve", "desc": "初始观察与复位点"},
                    {"name": "MoveToLidDrop", "desc": "盖子放置/取回暂存区"},
                    {"name": "MoveToLidDropRetreat", "desc": "盖子暂存区安全避让点"},
                    {"name": "MoveToGunGrab", "desc": "注液枪抓取与还枪架子"},
                    {"name": "MoveToPreInject", "desc": "注液前高空预备姿态"}
                ]
            },
            "2. 相对偏置动作 (Cartesian Offsets)": {
                "attrs": ["offset_x", "offset_y", "offset_z", "speed", "tol"],
                "nodes": [
                    {"name": "MoveToLidApproach", "desc": "抓盖前上方悬停偏移"},
                    {"name": "MoveToLidGrasp", "desc": "抓盖向下直插动作"},
                    {"name": "MoveRelativeZUp150", "desc": "通用安全拔高离场"},
                    {"name": "MoveToArUcoHover", "desc": "ArUco注液孔上方悬停"},
                    {"name": "MoveToArUcoInject", "desc": "注液枪口向下直插"}
                ]
            },
            "3. 系统杂项 (Misc & Delays)": {
                "attrs": ["delay_msec"],
                "nodes": [
                    {"tag": "Delay", "name": "Delay", "desc": "注液核心死等时间 (毫秒)"}
                ]
            }
        }
        
        self.setup_ui()
        self.load_xml()

    def setup_ui(self):
        """完全重写：基于 Grid 网格引擎的工业级极严谨对齐"""
        # 顶部标题
        header_frame = tk.Frame(self.root, bg="#2c3e50", pady=15)
        header_frame.pack(fill="x")
        tk.Label(header_frame, text="XML 战术参数全量配置中心", font=("Arial", 16, "bold"), fg="white", bg="#2c3e50").pack()

        # 核心 Tab 容器
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(expand=True, fill="both", padx=20, pady=15)

        # 动态渲染每个 Tab
        for tab_title, tab_data in self.config.items():
            frame = ttk.Frame(self.notebook, padding=10)
            self.notebook.add(frame, text=f"  {tab_title}  ")
            
            # 渲染表头 (Grid Row 0)
            tk.Label(frame, text="动作节点 (Action / Desc)", font=("Arial", 11, "bold"), anchor="w").grid(row=0, column=0, sticky="w", padx=10, pady=10)
            
            for col_idx, attr in enumerate(tab_data["attrs"]):
                tk.Label(frame, text=attr.upper(), font=("Arial", 10, "bold"), fg="#c0392b").grid(row=0, column=col_idx+1, padx=5, pady=10)

            # 渲染数据行 (Grid Row 1~N)
            for row_idx, node_info in enumerate(tab_data["nodes"]):
                actual_row = row_idx + 1
                node_name = node_info["name"]
                self.entries[node_name] = {}
                
                # 第一列：节点名与描述
                lbl_frame = tk.Frame(frame)
                lbl_frame.grid(row=actual_row, column=0, sticky="w", padx=10, pady=8)
                tk.Label(lbl_frame, text=node_name, font=("Arial", 11, "bold"), fg="#2980b9", anchor="w").pack(fill="x")
                tk.Label(lbl_frame, text=node_info["desc"], font=("Arial", 9), fg="gray", anchor="w").pack(fill="x")
                
                # 后续列：输入框
                for col_idx, attr in enumerate(tab_data["attrs"]):
                    # 针对超长的 target 数组，强行分配 38 字符的宽度
                    width = 38 if attr == "target" else 12 
                    entry = ttk.Entry(frame, width=width, font=("Consolas", 11), justify="center")
                    entry.grid(row=actual_row, column=col_idx+1, padx=10, pady=8)
                    self.entries[node_name][attr] = entry

        # 底部保存按钮
        bottom_frame = tk.Frame(self.root)
        bottom_frame.pack(fill="x", pady=15)
        
        self.save_btn = tk.Button(bottom_frame, text="💾 覆写保存至 XML 图纸", font=("Arial", 13, "bold"), 
                                  bg="#27ae60", fg="white", padx=30, pady=8, command=self.save_xml)
        self.save_btn.pack()

    def load_xml(self):
        if not os.path.exists(XML_PATH):
            messagebox.showerror("文件丢失", f"找不到 XML 文件:\n{XML_PATH}")
            return
        try:
            tree = ET.parse(XML_PATH)
            root = tree.getroot()
            for node in root.findall(".//*"):
                if "name" in node.attrib:
                    name = node.attrib["name"]
                    if name in self.entries:
                        for attr, entry_widget in self.entries[name].items():
                            if attr in node.attrib:
                                entry_widget.delete(0, tk.END)
                                entry_widget.insert(0, node.attrib[attr])
                
                if node.tag == "Delay" and "Delay" in self.entries:
                    if "delay_msec" in node.attrib:
                        self.entries["Delay"]["delay_msec"].delete(0, tk.END)
                        self.entries["Delay"]["delay_msec"].insert(0, node.attrib["delay_msec"])
        except Exception as e:
            messagebox.showerror("解析失败", f"XML 解析出错:\n{str(e)}")

    def save_xml(self):
        try:
            tree = ET.parse(XML_PATH)
            root = tree.getroot()
            update_count = 0
            
            for node in root.findall(".//*"):
                if "name" in node.attrib:
                    name = node.attrib["name"]
                    if name in self.entries:
                        for attr, entry_widget in self.entries[name].items():
                            new_val = entry_widget.get().strip()
                            if new_val:
                                node.attrib[attr] = new_val
                        update_count += 1
                
                if node.tag == "Delay" and "Delay" in self.entries:
                    new_val = self.entries["Delay"]["delay_msec"].get().strip()
                    if new_val:
                        node.attrib["delay_msec"] = new_val
                        update_count += 1
            
            tree.write(XML_PATH, encoding="utf-8", xml_declaration=False)
            messagebox.showinfo("保存成功", f"✅ 战术参数已成功覆写至 XML！\n共同步更新了 {update_count} 个底层动作节点。")
        except Exception as e:
            messagebox.showerror("保存失败", f"写入失败:\n{str(e)}")

if __name__ == "__main__":
    root_win = tk.Tk()
    style = ttk.Style()
    if "clam" in style.theme_names():
        style.theme_use("clam")
    app = FairinoXMLViewer(root_win)
    root_win.mainloop()