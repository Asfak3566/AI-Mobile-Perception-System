import zipfile
import xml.etree.ElementTree as ET
import os
import re

docx_path = "AI_Based_Mobile_Perception system(1).docx"
tex_path = "paper.tex"

def escape_latex(text):
    if not text:
        return ""
    # Escape special chars
    # Note: Text replacement order matters
    text = text.replace('\\', r'\textbackslash{}')
    text = text.replace('{', r'\{')
    text = text.replace('}', r'\}')
    text = text.replace('_', r'\_')
    text = text.replace('^', r'\^{}')
    text = text.replace('#', r'\#')
    text = text.replace('&', r'\&')
    text = text.replace('$', r'\$')
    text = text.replace('%', r'\%')
    text = text.replace('~', r'\~{}')
    
    # Try to convert "smart quotes" to normal latex quotes if present (basic handling)
    text = text.replace('“', "``").replace('”', "''")
    text = text.replace('‘', "`").replace('’', "'")
    return text

def get_docx_content(path):
    content = []
    if not os.path.exists(path):
        return [("Error", "File not found.")]
    
    namespaces = {
        'w': 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'
    }

    try:
        with zipfile.ZipFile(path) as document:
            xml_content = document.read('word/document.xml')
            tree = ET.fromstring(xml_content)
            
            for p in tree.iter():
                if p.tag.endswith('}p'):
                    # 1. Get Style
                    style = "Normal"
                    for pPr in p.findall(f"{{{namespaces['w']}}}pPr"):
                        for pStyle in pPr.findall(f"{{{namespaces['w']}}}pStyle"):
                            style = pStyle.attrib.get(f"{{{namespaces['w']}}}val", "Normal")
                    
                    # 2. Check for List Item
                    is_list = False
                    for pPr in p.findall(f"{{{namespaces['w']}}}pPr"):
                        if pPr.find(f"{{{namespaces['w']}}}numPr") is not None:
                            is_list = True

                    # 3. Get Text
                    para_text = ""
                    for t in p.iter():
                        if t.tag.endswith('}t'):
                            if t.text:
                                para_text += t.text
                    
                    if para_text.strip():
                        content.append({
                            "text": escape_latex(para_text),
                            "style": style,
                            "is_list": is_list
                        })
                    
        return content
    except Exception as e:
        print(f"Error: {e}")
        return []

content_nodes = get_docx_content(docx_path)

tex_output = r"""\documentclass[12pt]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage{geometry}
\geometry{a4paper, margin=1in}
\usepackage{parskip}
\usepackage{hyperref}
\usepackage{amsmath}
\usepackage{graphicx}

\title{AI Based Mobile Perception System}
\author{Team Project}
\date{\today}

\begin{document}

\maketitle
\tableofcontents
\newpage

"""

in_itemize = False

for node in content_nodes:
    text = node["text"]
    style = node["style"]
    is_list = node["is_list"]
    
    # Handle list state
    if is_list and not in_itemize:
        tex_output += r"\begin{itemize}" + "\n"
        in_itemize = True
    elif not is_list and in_itemize:
        tex_output += r"\end{itemize}" + "\n"
        in_itemize = False
        
    # Map styles to LaTeX
    if in_itemize:
        tex_output += f"    \\item {text}\n"
    else:
        if style == "Heading1":
            tex_output += f"\n\\section{{{text}}}\n"
        elif style == "Heading2":
            tex_output += f"\n\\subsection{{{text}}}\n"
        elif style == "Heading3":
            tex_output += f"\n\\subsubsection{{{text}}}\n"
        elif style == "Heading4":
            tex_output += f"\n\\paragraph{{{text}}}\n"
        elif style == "Title":
            # Skip title as we added it manually, or make it a huge text
            pass 
        else:
            tex_output += f"{text}\n\n"

if in_itemize:
    tex_output += r"\end{itemize}" + "\n"

tex_output += r"\end{document}"

with open(tex_path, "w", encoding="utf-8") as f:
    f.write(tex_output)

print(f"Generated {tex_path} with improved structure.")
