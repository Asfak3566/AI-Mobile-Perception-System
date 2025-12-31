import zipfile
import xml.etree.ElementTree as ET
import os
import re

docx_path = "AI_Based_Mobile_Perception system(1).docx"
tex_path = "paper.tex"

def escape_latex(text):
    if not text:
        return ""
    # Special characters: # $ % & ~ _ ^ \ { }
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
    return text

def get_docx_text(path):
    paragraphs = []
    if not os.path.exists(path):
        return ["Error: File not found."]
    
    try:
        with zipfile.ZipFile(path) as document:
            xml_content = document.read('word/document.xml')
            tree = ET.fromstring(xml_content)
            
            for p in tree.iter():
                if p.tag.endswith('}p'):
                    para_text = ""
                    for t in p.iter():
                        if t.tag.endswith('}t'):
                            if t.text:
                                para_text += t.text
                    if para_text.strip():
                        paragraphs.append(escape_latex(para_text))
                    
        return paragraphs
    except Exception as e:
        return [f"Error extracting text: {e}"]

paragraphs = get_docx_text(docx_path)

# Heuristic: First few lines are title/author info
# We will skip them in the body if we use them in the header, 
# or just dump everything for now to be safe.
# Let's create a robust template.

tex_content = r"""\documentclass[12pt]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage{geometry}
\geometry{a4paper, margin=1in}
\usepackage{parskip}
\usepackage{hyperref}

\title{AI Based Mobile Perception System}
\author{Team Project}
\date{\today}

\begin{document}

\maketitle

"""

for p in paragraphs:
    # simple heuristic: if it looks like a section header (short, title case, no period), make it a section?
    # For now, just paragraphs.
    tex_content += p + "\n\n"

tex_content += r"\end{document}"

with open(tex_path, "w", encoding="utf-8") as f:
    f.write(tex_content)

print(f"Generated {tex_path} with {len(paragraphs)} paragraphs.")
