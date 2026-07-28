#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 README.md 转换为 PDF 的辅助脚本
需要安装: pip install markdown pdfkit
或者使用: pip install markdown weasyprint
"""

import os
import sys

def convert_with_weasyprint():
    """使用 weasyprint 转换"""
    try:
        import markdown
        from weasyprint import HTML
        
        md_file = os.path.join(os.path.dirname(__file__), 'README.md')
        pdf_file = os.path.join(os.path.dirname(__file__), 'README.pdf')
        
        with open(md_file, 'r', encoding='utf-8') as f:
            md_content = f.read()
        
        # 转换为 HTML
        html = markdown.markdown(md_content, extensions=['codehilite', 'fenced_code'])
        
        # 添加基本样式
        html_with_style = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <style>
                body {{ font-family: Arial, sans-serif; margin: 40px; line-height: 1.6; }}
                code {{ background-color: #f4f4f4; padding: 2px 4px; border-radius: 3px; }}
                pre {{ background-color: #f4f4f4; padding: 10px; border-radius: 5px; overflow-x: auto; }}
                table {{ border-collapse: collapse; width: 100%; }}
                th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
                th {{ background-color: #f2f2f2; }}
            </style>
        </head>
        <body>
        {html}
        </body>
        </html>
        """
        
        # 转换为 PDF
        HTML(string=html_with_style).write_pdf(pdf_file)
        print(f"✓ PDF 已生成: {pdf_file}")
        return True
    except ImportError:
        print("需要安装: pip install markdown weasyprint")
        return False
    except Exception as e:
        print(f"转换失败: {e}")
        return False

if __name__ == "__main__":
    print("尝试使用 weasyprint 转换...")
    if not convert_with_weasyprint():
        print("\n请使用以下方法之一:")
        print("1. 在 Cursor 中按 Ctrl+Shift+P，输入 'Markdown PDF: Export (pdf)'")
        print("2. 右键点击 README.md，选择 'Markdown PDF: Export (pdf)'")
        print("3. 在浏览器中打开预览，然后按 Ctrl+P 打印为 PDF")

