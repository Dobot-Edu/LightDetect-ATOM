#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Markdown转PDF工具
将操作说明文档转换为PDF格式
"""

import re
import sys
from pathlib import Path

try:
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units import inch
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle
    from reportlab.lib import colors
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
except ImportError:
    print("错误: 需要安装 reportlab 库")
    print("请运行: pip install reportlab")
    sys.exit(1)

def parse_markdown(md_content):
    """解析Markdown内容，返回结构化数据"""
    lines = md_content.split('\n')
    elements = []
    current_table = []
    in_code_block = False
    code_block_content = []
    code_block_lang = ""
    
    i = 0
    while i < len(lines):
        line = lines[i]
        
        # 处理代码块
        if line.strip().startswith('```'):
            if in_code_block:
                # 结束代码块
                if code_block_content:
                    elements.append(('code', '\n'.join(code_block_content), code_block_lang))
                code_block_content = []
                code_block_lang = ""
                in_code_block = False
            else:
                # 开始代码块
                code_block_lang = line.strip()[3:].strip()
                in_code_block = True
            i += 1
            continue
        
        if in_code_block:
            code_block_content.append(line)
            i += 1
            continue
        
        # 处理表格
        if '|' in line and line.strip().startswith('|'):
            if not current_table:
                # 表格开始
                current_table = [line]
            else:
                current_table.append(line)
                # 检查是否是表格分隔行
                if all(c in '|: -' for c in line.strip()):
                    # 这是分隔行，继续收集
                    pass
                elif i + 1 >= len(lines) or '|' not in lines[i + 1] or not lines[i + 1].strip().startswith('|'):
                    # 表格结束
                    elements.append(('table', current_table))
                    current_table = []
            i += 1
            continue
        else:
            if current_table:
                elements.append(('table', current_table))
                current_table = []
        
        # 处理标题
        if line.startswith('#'):
            level = len(line) - len(line.lstrip('#'))
            text = line.lstrip('#').strip()
            elements.append(('heading', level, text))
        
        # 处理列表
        elif line.strip().startswith('- ') or line.strip().startswith('* '):
            text = line.strip()[2:].strip()
            elements.append(('list', text))
        
        # 处理有序列表
        elif re.match(r'^\d+\.\s', line.strip()):
            text = re.sub(r'^\d+\.\s', '', line.strip())
            elements.append(('list', text))
        
        # 处理分隔线
        elif line.strip() == '---':
            elements.append(('hr',))
        
        # 处理普通段落
        elif line.strip():
            elements.append(('paragraph', line))
        
        i += 1
    
    if current_table:
        elements.append(('table', current_table))
    
    return elements

def markdown_to_pdf(md_file, pdf_file):
    """将Markdown文件转换为PDF"""
    
    # 转换为字符串路径（reportlab需要字符串，不接受Path对象）
    md_file = str(md_file)
    pdf_file = str(pdf_file)
    
    # 读取Markdown文件
    with open(md_file, 'r', encoding='utf-8') as f:
        md_content = f.read()
    
    # 解析Markdown
    elements = parse_markdown(md_content)
    
    # 创建PDF文档
    doc = SimpleDocTemplate(
        pdf_file,
        pagesize=A4,
        rightMargin=72,
        leftMargin=72,
        topMargin=72,
        bottomMargin=18
    )
    
    # 创建样式
    styles = getSampleStyleSheet()
    
    # 自定义样式
    title_style = ParagraphStyle(
        'CustomTitle',
        parent=styles['Heading1'],
        fontSize=24,
        textColor=colors.HexColor('#1a1a1a'),
        spaceAfter=12,
        alignment=1  # 居中
    )
    
    heading1_style = ParagraphStyle(
        'CustomHeading1',
        parent=styles['Heading1'],
        fontSize=18,
        textColor=colors.HexColor('#2c3e50'),
        spaceAfter=12,
        spaceBefore=12
    )
    
    heading2_style = ParagraphStyle(
        'CustomHeading2',
        parent=styles['Heading2'],
        fontSize=16,
        textColor=colors.HexColor('#34495e'),
        spaceAfter=10,
        spaceBefore=10
    )
    
    heading3_style = ParagraphStyle(
        'CustomHeading3',
        parent=styles['Heading3'],
        fontSize=14,
        textColor=colors.HexColor('#34495e'),
        spaceAfter=8,
        spaceBefore=8
    )
    
    heading4_style = ParagraphStyle(
        'CustomHeading4',
        parent=styles['Heading4'],
        fontSize=12,
        textColor=colors.HexColor('#34495e'),
        spaceAfter=6,
        spaceBefore=6
    )
    
    code_style = ParagraphStyle(
        'CodeStyle',
        parent=styles['Code'],
        fontSize=9,
        fontName='Courier',
        textColor=colors.HexColor('#2c3e50'),
        backColor=colors.HexColor('#f5f5f5'),
        leftIndent=20,
        rightIndent=20,
        spaceAfter=10,
        spaceBefore=10
    )
    
    normal_style = ParagraphStyle(
        'NormalStyle',
        parent=styles['Normal'],
        fontSize=11,
        textColor=colors.HexColor('#333333'),
        spaceAfter=6
    )
    
    list_style = ParagraphStyle(
        'ListStyle',
        parent=styles['Normal'],
        fontSize=11,
        textColor=colors.HexColor('#333333'),
        leftIndent=20,
        spaceAfter=4
    )
    
    # 构建PDF内容
    story = []
    
    for element in elements:
        if element[0] == 'heading':
            level = element[1]
            text = element[2]
            
            # 处理粗体
            text = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', text)
            text = re.sub(r'`(.*?)`', r'<font name="Courier">\1</font>', text)
            
            if level == 1:
                story.append(Paragraph(text, title_style))
            elif level == 2:
                story.append(Paragraph(text, heading1_style))
            elif level == 3:
                story.append(Paragraph(text, heading2_style))
            elif level == 4:
                story.append(Paragraph(text, heading3_style))
            else:
                story.append(Paragraph(text, heading4_style))
            story.append(Spacer(1, 0.1*inch))
        
        elif element[0] == 'paragraph':
            text = element[1]
            
            # 处理Markdown格式
            text = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', text)
            text = re.sub(r'`(.*?)`', r'<font name="Courier">\1</font>', text)
            text = re.sub(r'✅', '✓', text)
            
            story.append(Paragraph(text, normal_style))
            story.append(Spacer(1, 0.05*inch))
        
        elif element[0] == 'list':
            text = element[1]
            
            # 处理Markdown格式
            text = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', text)
            text = re.sub(r'`(.*?)`', r'<font name="Courier">\1</font>', text)
            text = re.sub(r'✅', '✓', text)
            
            story.append(Paragraph('• ' + text, list_style))
        
        elif element[0] == 'code':
            code_text = element[1]
            # 代码块使用等宽字体
            story.append(Paragraph(
                '<font name="Courier" size="9">' + 
                code_text.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;') + 
                '</font>',
                code_style
            ))
            story.append(Spacer(1, 0.1*inch))
        
        elif element[0] == 'table':
            table_lines = element[1]
            table_data = []
            
            for line in table_lines:
                if '|' in line:
                    cells = [cell.strip() for cell in line.split('|')[1:-1]]
                    # 跳过分隔行
                    if not all(c.replace(':', '').replace('-', '').strip() == '' for c in cells):
                        # 处理格式
                        formatted_cells = []
                        for cell in cells:
                            cell = re.sub(r'\*\*(.*?)\*\*', r'<b>\1</b>', cell)
                            formatted_cells.append(cell)
                        table_data.append(formatted_cells)
            
            if table_data:
                # 创建表格
                table = Table(table_data)
                table.setStyle(TableStyle([
                    ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#34495e')),
                    ('TEXTCOLOR', (0, 0), (-1, 0), colors.whitesmoke),
                    ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
                    ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
                    ('FONTSIZE', (0, 0), (-1, 0), 11),
                    ('BOTTOMPADDING', (0, 0), (-1, 0), 12),
                    ('BACKGROUND', (0, 1), (-1, -1), colors.beige),
                    ('GRID', (0, 0), (-1, -1), 1, colors.black),
                    ('FONTSIZE', (0, 1), (-1, -1), 10),
                ]))
                story.append(table)
                story.append(Spacer(1, 0.2*inch))
        
        elif element[0] == 'hr':
            story.append(Spacer(1, 0.2*inch))
    
    # 生成PDF
    doc.build(story)
    print(f"✓ PDF文件已生成: {pdf_file}")

if __name__ == '__main__':
    # 获取当前脚本目录
    script_dir = Path(__file__).parent
    
    # 输入和输出文件
    md_file = script_dir / '操作说明文档.md'
    pdf_file = script_dir / '操作说明文档.pdf'
    
    if not md_file.exists():
        print(f"错误: 找不到Markdown文件: {md_file}")
        sys.exit(1)
    
    print(f"正在将 {md_file} 转换为PDF...")
    markdown_to_pdf(md_file, pdf_file)
    print("转换完成！")

