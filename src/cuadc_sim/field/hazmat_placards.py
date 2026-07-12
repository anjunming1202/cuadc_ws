"""The 10 hazmat placard designs, per 附件11：危险化学品标识.pdf.

The source PDF was only ever seen as rendered page images in conversation --
it was never saved to disk in this environment, so these are NOT pixel
extractions of the original. Each entry below is a simplified recreation
(correct diamond layout, exact colors/text, simplified pictogram icon) good
enough for a drone-camera view in simulation, not a print-accurate replica.
If the original PDF becomes available on disk, re-derive these via
pdfimages/pdftoppm and drop the "icon" simplification.

`layout`: "solid" (single background color) or "split_h" (horizontal split,
top half `bg` / bottom half `bg2`) or "split_h_rev" (top `bg2` / bottom `bg`).
`border`: outer diamond border color.
`icon`: simplified pictogram to draw, or None.
`class_number`: the digit(s) in the bottom corner, or None if absent.
"""

PLACARDS = {
    "explosive_1_5": {
        "bg": (237, 125, 49), "border": (30, 24, 20),
        "layout": "solid",
        "icon": None,
        "top_text": "1.5", "label": "爆炸品", "class_number": "1",
        "text_color": (30, 24, 20),
    },
    "nonflammable_gas": {
        "bg": (46, 139, 87), "border": (20, 20, 20),
        "layout": "solid",
        "icon": "gas_cylinder",
        "label": "不燃气体", "class_number": "2",
        "text_color": (20, 20, 20),
    },
    "irritant": {
        "bg": (255, 255, 255), "border": (20, 20, 20),
        "layout": "solid",
        "icon": "x_mark",
        "label": "刺激性", "sub_label": "IRRITANT", "class_number": None,
        "text_color": (20, 20, 20),
    },
    "radioactive": {
        "bg": (240, 210, 40), "bg2": (255, 255, 255), "border": (20, 20, 20),
        "layout": "split_h",
        "icon": "radiation",
        "label": "放射性物品", "class_number": "7", "class_mark": "II",
        "text_color": (20, 20, 20),
    },
    "corrosive": {
        "bg": (255, 255, 255), "bg2": (20, 20, 20), "border": (20, 20, 20),
        "layout": "split_h",
        "icon": "corrosive",
        "label": "腐蚀品", "sub_label": "CORROSIVE", "class_number": None,
        "text_color": (255, 255, 255),
    },
    "biohazard": {
        "bg": (240, 210, 40), "border": (20, 20, 20),
        "layout": "solid",
        "icon": "biohazard",
        "label": "生物危害", "class_number": None,
        "text_color": (20, 20, 20),
    },
    "wet_flammable": {
        "bg": (41, 171, 226), "border": (20, 20, 20),
        "layout": "solid",
        "icon": "flame",
        "label": "遇湿易燃物品", "class_number": "4",
        "text_color": (20, 20, 20),
    },
    "toxic": {
        "bg": (255, 255, 255), "border": (200, 30, 30),
        "layout": "solid",
        "icon": "skull",
        "label": "有毒品", "class_number": None,
        "text_color": (20, 20, 20),
    },
    "spontaneously_combustible": {
        "bg": (255, 255, 255), "bg2": (200, 30, 30), "border": (20, 20, 20),
        "layout": "split_h",
        "icon": "flame",
        "label": "自燃物品", "class_number": "4",
        "text_color": (20, 20, 20),
    },
    "flammable": {
        "bg": (200, 30, 30), "border": (100, 10, 10),
        "layout": "solid",
        "icon": "flame",
        "label": "易燃", "sub_label": "FLAMMABLE", "class_number": None,
        "text_color": (20, 20, 20),
    },
}
