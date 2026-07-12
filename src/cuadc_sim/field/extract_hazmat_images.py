#!/usr/bin/env python3
"""Extract the 10 hazmat placard images from the source PDF into
models/hazmat_placard/materials/textures/ -- pixel-accurate (embedded JPEGs
extracted directly, not re-rasterized), unlike the first pass at these
textures (gen_textures.py originally hand-recreated them because the PDF
wasn't available on disk yet; it's since been added under docs/reference/).

Page order matches hazmat_placards.PLACARDS' insertion order (verified by
eye against the PDF when it was first shared in conversation): explosive,
nonflammable gas, irritant, radioactive, corrosive, biohazard, wet
flammable, toxic, spontaneously combustible, flammable.

Usage: python3 field/extract_hazmat_images.py
"""
import os

import fitz  # PyMuPDF

import hazmat_placards

HERE = os.path.dirname(os.path.abspath(__file__))
PDF_PATH = os.path.join(HERE, '..', '..', '..', 'docs', 'reference',
                        '附件11：危险化学品标识.pdf')
OUT_DIR = os.path.join(HERE, '..', 'models', 'hazmat_placard', 'materials', 'textures')

PAGE_ORDER = [
    'explosive_1_5', 'nonflammable_gas', 'irritant', 'radioactive', 'corrosive',
    'biohazard', 'wet_flammable', 'toxic', 'spontaneously_combustible', 'flammable',
]


def main():
    assert set(PAGE_ORDER) == set(hazmat_placards.PLACARDS.keys()), (
        'PAGE_ORDER must match hazmat_placards.PLACARDS keys exactly')
    doc = fitz.open(PDF_PATH)
    assert doc.page_count == len(PAGE_ORDER), (
        f'expected {len(PAGE_ORDER)} pages, PDF has {doc.page_count}')
    os.makedirs(OUT_DIR, exist_ok=True)
    for i, key in enumerate(PAGE_ORDER):
        page = doc[i]
        imgs = page.get_images()
        assert len(imgs) == 1, f'page {i} ({key}) has {len(imgs)} images, expected 1'
        xref = imgs[0][0]
        base = doc.extract_image(xref)
        out_path = os.path.join(OUT_DIR, f'{key}.png')
        # Re-encode via Pillow to PNG for consistency with the rest of the
        # pipeline (ring/marking textures are all PNG).
        from PIL import Image
        import io
        Image.open(io.BytesIO(base['image'])).convert('RGB').save(out_path)
        print(f'wrote {out_path} ({base["width"]}x{base["height"]} source)')


if __name__ == '__main__':
    main()
