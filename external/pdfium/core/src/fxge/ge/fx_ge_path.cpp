// Copyright 2014 PDFium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
 
// Original code copyright 2014 Foxit Software Inc. http://www.foxitsoftware.com

#include "../../../include/fxcrt/fx_basic.h"
#include "../../../include/fxge/fx_ge.h"

#include <rtthread.h>
#define PDFIUM_PATH_LOG(fmt, ...) \
    do { \
        if (FXMEM_HasOOM()) { \
            rt_kprintf("pdfium_path: " fmt "\n", ##__VA_ARGS__); \
        } \
    } while (0)

CFX_ClipRgn::CFX_ClipRgn(int width, int height)
{
    m_Type = RectI;
    m_Box.left = m_Box.top = 0;
    m_Box.right = width;
    m_Box.bottom = height;
}
CFX_ClipRgn::CFX_ClipRgn(const FX_RECT& rect)
{
    m_Type = RectI;
    m_Box = rect;
}
CFX_ClipRgn::CFX_ClipRgn(const CFX_ClipRgn& src)
{
    m_Type = src.m_Type;
    m_Box = src.m_Box;
    m_Mask = src.m_Mask;
}
CFX_ClipRgn::~CFX_ClipRgn()
{
}
void CFX_ClipRgn::Reset(const FX_RECT& rect)
{
    m_Type = RectI;
    m_Box = rect;
    m_Mask.SetNull();
}
void CFX_ClipRgn::IntersectRect(const FX_RECT& rect)
{
    if (m_Type == RectI) {
        m_Box.Intersect(rect);
        return;
    }
    if (m_Type == MaskF) {
        IntersectMaskRect(rect, m_Box, m_Mask);
        return;
    }
}
void CFX_ClipRgn::IntersectMaskRect(FX_RECT rect, FX_RECT mask_rect, CFX_DIBitmapRef Mask)
{
    const CFX_DIBitmap* mask_dib = Mask;
    m_Type = MaskF;
    m_Box = rect;
    m_Box.Intersect(mask_rect);
    if (m_Box.IsEmpty()) {
        m_Type = RectI;
        return;
    } else if (m_Box == mask_rect) {
        m_Mask = Mask;
        return;
    }
    CFX_DIBitmap* new_dib = m_Mask.New();
    if (!new_dib) {
        return;
    }
    new_dib->Create(m_Box.Width(), m_Box.Height(), FXDIB_8bppMask);
    for (int row = m_Box.top; row < m_Box.bottom; row ++) {
        FX_LPBYTE dest_scan = new_dib->GetBuffer() + new_dib->GetPitch() * (row - m_Box.top);
        FX_LPBYTE src_scan = mask_dib->GetBuffer() + mask_dib->GetPitch() * (row - mask_rect.top);
        for (int col = m_Box.left; col < m_Box.right; col ++) {
            dest_scan[col - m_Box.left] = src_scan[col - mask_rect.left];
        }
    }
}
void CFX_ClipRgn::IntersectMaskF(int left, int top, CFX_DIBitmapRef Mask)
{
    const CFX_DIBitmap* mask_dib = Mask;
    ASSERT(mask_dib->GetFormat() == FXDIB_8bppMask);
    FX_RECT mask_box(left, top, left + mask_dib->GetWidth(), top + mask_dib->GetHeight());
    if (m_Type == RectI) {
        IntersectMaskRect(m_Box, mask_box, Mask);
        return;
    }
    if (m_Type == MaskF) {
        FX_RECT new_box = m_Box;
        new_box.Intersect(mask_box);
        if (new_box.IsEmpty()) {
            m_Type = RectI;
            m_Mask.SetNull();
            m_Box = new_box;
            return;
        }
        CFX_DIBitmapRef new_mask;
        CFX_DIBitmap* new_dib = new_mask.New();
        if (!new_dib) {
            return;
        }
        new_dib->Create(new_box.Width(), new_box.Height(), FXDIB_8bppMask);
        const CFX_DIBitmap* old_dib = m_Mask;
        for (int row = new_box.top; row < new_box.bottom; row ++) {
            FX_LPBYTE old_scan = old_dib->GetBuffer() + (row - m_Box.top) * old_dib->GetPitch();
            FX_LPBYTE mask_scan = mask_dib->GetBuffer() + (row - top) * mask_dib->GetPitch();
            FX_LPBYTE new_scan = new_dib->GetBuffer() + (row - new_box.top) * new_dib->GetPitch();
            for (int col = new_box.left; col < new_box.right; col ++) {
                new_scan[col - new_box.left] = old_scan[col - m_Box.left] * mask_scan[col - left] / 255;
            }
        }
        m_Box = new_box;
        m_Mask = new_mask;
        return;
    }
    ASSERT(FALSE);
}
CFX_PathData::CFX_PathData()
{
    m_PointCount = m_AllocCount = 0;
    m_pPoints = NULL;
}
CFX_PathData::~CFX_PathData()
{
    if (m_pPoints) {
        FX_Free(m_pPoints);
    }
}
FX_BOOL CFX_PathData::SetPointCount(int nPoints)
{
    PDFIUM_PATH_LOG("SetPointCount this=0x%08x pts=0x%08x old=%d alloc=%d new=%d",
                    (unsigned int)(FX_UINTPTR)this, (unsigned int)(FX_UINTPTR)m_pPoints,
                    m_PointCount, m_AllocCount, nPoints);
    if (nPoints <= 0) {
        m_PointCount = 0;
        return TRUE;
    }
    int old_count = m_PointCount;
    if (m_AllocCount < nPoints) {
        FX_PATHPOINT* pNewBuf = FX_Alloc(FX_PATHPOINT, nPoints);
        if (!pNewBuf) {
            PDFIUM_PATH_LOG("SetPointCount oom this=0x%08x old=%d alloc=%d new=%d",
                            (unsigned int)(FX_UINTPTR)this, m_PointCount, m_AllocCount, nPoints);
            return FALSE;
        }
        FXSYS_memset(pNewBuf, 0, sizeof(FX_PATHPOINT) * nPoints);
        if (m_pPoints) {
            FX_Free(m_pPoints);
        }
        m_pPoints = pNewBuf;
        m_AllocCount = nPoints;
    }
    if (m_pPoints && nPoints > old_count) {
        FXSYS_memset(m_pPoints + old_count, 0, sizeof(FX_PATHPOINT) * (nPoints - old_count));
    }
    m_PointCount = nPoints;
    return TRUE;
}
FX_BOOL CFX_PathData::AllocPointCount(int nPoints)
{
    PDFIUM_PATH_LOG("AllocPointCount this=0x%08x pts=0x%08x count=%d alloc=%d need=%d",
                    (unsigned int)(FX_UINTPTR)this, (unsigned int)(FX_UINTPTR)m_pPoints,
                    m_PointCount, m_AllocCount, nPoints);
    if (nPoints <= 0) {
        return TRUE;
    }
    if (m_AllocCount < nPoints) {
        FX_PATHPOINT* pNewBuf = FX_Alloc(FX_PATHPOINT, nPoints);
        if (!pNewBuf) {
            PDFIUM_PATH_LOG("AllocPointCount oom this=0x%08x count=%d alloc=%d need=%d",
                            (unsigned int)(FX_UINTPTR)this, m_PointCount, m_AllocCount, nPoints);
            return FALSE;
        }
        FXSYS_memset(pNewBuf, 0, sizeof(FX_PATHPOINT) * nPoints);
        if (m_PointCount && m_pPoints) {
            FXSYS_memcpy32(pNewBuf, m_pPoints, m_PointCount * sizeof(FX_PATHPOINT));
        }
        if (m_pPoints) {
            FX_Free(m_pPoints);
        }
        m_pPoints = pNewBuf;
        m_AllocCount = nPoints;
    }
    return TRUE;
}
CFX_PathData::CFX_PathData(const CFX_PathData& src)
{
    m_pPoints = NULL;
    m_PointCount = 0;
    m_AllocCount = 0;
    if (src.m_PointCount <= 0 || !src.m_pPoints) {
        return;
    }
    m_pPoints = FX_Alloc(FX_PATHPOINT, src.m_PointCount);
    if (!m_pPoints) {
        return;
    }
    m_PointCount = m_AllocCount = src.m_PointCount;
    FXSYS_memcpy32(m_pPoints, src.m_pPoints, sizeof(FX_PATHPOINT) * m_PointCount);
}
void CFX_PathData::TrimPoints(int nPoints)
{
    if (m_PointCount <= nPoints) {
        return;
    }
    if (!SetPointCount(nPoints)) {
        return;
    }
}
FX_BOOL CFX_PathData::AddPointCount(int addPoints)
{
    if (addPoints < 0 || m_PointCount < 0 || m_PointCount > 0x7fffffff - addPoints) {
        return FALSE;
    }
    int old_count = m_PointCount;
    int new_count = m_PointCount + addPoints;
    PDFIUM_PATH_LOG("AddPointCount this=0x%08x pts=0x%08x old=%d alloc=%d add=%d new=%d",
                    (unsigned int)(FX_UINTPTR)this, (unsigned int)(FX_UINTPTR)m_pPoints,
                    old_count, m_AllocCount, addPoints, new_count);
    if (!AllocPointCount(new_count)) {
        PDFIUM_PATH_LOG("AddPointCount fail this=0x%08x old=%d add=%d new=%d",
                        (unsigned int)(FX_UINTPTR)this, old_count, addPoints, new_count);
        return FALSE;
    }
    if (m_pPoints && new_count > old_count) {
        FXSYS_memset(m_pPoints + old_count, 0, sizeof(FX_PATHPOINT) * (new_count - old_count));
    }
    m_PointCount = new_count;
    return TRUE;
}
FX_BOOL CFX_PathData::Append(const CFX_PathData* pSrc, const CFX_AffineMatrix* pMatrix)
{
    if (!pSrc || (pSrc->m_PointCount && !pSrc->m_pPoints)) {
        return FALSE;
    }
    int old_count = m_PointCount;
    if (!AddPointCount(pSrc->m_PointCount)) {
        return FALSE;
    }
    FXSYS_memcpy32(m_pPoints + old_count, pSrc->m_pPoints, pSrc->m_PointCount * sizeof(FX_PATHPOINT));
    if (pMatrix == NULL) {
        return TRUE;
    }
    for (int i = 0; i < pSrc->m_PointCount; i ++) {
        pMatrix->Transform(m_pPoints[old_count + i].m_PointX, m_pPoints[old_count + i].m_PointY);
    }
    return TRUE;
}
void CFX_PathData::SetPoint(int index, FX_FLOAT x, FX_FLOAT y, int flag)
{
    ASSERT(index < m_PointCount);
    if (!m_pPoints || index < 0 || index >= m_PointCount) {
        return;
    }
    m_pPoints[index].m_PointX = x;
    m_pPoints[index].m_PointY = y;
    m_pPoints[index].m_Flag = flag;
}
FX_BOOL CFX_PathData::AppendRect(FX_FLOAT left, FX_FLOAT bottom, FX_FLOAT right, FX_FLOAT top)
{
    int old_count = m_PointCount;
    if (!AddPointCount(5)) {
        return FALSE;
    }
    FX_PATHPOINT* pPoints = m_pPoints + old_count;
    pPoints[0].m_PointX = pPoints[1].m_PointX = pPoints[4].m_PointX = left;
    pPoints[2].m_PointX = pPoints[3].m_PointX = right;
    pPoints[0].m_PointY = pPoints[3].m_PointY = pPoints[4].m_PointY = bottom;
    pPoints[1].m_PointY = pPoints[2].m_PointY = top;
    pPoints[0].m_Flag = FXPT_MOVETO;
    pPoints[1].m_Flag = pPoints[2].m_Flag = pPoints[3].m_Flag = FXPT_LINETO;
    pPoints[4].m_Flag = FXPT_LINETO | FXPT_CLOSEFIGURE;
    return TRUE;
}
CFX_FloatRect CFX_PathData::GetBoundingBox() const
{
    CFX_FloatRect rect;
    if (m_PointCount && m_pPoints) {
        rect.InitRect(m_pPoints[0].m_PointX, m_pPoints[0].m_PointY);
        for (int i = 1; i < m_PointCount; i ++) {
            rect.UpdateRect(m_pPoints[i].m_PointX, m_pPoints[i].m_PointY);
        }
    }
    return rect;
}
static void _UpdateLineEndPoints(CFX_FloatRect& rect, FX_FLOAT start_x, FX_FLOAT start_y, FX_FLOAT end_x, FX_FLOAT end_y,
                                 FX_FLOAT hw)
{
    if (start_x == end_x) {
        if (start_y == end_y) {
            rect.UpdateRect(end_x + hw, end_y + hw);
            rect.UpdateRect(end_x - hw, end_y - hw);
            return;
        }
        FX_FLOAT point_y;
        if (end_y < start_y) {
            point_y = end_y - hw;
        } else {
            point_y = end_y + hw;
        }
        rect.UpdateRect(end_x + hw, point_y);
        rect.UpdateRect(end_x - hw, point_y);
        return;
    } else if (start_y == end_y) {
        FX_FLOAT point_x;
        if (end_x < start_x) {
            point_x = end_x - hw;
        } else {
            point_x = end_x + hw;
        }
        rect.UpdateRect(point_x, end_y + hw);
        rect.UpdateRect(point_x, end_y - hw);
        return;
    }
    FX_FLOAT dx = end_x - start_x;
    FX_FLOAT dy = end_y - start_y;
    FX_FLOAT ll = FXSYS_sqrt2(dx, dy);
    FX_FLOAT mx = end_x + hw * dx / ll;
    FX_FLOAT my = end_y + hw * dy / ll;
    FX_FLOAT dx1 = hw * dy / ll;
    FX_FLOAT dy1 = hw * dx / ll;
    rect.UpdateRect(mx - dx1, my + dy1);
    rect.UpdateRect(mx + dx1, my - dy1);
}
static void _UpdateLineJoinPoints(CFX_FloatRect& rect, FX_FLOAT start_x, FX_FLOAT start_y,
                                  FX_FLOAT middle_x, FX_FLOAT middle_y, FX_FLOAT end_x, FX_FLOAT end_y,
                                  FX_FLOAT half_width, FX_FLOAT miter_limit)
{
    FX_FLOAT start_k = 0, start_c = 0, end_k = 0, end_c = 0, start_len = 0, start_dc = 0, end_len = 0, end_dc = 0;
    FX_BOOL bStartVert = FXSYS_fabs(start_x - middle_x) < 1.0f / 20;
    FX_BOOL bEndVert = FXSYS_fabs(middle_x - end_x) < 1.0f / 20;
    if (bStartVert && bEndVert) {
        int start_dir = middle_y > start_y ? 1 : -1;
        FX_FLOAT point_y = middle_y + half_width * start_dir;
        rect.UpdateRect(middle_x + half_width, point_y);
        rect.UpdateRect(middle_x - half_width, point_y);
        return;
    }
    if (!bStartVert) {
        start_k = FXSYS_Div(middle_y - start_y, middle_x - start_x);
        start_c = middle_y - FXSYS_Mul(start_k, middle_x);
        start_len = FXSYS_sqrt2(start_x - middle_x, start_y - middle_y);
        start_dc = (FX_FLOAT)FXSYS_fabs(FXSYS_MulDiv(half_width, start_len, start_x - middle_x));
    }
    if (!bEndVert) {
        end_k = FXSYS_Div(end_y - middle_y, end_x - middle_x);
        end_c = middle_y - FXSYS_Mul(end_k, middle_x);
        end_len = FXSYS_sqrt2(end_x - middle_x, end_y - middle_y);
        end_dc = (FX_FLOAT)FXSYS_fabs(FXSYS_MulDiv(half_width, end_len, end_x - middle_x));
    }
    if (bStartVert) {
        FX_FLOAT outside_x = start_x;
        if (end_x < start_x) {
            outside_x += half_width;
        } else {
            outside_x -= half_width;
        }
        FX_FLOAT outside_y;
        if (start_y < FXSYS_Mul(end_k, start_x) + end_c) {
            outside_y = FXSYS_Mul(end_k, outside_x) + end_c + end_dc;
        } else {
            outside_y = FXSYS_Mul(end_k, outside_x) + end_c - end_dc;
        }
        rect.UpdateRect(outside_x, outside_y);
        return;
    }
    if (bEndVert) {
        FX_FLOAT outside_x = end_x;
        if (start_x < end_x) {
            outside_x += half_width;
        } else {
            outside_x -= half_width;
        }
        FX_FLOAT outside_y;
        if (end_y < FXSYS_Mul(start_k, end_x) + start_c) {
            outside_y = FXSYS_Mul(start_k, outside_x) + start_c + start_dc;
        } else {
            outside_y = FXSYS_Mul(start_k, outside_x) + start_c - start_dc;
        }
        rect.UpdateRect(outside_x, outside_y);
        return;
    }
    if (FXSYS_fabs(start_k - end_k) < 1.0f / 20) {
        int start_dir = middle_x > start_x ? 1 : -1;
        int end_dir = end_x > middle_x ? 1 : -1;
        if (start_dir == end_dir) {
            _UpdateLineEndPoints(rect, middle_x, middle_y, end_x, end_y, half_width);
        } else {
            _UpdateLineEndPoints(rect, start_x, start_y, middle_x, middle_y, half_width);
        }
        return;
    }
    FX_FLOAT start_outside_c = start_c;
    if (end_y < FXSYS_Mul(start_k, end_x) + start_c) {
        start_outside_c += start_dc;
    } else {
        start_outside_c -= start_dc;
    }
    FX_FLOAT end_outside_c = end_c;
    if (start_y < FXSYS_Mul(end_k, start_x) + end_c) {
        end_outside_c += end_dc;
    } else {
        end_outside_c -= end_dc;
    }
    FX_FLOAT join_x = FXSYS_Div(end_outside_c - start_outside_c, start_k - end_k);
    FX_FLOAT join_y = FXSYS_Mul(start_k, join_x) + start_outside_c;
    rect.UpdateRect(join_x, join_y);
}
CFX_FloatRect CFX_PathData::GetBoundingBox(FX_FLOAT line_width, FX_FLOAT miter_limit) const
{
    CFX_FloatRect rect(100000 * 1.0f, 100000 * 1.0f, -100000 * 1.0f, -100000 * 1.0f);
    if (m_PointCount && !m_pPoints) {
        return rect;
    }
    if (m_PointCount <= 0) {
        return rect;
    }
    rect = GetBoundingBox();
    FX_FLOAT inflate = line_width < 0 ? -line_width : line_width;
    FX_FLOAT miter = miter_limit < 0 ? -miter_limit : miter_limit;
    if (miter > 1.0f) {
        inflate *= miter;
    }
    rect.Inflate(inflate, inflate);
    return rect;
#if 0
    int iPoint = 0;
    FX_FLOAT half_width = line_width;
    int iStartPoint, iEndPoint, iMiddlePoint;
    FX_BOOL bJoin;
    while (iPoint < m_PointCount) {
        if (m_pPoints[iPoint].m_Flag == FXPT_MOVETO) {
            if (iPoint + 1 >= m_PointCount) {
                break;
            }
            iStartPoint = iPoint + 1;
            iEndPoint = iPoint;
            bJoin = FALSE;
        } else {
            if (m_pPoints[iPoint].m_Flag == FXPT_BEZIERTO) {
                if (iPoint + 2 >= m_PointCount) {
                    break;
                }
                rect.UpdateRect(m_pPoints[iPoint].m_PointX, m_pPoints[iPoint].m_PointY);
                rect.UpdateRect(m_pPoints[iPoint + 1].m_PointX, m_pPoints[iPoint + 1].m_PointY);
                iPoint += 2;
                if (iPoint >= m_PointCount) {
                    break;
                }
            }
            if (iPoint == m_PointCount - 1 || m_pPoints[iPoint + 1].m_Flag == FXPT_MOVETO) {
                if (iPoint <= 0) {
                    iPoint ++;
                    continue;
                }
                iStartPoint = iPoint - 1;
                iEndPoint = iPoint;
                bJoin = FALSE;
            } else {
                if (iPoint <= 0 || iPoint + 1 >= m_PointCount) {
                    iPoint ++;
                    continue;
                }
                iStartPoint = iPoint - 1;
                iMiddlePoint = iPoint;
                iEndPoint = iPoint + 1;
                bJoin = TRUE;
            }
        }
        FX_FLOAT start_x = m_pPoints[iStartPoint].m_PointX;
        FX_FLOAT start_y = m_pPoints[iStartPoint].m_PointY;
        FX_FLOAT end_x = m_pPoints[iEndPoint].m_PointX;
        FX_FLOAT end_y = m_pPoints[iEndPoint].m_PointY;
        if (bJoin) {
            FX_FLOAT middle_x = m_pPoints[iMiddlePoint].m_PointX;
            FX_FLOAT middle_y = m_pPoints[iMiddlePoint].m_PointY;
            _UpdateLineJoinPoints(rect, start_x, start_y, middle_x, middle_y, end_x, end_y, half_width, miter_limit);
        } else {
            _UpdateLineEndPoints(rect, start_x, start_y, end_x, end_y, half_width);
        }
        iPoint ++;
    }
    return rect;
#endif
}
void CFX_PathData::Transform(const CFX_AffineMatrix* pMatrix)
{
    if (pMatrix == NULL || (m_PointCount && !m_pPoints)) {
        return;
    }
    for (int i = 0; i < m_PointCount; i ++) {
        pMatrix->Transform(m_pPoints[i].m_PointX, m_pPoints[i].m_PointY);
    }
}
FX_BOOL CFX_PathData::GetZeroAreaPath(CFX_PathData& NewPath, CFX_AffineMatrix* pMatrix, FX_BOOL&bThin, FX_BOOL bAdjust) const
{
    PDFIUM_PATH_LOG("GetZeroAreaPath enter this=0x%08x pts=0x%08x count=%d alloc=%d newpath=0x%08x newcnt=%d matrix=0x%08x adj=%d",
                    (unsigned int)(FX_UINTPTR)this, (unsigned int)(FX_UINTPTR)m_pPoints,
                    m_PointCount, m_AllocCount, (unsigned int)(FX_UINTPTR)&NewPath,
                    NewPath.GetPointCount(), (unsigned int)(FX_UINTPTR)pMatrix, bAdjust);
    if (m_PointCount < 3 || !m_pPoints) {
        PDFIUM_PATH_LOG("GetZeroAreaPath reject this=0x%08x pts=0x%08x count=%d",
                        (unsigned int)(FX_UINTPTR)this, (unsigned int)(FX_UINTPTR)m_pPoints,
                        m_PointCount);
        return FALSE;
    }
    if (m_PointCount == 3 && (m_pPoints[0].m_Flag & FXPT_TYPE) == FXPT_MOVETO &&
            (m_pPoints[1].m_Flag & FXPT_TYPE) == FXPT_LINETO && (m_pPoints[2].m_Flag & FXPT_TYPE) == FXPT_LINETO
            && m_pPoints[0].m_PointX == m_pPoints[2].m_PointX && m_pPoints[0].m_PointY == m_pPoints[2].m_PointY) {
        if (!NewPath.AddPointCount(2)) {
            return FALSE;
        }
        if (bAdjust) {
            if (pMatrix) {
                FX_FLOAT x = m_pPoints[0].m_PointX, y = m_pPoints[0].m_PointY;
                pMatrix->TransformPoint(x, y);
                x = (int)x + 0.5f;
                y = (int)y + 0.5f;
                NewPath.SetPoint(0, x, y, FXPT_MOVETO);
                x = m_pPoints[1].m_PointX, y = m_pPoints[1].m_PointY;
                pMatrix->TransformPoint(x, y);
                x = (int)x + 0.5f;
                y = (int)y + 0.5f;
                NewPath.SetPoint(1, x, y, FXPT_LINETO);
                pMatrix->SetIdentity();
            } else {
                FX_FLOAT x = (int)m_pPoints[0].m_PointX + 0.5f, y = (int)m_pPoints[0].m_PointY + 0.5f;
                NewPath.SetPoint(0, x, y, FXPT_MOVETO);
                x = (int)m_pPoints[1].m_PointX + 0.5f, y = (int)m_pPoints[1].m_PointY + 0.5f;
                NewPath.SetPoint(1, x, y, FXPT_LINETO);
            }
        } else {
            NewPath.SetPoint(0, m_pPoints[0].m_PointX, m_pPoints[0].m_PointY, FXPT_MOVETO);
            NewPath.SetPoint(1, m_pPoints[1].m_PointX, m_pPoints[1].m_PointY, FXPT_LINETO);
        }
        if (m_pPoints[0].m_PointX != m_pPoints[1].m_PointX && m_pPoints[0].m_PointY != m_pPoints[1].m_PointY) {
            bThin = TRUE;
        }
        return TRUE;
    }
    if (((m_PointCount > 3) && (m_PointCount % 2))) {
        int mid = m_PointCount / 2;
        FX_BOOL bZeroArea = FALSE;
        CFX_PathData t_path;
        for (int i = 0; i < mid; i++) {
            PDFIUM_PATH_LOG("GetZeroAreaPath mirror this=0x%08x i=%d mid=%d left=%d right=%d count=%d flagL=0x%x flagR=0x%x",
                            (unsigned int)(FX_UINTPTR)this, i, mid, mid - i - 1,
                            mid + i + 1, m_PointCount,
                            m_pPoints[mid - i - 1].m_Flag, m_pPoints[mid + i + 1].m_Flag);
            if (!(m_pPoints[mid - i - 1].m_PointX == m_pPoints[mid + i + 1].m_PointX
                    && m_pPoints[mid - i - 1].m_PointY == m_pPoints[mid + i + 1].m_PointY &&
                    ((m_pPoints[mid - i - 1].m_Flag & FXPT_TYPE) != FXPT_BEZIERTO && (m_pPoints[mid + i + 1].m_Flag & FXPT_TYPE) != FXPT_BEZIERTO))) {
                bZeroArea = TRUE;
                break;
            }
            int new_count = t_path.GetPointCount();
            if (!t_path.AddPointCount(2)) {
                return FALSE;
            }
            t_path.SetPoint(new_count, m_pPoints[mid - i].m_PointX, m_pPoints[mid - i].m_PointY, FXPT_MOVETO);
            t_path.SetPoint(new_count + 1, m_pPoints[mid - i - 1].m_PointX, m_pPoints[mid - i - 1].m_PointY, FXPT_LINETO);
        }
        if (!bZeroArea) {
            if (!NewPath.Append(&t_path, NULL)) {
                return FALSE;
            }
            bThin = TRUE;
            return TRUE;
        }
    }
    int stratPoint = 0;
    int next = 0, i;
    for (i = 0; i < m_PointCount; i++) {
        int point_type = m_pPoints[i].m_Flag & FXPT_TYPE;
        PDFIUM_PATH_LOG("GetZeroAreaPath loop this=0x%08x i=%d count=%d flag=0x%x type=%d strat=%d",
                        (unsigned int)(FX_UINTPTR)this, i, m_PointCount,
                        m_pPoints[i].m_Flag, point_type, stratPoint);
        if (point_type == FXPT_MOVETO) {
            stratPoint = i;
        } else if (point_type == FXPT_LINETO) {
            if (i <= 0 || stratPoint < 0 || stratPoint >= m_PointCount || stratPoint >= i) {
                PDFIUM_PATH_LOG("GetZeroAreaPath skip_bad_i this=0x%08x i=%d strat=%d count=%d",
                                (unsigned int)(FX_UINTPTR)this, i, stratPoint, m_PointCount);
                continue;
            }
            next = (i + 1 - stratPoint) % (m_PointCount - stratPoint) + stratPoint;
            if (next < 0 || next >= m_PointCount) {
                PDFIUM_PATH_LOG("GetZeroAreaPath skip_bad_next this=0x%08x i=%d strat=%d next=%d count=%d",
                                (unsigned int)(FX_UINTPTR)this, i, stratPoint, next, m_PointCount);
                continue;
            }
            PDFIUM_PATH_LOG("GetZeroAreaPath line this=0x%08x i=%d pre=%d next=%d count=%d flagPre=0x%x flagCur=0x%x flagNext=0x%x",
                            (unsigned int)(FX_UINTPTR)this, i, i - 1, next, m_PointCount,
                            m_pPoints[i - 1].m_Flag, m_pPoints[i].m_Flag, m_pPoints[next].m_Flag);
            if ((m_pPoints[next].m_Flag & FXPT_TYPE) != FXPT_BEZIERTO && (m_pPoints[next].m_Flag & FXPT_TYPE) != FXPT_MOVETO) {
                if((m_pPoints[i - 1].m_PointX == m_pPoints[i].m_PointX && m_pPoints[i].m_PointX == m_pPoints[next].m_PointX)
                        && ((m_pPoints[i].m_PointY - m_pPoints[i - 1].m_PointY) * (m_pPoints[i].m_PointY - m_pPoints[next].m_PointY) > 0)) {
                    int pre = i;
                    if (FXSYS_fabs(m_pPoints[i].m_PointY - m_pPoints[i - 1].m_PointY)
                            < FXSYS_fabs(m_pPoints[i].m_PointY - m_pPoints[next].m_PointY)) {
                        pre --;
                        next--;
                    }
                    if (pre < 0 || pre >= m_PointCount || next < 0 || next >= m_PointCount) {
                        continue;
                    }
                    int new_count = NewPath.GetPointCount();
                    if (!NewPath.AddPointCount(2)) {
                        return FALSE;
                    }
                    NewPath.SetPoint(new_count, m_pPoints[pre].m_PointX, m_pPoints[pre].m_PointY, FXPT_MOVETO);
                    NewPath.SetPoint(new_count + 1, m_pPoints[next].m_PointX, m_pPoints[next].m_PointY, FXPT_LINETO);
                } else if((m_pPoints[i - 1].m_PointY == m_pPoints[i].m_PointY && m_pPoints[i].m_PointY == m_pPoints[next].m_PointY)
                          && ((m_pPoints[i].m_PointX - m_pPoints[i - 1].m_PointX) * (m_pPoints[i].m_PointX - m_pPoints[next].m_PointX) > 0)) {
                    int pre = i;
                    if (FXSYS_fabs(m_pPoints[i].m_PointX - m_pPoints[i - 1].m_PointX)
                            < FXSYS_fabs(m_pPoints[i].m_PointX - m_pPoints[next].m_PointX)) {
                        pre --;
                        next--;
                    }
                    if (pre < 0 || pre >= m_PointCount || next < 0 || next >= m_PointCount) {
                        continue;
                    }
                    int new_count = NewPath.GetPointCount();
                    if (!NewPath.AddPointCount(2)) {
                        return FALSE;
                    }
                    NewPath.SetPoint(new_count, m_pPoints[pre].m_PointX, m_pPoints[pre].m_PointY, FXPT_MOVETO);
                    NewPath.SetPoint(new_count + 1, m_pPoints[next].m_PointX, m_pPoints[next].m_PointY, FXPT_LINETO);
                } else if ((m_pPoints[i - 1].m_Flag & FXPT_TYPE) == FXPT_MOVETO && (m_pPoints[next].m_Flag & FXPT_TYPE) == FXPT_LINETO &&
                           m_pPoints[i - 1].m_PointX == m_pPoints[next].m_PointX && m_pPoints[i - 1].m_PointY == m_pPoints[next].m_PointY
                           && m_pPoints[next].m_Flag & FXPT_CLOSEFIGURE) {
                    int new_count = NewPath.GetPointCount();
                    if (!NewPath.AddPointCount(2)) {
                        return FALSE;
                    }
                    NewPath.SetPoint(new_count, m_pPoints[i - 1].m_PointX, m_pPoints[i - 1].m_PointY, FXPT_MOVETO);
                    NewPath.SetPoint(new_count + 1, m_pPoints[i].m_PointX, m_pPoints[i].m_PointY, FXPT_LINETO);
                    bThin = TRUE;
                }
            }
        } else if (point_type == FXPT_BEZIERTO) {
            i += 2;
            continue;
        }
    }
    if (m_PointCount > 3 && NewPath.GetPointCount()) {
        bThin = TRUE;
    }
    if (NewPath.GetPointCount() == 0) {
        PDFIUM_PATH_LOG("GetZeroAreaPath exit_empty this=0x%08x count=%d", (unsigned int)(FX_UINTPTR)this, m_PointCount);
        return FALSE;
    }
    PDFIUM_PATH_LOG("GetZeroAreaPath exit_true this=0x%08x count=%d newcnt=%d thin=%d",
                    (unsigned int)(FX_UINTPTR)this, m_PointCount, NewPath.GetPointCount(), bThin);
    return TRUE;
}
FX_BOOL CFX_PathData::IsRect() const
{
    PDFIUM_PATH_LOG("IsRect enter this=0x%08x pts=0x%08x count=%d alloc=%d",
                    (unsigned int)(FX_UINTPTR)this, (unsigned int)(FX_UINTPTR)m_pPoints,
                    m_PointCount, m_AllocCount);
    if (!m_pPoints) {
        return FALSE;
    }
    if (m_PointCount != 5 && m_PointCount != 4) {
        return FALSE;
    }
    if ((m_PointCount == 5 && (m_pPoints[0].m_PointX != m_pPoints[4].m_PointX ||
                               m_pPoints[0].m_PointY != m_pPoints[4].m_PointY)) ||
            (m_pPoints[0].m_PointX == m_pPoints[2].m_PointX && m_pPoints[0].m_PointY == m_pPoints[2].m_PointY) ||
            (m_pPoints[1].m_PointX == m_pPoints[3].m_PointX && m_pPoints[1].m_PointY == m_pPoints[3].m_PointY)) {
        return FALSE;
    }
    if (m_pPoints[0].m_PointX != m_pPoints[3].m_PointX && m_pPoints[0].m_PointY != m_pPoints[3].m_PointY) {
        return FALSE;
    }
    for (int i = 1; i < 4; i ++) {
        if ((m_pPoints[i].m_Flag & FXPT_TYPE) != FXPT_LINETO) {
            return FALSE;
        }
        if (m_pPoints[i].m_PointX != m_pPoints[i - 1].m_PointX && m_pPoints[i].m_PointY != m_pPoints[i - 1].m_PointY) {
            return FALSE;
        }
    }
    return m_PointCount == 5 || (m_pPoints[3].m_Flag & FXPT_CLOSEFIGURE);
}
FX_BOOL CFX_PathData::IsRect(const CFX_AffineMatrix* pMatrix, CFX_FloatRect* pRect) const
{
    PDFIUM_PATH_LOG("IsRectM enter this=0x%08x pts=0x%08x count=%d alloc=%d matrix=0x%08x rect=0x%08x",
                    (unsigned int)(FX_UINTPTR)this, (unsigned int)(FX_UINTPTR)m_pPoints,
                    m_PointCount, m_AllocCount, (unsigned int)(FX_UINTPTR)pMatrix,
                    (unsigned int)(FX_UINTPTR)pRect);
    if (!m_pPoints) {
        return FALSE;
    }
    if (pMatrix == NULL) {
        if (!IsRect()) {
            return FALSE;
        }
        if (pRect) {
            pRect->left = m_pPoints[0].m_PointX;
            pRect->right = m_pPoints[2].m_PointX;
            pRect->bottom = m_pPoints[0].m_PointY;
            pRect->top = m_pPoints[2].m_PointY;
            pRect->Normalize();
        }
        return TRUE;
    }
    if (m_PointCount != 5 && m_PointCount != 4) {
        return FALSE;
    }
    if ((m_PointCount == 5 && (m_pPoints[0].m_PointX != m_pPoints[4].m_PointX || m_pPoints[0].m_PointY != m_pPoints[4].m_PointY)) ||
            (m_pPoints[1].m_PointX == m_pPoints[3].m_PointX && m_pPoints[1].m_PointY == m_pPoints[3].m_PointY)) {
        return FALSE;
    }
    if (m_PointCount == 4 && m_pPoints[0].m_PointX != m_pPoints[3].m_PointX && m_pPoints[0].m_PointY != m_pPoints[3].m_PointY) {
        return FALSE;
    }
    FX_FLOAT x[5], y[5];
    for (int i = 0; i < m_PointCount; i ++) {
        pMatrix->Transform(m_pPoints[i].m_PointX, m_pPoints[i].m_PointY, x[i], y[i]);
        if (i) {
            if ((m_pPoints[i].m_Flag & FXPT_TYPE) != FXPT_LINETO) {
                return FALSE;
            }
            if (x[i] != x[i - 1] && y[i] != y[i - 1]) {
                return FALSE;
            }
        }
    }
    if (pRect) {
        pRect->left = x[0];
        pRect->right = x[2];
        pRect->bottom = y[0];
        pRect->top = y[2];
        pRect->Normalize();
    }
    return TRUE;
}
FX_BOOL CFX_PathData::Copy(const CFX_PathData &src)
{
    if (src.m_PointCount && !src.m_pPoints) {
        return FALSE;
    }
    if (!SetPointCount(src.m_PointCount)) {
        return FALSE;
    }
    if (m_PointCount) {
        FXSYS_memcpy32(m_pPoints, src.m_pPoints, sizeof(FX_PATHPOINT) * m_PointCount);
    }
    return TRUE;
}
CFX_GraphStateData::CFX_GraphStateData()
{
    m_LineCap = LineCapButt;
    m_DashCount = 0;
    m_DashArray = NULL;
    m_DashPhase = 0;
    m_LineJoin = LineJoinMiter;
    m_MiterLimit = 10 * 1.0f;
    m_LineWidth = 1.0f;
}
CFX_GraphStateData::CFX_GraphStateData(const CFX_GraphStateData& src)
{
    m_DashArray = NULL;
    Copy(src);
}
void CFX_GraphStateData::Copy(const CFX_GraphStateData& src)
{
    m_LineCap = src.m_LineCap;
    m_DashCount = src.m_DashCount;
    if (m_DashArray) {
        FX_Free(m_DashArray);
    }
    m_DashArray = NULL;
    m_DashPhase = src.m_DashPhase;
    m_LineJoin = src.m_LineJoin;
    m_MiterLimit = src.m_MiterLimit;
    m_LineWidth = src.m_LineWidth;
    if (m_DashCount) {
        m_DashArray = FX_Alloc(FX_FLOAT, m_DashCount);
        if (!m_DashArray) {
            m_DashCount = 0;
            return;
        }
        FXSYS_memcpy32(m_DashArray, src.m_DashArray, m_DashCount * sizeof(FX_FLOAT));
    }
}
CFX_GraphStateData::~CFX_GraphStateData()
{
    if (m_DashArray) {
        FX_Free(m_DashArray);
    }
}
void CFX_GraphStateData::SetDashCount(int count)
{
    if (m_DashArray) {
        FX_Free(m_DashArray);
    }
    m_DashArray = NULL;
    m_DashCount = count;
    if (count == 0) {
        return;
    }
    m_DashArray = FX_Alloc(FX_FLOAT, count);
    if (!m_DashArray) {
        m_DashCount = 0;
    }
}
