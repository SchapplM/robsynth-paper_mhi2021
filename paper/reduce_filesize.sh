#!/bin/bash -e
# http://milan.kupcevic.net/ghostscript-ps-pdf/
#-dPDFSETTINGS=/screen   (screen-view-only quality, 72 dpi images)
#-dPDFSETTINGS=/ebook    (low quality, 150 dpi images)
#-dPDFSETTINGS=/printer  (high quality, 300 dpi images)
#-dPDFSETTINGS=/prepress (high quality, color preserving, 300 dpi imgs)
#-dPDFSETTINGS=/default  (almost identical to /screen)
gs -dNOPAUSE -dBATCH -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/prepress -sOutputFile=mhi2021_prepress.pdf mhi2021.pdf
