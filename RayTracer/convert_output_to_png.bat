magick convert canvas.ppm canvas.png
rem magick convert canvas.ppm canvas_ppm.bmp
rem magick convert canvas.png canvas_bmp.bmp
rem magick compare -verbose canvas.ppm canvas.png -compose src diff.txt
rem echo Exit Code is %errorlevel%
rem pause