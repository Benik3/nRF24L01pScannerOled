# nRF24L01pScannerOled
2,4GHz spectral scanner with nRF24L01 and SSD1306/SSD1106 OLED display.  
Original came from Ceptimus: https://www.rcgroups.com/forums/showthread.php?2777178-Very-cheap-Arduino-based-2-4-GHz-band-monitor

I made some changes for faster show up of data:
- Asynchronous scan and OLED draw - scanning is made with interrupt
- Redraw only bytes which really changed from previous draw
- Function to draw line from array for faster draw of long line
- Faster I2C clock by default (800kHz)
- faster SSD1306 OLED clock

Tested with SSD1306, if you will find any errors (e.g. with SSD1106) let me know and I will try to fix it :)
