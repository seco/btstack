# 
# Makefile to download and convert .hex files from DA14581 HCI Firmware release
#

all:
	./convert_hex_files.py

clean:
	rm -f *.h *.c

clean-scripts:
	rm -f *.hex
