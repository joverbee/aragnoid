# Aragnoid
 Tame that tractor, an ARAG wisperer
 Use RTKsimple2b to receive high precision GPS coordinates in RTK
 Transform these coordinates in a way that the closed system ARAG400
 accepts this datastream as if it is coming from its own GPS module but now with higher localisation precision

# HW setup
Using Arduin MKRZero board
Connect Arag serial to RX and TX on pin 13/14, make use of a proper RS232 level interface chip
Connect RTKsimple module RX and TX on PIN 1/0, this is the connection to RTKsimple,make use of a RS232 interface board
Connect Computer for debug messages to conventional usbserial port

# Example data
GPGGA parsed
$GPGGA,183538.70,5056.7186660,N,00446.6231208,E,4,12,0.64,17.998,M,46.220,M,0.7,4035*48
$GNVTG,335.788,T,335.788,M,0.001,N,0.002,K,A*3E
GPGGA parsed
$GPGGA,183538.80,5056.7186657,N,00446.6231210,E,4,12,0.64,18.002,M,46.220,M,0.8,4035*40

# Upgrade ideas
add 2 RTK to allow for proper heading information, this is important when tractor is changing direction
add gyro and compasss module to do the same
add tilt for sloping fields?

# config RTKsimple (very important!)
output on UART1 115200 8N1 LSB FIRST
nmea message in main talker ID 1-GP mode (P prefix)
GGA on
VTG on
ZDA on with factor 10 to send only every 1s
PUBX04 on
NAV5 dynamic model 'pedestrian' (important otherwise you get filtering of the position in time)

update speed 100ms




