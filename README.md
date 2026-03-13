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

# Example data from novatel
$GNVTG,15.228,T,15.228,M,0.000,N,0.000,K,A*3D
$GNVTG,15.226,T,15.226,M,0.010,N,0.019,K,A*34
$GNVTG,15.226,T,15.226,M,0.010,N,0.019,K,A*34
$GNVTG,323.767,T,323.767,M,0.005,N,0.008,K,A*30


# Example data
$GPGGA,183538.70,5043.7286660,N,00436.5241308,E,4,12,0.64,17.998,M,46.220,M,0.7,4035*48
$GNVTG,335.788,T,335.788,M,0.001,N,0.002,K,A*3E
$GPGGA,183538.80,5043.7286657,N,00436.5241310,E,4,12,0.64,18.002,M,46.220,M,0.8,4035*40

# Upgrade ideas
add 2 RTK to allow for proper heading information, this is important when tractor is changing direction
add gyro and compasss module to do the same (CHECK)
add tilt for sloping fields? (CHECK)

# config RTKsimple (very important!)
output on UART1 115200 8N1 LSB FIRST
nmea message in main talker ID 1-GP mode (P prefix)
GGA on
VTG on
ZDA on with factor 10 to send only every 1s (read in arag documentation, I think this needs to be only 0.1Hz so factor 100)
PUBX04 on (needed?)
NAV5 dynamic model 'pedestrian' (important: this filters for slow moving objects, precision will be higher)
update speed 100ms

# correction of heading when driving in reverse
A single gps receiver doesn't know what is front or back and calculates heading as a vector from previous position to current position (possibily filtered)
A tractor however can not rotate around its axis without some radius of curvature
If we detect a sudden change in direction that is closer to 180 degrees than 0 we know we are in reverse and the heading as far as the tractor is concerned 
is still forward. We correct the heading by adding 180 to the gps heading and correcting for cases where it would fall outside the [0-360[ range
This is important for the arag as otherwise it will assume we made a full circle and we sprayed a semicircular area while we didn't.

There is a fault scenario however as the GPS still doesnt know what is forward and backward and could be thinking the whole time that we are driving backward depending on the initial conditions when starting up.
To solve this you can tie a pushbutton between pin 3 and gnd and press it once when driving forward to reset the direction.
Hopefully this is only rarely needed?
Taking actual heading from an IMU like BNO085 would avoid this problem

# sensor fusion
todo for better accuracy: needed in practice?

# known problems
orange lines from dropped messages?
reversing fails
crossing 10 kmh seems to drop communication?

# with ardusimple
$GPVTG,,T,,M,,N,,K,D*26
$GPVTG,,T,,M,,N,,K,D*26
$GPVTG,,T,,M,,N,,K,D*26

# with NMEA simulator
$GNVTG,25.788,T,25.788,M,0.001,N,0.002,K,A*3E


