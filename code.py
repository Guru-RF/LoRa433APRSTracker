import aprslib
import time
from math import sin, cos, sqrt, atan2, radians

packet = aprslib.packets.PositionReport()
packet.fromcall='ON3URE'  #Insert your hamradio call sign
packet.tocall='WIDE1'
packet.comment=' '
packet.symbol='>'

print(packet)

        
