# TSDZ2 Pinout

## Display Connector (8pin)

color  | description                                       
------ | -------------------------------------------------
blue   | V_BAT+                                           
black  | GND                                              
red    | ENABLE (connector to V_BAT+ to enable controller)
brown  | UART_TX                                          
yellow | UART_RX                                          
green  | BRAKE                                            
white  | 5V                                               
orange | THROTTLE                                         


## Speed Sensor Connector
color  | type   | description
------ | ------ | -----------
black  | IN/OUT | SWIM (programming interface)
brown  | IN/OUT | V_MCU (e.g. supply for programming)
green  | OUTPUT | V_HEADLIGHT
orange | -      | GROUND
purple | INPUT  | RST (not needed with stlink v2)
white  | IN/OUT | speed sensor / low active

## Remote Controller

color  | description
------ | ----------- 
black  | GND
brown  | BTN (-)
green  | BTN INFO
orange | BTN PWR
white  | BTN (+)

## Throttle

color  | description
------ | -----------
white  | GND
orange | +5V
blue   | SIGNAL



## Custom Cable Harness

orignal  | custom
-------- | -------------------------- 
UART_TX  | Remote BTN (+)
UART_RX  | Remote BTN (-)
BRAKE    | BTN INFO || BTN PWR
THROTTLE | THROTTLE
+5V      | +5V THROTTLE
V_BAT+	 | -> ENABLE
ENABLE   | -> VBAT+
