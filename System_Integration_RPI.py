import smbus2
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [0, 100, 0]
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus2.SMBus(1)
time.sleep(1)

#this is the address we setup in the Arduino Program
address = 0x04
while True:
    #result = "3pi/2" Testing values
    #result1 = 4 Tesing Values
    
    #Writes the quadrant to the UNO, converts to angle there
    bus.write_byte_data(0x04, 0, quad)
    
    #Display desried angle on lcd
    lcd.color = [0, 100, 0]
    setPointDisp = "Setpoint: " + result
    lcd.message = setPointDisp
    
    #time.sleep(1)
        
    #Reads current position from UNO
    readPosition = bus.read_i2c_block_data(0x04, 0, 2)
    
    #bitshifts back to full number
    fullPosition = (readPosition[0] << 8) | readPosition[1]
    #print(fullPosition)
    
    #time.sleep(1)
    
    #converts to actual angle
    actualPosition = fullPosition/3210 * 2*3.14;
        
    #Displays current position along with desired setpoint on two lines
    fullMessage = setPointDisp + "\nActual: " + str(actualPosition)
    lcd.message = fullMessage