from  pycreate2 import Create2
import time

# Create a Create2.
port = "COM3"  # where is your serial port?
bot = Create2(port)

# Start the Create 2
bot.start()

# Put the Create2 into 'safe' mode so we can drive it
# This will still provide some protection
bot.safe()


for i in range(10):
    if i % 2 == 0:
        bot.drive_direct(100, 100)
    else:
        bot.drive_direct(-100, -100)
    
    time.sleep(0.1)

# Stop the bot
bot.drive_stop()

#go back to passive mode
bot.power()

# query some sensors
#sensors = bot.get_sensors()  # returns all data
#print(sensors.battery_charge)
#print(sensors.battery_capacity)

#Puts the Create 2 into OFF mode. Use this command when you are finished
bot.stop()

# Close the connection
bot.close()
