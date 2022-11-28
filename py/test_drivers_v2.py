import drivers_v2.drivers_v2 as drv2
dart_bot = drv2.DartV2DriverV2()
df,dl,db,dr = dart_bot.sonars.read_4_sonars()
print ("sonars",df,dl,db,dr)
dart_bot.end()
