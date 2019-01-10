Algo main TP

start-up procedure
 create rover
 do some checks eg. I2C devices
 wait for routing to be activated from front end
loop forever
  every 0.1s
  check if routing has been updated, if so load it, stop mower and push new routing to it
  check rover status (segment and power)
  update database
endloop
  
