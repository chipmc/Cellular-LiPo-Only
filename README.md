# Cellular-LiPo-Only

This project is meant to demonstrate an approach to data collection, power management and reporting for a soil monitoring use case.

# Cellular-LiPo-Only

This is a proof of concept outline.  Here is what you will need to do in order to put it to work:

1) Add your libraries and code to collect data from your sensors.  Data collection from the sensors is done in the takeMeasurements() function.

2) Make sure you have added your pinMode commands to startup

3) Set up your Webhook using SendCounts.json: add your Ubidots Token to the target URL (note: industrial Ubidots API endpoint included - you may need a different one)

4) Deploy the Webhook using the Particle CLI - particle webhook create SendCounts.json

5) Flash your Electron / Boron with the code and monitor using the Particle Console

6) Use the Particle.functions to control the device (1 for yes, 0 for no): 
    - MeasureNow - take measurement and send to Ubidots
    - Verbose Mode - Turns on and off more verbose meassages for monitoring device
    - LowPowerMode - Enables the device to sleep during the time between measurements