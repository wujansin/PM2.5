# pm2.5, temperature and humidity sensor for CHT IOT Platform
## 1. prepare the 'libcurl' first

###	sudo apt-get install libcurl-devel

## 2. login to 'https://iot.cht.com.tw'

###   a) create a project and read the API KEY from detail.
###   b) create a device and get it's numeric ID from URL.
###   c) create 2 sensors - names 'Input' and 'Output'.

## 3. modify the API_KEY, DEVICE_ID and SENSOR IDs in pm_sys.c file

## 4. make file
###   $ make
## 5. Execute program
###   $ ./iot_demo
