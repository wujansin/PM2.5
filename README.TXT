
1. prepare the 'libcurl' first

	apt-get install libcurl-devel

2. login to 'https://iot.cht.com.tw'

   a) create a project and read the API KEY from detail.
   b) create a device and get it's numeric ID from URL.
   c) create  sensors 

3. modify the API_KEY, DEVICE_ID, SENSOR_ID in pm_sys.c

4. make

5. ./iot_demo

