#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <curl/curl.h>
#include <wiringPi.h> 
#include <termios.h>
#include <termio.h>
#include <fcntl.h>

#include "mosquitto.h"
#include "dht22.h"

/* IOT 設備基本資料 */
#define HOST        "iot.cht.com.tw"			//CHT IOT平台網址
#define MQTT_PORT   1883
#define API_KEY		" "							//填入Device 的 Key 
#define DEVICE_ID   " "							//填入Device 的 ID  

/* 感測器與硬體定義 */
#define LED     		0						//GPIO#17	 接pin11  LED
#define LED_ON			HIGH					//LED ON
#define LED_OFF			LOW						//LED OFF
#define LED_SENSOR_ID  	"LED1"					//填入Device 的 Sensor ID
#define HUM_SENSOR_ID	"HUM01"					//填入Device 的 Sensor ID
#define TEMP_SENSOR_ID 	"TEMP01"				//填入Device 的 Sensor ID
#define PM_SENSOR_ID	"PM01"					//填入Device 的 Sensor ID  
#define ERR_SENSOR_ID	"ERR01"					//填入Device 的 Sensor ID  

#define BUFFER_SIZE 128

int	g_thread_running = 1;
pthread_t  g_thread_to_post = { 0 };
int com_fd;

/* Send a HTTP POST */
int http_post(char *url, char *ck, char *body)
{
  	CURL                *pCurl = NULL;
  	struct  curl_slist  *pHeaders = NULL;
  	CURLcode             curlCode = { 0 };
  	char                 strCK[BUFFER_SIZE] = { 0 };
  	long                 sc = 0; // HTTP status code
  	int                  r = -1;
  
  	if ((pCurl = curl_easy_init()) == NULL)
    	return -1; // failed to initialize cURL
  
  	curl_easy_setopt(pCurl, CURLOPT_URL, url);
  
  	snprintf(strCK, BUFFER_SIZE, "CK: %s", ck);  
  	pHeaders = curl_slist_append(pHeaders, strCK); // API KEY
  
  	pHeaders = curl_slist_append(pHeaders, "Content-Type: application/json"); // my body is JSON
  
  	if (pHeaders == NULL)
    	goto  FUNCTION_END;
  
  	curl_easy_setopt(pCurl, CURLOPT_HTTPHEADER, pHeaders);
  
  	curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS, body);
  
  	r = -2;
  
  	if (curl_easy_perform(pCurl) == CURLE_OK) // perform the request
  	{ 
    	curlCode = curl_easy_getinfo(pCurl, CURLINFO_RESPONSE_CODE , &sc);
    	if (curlCode == CURLE_OK) // check status code
    	{
      		r = sc;
    	}
  	}
  
FUNCTION_END:
  
  	if (pHeaders)
    	curl_slist_free_all(pHeaders);
  
  	curl_easy_cleanup(pCurl);
  
  	return r;
}

/* Fore a thread to change rawdata of the input sensor */

void thread_to_post(void *arg)
{
  	int  sc = 0;
  	char url[BUFFER_SIZE];
  	char json[BUFFER_SIZE];
  	float hum,tempe;
  	char buf[50];
  	int ret, buf_len;
  	int pm25_value;
  	  	
	struct timeval tm;
	fd_set rfd, wfd, efd;  	
	
	tm.tv_sec = 5;	
	tm.tv_usec = 0;     	
	buf_len = 0;
	
  	// specify the URL
  	//先送Error code 初值 
  	snprintf(url, BUFFER_SIZE, "http://%s/iot/v1/device/%s/rawdata", HOST, DEVICE_ID); 
    snprintf(json, BUFFER_SIZE, "[{\"id\":\"%s\",\"value\":[\"%s\"]}]", ERR_SENSOR_ID, "0000");
    sc = http_post(url, API_KEY, json);
  	printf("%s%s\n",url,json);	    
	if (sc != 200)   
    	printf("HTTP POST is failed - %d\n", sc);    	    
	    	            	 	
  	while (g_thread_running)
  	{
		//*偵測溫濕度
		if(read_dht22_dat(&hum, &tempe) == 1)
		{
			printf("Humidity = %.2f %% Temperature = %.2f *C \n", hum, tempe);	  	
		  	//送Temperature value
		  	snprintf(url, BUFFER_SIZE, "http://%s/iot/v1/device/%s/rawdata", HOST, DEVICE_ID); 
		    snprintf(json, BUFFER_SIZE, "[{\"id\":\"%s\",\"value\":[\"%.2f\"]}]", TEMP_SENSOR_ID, tempe);        	
		    sc = http_post(url, API_KEY, json);
		  	printf("%s%s\n",url,json);		    
			if (sc != 200)   
		    	printf("HTTP POST is failed - %d\n", sc);    	    
		
		  	//先Humidity value
		  	snprintf(url, BUFFER_SIZE, "http://%s/iot/v1/device/%s/rawdata", HOST, DEVICE_ID); 
		    snprintf(json, BUFFER_SIZE, "[{\"id\":\"%s\",\"value\":[\"%.2f\"]}]", HUM_SENSOR_ID, hum);
		    sc = http_post(url, API_KEY, json);
		  	printf("%s%s\n",url,json);		
			if (sc != 200)   
		    	printf("HTTP POST is failed - %d\n", sc);    	
	    
		}      	
			
		//偵測空氣品質
		FD_ZERO(&rfd);
		FD_ZERO(&wfd);
		FD_ZERO(&efd);
		FD_SET(com_fd, &rfd);
		
		ret = select(com_fd+1, &rfd, &wfd, &efd, &tm);
		if(ret < 0) 
		{
			printf("select fail\n");
			break;
		}
		else if(ret == 0) 
		{
			continue;
		}
		
		if(FD_ISSET(com_fd, &rfd)) 
		{
			//ret = read(com_fd, buf+buf_len, sizeof(buf));		
			ret = read(com_fd,buf,1);	
			if(ret <= 0) 
			{
				printf("Read com port fail\n");
				break;
			}		
			buf_len +=ret;
			if(buf[0] ==0x42)
			{
				ret = read(com_fd,&buf[1],1);
				buf_len +=ret;
				if(buf[1] ==0x4D)
				{
					ret = read(com_fd,&buf[2],22);	
					buf_len +=ret;
				}	
			}	 		
			
		}				  	
		if((buf[0] != 0x42)  && (buf[1] != 0x4D))
		{
			buf_len = 0;
			sleep(1);   //1 秒
			continue;
		}
		else
		{
			if(buf_len == 24)
			{			
				printf("PMS3003 Sensor Data Get: ");
				//for(i=0;i<buf_len;i++)
				//	printf("%02X ",buf[i]); 
				//printf("\n");	
				printf("PM1.0:%i ,PM2.5:%i ,PM10:%i\n",buf[4]*256+buf[5],buf[6]*256+buf[7],buf[8]*256+buf[9]);			
				pm25_value = buf[6]*256+buf[7];
			  	
			  	//送出PM2.5的值到IOT Server
			  	snprintf(url, BUFFER_SIZE, "http://%s/iot/v1/device/%s/rawdata", HOST, DEVICE_ID); 
			    snprintf(json, BUFFER_SIZE, "[{\"id\":\"%s\",\"value\":[\"%d\"]}]", PM_SENSOR_ID, pm25_value);
			    sc = http_post(url, API_KEY, json);
			  	printf("%s%s\n",url,json);			    
				if (sc != 200)   
			    	printf("HTTP POST is failed - %d\n", sc);    								
			}
			buf_len = 0;
			sleep(9);   //1 秒
		}	 	
		sleep(1);   //1 秒  	
			
  	}
  
  	pthread_detach(g_thread_to_post);
}

/* MQTT connection is ready */
void mqtt_callback_connect(struct mosquitto *mosq, void *obj, int result)
{
  	int  r = 0;
  	char topic[BUFFER_SIZE];
	
	/* ERROR Code 異常狀態 */
  	snprintf(topic, BUFFER_SIZE, "/v1/device/%s/sensor/%s/rawdata", DEVICE_ID, ERR_SENSOR_ID);  
  	printf("MQTT: %s is connected.\n",ERR_SENSOR_ID );  
  	
  	r = mosquitto_subscribe(mosq, NULL, topic, 0);  
  	if (r == 0)
    	printf("MQTT topic %s is subscribed.\n",ERR_SENSOR_ID);
  	else
    	printf("Failed to subscribe the topic %s!\n", ERR_SENSOR_ID);
	
	/* Humidity 濕度 */
  	snprintf(topic, BUFFER_SIZE, "/v1/device/%s/sensor/%s/rawdata", DEVICE_ID, HUM_SENSOR_ID);  
  	printf("MQTT: %s is connected.\n",HUM_SENSOR_ID );  
  	
  	r = mosquitto_subscribe(mosq, NULL, topic, 0);  
  	if (r == 0)
    	printf("MQTT topic %s is subscribed.\n",HUM_SENSOR_ID);
  	else
    	printf("Failed to subscribe the topic %s!\n", HUM_SENSOR_ID);
    	
	/* Temperature 溫度 */
  	snprintf(topic, BUFFER_SIZE, "/v1/device/%s/sensor/%s/rawdata", DEVICE_ID, TEMP_SENSOR_ID);  
  	printf("MQTT: %s is connected.\n",TEMP_SENSOR_ID );  
  	
  	r = mosquitto_subscribe(mosq, NULL, topic, 0);  
  	if (r == 0)
    	printf("MQTT topic %s is subscribed.\n",TEMP_SENSOR_ID);
  	else
    	printf("Failed to subscribe the topic %s!\n", TEMP_SENSOR_ID);    	

	/* PM2.5 */
  	snprintf(topic, BUFFER_SIZE, "/v1/device/%s/sensor/%s/rawdata", DEVICE_ID, PM_SENSOR_ID);  
  	printf("MQTT: %s is connected.\n",PM_SENSOR_ID );  
  	
  	r = mosquitto_subscribe(mosq, NULL, topic, 0);  
  	if (r == 0)
    	printf("MQTT topic %s is subscribed.\n",PM_SENSOR_ID);
  	else
    	printf("Failed to subscribe the topic %s!\n", PM_SENSOR_ID);    		
}

/* MQTT connection is lost */

void mqtt_callback_disconnect(struct mosquitto *mosq, void *obj, int result)
{
  printf("MQTT is disconnected!");
 
  mosquitto_disconnect(mosq);
}

/* Received the MQTT message */

void mqtt_callback_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
  	if ((message->topic == NULL) || (message->payload == NULL) || (message->payloadlen <= 0))   
    	return;
 
 	// just print out the payload
  	//printf("[%s]\n", message->topic);
 	//printf("%s\n", (char *)message->payload);
}

/* Establish the MQTT connection to subscribe the rawdata topic */
int mqtt_loop(char *host, int port, char *username, char *password)
{
  	struct mosquitto  *mosq;
  	int r = 0;
  
  	mosquitto_lib_init();
  
  	if ((mosq = mosquitto_new(NULL, true, NULL)) == NULL)
  	{    
    	r = -1;    
    	printf("Failed to build the MQTT object.\n" );    
    	goto  FUNCTION_END;
  	}
  	// declare the callback functions  
  	mosquitto_connect_callback_set(mosq, mqtt_callback_connect);    		
  	mosquitto_message_callback_set(mosq, mqtt_callback_message);    	
  	mosquitto_disconnect_callback_set(mosq, mqtt_callback_disconnect);
  	// authentication  
  	r = mosquitto_username_pw_set(mosq, username, password);  
  	if (r)
  	{    
    	r = -2;    
    	printf("Failed to setup the MQTT authentication - %d.\n", r);    
    	goto  FUNCTION_END;
  	}
  	// connect and wait  
  	if (mosquitto_connect(mosq, host, port, 60) == MOSQ_ERR_SUCCESS)
  	{    
    	r = 0;    
    	mosquitto_loop_forever(mosq, -1, 1);
  	}
  	else
  	{    
    	r = -3;    
    	printf("Failed to connect to MQTT broker.\n");
  	}
  
  	mosquitto_destroy(mosq);
  
FUNCTION_END:  
  	mosquitto_lib_cleanup();  
  	return  r;
}

//LED 閃爍  count:閃爍次數  ms:閃爍間隔(m sec)
void led_flash(int count, int ms)
{
	while(count >0)
  	{
  		digitalWrite(LED,LED_ON);
  		delay(ms);
  		digitalWrite(LED,LED_OFF);
  		delay(ms);
  		count--;
  	}  
}	

int init_comport(char *port, int baudrate)
{
	struct termios oldtio; 
	struct termios newtio;  	
	//char com_port[12];
	int fd;				

//	/* 開啟 COM2 */	
//	memset(com_port, 0x00, 12);
//	switch(port)
//	{
//			case 0:
//					memcpy(com_port, "/dev/ttyS0", 10);
//					break;			
//			case 1:
//					memcpy(com_port, "/dev/ttyS1", 10);
//					break;
//			case 2:
//					memcpy(com_port, "/dev/ttyS2", 10);
//					break;
//			case 3:
//					memcpy(com_port, "/dev/ttyS3", 10);
//					break;
//			case 4:
//					memcpy(com_port, "/dev/ttyS4", 10);
//					break;		
//	}
	fd = open(port, O_RDWR | O_NOCTTY |O_NDELAY);
	if(fd <0)
	{
		printf("Open com port %s fail!!\n", port);
		return(fd);
	}	
	else
	{
		printf("Open com port %s OK\n", port);
	}		
 
	/* 設定COM 通訊參數  */
	tcgetattr(fd, &oldtio);	
	memset(&newtio, 0, sizeof(newtio));
	newtio = oldtio;
	newtio.c_iflag = 0;
	newtio.c_oflag &=  ~(OPOST | ONLCR | OCRNL);
	newtio.c_cflag &=  CSIZE | CSTOPB | CREAD | PARENB | PARODD | HUPCL | CLOCAL;
	newtio.c_cflag |= (baudrate | CS8 | CLOCAL | CREAD);
		
	newtio.c_lflag &= ~(ECHO | ICANON | ECHONL | IEXTEN | ISIG);
	newtio.c_cc[VMIN]=1;
	newtio.c_cc[VTIME]=1;
	tcflush(fd,TCIFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);
	return(fd);	
}	

int main(int argc, char **argv)
{
  	int ret = 0;
	
  	//GPIO 初始化
  	wiringPiSetup();
  	pinMode(LED, OUTPUT);
  	
  	//Serial Port 初始化
  	com_fd = init_comport("/dev/ttyAMA0",B9600);			//ttyS0
  	if(com_fd <0)
  		printf("Can not open ttyAMA0\n");
  	else
  		printf("Open ttyS0 OK, baudrate 9600 bps\n");
  		
  	//開始執行程式 LED閃三下
  	led_flash(3,200);
  	//測試溫濕度Seneor
	//測試PM2.5 Sensor
		  	
  	//fork a thread to change the rawdata of the INPUT mode sensor  
  	if (pthread_create(&g_thread_to_post, NULL, (void *) thread_to_post, NULL))
  	{ 
    	printf("Failed to create a thread to change the rawdata.\n");    
    	return 1;
  	}
  
  	ret = mqtt_loop(HOST, MQTT_PORT, API_KEY, API_KEY); // blocking here
  	if (ret)
    	printf("MQTT is broken - %d\n", ret);

  	// stop the rawdata saving thread  
  	g_thread_running = 0;
  
 	pthread_join(g_thread_to_post, NULL);
  
  	return  0;
}

