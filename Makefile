TARGET = iot_demo

CC = gcc

CFLAGS = -Wall 

INCLUDES = -I libmqtt

LIBS = libmqtt/*.c -lcurl -lpthread -lcrypt -lrt -lm -lwiringPi

SOURCES = pm_sys.c dht22.c

all : 
	${CC} ${CFLAGS} ${INCLUDES} ${LIBS} ${SOURCES} -o ${TARGET}

clean :
	rm -fr ${TARGET}

