CONTIKI_PROJECT=border-router
all: $(CONTIKI_PROJECT) 

CONTIKI = $(HOME)/contiki-git
TARGET=srf06-cc26xx
BOARD=sensortag/cc2650
TARGET=srf06-cc26xx
BOARD=sensortag/cc2650

#linker optimizations
SMALL=1

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"
CFLAGS += -DRF_CORE_CONF_CHANNEL=18
PROJECT_SOURCEFILES += slip-bridge.c

#Simple built-in webserver is the default.
#Override with make WITH_WEBSERVER=0 for no webserver.
#WITH_WEBSERVER=webserver-name will use /apps/webserver-name if it can be
#found in the /apps, /platform/$(TARGET)/apps/, or current directory (in that order).
# WITH_WEBSERVER=webserver for /apps/webserver
# WITH_WEBSERVER=raven-webserver for /platform/avr-raven/apps/raven-webserver/
#make clean before changing webservers!

#Note /apps/webserver contains a 2500 byte style sheet which is a severe test
#of the slip connection. Large MSS together with low baud rates without flow
#control will overrun the transmit buffer when the style sheet is requested.

WITH_WEBSERVER=1
ifeq ($(WITH_WEBSERVER),1)
CFLAGS += -DUIP_CONF_TCP=1
CFLAGS += -DWEBSERVER=1
PROJECT_SOURCEFILES += httpd-simple.c
else ifneq ($(WITH_WEBSERVER), 0)
APPS += $(WITH_WEBSERVER)
CFLAGS += -DUIP_CONF_TCP=1
CFLAGS += -DWEBSERVER=2
endif

ifeq ($(PREFIX),)
 PREFIX = aaaa::1/64
endif

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include

connect:
	sudo $(CONTIKI)/tools/tunslip6 -B 115200 -s /dev/ttyACM0 aaaa::1/64

$(CONTIKI)/tools/tunslip6:	$(CONTIKI)/tools/tunslip6.c
	(cd $(CONTIKI)/tools && $(MAKE) tunslip6)

connect-router:	$(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 $(PREFIX)

connect-router-cooja:	$(CONTIKI)/tools/tunslip6
	sudo $(CONTIKI)/tools/tunslip6 -a 127.0.0.1 $(PREFIX)

prog: 
	#/home/csse3010/ti/uniflash_3.4/uniflash.sh -ccxml ~/Downloads/sensortag.ccxml -targetOp reset -operation Erase
	#/home/csse3010/ti/uniflash_3.4/uniflash.sh -ccxml ~/Downloads/sensortag.ccxml -targetOp reset -operation Erase -program $(CONTIKI_PROJECT).elf
	/home/csse3010/Downloads/DSLite/DebugServer/bin/DSLite load -c ~/Downloads/DSLite/CC2650F128_TIXDS110_Connection.ccxml -f $(CONTIKI_PROJECT).elf
