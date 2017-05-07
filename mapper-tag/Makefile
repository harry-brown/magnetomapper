CONTIKI_PROJECT=mapper
all: $(CONTIKI_PROJECT)

CONTIKI = $(HOME)/contiki-git
CONTIKI_WITH_IPV6 = 1
TARGET=srf06-cc26xx
BOARD=sensortag/cc2650
#UIP_CONF_ROUTER = 0
CFLAGS += -DUIP_CONF_ND6_SEND_NA=1
CFLAGS += -DUIP_CONF_ND6_SEND_NS=1
CFLAGS += -DRF_CORE_CONF_CHANNEL=18
APPS += json
include $(CONTIKI)/Makefile.include


erase: 
	$(HOME)/uniflash_3.4/uniflash.sh -ccxml ~/Downloads/sensortag.ccxml -targetOp reset -operation Erase

prog: 
	$(HOME)/Downloads/DSLite/DebugServer/bin/DSLite load -c ~/Downloads/DSLite/CC2650F128_TIXDS110_Connection.ccxml -f $(CONTIKI_PROJECT).elf

progu: 
	$(HOME)/uniflash_3.4/uniflash.sh -ccxml ~/Downloads/sensortag.ccxml -targetOp reset -operation Erase
	$(HOME)/uniflash_3.4/uniflash.sh -ccxml ~/Downloads/sensortag.ccxml -program $(CONTIKI_PROJECT).hex
