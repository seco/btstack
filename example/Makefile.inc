
VPATH += ${BTSTACK_ROOT}/src
VPATH += ${BTSTACK_ROOT}/src/ble
VPATH += ${BTSTACK_ROOT}/src/ble/gatt-service
VPATH += ${BTSTACK_ROOT}/src/classic
VPATH += ${BTSTACK_ROOT}/example
VPATH += ${BTSTACK_ROOT}/3rd-party/mbedtls/library
VPATH += ${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/srce 
VPATH += ${BTSTACK_ROOT}/3rd-party/bluedroid/encoder//srce

CFLAGS += -I.
CFLAGS += -I${BTSTACK_ROOT}/src/ble
CFLAGS += -I${BTSTACK_ROOT}/src/classic
CFLAGS += -I${BTSTACK_ROOT}/src
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/mbedtls/include
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/include
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/include

CFLAGS += -I${BTSTACK_ROOT}/test/security_manager

# for CVSD/SBC PLC
LDFLAGS += -lm

CORE += \
	btstack_memory.c            \
	btstack_linked_list.c	    \
	btstack_memory_pool.c       \
	btstack_run_loop.c		    \
	btstack_util.c 	            \

COMMON += \
	hci.c			            \
	hci_cmd.c		            \
	hci_dump.c		            \
	l2cap.c			            \
	l2cap_signaling.c	        \

CLASSIC += \
	btstack_link_key_db_memory.c   \
	sdp_util.c	                \
	spp_server.c  				\
	rfcomm.c	                \
	bnep.c	                    \
	sdp_server.c			            \
	device_id_server.c          \

SDP_CLIENT += \
	sdp_client.o		        \
	sdp_client_rfcomm.o		    \

ATT	+= \
	att_dispatch.c       	    \

GATT_SERVER += \
	att_db.c 				 	    \
	att_server.c        	    \

GATT_CLIENT += \
	ad_parser.c                 \
	gatt_client.c        	    \

SM += \
	sm.c 				 	    \

PAN += \
	pan.c \

MBEDTLS = 					\
	bignum.c 				\
	ecp.c 					\
	ecp_curves.c 			\
	sm_mbedtls_allocator.c  \
	memory_buffer_alloc.c   \
	platform.c 				\

# List of files for Bluedroid SBC codec
include ${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/Makefile.inc
include ${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/Makefile.inc

SBC_DECODER += \
	btstack_sbc_plc.c \
	btstack_sbc_bludroid.c \

SBC_ENCODER += \
	btstack_sbc_bludroid.c \
	hfp_msbc.c \

CVSD_PLC = \
	btstack_cvsd_plc.c \


EXAMPLES = 					\
	ancs_client_demo		\
	gap_dedicated_bonding	\
	gap_inquiry 			\
	gatt_battery_query		\
	gatt_browser            \
	le_counter              \
	le_streamer				\
	le_streamer_client      \
	led_counter				\
	sdp_bnep_query 			\
	sdp_general_query		\
	sdp_rfcomm_query		\
	spp_and_le_counter 		\
	spp_counter 			\
	spp_streamer			\
	gap_le_advertisements   \
	hfp_ag_demo             \
	hfp_hf_demo             \
	hsp_ag_demo             \
	hsp_hs_demo             \
	sm_pairing_peripheral   \
	sm_pairing_central      \
	pbap_client_demo		\

EXAMPLES_USING_LE =			\
	ancs_client_demo		\
	gatt_battery_query      \
	gatt_browser			\
	le_counter              \
	le_streamer				\
	le_streamer_client      \
	spp_and_le_counter	    \
	gap_le_advertisements   \
	sm_pairing_peripheral   \
	sm_pairing_central      \

# .o for .c
CORE_OBJ    = $(CORE:.c=.o)
COMMON_OBJ  = $(COMMON:.c=.o) 
CLASSIC_OBJ  = $(CLASSIC:.c=.o) 
SM_OBJ = $(SM:.c=.o) $(MBEDTLS:.c=.o)
ATT_OBJ     = $(ATT:.c=.o)
GATT_CLIENT_OBJ = $(GATT_CLIENT:.c=.o)
GATT_SERVER_OBJ = $(GATT_SERVER:.c=.o)
PAN_OBJ = $(PAN:.c=.o)
SBC_DECODER_OBJ  = $(SBC_DECODER:.c=.o) 
SBC_ENCODER_OBJ  = $(SBC_ENCODER:.c=.o)
CVSD_PLC_OBJ = $(CVSD_PLC:.c=.o)

default_target: all

# compile .gatt descriptions
profile.h: profile.gatt
	python ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@ 
ancs_client_demo.h: ancs_client_demo.gatt
	python ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@ 
gatt_browser.h: gatt_browser.gatt
	python ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@ 
gatt_battery_query.h: gatt_battery_query.gatt
	python ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@ 
spp_and_le_counter.h: spp_and_le_counter.gatt
	python ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@ 
le_counter.h: le_counter.gatt
	python ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@ 
le_streamer.h: le_streamer.gatt
	python ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@ 
sm_pairing_peripheral.h: sm_pairing_peripheral.gatt
	python ${BTSTACK_ROOT}/tool/compile_gatt.py $< $@ 

# examples
sdp_rfcomm_query: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${PAN_OBJ} ${SDP_CLIENT} sdp_rfcomm_query.c
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

pbap_client_demo: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} obex_iterator.c goep_client.c pbap_client.c pbap_client_demo.c
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

sdp_general_query: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} sdp_general_query.c  
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

sdp_bnep_query: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} sdp_bnep_query.c  
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

spp_counter: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} spp_counter.c
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

le_counter: le_counter.h ${CORE_OBJ} ${COMMON_OBJ} ${ATT_OBJ} ${GATT_SERVER_OBJ} ${SM_OBJ} battery_service_server.o le_counter.c 
	${CC} $(filter-out le_counter.h,$^) ${CFLAGS} ${LDFLAGS} -o $@

sm_pairing_peripheral: sm_pairing_peripheral.h ${CORE_OBJ} ${COMMON_OBJ} ${ATT_OBJ} ${GATT_SERVER_OBJ} ${SM_OBJ} sm_pairing_peripheral.c 
	${CC} $(filter-out sm_pairing_peripheral.h,$^) ${CFLAGS} ${LDFLAGS} -o $@

sm_pairing_central: ${CORE_OBJ} ${COMMON_OBJ} ${SM_OBJ} ad_parser.o sm_pairing_central.o 
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

le_streamer: le_streamer.h ${CORE_OBJ} ${COMMON_OBJ} ${ATT_OBJ} ${GATT_SERVER_OBJ} ${SM_OBJ} le_streamer.c 
	${CC} $(filter-out le_streamer.h,$^) ${CFLAGS} ${LDFLAGS} -o $@

le_streamer_client: ${CORE_OBJ} ${COMMON_OBJ} ${ATT_OBJ} ${GATT_CLIENT_OBJ} ${SM_OBJ} le_streamer_client.c
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

spp_and_le_counter: spp_and_le_counter.h ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${ATT_OBJ} ${GATT_SERVER_OBJ} ${SM_OBJ} spp_and_le_counter.c 
	${CC} $(filter-out spp_and_le_counter.h,$^)  ${CFLAGS} ${LDFLAGS} -o $@

spp_counter_ssp: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} spp_counter_ssp.c
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

spp_streamer: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} spp_streamer.c  
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

gap_dedicated_bonding: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} gap_dedicated_bonding.c
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

gap_inquiry: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ad_parser.o gap_inquiry.c
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

panu_demo: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} panu_demo.c  
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

gatt_browser: gatt_browser.h ${CORE_OBJ} ${COMMON_OBJ} ${ATT_OBJ} ${GATT_CLIENT_OBJ} ${GATT_SERVER_OBJ} ${SM_OBJ} gatt_browser.c
	${CC} $(filter-out gatt_browser.h,$^) ${CFLAGS} ${LDFLAGS} -o $@

gatt_battery_query: gatt_battery_query.h ${CORE_OBJ} ${COMMON_OBJ} ${ATT_OBJ} ${GATT_CLIENT_OBJ} ${GATT_SERVER_OBJ} ${SM_OBJ} gatt_battery_query.c
	${CC} $(filter-out gatt_battery_query.h,$^) ${CFLAGS} ${LDFLAGS} -o $@

ancs_client_demo: ancs_client_demo.h ${CORE_OBJ} ${COMMON_OBJ} ${ATT_OBJ} ${GATT_SERVER_OBJ} ${GATT_CLIENT_OBJ} ${SM_OBJ} ancs_client.c ancs_client_demo.c
	${CC} $(filter-out ancs_client_demo.h,$^) ${CFLAGS} ${LDFLAGS} -o $@

led_counter: ${CORE_OBJ} ${COMMON_OBJ} led_counter.c
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

gap_le_advertisements: ${CORE_OBJ} ${COMMON_OBJ} ad_parser.c gap_le_advertisements.c
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

hsp_hs_demo: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} ${SBC_DECODER_OBJ} ${SBC_ENCODER_OBJ} ${CVSD_PLC_OBJ} wav_util.o sco_demo_util.o btstack_ring_buffer.o hsp_hs.o hsp_hs_demo.c
	${CC} $^ ${CFLAGS} -I${BTSTACK_ROOT}/platform/posix ${LDFLAGS} -o $@

hsp_ag_demo: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} ${SBC_DECODER_OBJ} ${SBC_ENCODER_OBJ} ${CVSD_PLC_OBJ} wav_util.o sco_demo_util.o btstack_ring_buffer.o hsp_ag.o hsp_ag_demo.c
	${CC} $^ ${CFLAGS} -I${BTSTACK_ROOT}/platform/posix ${LDFLAGS} -o $@

hfp_ag_demo: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} ${SBC_DECODER_OBJ} ${SBC_ENCODER_OBJ} ${CVSD_PLC_OBJ} wav_util.o sco_demo_util.o btstack_ring_buffer.o hfp.o hfp_gsm_model.o hfp_ag.o hfp_ag_demo.c
	${CC} $^ ${CFLAGS} -I${BTSTACK_ROOT}/platform/posix ${LDFLAGS} -o $@

hfp_hf_demo: ${CORE_OBJ} ${COMMON_OBJ} ${CLASSIC_OBJ} ${SDP_CLIENT} ${SBC_DECODER_OBJ} ${SBC_ENCODER_OBJ} ${CVSD_PLC_OBJ} wav_util.o sco_demo_util.o btstack_ring_buffer.o hfp.o hfp_hf.o hfp_hf_demo.c
	${CC} $^ ${CFLAGS} -I${BTSTACK_ROOT}/platform/posix ${LDFLAGS} -o $@

clean:
	rm -f ${EXAMPLES}
	rm -f  *.o *.out *.hex *.exe
	rm -f  ancs_client_demo.h profile.h spp_and_le_counter.h le_counter.h le_streamer.h
	rm -rf *.dSYM
	rm -rf ${BTSTACK_ROOT}/src/*.o
	rm -rf ${BTSTACK_ROOT}/src/ble/*.o
	rm -rf ${BTSTACK_ROOT}/src/ble/gatt-service/*.o
	rm -rf ${BTSTACK_ROOT}/example/*.o
	rm -rf ${BTSTACK_ROOT}/example/ancs_client_demo.h
	rm -rf ${BTSTACK_ROOT}/example/profile.h
	rm -rf ${BTSTACK_ROOT}/example/spp_and_le_counter.h
	rm -rf ${BTSTACK_ROOT}/example/le_counter.h
	rm -rf ${BTSTACK_ROOT}/example/le_streamer.h
	
