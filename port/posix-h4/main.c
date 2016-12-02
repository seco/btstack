/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

// *****************************************************************************
//
// minimal setup for HCI code
//
// *****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include "btstack_config.h"

#include "btstack_debug.h"
#include "btstack_event.h"
#include "btstack_link_key_db_fs.h"
#include "btstack_memory.h"
#include "btstack_run_loop.h"
#include "btstack_run_loop_posix.h"
#include "hci.h"
#include "hci_dump.h"
#include "stdin_support.h"

#include "btstack_chipset_bcm.h"
#include "btstack_chipset_csr.h"
#include "btstack_chipset_cc256x.h"
#include "btstack_chipset_em9301.h"
#include "btstack_chipset_stlc2500d.h"
#include "btstack_chipset_tc3566x.h"

#define HAVE_DA14581

#ifdef HAVE_DA14581
#include "hci_581_active_uart.h"
#endif

int is_bcm;

int btstack_main(int argc, const char * argv[]);

static hci_transport_config_uart_t config = {
    HCI_TRANSPORT_CONFIG_UART,
    115200,
    0,  // main baudrate
    1,  // flow control
    NULL,
};

static btstack_packet_callback_registration_t hci_event_callback_registration;

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    if (packet_type != HCI_EVENT_PACKET) return;
    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
            printf("BTstack up and running.\n");
            break;
        case HCI_EVENT_COMMAND_COMPLETE:
            if (HCI_EVENT_IS_COMMAND_COMPLETE(packet, hci_read_local_name)){
                if (hci_event_command_complete_get_return_parameters(packet)[0]) break;
                // terminate, name 248 chars
                packet[6+248] = 0;
                printf("Local name: %s\n", &packet[6]);
                if (is_bcm){
                    btstack_chipset_bcm_set_device_name((const char *)&packet[6]);
                }
            }        
            break;
        default:
            break;
    }
}

static void sigint_handler(int param){

#ifndef _WIN32
    // reset anyway
    btstack_stdin_reset();
#endif

    log_info(" <= SIGINT received, shutting down..\n");   
    hci_power_control(HCI_POWER_OFF);
    hci_close();
    log_info("Good bye, see you.\n");    
    exit(0);
}

static int led_state = 0;
void hal_led_toggle(void){
    led_state = 1 - led_state;
    printf("LED State %u\n", led_state);
}
static void use_fast_uart(void){
#if defined(HAVE_POSIX_B240000_MAPPED_TO_3000000) || defined(HAVE_POSIX_B600_MAPPED_TO_3000000)
    printf("Using 3000000 baud.\n");
    config.baudrate_main = 3000000;
#elif defined(HAVE_POSIX_B1200_MAPPED_TO_2000000) || defined(HAVE_POSIX_B300_MAPPED_TO_2000000)
    printf("Using 2000000 baud.\n");
    config.baudrate_main = 2000000;
#else
    printf("Using 921600 baud.\n");
    config.baudrate_main = 921600;
#endif
}

static void local_version_information_callback(uint8_t * packet){
    printf("Local version information:\n");
    uint16_t hci_version    = little_endian_read_16(packet, 4);
    uint16_t hci_revision   = little_endian_read_16(packet, 6);
    uint16_t lmp_version    = little_endian_read_16(packet, 8);
    uint16_t manufacturer   = little_endian_read_16(packet, 10);
    uint16_t lmp_subversion = little_endian_read_16(packet, 12);
    printf("- HCI Version  0x%04x\n", hci_version);
    printf("- HCI Revision 0x%04x\n", hci_revision);
    printf("- LMP Version  0x%04x\n", lmp_version);
    printf("- LMP Revision 0x%04x\n", lmp_subversion);
    printf("- Manufacturer 0x%04x\n", manufacturer);
    switch (manufacturer){
        case COMPANY_ID_CAMBRIDGE_SILICON_RADIO:
            printf("Cambridge Silicon Radio - CSR chipset.\n");
            use_fast_uart();
            hci_set_chipset(btstack_chipset_csr_instance());
            break;
        case COMPANY_ID_TEXAS_INSTRUMENTS_INC: 
            printf("Texas Instruments - CC256x compatible chipset.\n");
            use_fast_uart();
            hci_set_chipset(btstack_chipset_cc256x_instance());
#ifdef ENABLE_EHCILL
            printf("eHCILL enabled.\n");
#else
            printf("eHCILL disable.\n");
#endif
            break;
        case COMPANY_ID_BROADCOM_CORPORATION:   
            printf("Broadcom - using BCM driver.\n");
            hci_set_chipset(btstack_chipset_bcm_instance());

            use_fast_uart();
            is_bcm = 1;
            break;
        case COMPANY_ID_ST_MICROELECTRONICS:   
            printf("ST Microelectronics - using STLC2500d driver.\n");
            use_fast_uart();
            hci_set_chipset(btstack_chipset_stlc2500d_instance());
            break;
        case COMPANY_ID_EM_MICROELECTRONICS_MARIN:
            printf("EM Microelectronics - using EM9301 driver.\n");
            hci_set_chipset(btstack_chipset_em9301_instance());
            break;
        case COMPANY_ID_NORDIC_SEMICONDUCTOR_ASA:
            printf("Nordic Semiconductor nRF5 chipset.\n");
            break;        
        default:
            printf("Unknown manufacturer / manufacturer not supported yet.\n");
            break;
    }
}

#ifdef HAVE_DA14581

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>  /* POSIX terminal control definitions */

//
#define SOH         0x01
#define STX         0x02
#define ACK         0x06
#define NACK        0x15
#define CRC_INIT    0x00

static int safe_write(int fd, const void* buf, size_t len)
{
    ssize_t n;

    do {
        n = write(fd, buf, len);
        if (n > 0) {
            // printf("Wrote %u from %u: ", (int) n, (int) len);
            // printf_hexdump(buf, n);
            len -= n;
            buf += n;
        } else if (n < 0  &&  (errno != EINTR && errno != EAGAIN))
            return -1;
    } while (len);

    return 0;
}

static int send_file(int ser_fd, int file_fd, int file_size, uint8_t* file_crc)
{
    uint8_t fcrc = CRC_INIT;
    int n, pos;

    for (pos = 0; pos < file_size; ) {
        // printf("send pos %u\n", pos);
#define FBUF_SIZE 4096
        uint8_t buf[FBUF_SIZE];
        int i, count;

        /* read a chunk */
        count = file_size - pos;
        if (count > FBUF_SIZE)
            count = FBUF_SIZE;
        n = read(file_fd, buf, count);
        if (-1 == n) {
            if (errno != EINTR) {
                perror("reading from file");
                return -1;
            } else
                continue;
        }

        /* update crc */
        for (i = 0; i < n; i++)
            fcrc ^= buf[i];

        /* write chunk */
        if (safe_write(ser_fd, buf, n) < 0) {
            perror("writing to serial port");
            return -1;
        }

        pos += n;
    }

    *file_crc = fcrc;
    return 0;
}

static int dialog_download_hci_firmware(int fd)
{
    struct stat sbuf;
    const char fw_fname[] = "hci-firmware.bin";
    int cnt = 0, fw = -1, fw_size, res = -1, done = 0, download_state = 0;
    int read_from_serial = 1;

    /* open firmware file and get its size */
    fw = open(fw_fname, O_RDONLY);
    if (-1 == fw) {
        perror(fw_fname);
        exit(EXIT_FAILURE);
    }
    if (fstat(fw, &sbuf)) {
        perror(fw_fname);
        goto cleanup_and_exit;
    }
    fw_size = sbuf.st_size;

    printf("Downloading %s...\n", fw_fname);
    uint8_t b = 0, fw_crc = 0;

    while (!done) {

        if (read_from_serial && read(fd, &b, 1) < 0) {
            if (errno == EAGAIN) continue;

            perror("Reading from serial port");
            continue;
            // goto cleanup_and_exit;
        }

        // printf("State %u, read 0x%02x\n", download_state, b);
        switch (download_state)
        {
            case 0:
                if (STX == b) {
                    uint8_t buf[3];

                    buf[0] = SOH;
                    buf[1] = fw_size;
                    buf[2] = (fw_size >> 8);
                    if (safe_write(fd, buf, 3)) {
                        perror("responding to STX");
                        goto cleanup_and_exit;
                    }
                    download_state = 1;
                }
                break ;

            case 1:
                if (ACK == b) {
                    download_state = 2;
                    read_from_serial = 0;
                } else if (NACK == b) {
                    fprintf(stderr, "Received NACK.\n");
                    done = 1;
                } else {
#if 0
                    fprintf(stderr, "Received %02x while "
                            "expecting ACK (%02x) "
                            "or NACK (%02x).\n",
                            b, ACK, NACK);
#endif
                    if (++cnt == 10) {
                        printf("Restarting.\n");
                        download_state = 0;
                        cnt = 0;
                    }
                }
                break ;

            case 2:
                read_from_serial = 1;
                if (send_file(fd, fw, fw_size, &fw_crc))
                    goto cleanup_and_exit;
                download_state = 3;
                break;

            case 3:
                if (fw_crc != b) {
                    fprintf(stderr, "Received CRC %02x, "
                            "which does not match "
                            "computed CRC %02x.\n",
                            b, fw_crc);
                } else {
                    printf("CRC OK (%02x).\n", b);
                    b = ACK;
                    if (safe_write(fd, &b, 1) < 0) {
                        perror("sending final ACK");
                        goto cleanup_and_exit;
                    }
                }
                done = 1;
                break;

            default:
                fprintf(stderr, "Unknown download_state=%d\n",
                        download_state);
                goto cleanup_and_exit;
        }
    }

    res = 0;

cleanup_and_exit:
    if (fw != -1) {
        if (close(fw))
            perror(fw_fname);
    }

    return res;
}

int btstack_chipset_da14581_download_firmware(const char * device_name){

    // open serial port
    int flowcontrol = 1;

    struct termios toptions;
    int flags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    int fd = open(device_name, flags);
    if (fd == -1)  {
        log_error("posix_open: Unable to open port %s", device_name);
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        log_error("posix_open: Couldn't get term attributes");
        return -1;
    }
    
    cfmakeraw(&toptions);   // make raw

    // 8N1
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag |= CS8;
    if (flowcontrol) {
        // with flow control
        toptions.c_cflag |= CRTSCTS;
    } else {
        // no flow control
        toptions.c_cflag &= ~CRTSCTS;
    }
    
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    
    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 1;
    toptions.c_cc[VTIME] = 0;
    
    speed_t brate = B57600;
    cfsetospeed(&toptions, brate);
    cfsetispeed(&toptions, brate);

    if(tcsetattr(fd, TCSANOW, &toptions) < 0) {
        log_error("posix_open: Couldn't set term attributes");
        return -1;
    }

    tcflush(fd,TCIOFLUSH);
    
    dialog_download_hci_firmware(fd);

    close(fd);

    return 0;
}
#endif

int main(int argc, const char * argv[]){

	/// GET STARTED with BTstack ///
	btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_posix_get_instance());
	    
    // use logger: format HCI_DUMP_PACKETLOGGER, HCI_DUMP_BLUEZ or HCI_DUMP_STDOUT
    hci_dump_open("/tmp/hci_dump_da.pklg", HCI_DUMP_PACKETLOGGER);

    // pick serial port
    config.device_name = "/dev/tty.usbmodem1411";

#ifdef HAVE_DA14581
    btstack_chipset_da14581_download_firmware(config.device_name);
#endif    

    // init HCI
    const btstack_uart_block_t * uart_driver = btstack_uart_block_posix_instance();
    const hci_transport_t * transport = hci_transport_h4_instance(uart_driver);
    const btstack_link_key_db_t * link_key_db = btstack_link_key_db_fs_instance();
	hci_init(transport, (void*) &config);
    hci_set_link_key_db(link_key_db);
    
    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // setup dynamic chipset driver setup
    hci_set_local_version_information_callback(&local_version_information_callback);

    // handle CTRL-c
    signal(SIGINT, sigint_handler);

    // setup app
    btstack_main(argc, argv);

    // go
    btstack_run_loop_execute();    

    return 0;
}
