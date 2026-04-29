/*$$HEADER*/
/******************************************************************************/
/*                                                                            */
/*                    H E A D E R   I N F O R M A T I O N                     */
/*                                                                            */
/******************************************************************************/

// Project Name                   : OpenRISC Debug Proxy
// File Name                      : or_debug_proxy.c
// Prepared By                    : jb
// Project Start                  : 2008-10-01

/*$$COPYRIGHT NOTICE*/
/******************************************************************************/
/*                                                                            */
/*                      C O P Y R I G H T   N O T I C E                       */
/*                                                                            */
/******************************************************************************/
/*
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation;
  version 2.1 of the License, a copy of which is available from
  http://www.gnu.org/licenses/old-licenses/lgpl-2.1.txt.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*$$DESCRIPTION*/
/******************************************************************************/
/*                                                                            */
/*                           D E S C R I P T I O N                            */
/*                                                                            */
/******************************************************************************/
//
// The entry point for the OpenRISC debug proxy console application. Is 
// compilable under both Linux/Unix systems and Cygwin Windows.
//

#include <assert.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>

/* use multithreading for debugging two DSP cores simultaneously */
#include <pthread.h>

// Windows includes
#ifdef CYGWIN_COMPILE
#include <windows.h>
#include "win_FTCJTAG.h"
//#include "win_FTCJTAG_ptrs.h"
#else
#include <signal.h>
void catch_sigint(int sig_num);	// First param must be "int"
#endif

#include "gdb.h"

#ifdef USB_ENDPOINT_ENABLED
#include "usb_functions.h"
#endif

#include "chip_desc.h"
#include "or_debug_proxy.h"

static const char * gVersionStr = "1.9.2.20260420";

// Defines of endpoint numbers
#define ENDPOINT_TARGET_NONE 0
#define ENDPOINT_TARGET_USB 1
#define ENDPOINT_TARGET_OTHER 2

static int endpoint_target;	// Value to hold targeted endpoint

#define GDB_PROTOCOL_JTAG  1
#define GDB_PROTOCOL_RSP   2
#define GDB_PROTOCOL_NONE  3

int err;			// Global error value

/* Currently selected scan chain - just to prevent unnecessary transfers. */
int current_chain = -1;

/* The chain that should be currently selected. */
int dbg_chain = -1;

/* By default, provide access to CPU */
int no_cpu = 0;

//为了解决重复定义的问题，上面注释了#include "win_FTCJTAG_ptrs.h"，同时在这里使用extern来获取函数
extern int getFTDIJTAGFunctions();
extern void * handle_rsp(void * func_arg);
extern int g_dsp_core_index;
sem_t dual_core_sem;

ProxyDebugModeType g_option_debug_mode = STOP_MODE;
int g_option_show_pc = 0;

extern int g_usb_jtag_clk_divider;

int g_show_verbose = 0;
int g_trace_flow = 0;
enum CHIP_TYPE g_chip_type = CHIP_QXS320F28004x;
int g_core_jtag_chain_start = 0;
int g_dual_core_dsp = 0;
int g_stalldsp_on_terminate = 0;
int g_download_program_stage = 1;

int g_cla_debug = 0;

iram_range_t g_iram_range;

// JTAG port channel of debugger, "A" | "B" (default)
char g_jtag_channel[2] = { 'B', '\0' };

int main(int argc, char* argv[]) {
	int gdb_protocol = GDB_PROTOCOL_RSP;
	int core0_debug_port = 3333;

	endpoint_target = ENDPOINT_TARGET_USB;

	// init our global error number
	err = DBG_ERR_OK;

	// Check we were compiled with at least one endpoint enabled
#ifndef USB_ENDPOINT_ENABLED
	printf("No endpoints enabled.\nRecompile the proxy with at least one endpoint enabled\n");
	exit(EXIT_FAILURE);
#endif

	srand(getpid());

	// init all to invalid
	g_iram_range.value_invalid = 0xFFFFFFFF;
	g_iram_range.core0_iram_base = g_iram_range.value_invalid;
	g_iram_range.core0_iram_size = g_iram_range.value_invalid;
	g_iram_range.core1_iram_base = g_iram_range.value_invalid;
	g_iram_range.core1_iram_size = g_iram_range.value_invalid;
	g_iram_range.core0cla_iram_base = g_iram_range.value_invalid;
	g_iram_range.core0cla_iram_size = g_iram_range.value_invalid;
	g_iram_range.core1cla_iram_base = g_iram_range.value_invalid;
	g_iram_range.core1cla_iram_size = g_iram_range.value_invalid;

	// Parse through the input, check what we've been given
	char* s = NULL;
	int argv_idx = 1;
	while (argv[argv_idx] != NULL) {
		if (strcmp(argv[argv_idx], "-r") == 0) {
			// TCP port number for GDB connect with DSP core0
			core0_debug_port = strtol(argv[argv_idx+1], &s, 10);
			argv_idx += 2;
			// reserve at least 4 ports for dual-core and dual-cla chip
			if (core0_debug_port > 65532 || *s != '\0') {
				print_usage();
				exit(1);
			}
		} else if ((strcmp(argv[argv_idx], "2>CON") == 0) ||
				   (strcmp(argv[argv_idx], "1>CON") == 0) ||
				   (strcmp(argv[argv_idx], "<CON") == 0)) {
			/* skip CLI from vscode debugging */
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "-c") == 0) {
			// enable real-time refresh mode
			g_option_debug_mode = NON_STOP_MODE;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "-p") == 0) {
			// print PC after each GDB RSP packet
			g_option_show_pc = 1;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "-d") == 0) {
			// specify divisor; JTAG TCK frequency = 60Mhz/((1+divisor)*2)
			g_usb_jtag_clk_divider = strtol(argv[argv_idx+1], &s, 10);
			argv_idx += 2;
		} else if (strcmp(argv[argv_idx], "-v") == 0) {
			// show verbose log
			g_show_verbose = 1;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "-t") == 0) {
			// trace program flow
			g_trace_flow = 1;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--cla") == 0) {
			g_cla_debug = 1;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--chip03x") == 0) {
			g_chip_type = CHIP_QXS320F2803x;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--chip13x") == 0) {
			g_chip_type = CHIP_QXS320F280013x;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--chip33x") == 0) {
			g_chip_type = CHIP_QXS320F2833x;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--chip65x") == 0) {
			g_chip_type = CHIP_QXS320F28P65x;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--chip37xs") == 0) {
			g_chip_type = CHIP_QXS320F2837xS;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--chip37xd") == 0) {
			g_chip_type = CHIP_QXS320F2837xD;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--chip003x") == 0) {
			g_chip_type = CHIP_QXS320F28003x;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--c0ibase") == 0) {
			g_iram_range.core0_iram_base = strtol(argv[argv_idx+1], &s, 16);
			argv_idx += 2;
		} else if (strcmp(argv[argv_idx], "--c0isize") == 0) {
			g_iram_range.core0_iram_size = strtol(argv[argv_idx+1], &s, 16);
			argv_idx += 2;
		} else if (strcmp(argv[argv_idx], "--c1ibase") == 0) {
			g_iram_range.core1_iram_base = strtol(argv[argv_idx+1], &s, 16);
			argv_idx += 2;
		} else if (strcmp(argv[argv_idx], "--c1isize") == 0) {
			g_iram_range.core1_iram_size = strtol(argv[argv_idx+1], &s, 16);
			argv_idx += 2;
		} else if (strcmp(argv[argv_idx], "--c0claibase") == 0) {
			g_iram_range.core0cla_iram_base = strtol(argv[argv_idx+1], &s, 16);
			argv_idx += 2;
		} else if (strcmp(argv[argv_idx], "--c0claisize") == 0) {
			g_iram_range.core0cla_iram_size = strtol(argv[argv_idx+1], &s, 16);
			argv_idx += 2;
		} else if (strcmp(argv[argv_idx], "--c1claibase") == 0) {
			g_iram_range.core1cla_iram_base = strtol(argv[argv_idx+1], &s, 16);
			argv_idx += 2;
		} else if (strcmp(argv[argv_idx], "--c1claisize") == 0) {
			g_iram_range.core1cla_iram_size = strtol(argv[argv_idx+1], &s, 16);
			argv_idx += 2;
		} else if (strcmp(argv[argv_idx], "--stall-on-terminate") == 0) {
			g_stalldsp_on_terminate = 1;
			argv_idx += 1;
		} else if (strcmp(argv[argv_idx], "--jtagchannel") == 0) {
			char jchan = toupper(argv[argv_idx+1][0]);
			assert(jchan == 'A' || jchan == 'B');
			g_jtag_channel[0] = jchan;
			g_jtag_channel[1] = '\0';
			argv_idx += 2;
		} else {
			print_usage();
			exit(1);
		}
	}

	// get chip descriptor for selected chip type, and print info
	//
	const chipdesc_t * p_chipdesc = get_chipdesc(g_chip_type);
	g_core_jtag_chain_start = p_chipdesc->core_jtag_chain_start;

	assert((1 == p_chipdesc->corecnt) || (2 == p_chipdesc->corecnt));
	g_dual_core_dsp = p_chipdesc->corecnt == 2 ? 1 : 0;

	printf("\nor_debug_proxy: version %s\n", gVersionStr);
	printf("\n[INFO]: debug `%s (%s-core", p_chipdesc->cname, g_dual_core_dsp ? "dual" : "single");
	if (p_chipdesc->clacnt != 0) {
		printf(", %s-cla", p_chipdesc->clacnt == 2 ? "dual" : "single");
	}
	printf(")` via JTAG channel `%s`\n\n", g_jtag_channel);
	fflush(stdout);

	if (g_cla_debug && p_chipdesc->clacnt == 0) {
		printf("[WARNING]: try to debug CLA on no-cla chip, disable cla debug.\n\n");
		fflush(stdout);
		g_cla_debug = 0;
	}

#ifdef CYGWIN_COMPILE
	// Load the FTCJTAG DLL function pointers
	if (getFTDIJTAGFunctions() < 0) {
		printf("getFTDIJTAGFunctions error");
		exit(EXIT_FAILURE);
	}
#endif

#ifndef CYGWIN_COMPILE
	// Install a signal handler to exit gracefully
	// when we receive a sigint
	signal(SIGINT, catch_sigint);
#endif

	/* Initialise connection to our DSP system */
	current_chain = -1;
#ifdef USB_ENDPOINT_ENABLED
	/* USB Endpoint */
	if (endpoint_target == ENDPOINT_TARGET_USB) {
        err = usb_dbg_reset();
        if (err != 0) {
            fprintf(stderr, "Connection via USB debug cable failed (err = %d).\nPlease ensure the device is attached and correctly installed\n\n", err);
            exit(-1);
        }

        g_dsp_core_index = 0;
        dbg_test();
	}
#endif

	/* We have a connection to the target system. Now establish server connection. */
	assert(gdb_protocol == GDB_PROTOCOL_RSP);
	if (gdb_protocol == GDB_PROTOCOL_RSP) {
		sem_init(&dual_core_sem, 0, 1);

		// create threads private data
		//
		struct global_rsp core0_data = { .core_index = 0, .debug_port = core0_debug_port + 0 };
		struct global_rsp core1_data = { .core_index = 1, .debug_port = core0_debug_port + 1 };
		struct global_rsp cla0_data  = { .core_index = 2, .debug_port = core0_debug_port + 2 };
		struct global_rsp cla1_data  = { .core_index = 3, .debug_port = core0_debug_port + 3 };

		// create threads
		//
		pthread_t core0_thread, core1_thread;
		if (pthread_create(&core0_thread, NULL, handle_rsp, (void*)&core0_data) != 0) {
			printf("can't create thread for port: %d\n", core0_data.debug_port);
			return EXIT_FAILURE;
		}
		if (g_dual_core_dsp) {
			if (pthread_create(&core1_thread, NULL, handle_rsp, (void*)&core1_data) != 0) {
				printf("can't create thread for port: %d\n", core1_data.debug_port);
				return EXIT_FAILURE;
			}
		}

		pthread_t cla0_thread, cla1_thread;
		if (g_cla_debug) {
			if (pthread_create(&cla0_thread, NULL, handle_rsp, (void*)&cla0_data) != 0) {
				printf("can't create thread for port: %d\n", cla0_data.debug_port);
				return EXIT_FAILURE;
			}
			if (g_dual_core_dsp && p_chipdesc->clacnt == 2) {
				if (pthread_create(&cla1_thread, NULL, handle_rsp, (void*)&cla1_data) != 0) {
					printf("can't create thread for port: %d\n", cla1_data.debug_port);
					return EXIT_FAILURE;
				}
			}
		}

		// join threads
		//
		int core0_thread_ret = pthread_join(core0_thread, NULL);
		if (g_dual_core_dsp) {
			int core1_thread_ret = pthread_join(core1_thread, NULL);
		}

		if (g_cla_debug) {
			int cla0_thread_ret = pthread_join(cla0_thread, NULL);
			if (g_dual_core_dsp && p_chipdesc->clacnt == 2) {
				int cla1_thread_ret = pthread_join(cla1_thread, NULL);
			}
		}
	}

	return 0;
}

int dbg_reset() {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_reset();
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

void dbg_test() {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		usb_dbg_test();
#endif
}

/* Set TAP instruction register */
int dbg_set_tap_ir(uint32_t ir) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		usb_set_tap_ir(ir);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* Sets scan chain.  */
int dbg_set_chain(uint32_t chain) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_set_chain(chain);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* sends out a command with 32bit address and 16bit length, if len >= 0 */
int dbg_command(uint32_t type, uint32_t adr, uint32_t len) {
	// This is never called by any of the VPI functions, so only USB 
	// endpoint
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_command(type, adr, len);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* writes a ctrl reg */
int dbg_ctrl(uint32_t reset, uint32_t stall) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_ctrl(reset, stall);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* reads control register */
int dbg_ctrl_read(uint32_t* reset, uint32_t* stall) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_ctrl_read(reset, stall);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* issues a burst read/write */
int dbg_go(unsigned char* data, uint16_t len, uint32_t read) {
	// Only USB endpouint32_t option here
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_go(data, len, read);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* read a byte from wishbone */
int dbg_wb_read8(uint32_t adr, uint8_t* data) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_wb_read8(adr, data);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* read a word from wishbone */
int dbg_wb_read32(uint32_t adr, uint32_t* data) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_wb_read32(adr, data);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* write a word to wishbone */
int dbg_wb_write8(uint32_t adr, uint8_t data) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_wb_write8(adr, data);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

int dbg_wb_write16(uint32_t adr, uint16_t data) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_wb_write16(adr, data);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* write a word to wishbone */
int dbg_wb_write32(uint32_t adr, uint32_t data) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_wb_write32(adr, data);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* read a block from wishbone */
int dbg_wb_read_block32(uint32_t adr, uint32_t* data, uint32_t len) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_wb_read_block32(adr, data, len);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* write a block to wishbone */
int dbg_wb_write_block32(uint32_t adr, uint32_t* data, uint32_t len) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_wb_write_block32(adr, data, len);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* read a register from cpu */
int dbg_cpu0_read(uint32_t adr, uint32_t* data, uint32_t length) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_cpu0_read(adr, data, length);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* write a cpu register */
int dbg_cpu0_write(uint32_t adr, uint32_t* data, uint32_t length) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_cpu0_write(adr, data, length);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* write a cpu module register */
int dbg_cpu0_write_ctrl(uint32_t adr, unsigned char data) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_cpu0_write_ctrl(adr, data);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

/* read a register from cpu module */
int dbg_cpu0_read_ctrl(uint32_t adr, unsigned char* data) {
#ifdef USB_ENDPOINT_ENABLED
	if (endpoint_target == ENDPOINT_TARGET_USB)
		return usb_dbg_cpu0_read_ctrl(adr, data);
#endif
	return DBG_ERR_INVALID_ENDPOINT;
}

void test_sdram(void) {
	return;
}

// Close down gracefully when we receive any kill signals
void catch_sigint(int sig_num) {
	// Close down any potentially open sockets and USB handles
	if (server_fd)
		close(server_fd);
	gdb_close();
#ifdef USB_ENDPOINT_ENABLED
	usb_close_device_handle();
#endif
	printf
	("\nInterrupt signal received. Closing down connections and exiting\n\n");
	exit(0);
}

void print_usage() {
	printf("\nor_debug_proxy: version %s\n", gVersionStr);
	printf("\nUsage: or_debug_proxy <OPTIONS>\n"
		   "\nGDB Proxy Server for dual-core DSP\n"
		   "\n<OPTIONS>:\n"
		   "\n    -r <port_for_core0>        TCP port number for GDB connect with DSP core0, [0,65535], default: 3333"
		   "\n                               For dual-core DSP, core1 will have port <port_for_core0>+1\n"
		   "\n    -d <usb_jtag_clk_divider>  JTAG TCK frequency = 12Mhz/((1+divisor)*2), [0, 65535], default: 3 (1.5MHz)\n"
		   "\n    -c                         enable real-time refresh mode\n"
		   "\n    -p                         print PC after each GDB RSP packet\n"
		   "\n    -v                         show verbose log\n"
		   "\n    -t                         trace program flow\n"
		   "\n    --cla                      enable CLA debug\n"
		   "\n    --jtagchannel              specify JTAG channel of debugger, A | B (default)\n"
		   "\n    --chip03x                  specify that chip to debug has QXS320F2803x DSP\n"
		   "\n    --chip13x                  specify that chip to debug has QXS320F280013x DSP\n"
		   "\n    --chip33x                  specify that chip to debug has QXS320F2833x DSP\n"
		   "\n    --chip65x                  specify that chip to debug has QXS320F28P65x DSP\n"
		   "\n    --chip37xs                 specify that chip to debug has QXS320F2837xS DSP\n"
		   "\n    --chip37xd                 specify that chip to debug has QXS320F2837xD DSP\n"
		   "\n    --chip003x                 specify that chip to debug has QXS320F28003x DSP\n"
		   "\n    --c0ibase <hex_number>     core0 IRAM base address , format: 0x----\n"
		   "\n    --c0isize <hex_number>     core0 IRAM size in Bytes, format: 0x----\n"
		   "\n    --c1ibase <hex_number>     core1 IRAM base address , format: 0x----\n"
		   "\n    --c1isize <hex_number>     core1 IRAM size in Bytes, format: 0x----\n"
		   "\n    --c0claibase <hex_number>  core0 CLA IRAM base address , format: 0x----\n"
		   "\n    --c0claisize <hex_number>  core0 CLA IRAM size in Bytes, format: 0x----\n"
		   "\n    --c1claibase <hex_number>  core1 CLA IRAM base address , format: 0x----\n"
		   "\n    --c1claisize <hex_number>  core1 CLA IRAM size in Bytes, format: 0x----\n"
		   "\n    --stall-on-terminate       DSP stalls when debug terminates\n\n"
		   "    NOTE: only one --chipxxx argument should be given, and\n"
		   "          the last one takes precedence if multiple are provided.\n\n");
	fflush(stdout);
}
