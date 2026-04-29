/*$$HEADER*/
/******************************************************************************/
/*                                                                            */
/*                    H E A D E R   I N F O R M A T I O N                     */
/*                                                                            */
/******************************************************************************/

// Project Name                   : OpenRISC Debug Proxy
// File Name                      : gdb.c
// Prepared By                    : jb, rmd
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
// Implements GDB stub for OpenRISC debug proxy.
//

/*$$CHANGE HISTORY*/
/******************************************************************************/
/*                                                                            */
/*                         C H A N G E  H I S T O R Y                         */
/*                                                                            */
/******************************************************************************/

// Date         Version Description
//------------------------------------------------------------------------
// 081101               Imported code from "jp" project                 jb
// 090219               Adapted code from Jeremy Bennett's RSP server
//                      for the or1ksim project.                       rmb
// 090304               Finished RSP server code import, added extra
//                      functions, adding stability when debugging on
//                      a remote target.                                jb
// 090505               Remove PPC polling - caused intermittent errors 
//                      in or1k                                         jb
// 090828               Fixed byte/non-aligned accesses. Removed legacy
//                      "remote JTAG" protocol.                         jb
// 100924               Added RSP query handlers for things new to with
//                      gdb-7.2                                         jb

#ifdef CYGWIN_COMPILE

#else
// linux includes                  
#include <time.h>
#include <sched.h>
#endif

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <assert.h>

/* Libraries for JTAG proxy server.  */
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/poll.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <inttypes.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/time.h>

/*! Name of the Or1ksim RSP service */
#define OR1KSIM_RSP_SERVICE  "or1ksim-rsp"

#include "gdb.h"		/* partially copied from gdb/config/or1k */
#include "or_debug_proxy.h"
#include "usb_functions.h"

#define MAX_GPRS        (44)  
#define MAX_GPRS_all    (48)  
#define GR_START        (0x007f0500)
#define GR_NUM          (32)
#define	VR_START        (0x007f1000)
#define VR_NUM          (0)
#define VR_LENGTH       (0)	//VR有160位宽度
#define VR_GAP          (0x200)	//相邻VR之间的地址差异
#define MOB_START       (0x007f0610)

/* Indices of GDB registers that are not GPRs. Must match GDB settings! */
#define PPC_REGNUM  (MAX_GPRS + 0)	/*!< Previous PC */
#define NPC_REGNUM  (MAX_GPRS + 1)	/*!< Next PC */
#define SR_REGNUM   (MAX_GPRS + 2)	/*!< Supervision Register */
#define NUM_REGS    (MAX_GPRS + 4)	/*!< Total GDB registers */

/* OR1k CPU registers address */
#define ADDR_SPR_BOOT 		0x007f0000
#define NPC_CPU_REG_ADD  	0x007f040c	/* Next PC */
#define SR_CPU_REG_ADD   	11	/* Supervision Register */
#define PPC_CPU_REG_ADD  	0x007f0410	/* Previous PC */
#define EXPC_CPU_REG_ADD	0x007f0414	/* Previous PC */
#define DMR1_CPU_REG_ADD	(0x007f0200 + (16<<2))	// 0x007f0240	/* Debug Mode Register 1 (DMR1) */
#define DMR2_CPU_REG_ADD	(0x007f0200 + (17<<2))	// 0x007f0244	/* Debug Mode Register 2 (DMR2) */
#define DSR_CPU_REG_ADD		(0x007f0200 + (20<<2))	// 0x007f0250	/* Debug Stop Register   (DSR)  */
#define DRR_CPU_REG_ADD  	(0x007f0200 + (21<<2))	// 0x007f0244	/* Debug Reason Register (DRR)  */
#define GR_CPU_REG_ADD  	(ADDR_SPR_BOOT+0x0500)	/* GR */

#define GRN_ADDR(N)         (GR_START + (N<<2))

#define VR_CPU_REG_ADD(N)	(VR_START   + (N<<9))
#define DVR_CPU_REG_ADD(N)  (0x007f0200 + (N<<2))
#define DCR_CPU_REG_ADD(N)  (0x007f0200 + ((N +8)<<2))

/*! Trap instruction for OR32 */
#define OR1K_TRAP_INSTR  0x21000001

/*! The maximum number of characters in inbound/outbound buffers.  The largest
  packets are the 'G' packet, which must hold the 'G' and all the registers
  with two hex digits per byte and the 'g' reply, which must hold all the
  registers, and (in our implementation) an end-of-string (0)
  character. Adding the EOS allows us to print out the packet as a
  string. So at least NUMREGBYTES*2 + 1 (for the 'G' or the EOS) are needed
  for register packets */
  //#define GDB_BUF_MAX  ((NUM_REGS) * 8 + 1)

/* GDB_BUF_MAX is chosen based on tests of reading/writing IRAM/DRAM with program_flash.
 * read/write from/to RAM through JTAG interface is stable by invoking function
 * usb_dbg_wb_write_block32, where @len argument to this function is 2048.
 * using 4096 for @len will make JTAG read/write stuck ...
 */
#define GDB_BUF_MAX  2048
#define GDB_BUF_MAX_TIMES_TWO (GDB_BUF_MAX*2)

/*! Size of the matchpoint hash table. Largest prime < 2^10 */
#define MP_HASH_SIZE  1021

/* Definition of special-purpose registers (SPRs). */
#define MAX_SPRS (0x10000)

#define SPR_DMR1_ST	  		0x00400000	/* Single-step trace */
#define SPR_DMR2_WGB	   	0x003FF000	/* Watchpoints generating breakpoint */
#define SPR_DSR_TE				0x00002000	/* Trap exception */
#define SPR_DCR				0x00000023 /* DCR  */	
#define HWP_MAX_WP 8
unsigned char hwp_in_use[HWP_MAX_WP];

//#define WORDSBIGENDIAN_N

/* Definition of OR1K exceptions */
#define EXCEPT_NONE     0x0000
#define EXCEPT_RESET	0x0100
#define EXCEPT_BUSERR	0x0200
#define EXCEPT_DPF	0x0300
#define EXCEPT_IPF	0x0400
#define EXCEPT_TICK	0x0500
#define EXCEPT_ALIGN	0x0600
#define EXCEPT_ILLEGAL	0x0700
#define EXCEPT_INT	0x0800
#define EXCEPT_DTLBMISS	0x0900
#define EXCEPT_ITLBMISS	0x0a00
#define EXCEPT_RANGE	0x0b00
#define EXCEPT_SYSCALL	0x0c00
#define EXCEPT_FPE	0x0d00
#define EXCEPT_TRAP	0x0e00

// Changed to #defines from static const int's due to compile error
// DRR (Debug Reason Register) Bits
#define SPR_DRR_RSTE  0x00000001	//!< Reset
#define SPR_DRR_BUSEE 0x00000002	//!< Bus error
#define SPR_DRR_DPFE  0x00000004	//!< Data page fault
#define SPR_DRR_IPFE  0x00000008	//!< Insn page fault
#define SPR_DRR_TTE   0x00000010	//!< Tick timer
#define SPR_DRR_AE    0x00000020	//!< Alignment
#define SPR_DRR_IIE   0x00000040	//!< Illegal instruction
#define SPR_DRR_IE    0x00000080	//!< Interrupt
#define SPR_DRR_DME   0x00000100	//!< DTLB miss
#define SPR_DRR_IME   0x00000200	//!< ITLB miss
#define SPR_DRR_RE    0x00000400	//!< Range fault
#define SPR_DRR_SCE   0x00000800	//!< System call
#define SPR_DRR_FPE   0x00001000	//!< Floating point
#define SPR_DRR_TE    0x00002000	//!< Trap

/*! Definition of GDB target signals. Data taken from the GDB 6.8
  source. Only those we use defined here. The exact meaning of
  signal number is defined by the header `include/gdb/signals.h'
  in the GDB source code. For an explanation of what each signal
  means, see target_signal_to_string.*/
enum target_signal {
    TARGET_SIGNAL_NONE = 0,
    TARGET_SIGNAL_INT = 2,
    TARGET_SIGNAL_ILL = 4,
    TARGET_SIGNAL_TRAP = 5,
    TARGET_SIGNAL_FPE = 8,
    TARGET_SIGNAL_BUS = 10,
    TARGET_SIGNAL_SEGV = 11,
    TARGET_SIGNAL_ALRM = 14,
    TARGET_SIGNAL_USR2 = 31,
    TARGET_SIGNAL_PWR = 32
};

/*! String to map hex digits to chars */
static const char hexchars[] = "0123456789abcdef";

/* OR32 Linux kernel debugging hacks */
int kernel_debug = 0; // Disabled by default. Enabled via command line.
/* If we're operating in virtual memory space, currently the standard VM base
   is 0xc0000000, since the GDB we're currently using is bare-metal we will
   do a translation of these VM addresses to their physical address - although
   not with any sophistication, we simply map all 0xc0000000 based addresses
   to 0x0. This allows GDB to read/wite the addresses the PC and NPC point to,
   access VM data areas, etc. */
   /* Define OR32_KERNEL_DBUG_COMPAT=1 when compiling to enable this */
#define OR32_KERNEL_DBG_COMPAT 1
#define OR32_LINUX_VM_OFFSET 0xc0000000
#define OR32_LINUX_VM_MASK 0xf0000000

#define IS_VM_ADDR(adr) ((adr & OR32_LINUX_VM_MASK) == OR32_LINUX_VM_OFFSET)

#define INVALID_PC_LOCATION (0xFFFFFFFF)

/************************
   JTAG Server Routines
************************/
int server_fd = 0;
int gdb_fd = 0;

/* global to store what chain the debug unit is currently connected to
   (not the JTAG TAP, but the onchip debug module has selected) */
int gdb_chain = -1;

/*! Data structure for RSP buffers. Can't be null terminated, since it may
  include zero bytes */
struct rsp_buf {
    char data[GDB_BUF_MAX_TIMES_TWO];
    int len;
};

/*! global to get which cpu is to run */
extern sem_t dual_core_sem;
int cpu0_or_cpu1 = 1;
int g_cpu_no_adjust = 0;
static void change_cpu_01(struct global_rsp* rspptr) {
    assert(0 == rspptr->cpu_no || 1 == rspptr->cpu_no);
    cpu0_or_cpu1 = rspptr->cpu_no + 1;
}

/* Forward declarations of static functions */
static char* printTime(void);
static int gdb_read(void*, int);
static int gdb_write(void*, int);
static void rsp_interrupt(struct global_rsp* rspptr);
static char rsp_peek(struct global_rsp* rspptr);
static struct rsp_buf* get_packet(struct global_rsp* rspptr);
static void rsp_init(struct global_rsp* rspptr);
static void set_npc(uint32_t addr, struct global_rsp* rspptr);
static uint32_t get_npc(struct global_rsp* rspptr);
static void rsp_check_for_exception(struct global_rsp* rspptr);
static int check_for_exception_vector(uint32_t ppc, struct global_rsp* rspptr);
static void rsp_exception(uint32_t except, struct global_rsp* rspptr);
static int get_rsp_char(struct global_rsp* rspptr);
static int hex(int c);
static void rsp_get_client(struct global_rsp* rspptr);
static void rsp_client_request(struct global_rsp* rspptr);
static void rsp_client_close(struct global_rsp* rspptr);
static void client_close(char err);
static void put_str_packet(const char* str, struct global_rsp* rspptr);
static void rsp_report_exception(struct global_rsp* rspptr);
static void put_packet(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void send_rsp_str(unsigned char* data, int len, struct global_rsp* rspptr);
static void rsp_query(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_vpkt(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_step(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_step_with_signal(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_step_generic(uint32_t addr, uint32_t except, struct global_rsp* rspptr);
static void rsp_step_continue(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_hardware_continue(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_continue(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_continue_with_signal(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_continue_generic(uint32_t addr, uint32_t except, struct global_rsp* rspptr);
static void rsp_read_all_regs(struct global_rsp* rspptr);
static void rsp_write_all_regs(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_read_mem(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_read_mem_seprate(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_write_mem(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_write_mem_bin(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static int rsp_unescape(char* data, int len);
//static void rsp_read_reg(struct rsp_buf* p_buf, struct global_rsp* rspptr);
//static void rsp_write_reg(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void mp_hash_init(struct global_rsp* rspptr);
int hwp_get_available_watchpoint(void);
void hwp_return_watchpoint(int wp);
static void mp_hash_add(enum mp_type type, uint32_t addr, uint32_t instr, struct global_rsp* rspptr);
static struct mp_entry* mp_hash_lookup(enum mp_type type, uint32_t addr, struct global_rsp* rspptr);
static struct mp_entry* mp_hash_delete(enum mp_type type, uint32_t addr, struct global_rsp* rspptr);
static void rsp_remove_matchpoint(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_insert_matchpoint(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_command(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_set(struct rsp_buf* p_buf, struct global_rsp* rspptr);
static void rsp_restart(struct global_rsp* rspptr);
static void ascii2hex(char* dest, char* src);
static void hex2ascii(char* dest, char* src);
static uint32_t hex2reg(char* p_buf);
static void reg2hex(uint32_t val, char* p_buf);
static void swap_buf(char* p_buf, int len);
static int swap_buf_128bit(char* p_buf, int len);
static void swap_buf_256bit(char* p_buf, int* length);
static void set_stall_state(int state, struct global_rsp* rspptr);
static void reset_or1k(void);
static void unreset_or1k(void);
static void gdb_ensure_or1k_stalled();
static int gdb_set_chain(int chain);
static int gdb_write_byte(uint32_t adr, uint8_t data);
static int gdb_write_halfword(uint32_t adr, uint16_t data);
static int gdb_write_reg(uint32_t adr, uint32_t data);
static int gdb_read_byte(uint32_t adr, uint8_t* data);
static int gdb_read_reg(uint32_t adr, uint32_t* data);
static int gdb_write_block(uint32_t adr, uint32_t* data, int len);
static int gdb_read_block(uint32_t adr, uint32_t* data, int len);

static int send_file_request(int trapImm, struct global_rsp* rspptr);
static int target_recv_trap_back(struct rsp_buf* p_buf, struct global_rsp* rspptr);

static uint32_t read_id_pc();
static void print_pc();
static int is_chip_alive();
static uint64_t getRelativeMicroseconds();

static void rsp_check_report_stall_status(struct global_rsp* rspptr);

static int address_in_bootram_iram(unsigned int addr);

char* printTime(void) {
    time_t tid;
    struct tm* strtm;
    static char timeBuf[20];

    time(&tid);
    strtm = localtime(&tid);
    sprintf(timeBuf, "[%.02d:%.02d:%.02d] ", strtm->tm_hour, strtm->tm_min,
        strtm->tm_sec);
    return timeBuf;
}

/*---------------------------------------------------------------------------*/
/*!Initialize the Remote Serial Protocol connection

  Set up the central data structures.                                       */
  /*---------------------------------------------------------------------------*/
void rsp_init(struct global_rsp* rspptr) {
    if (g_trace_flow) {
        printf("[TRACE_FLOW] rsp_init() on port `%d`: enter\n", rspptr->serverPort);
    }

    /* Clear out the central data structure */
    rspptr->client_waiting = 0;	/* GDB client is not waiting for us */
    rspptr->client_fd = -1;	/* i.e. invalid */
    rspptr->sigval = 0;		/* No exception */
    rspptr->start_addr = EXCEPT_RESET;	/* Default restart point */
    rspptr->trap_imm = 0;
    /* Set up the matchpoint hash table */
    mp_hash_init(rspptr);
    change_cpu_01(rspptr);
    /* RSP always starts stalled as though we have just reset the processor. */
    rsp_exception(EXCEPT_TRAP, rspptr);
    if (g_chip_type == CHIP_QXS320F2833x) {
        usb_dbg_cpu0_write_ctrl_nocrc(0, 0x03);
        usb_dbg_cpu0_write_ctrl_nocrc(0, 0x01);
        // set JTAG TDO (GPIO 37) in output mode
        uint32_t val2wr = 0x20;
        usb_dbg_wb_write_block32_nocrc(0x01003064, &val2wr, 4);
    } else {
        dbg_cpu0_write_ctrl(0, 0x03);  //added by wangwk 17-05-22
        dbg_cpu0_write_ctrl(0, 0x01);  //added by wangwk 17-05-22
    }
    /* Setup the NPC caching variables */
    rspptr->stallState = STALLED;
    //printf("rspptr->stallState == STALLED\n");

    gdb_set_chain(SC_WISHBONE);

    // Force a caching of the NPC
    rspptr->npcIsCached = 0;
    get_npc(rspptr);

    uint32_t temp_uint32 = 0xff3fffff;
    if (gdb_write_reg(DMR1_CPU_REG_ADD, temp_uint32))
        printf("Error write to DMR1 register\n");
    temp_uint32 = 0x0;
    if (gdb_write_reg(DMR2_CPU_REG_ADD, temp_uint32))
        printf("Error write to DMR2 register\n");

    rspptr->just_handle_packet_c = 0;
    rspptr->just_handle_packet_s = 0;
    rspptr->expected_location_packet_s = INVALID_PC_LOCATION;
    rspptr->just_handle_packet_F = 0;

    /* init to reported, and proxy takes no action */
    rspptr->nonstop_dsp_stall_status_reported = 1;

    if (g_trace_flow) {
        printf("[TRACE_FLOW] rsp_init() on port `%d`: done and exit.\n", rspptr->serverPort);
    }
}				/* rsp_init () */

/*---------------------------------------------------------------------------*/
/*!Look for action on RSP

  This function is called when the processor has stalled, which, except for
  initialization, must be due to an interrupt.

  If we have no RSP client, we get one. We can make no progress until the
  client is available.

  Then if the cause is an exception following a step or continue command, and
  the exception not been notified to GDB, a packet reporting the cause of the
  exception is sent.

  The next client request is then processed.                                */
  /*---------------------------------------------------------------------------*/
static uint32_t cnt = 0;
static int current_filefunc = 0;//现在是哪个fileio函数在作用，用于接收到回复信息的函数
static int traping = 0;

static uint64_t g_handle_rsp_start_us = 0;

static uint64_t GetTimeStampMicrosecond() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec * 1000000UL + tv.tv_usec;
}

void* handle_rsp(void* func_arg) {
    g_handle_rsp_start_us = GetTimeStampMicrosecond();

    struct global_rsp* rspptr = (struct global_rsp*)func_arg;
    uint32_t temp_uint32;

    sem_wait(&dual_core_sem);
    /* The following printf is better to put outside of the critial region protected by dual_core_sem.
     * But, put it inside to make proxy log cleaner. It will not affect performance much since it is
     * only invoked once.
     */
    printf("\nCore %d: handle_rsp(): rspptr->serverPort: %d\n", rspptr->cpu_no, rspptr->serverPort);
    rsp_init(rspptr);
    sem_post(&dual_core_sem);

    while (1) {
        /* If we have no RSP client, wait until we get one. */
        while (-1 == rspptr->client_fd) {
            rsp_get_client(rspptr);
            rspptr->client_waiting = 0;	/* No longer waiting */
        }

        /* If we have an unacknowledged exception tell the GDB client. If this
           exception was a trap due to a memory breakpoint, then adjust the NPC. */
        if (rspptr->client_waiting) {
            sem_wait(&dual_core_sem);
            /* update of global variable `cpu0_or_cpu1` should be protected by mutex.
             * The wait/post is better to put inside function change_cpu_01(), but according to existing
             * program flow, sometimes `cpu0_or_cpu1` is already protected. So, wait/post is put outside.
             *
             * Another better way to do this is to update the interface of the 4 functions in usb_functions.c,
             * who uses `cpu0_or_cpu1` to set debug chain. Add another argument for debug chain number.
             * Maybe, will do this in the future.
             */
            change_cpu_01(rspptr);

            rsp_check_for_exception(rspptr);

            //printf("1. client_waiting: %d, nonstop_dsp_stall_status_reported: %d\n", rspptr->client_waiting, rspptr->nonstop_dsp_stall_status_reported);

            if (rspptr->stallState == STALLED) {	// Get the EXPC if we're stalled
                gdb_set_chain(SC_RISC_DEBUG);
                gdb_read_reg(PPC_CPU_REG_ADD, &temp_uint32);//zsy change npc to expc
            }

            if (TARGET_SIGNAL_USR2 == rspptr->sigval) {
                if (g_show_verbose)
                    printf("tarp occurs, the trap imm is %d\n", rspptr->trap_imm);
                rspptr->client_waiting = 0;
            } else if ((TARGET_SIGNAL_TRAP == rspptr->sigval)
                && (NULL != mp_hash_lookup(BP_MEMORY, temp_uint32, rspptr))) {
                if (rspptr->stallState != STALLED)
                    // This is a quick fix for a strange situation seen in some of 
                    // the simulators where the sw bp would be detected, but the 
                    // stalled state variable wasn't updated correctly indicating 
                    // that last time it checked, it wasn't set but the processor 
                    // had hit the breakpoint. So run rsp_check_for_exception() to 
                    // bring everything up to date.
                    rsp_check_for_exception(rspptr);

                if (g_show_verbose) {
                    printf("Software breakpoint hit at 0x%08x.\n", temp_uint32);
                }

                // set_npc(temp_uint32,rspptr);

                rsp_report_exception(rspptr);
                rspptr->client_waiting = 0;	/* No longer waiting */
            } else if (rspptr->stallState == STALLED) {
                // If we're here, the thing has stalled, but not because of a breakpoint we set
                // report back the exception
                if (g_show_verbose) {
                    printf("core%d: %s\n", rspptr->cpu_no, "the processor is stalled, but not because of a breakpoint we set");
                }

                if (rspptr->just_handle_packet_c) {
                    /* To work around a 'step return' bug observed from IDE, but actually in hardware.
                     * Sometimes, when handling 'step return', program does not go out of current function, 
                     * the reason from SW is that when telling DSP to do packet c action, it stalls when RET in ID stage.
                     * TODO: remove this work around when HW bug fixed
                     */
                    print_pc();
                    if (g_show_verbose) {
                        printf("core%d: %s\n", rspptr->cpu_no, "DSP stalls at unexpected location on c packet, simulate one extra s packet");
                    }
                    struct rsp_buf buf;
                    buf.data[0] = 's';
                    buf.data[1] = '\0';
                    rsp_step(&buf, rspptr);
                    rspptr->just_handle_packet_c = 0;
                }

                if (rspptr->just_handle_packet_s) {
                    /* To work around a 'step over' bug observed from IDE, but actually in hardware.
                     * After returning from CALL instruction, and a packet s action send to DSP, DSP is expected to
                     * stall after executing exactly one instruction, but not.
                     * TODO: remove this work around when HW bug fixed
                     */
                    print_pc();
                    assert(rspptr->expected_location_packet_s != INVALID_PC_LOCATION);
                    uint32_t idpc = read_id_pc();
                    uint32_t pcdiff = idpc - rspptr->expected_location_packet_s;
                    if (0x8 == pcdiff || 0xc == pcdiff) {
                        if (g_show_verbose) {
                            printf("core%d: DSP stalls at unexpected location on s packet, tell gdb not to further run DSP\n", rspptr->cpu_no);
                        }
                        rspptr->sigval = 0;
                    }
                }

                rsp_report_exception(rspptr);
                rspptr->client_waiting = 0;	/* No longer waiting */
            } else if ((TARGET_SIGNAL_TRAP == rspptr->sigval) && (rspptr->stallState != STALLED)) {
                printf("rspptr->sigval = TARGET_SIGNAL_TRAP on unstalled core\n");
                rsp_report_exception(rspptr);
                rspptr->client_waiting = 0;	/* No longer waiting */
            }
#ifndef CYGWIN_COMPILE
            sched_yield();
#endif
            sem_post(&dual_core_sem);
        }

        // See if there's any incoming data from the client by peeking at the socket
        if (rsp_peek(rspptr) > 0) {
            sem_wait(&dual_core_sem);
            change_cpu_01(rspptr);

            //printf("2. client_waiting: %d, nonstop_dsp_stall_status_reported: %d\n", rspptr->client_waiting, rspptr->nonstop_dsp_stall_status_reported);

            if (rsp_peek(rspptr) == 0x03)	// ETX, end of text control char
            {
                if (rspptr->stallState != STALLED) {
                    // Got an interrupt command from GDB, this function should
                    // pull the packet off the socket and stall the processor.
                    // and then send a stop reply packet with signal TARGET_SIGNAL_NONE
                    rsp_interrupt(rspptr);
                } else {
                    if (g_show_verbose)
                        printf("Interrupt received from GDB when DSP is stalled.\n");
                    rspptr->sigval = TARGET_SIGNAL_NONE;
                    rsp_report_exception(rspptr);
                }
                rspptr->client_waiting = 0;
            } else if (rspptr->client_waiting == 0) {
                // Default handling of data from the client:
                /* Get a RSP client request */
                rsp_client_request(rspptr);
            } else if (NON_STOP_MODE == g_option_debug_mode) {
                /* client_waiting must be 1 here */
                /* in non-stop mode, set client_waiting to 0 when received another packet from GDB, and
                 * give no response to previous GDB packet.
                 * It is assumed that when GDB send another packet, it does not require response to its previous packet.
                 * Change this logic if the assumption is NOT right.
                 */
                assert(1 == rspptr->client_waiting);
                if (g_show_verbose) {
                    printf("core %d: Received another GDB packet in non-stop mode while client_waiting = %d, set it to 0.\n",
                            rspptr->cpu_no, rspptr->client_waiting);
                }
                rspptr->client_waiting = 0;
            }

            sem_post(&dual_core_sem);
        } /* end if (rsp_peek(rspptr) > 0) */
        else
        {
#ifndef CYGWIN_COMPILE
            sched_yield();
#endif
        }

        /* if the following condition is true, it means in non-stop mode, DSP is resumed by c/s packet,
         * but another packet comes which disables proxy to respond to c/s packet.
         * 
         * Check whether DSP stalls in this situation, usually because of hitting a breakpoint,
         * and report to IDE if DSP stall.
         */
        sem_wait(&dual_core_sem);
        if ((NON_STOP_MODE == g_option_debug_mode) && (0 == rspptr->client_waiting) && (0 == rspptr->nonstop_dsp_stall_status_reported)) {
            //printf("3. client_waiting: %d, nonstop_dsp_stall_status_reported: %d\n", rspptr->client_waiting, rspptr->nonstop_dsp_stall_status_reported);
            rsp_check_report_stall_status(rspptr);
        }
        sem_post(&dual_core_sem);

#if 0
        if (!is_chip_alive()) {
            printf("Lost connection from chip, exit.\n");
            fflush(stdout);
            exit(EXIT_FAILURE);
        }
#endif
    } // while (1) {

    return NULL;
}				/* handle_rsp () */

static int is_chip_alive() {
    static uint64_t last_check_us = 0;
    uint64_t current_us = getRelativeMicroseconds();
    // check every 1 second
    if (current_us - last_check_us < 1000000) {
        return 1;
    }

    if (g_show_verbose) printf("[@ %lu us] checking DSP alive ... ", current_us);
    int saved_gdb_chain = gdb_chain;
    gdb_set_chain(SC_WISHBONE);

    /* regard chip is not alive if reading from DRAM below is fail */
    int chip_is_alive = 0;
    uint32_t addr = 0x0;
    char word_tem[5] = { '\0' };
    if (gdb_read_block(addr, (uint32_t*)word_tem, bytes_per_word) == DBG_ERR_OK) {
        chip_is_alive = 1;
    }

    gdb_set_chain(saved_gdb_chain);
    if (g_show_verbose) printf("done, chip_is_alive = %d\n", chip_is_alive);
    fflush(stdout);

    last_check_us = current_us;

    return chip_is_alive;
}

static void rsp_check_report_stall_status(struct global_rsp* rspptr) {
    uint32_t reset, stall;
    usb_dbg_cpu_read_ctrl(rspptr->cpu_no, &reset, &stall);
    if (stall) {
        printf("[MSG2IDE] CORE %d STALLED AFTER c PACKET\n", rspptr->cpu_no);
        fflush(stdout);
        rspptr->nonstop_dsp_stall_status_reported = 1;
    }
}

/*
  Check if processor is stalled - if it is, read the DRR
  and return the target signal code
*/
static void rsp_check_for_exception(struct global_rsp* rspptr) {

    unsigned char stalled;
    uint32_t drr, drr_tem, trap_imm;
    //printf("rsp_check_for_exception()\n");
    err = dbg_cpu0_read_ctrl(0, &stalled);	/* check if we're stalled */

    if (!(stalled & 0x01)) {
        // Processor not stalled. Just return;
        return;
    }

    if (g_show_verbose)
        printf("[DEBUG_GDB] rsp_check_for_exception() detected processor was stalled, Checking DRR...\n");

    // We're stalled
    rspptr->stallState = STALLED;
    if (g_show_verbose) printf("rspptr->stallState == STALLED\n");
    rspptr->npcIsCached = 0;

    gdb_set_chain(SC_RISC_DEBUG);

    // Now read the DRR (Debug Reason Register)
    gdb_read_reg(DRR_CPU_REG_ADD, &drr_tem);
    drr = drr_tem & 0x3fff; //取出后14位
    trap_imm = drr_tem >> 14;
    if (g_show_verbose)
        printf("[DEBUG_GDB] DRR = 0x%08x\n", drr);
    switch ((int)(drr & 0xffffffff)) {
    case SPR_DRR_RSTE:
        rspptr->sigval = TARGET_SIGNAL_PWR;
        break;
    case SPR_DRR_BUSEE:
        rspptr->sigval = TARGET_SIGNAL_BUS;
        break;
    case SPR_DRR_DPFE:
        rspptr->sigval = TARGET_SIGNAL_SEGV;
        break;
    case SPR_DRR_IPFE:
        rspptr->sigval = TARGET_SIGNAL_SEGV;
        break;
    case SPR_DRR_TTE:
        rspptr->sigval = TARGET_SIGNAL_ALRM;
        break;
    case SPR_DRR_AE:
        rspptr->sigval = TARGET_SIGNAL_BUS;
        break;
    case SPR_DRR_IIE:
        rspptr->sigval = TARGET_SIGNAL_ILL;
        break;
    case SPR_DRR_IE:
        rspptr->sigval = TARGET_SIGNAL_INT;
        break;
    case SPR_DRR_DME:
        rspptr->sigval = TARGET_SIGNAL_SEGV;
        break;
    case SPR_DRR_IME:
        rspptr->sigval = TARGET_SIGNAL_SEGV;
        break;
    case SPR_DRR_RE:
        rspptr->sigval = TARGET_SIGNAL_FPE;
        break;
    case SPR_DRR_SCE://trap
    case SPR_DRR_TE | SPR_DRR_SCE: // system call trap
        rspptr->sigval = TARGET_SIGNAL_USR2;

        // get system call number from gr2 according to libc
        gdb_read_reg(GRN_ADDR(2), &trap_imm);
        rspptr->trap_imm = trap_imm;
        printf("trap_imm:%d\n", rspptr->trap_imm);
        uint32_t temp_uint32;
        err = gdb_read_reg(PPC_CPU_REG_ADD, &temp_uint32);//zsy use expc instead of ppc
        
        printf("PPC     0x%08x\n", temp_uint32);
        traping = 1;
        //if (trap_imm != 20 && (rspptr->just_handle_packet_F == 0))
        if (trap_imm != 20)
            send_file_request(rspptr->trap_imm, rspptr);
        else
            rspptr->sigval = TARGET_SIGNAL_TRAP;
        break;
    case SPR_DRR_FPE:
        rspptr->sigval = TARGET_SIGNAL_FPE;
        break;
    case SPR_DRR_TE:
        rspptr->sigval = TARGET_SIGNAL_TRAP;
        break;

    default:
        // This must be the case of single step (which does not set DRR)
        rspptr->sigval = TARGET_SIGNAL_TRAP;
        break;
    }

    if (g_show_verbose)
        printf("[DEBUG_GDB] rspptr->sigval: 0x%x\n", rspptr->sigval);

    return;
}

/*---------------------------------------------------------------------------*/
/*!Check if PPC is in an exception vector that halts program flow

  Compare the provided PPC with known exception vectors that are fatal
  to a program's execution. Call rsp_exception(ppc) to set the appropriate
  sigval and return.

  @param[in] ppc  Value of current PPC, as read from debug unit
  @return: 1 if we set a sigval and should return control to GDB, else 0       */
  /*---------------------------------------------------------------------------*/
static int check_for_exception_vector(uint32_t ppc, struct global_rsp* rspptr) {
    switch (ppc) {
        // The following should return sigvals to GDB for processing
    case EXCEPT_BUSERR:
    case EXCEPT_ALIGN:
    case EXCEPT_ILLEGAL:
    case EXCEPT_TRAP:
        if (g_show_verbose)
            printf("PPC at exception address\n");
        rsp_exception(ppc, rspptr);
        return 1;

    default:
        return 0;
    }
    return 1;
}

/*---------------------------------------------------------------------------*/
/*!Note an exception for future processing

  The simulator has encountered an exception. Record it here, so that a
  future call to handle_exception will report it back to the client. The
  signal is supplied in Or1ksim form and recorded in GDB form.

  We flag up a warning if an exception is already pending, and ignore the
  earlier exception.

  @param[in] except  The exception (Or1ksim form)                           */
  /*---------------------------------------------------------------------------*/
void rsp_exception(uint32_t except, struct global_rsp* rspptr) {
    int sigval;		/* GDB signal equivalent to exception */

    switch (except) {
    case EXCEPT_RESET:
        sigval = TARGET_SIGNAL_PWR;
        break;
    case EXCEPT_BUSERR:
        sigval = TARGET_SIGNAL_BUS;
        break;
    case EXCEPT_DPF:
        sigval = TARGET_SIGNAL_SEGV;
        break;
    case EXCEPT_IPF:
        sigval = TARGET_SIGNAL_SEGV;
        break;
    case EXCEPT_TICK:
        sigval = TARGET_SIGNAL_ALRM;
        break;
    case EXCEPT_ALIGN:
        sigval = TARGET_SIGNAL_BUS;
        break;
    case EXCEPT_ILLEGAL:
        sigval = TARGET_SIGNAL_ILL;
        break;
    case EXCEPT_INT:
        sigval = TARGET_SIGNAL_INT;
        break;
    case EXCEPT_DTLBMISS:
        sigval = TARGET_SIGNAL_SEGV;
        break;
    case EXCEPT_ITLBMISS:
        sigval = TARGET_SIGNAL_SEGV;
        break;
    case EXCEPT_RANGE:
        sigval = TARGET_SIGNAL_FPE;
        break;
    case EXCEPT_SYSCALL:
        sigval = TARGET_SIGNAL_USR2;
        break;
    case EXCEPT_FPE:
        sigval = TARGET_SIGNAL_FPE;
        break;
    case EXCEPT_TRAP:
        sigval = TARGET_SIGNAL_TRAP;
        break;

    default:
        fprintf(stderr, "Warning: Unknown RSP exception %u: Ignored\n",
            except);
        return;
    }

    if ((0 != rspptr->sigval) && (sigval != rspptr->sigval)) {
        fprintf(stderr, "Warning: RSP signal %d received while signal "
            "%d pending: Pending exception replaced\n", sigval,
            rspptr->sigval);
    }

    rspptr->sigval = sigval;	/* Save the signal value */

}				/* rsp_exception () */

/*---------------------------------------------------------------------------*/
/*!Get a new client connection.

  Blocks until the client connection is available.

  A lot of this code is copied from remote_open in gdbserver remote-utils.c.

  This involves setting up a socket to listen on a socket for attempted
  connections from a single GDB instance (we couldn't be talking to multiple
  GDBs at once!).

  The service is specified either as a port number in the Or1ksim configuration
  (parameter rsp_port in section debug, default 51000) or as a service name
  in the constant OR1KSIM_RSP_SERVICE.

  The protocol used for communication is specified in OR1KSIM_RSP_PROTOCOL. */
  /*---------------------------------------------------------------------------*/
static void rsp_get_client(struct global_rsp* rspptr) {
    if (g_trace_flow) {
        printf("[TRACE_FLOW] rsp_get_client() on port `%d`: enter\n", rspptr->serverPort);
    }

    int tmp_fd;		/* Temporary descriptor for socket */
    int optval;		/* Socket options */
    struct sockaddr_in sock_addr;	/* Socket address */
    socklen_t len;		/* Size of the socket address */

    /* 0 is used as the RSP port number to indicate that we should use the
       service name instead. */
    if (0 == rspptr->serverPort) {
        struct servent* service =
            getservbyname(OR1KSIM_RSP_SERVICE, "tcp");
        if (NULL == service) {
            fprintf(stderr,
                "Warning: RSP unable to find service \"%s\": %s\n",
                OR1KSIM_RSP_SERVICE, strerror(errno));
            return;
        }
        rspptr->serverPort = ntohs(service->s_port);
    }

    /* Open a socket on which we'll listen for clients */
    tmp_fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (tmp_fd < 0) {
        fprintf(stderr, "ERROR: Cannot open RSP socket\n");
        exit(0);
    }

    /* Allow rapid reuse of the port on this socket */
    optval = 1;
    setsockopt(tmp_fd, SOL_SOCKET, SO_REUSEADDR, (char*)&optval,
        sizeof(optval));

    /* Bind the port to the socket */
    sock_addr.sin_family = PF_INET;
    sock_addr.sin_port = htons(rspptr->serverPort);
    sock_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(tmp_fd, (struct sockaddr*)&sock_addr, sizeof(sock_addr))) {
        fprintf(stderr, "ERROR: Cannot bind to RSP socket\n");
        exit(0);
    }

    /* Listen for (at most one) client */
    if (0 != listen(tmp_fd, 1)) {
        fprintf(stderr, "ERROR: Cannot listen on RSP socket\n");
        exit(0);
    }

    printf("\nCore %d: waiting for gdb connection on localhost:%d\n", rspptr->cpu_no, rspptr->serverPort);
    fflush(stdout);

    printf("Core %d: press CTRL+c to exit.\n", rspptr->cpu_no);
    fflush(stdout);

    /* Accept a client which connects */
    len = sizeof(sock_addr);
    rspptr->client_fd = accept(tmp_fd, (struct sockaddr*)&sock_addr, &len);

    if (-1 == rspptr->client_fd) {
        fprintf(stderr, "Warning: Failed to accept RSP client\n");
        return;
    }

    /* Enable TCP keep alive process */
    optval = 1;
    setsockopt(rspptr->client_fd, SOL_SOCKET, SO_KEEPALIVE, (char*)&optval,
        sizeof(optval));

    int flags;

    /* If they have O_NONBLOCK, use the Posix way to do it */

#if defined(O_NONBLOCK)
    /* Fixme: O_NONBLOCK is defined but broken on SunOS 4.1.x and AIX 3.2.5. */
    if (-1 == (flags = fcntl(rspptr->client_fd, F_GETFL, 0)))
        flags = 0;

    fcntl(rspptr->client_fd, F_SETFL, flags | O_NONBLOCK);
#else
    /* Otherwise, use the old way of doing it */
    flags = 1;
    ioctl(rspptr->client_fd, FIOBIO, &flags);
#endif

    /* Set socket to be non-blocking.

       We do this because when we're given a continue, or step
       instruction,command we set the processor stall off, then instantly check
       if it's stopped. If it hasn't then we drop through and wait for input
       from GDB. Obviously this will cause problems when it will stop after we
       do the check. So now, rsp_peek(rspptr) been implemented to simply check if
       there's an incoming command from GDB (although, mainly interested in
       int. commands), otherwise it returns back to poll the processor's
       stall bit. It can only do this if the socket is non-blocking.

       At first test, simply adding this line appeared to give no problems with
       the existing code. No "simulation" of blocking behaviour on the
       non-blocking socket was required (in the event that a read/write throws
       back a EWOULDBLOCK error, as was looked to be the case in the previous
       GDB handling code) -- Julius
     */

    if (ioctl(rspptr->client_fd, FIONBIO, (char*)&optval) > 0) {
        perror("ioctl() failed");
        close(rspptr->client_fd);
        close(tmp_fd);
        exit(0);
    }

    /* Don't delay small packets, for better interactive response (disable
       Nagel's algorithm) */
    optval = 1;
    setsockopt(rspptr->client_fd, IPPROTO_TCP, TCP_NODELAY, (char*)&optval,
        sizeof(optval));

    /* Socket is no longer needed */
    close(tmp_fd);		/* No longer need this */
    signal(SIGPIPE, SIG_IGN);	/* So we don't exit if client dies */

    printf("\nCore %d: remote `%s mode` debugging from host %s\n",
            rspptr->cpu_no, (STOP_MODE == g_option_debug_mode) ? "stop" : "non-stop", inet_ntoa(sock_addr.sin_addr));

    if (g_trace_flow) {
        printf("[TRACE_FLOW] rsp_get_client() on port `%d`: done and exit.\n", rspptr->serverPort);
    }
}				/* rsp_get_client () */

static uint64_t getRelativeMicroseconds() {
    uint64_t current_us = GetTimeStampMicrosecond();
    return (current_us - g_handle_rsp_start_us);
}

static uint32_t read_id_pc() {
    uint32_t idpc;
    int saved_gdb_chain = gdb_chain;
    gdb_read_reg(PPC_CPU_REG_ADD, &idpc);
    gdb_set_chain(saved_gdb_chain);
    return idpc;
}

static void print_pc() {
    if (!g_option_show_pc) return;
    uint32_t npc, ppc, expc;
    int saved_gdb_chain = gdb_chain;
    gdb_set_chain(SC_RISC_DEBUG);
    gdb_read_reg(NPC_CPU_REG_ADD, &npc);
    gdb_read_reg(PPC_CPU_REG_ADD, &ppc);
    gdb_read_reg(EXPC_CPU_REG_ADD, &expc);
    gdb_set_chain(saved_gdb_chain);
    printf("[Program Counter] if3_pc = 0x%08x, id_pc = 0x%08x, ex1_pc = 0x%08x\n", npc, ppc, expc);
}

/*---------------------------------------------------------------------------*/
/*!Deal with a request from the GDB client session

  In general, apart from the simplest requests, this function replies on
  other functions to implement the functionality.                           */
  /*---------------------------------------------------------------------------*/
static void rsp_client_request(struct global_rsp* rspptr) {
    struct rsp_buf* p_buf = get_packet(rspptr);	/* Message sent to us */

    // Null packet means we hit EOF or the link was closed for some other
    // reason. Close the client and return
    if (NULL == p_buf) {
        rsp_client_close(rspptr);
        return;
    }

    if (g_show_verbose) {
        printf("\n---------------- Core %d: got packet from GDB ----------------\n", rspptr->cpu_no);
        if (p_buf->data[0] == 'G') {
            printf("[@ %lu us] [GDB => proxy] G XX..., %d character(s)\n", getRelativeMicroseconds(), p_buf->len);
            fflush(stdout);
        } else {
            printf("[@ %lu us] [GDB => proxy] %s, %d character(s)\n", getRelativeMicroseconds(), p_buf->data, p_buf->len);
            fflush(stdout);
        }
    }

    print_pc();

    /* In normal GDB RSP packet sequence, after c/s packet responded, g packet is expected
     * If not, it is most likely that Live Expression refresh overwrites g packet and IDE does not
     * know which PC DSP stalls, so clear nonstop_dsp_stall_status_reported for IDE to send interrupt signal
     */
    if (rspptr->just_handle_packet_c || rspptr->just_handle_packet_s) {
        if ('g' != p_buf->data[0]) {
            rspptr->nonstop_dsp_stall_status_reported = 0;
        }
    }

    rspptr->just_handle_packet_c = 0;
    rspptr->just_handle_packet_s = 0;
    rspptr->expected_location_packet_s = INVALID_PC_LOCATION;
    rspptr->just_handle_packet_F = 0;

    switch (p_buf->data[0]) {
    case '!':
        /* Request for extended remote mode */
        put_str_packet("OK", rspptr);	// OK = supports and has enabled extended mode.
        return;

    case '?':
        /* Return last signal ID */
        rsp_report_exception(rspptr);
        return;

    case 'A':
        /* Initialization of argv not supported */
        fprintf(stderr,
            "Warning: RSP 'A' packet not supported: ignored\n");
        put_str_packet("E01", rspptr);
        return;

    case 'b':
        /* Setting baud rate is deprecated */
        fprintf(stderr, "Warning: RSP 'b' packet is deprecated and not "
            "supported: ignored\n");
        return;

    case 'B':
        /* Breakpoints should be set using Z packets */
        fprintf(stderr,
            "Warning: RSP 'B' packet is deprecated (use 'Z'/'z' "
            "packets instead): ignored\n");
        return;

    case 'c':
        /* Continue */
//		rsp_continue(p_buf,rspptr);
//		rsp_step_continue(p_buf);
        rsp_hardware_continue(p_buf, rspptr);
        rspptr->current_stat = 2;
        rspptr->just_handle_packet_c = 1;
        if (NON_STOP_MODE == g_option_debug_mode) {
            rspptr->nonstop_dsp_stall_status_reported = 0;
        }
        g_download_program_stage = 0;
        return;

    case 'C':
        /* Continue with signal */
        rsp_continue_with_signal(p_buf, rspptr);
        rspptr->current_stat = 2;
        g_download_program_stage = 0;
        return;

    case 'd':
        /* Disable debug using a general query */
        fprintf(stderr,
            "Warning: RSP 'd' packet is deprecated (define a 'Q' "
            "packet instead: ignored\n");
        return;

    case 'D':
        /* Detach GDB. Do this by closing the client. The rules say that
           execution should continue. TODO. Is this really then intended
           meaning? Or does it just mean that only vAttach will be recognized
           after this? */
        put_str_packet("OK", rspptr);
        rsp_client_close(rspptr);
        reset_or1k();
        set_stall_state(0, rspptr);
        return;

    case 'F':
        /* File I/O is not currently supported */
        target_recv_trap_back(p_buf, rspptr);
        rspptr->just_handle_packet_F = 1;
        return;

    case 'g':
        rsp_read_all_regs(rspptr);
        return;

    case 'G':
        //rsp_write_all_regs(p_buf,rspptr);
        put_str_packet("OK", rspptr);
        return;

    case 'H':
        /* Set the thread number of subsequent operations. For now ignore
           silently and just reply "OK" */
        put_str_packet("OK", rspptr);
        return;

    case 'i':
        /* Single instruction step */
        fprintf(stderr,
            "Warning: RSP cycle stepping not supported: target "
            "stopped immediately\n");
        rspptr->client_waiting = 1;	/* Stop reply will be sent */
        return;

    case 'I':
        /* Single instruction step with signal */
        fprintf(stderr,
            "Warning: RSP cycle stepping not supported: target "
            "stopped immediately\n");
        rspptr->client_waiting = 1;	/* Stop reply will be sent */
        return;

    case 'k':
        /* Kill request. Do nothing for now. */
        printf("Core %d: disconnect from debugger on port %d\n", rspptr->cpu_no, rspptr->serverPort);

        if (!g_stalldsp_on_terminate) {
            // resume all DSP cores when debug terminates
            usb_dbg_cpu_write_ctrl(0, 0, 0);
            if (g_dual_core_dsp) {
                usb_dbg_cpu_write_ctrl(1, 0, 0);
            }
        }

#if 0
        {
            uint32_t reset0, reset1;
            uint32_t stall0, stall1;
            usb_dbg_cpu_read_ctrl(0, &reset0, &stall0);
            usb_dbg_cpu_read_ctrl(0, &reset1, &stall1);
            printf("stall_0 = %d, stall_1 = %d\n", stall0, stall1);
        }
#endif
        fflush(stdout);
        return;

    case 'm':
        /* Read memory (symbolic) */
        rsp_read_mem(p_buf, rspptr);
        //rsp_read_mem_seprate(p_buf,rspptr);
        return;

    case 'M':
        /* Write memory (symbolic) */
        rsp_write_mem(p_buf, rspptr);
        return;

    case 'p':
        /* Read a register */
        //rsp_read_reg(p_buf,rspptr);
        put_str_packet("", rspptr);
        return;

    case 'P':
        /* Write a register */
        //rsp_write_reg(p_buf,rspptr);
        put_str_packet("", rspptr);
        return;

    case 'q':
        /* Any one of a number of query packets */
        rsp_query(p_buf, rspptr);
        //put_str_packet("",rspptr);
        return;

    case 'Q':
        /* Any one of a number of set packets */
        rsp_set(p_buf, rspptr);
        return;

    case 'r':
        /* Reset the system. Deprecated (use 'R' instead) */
        fprintf(stderr,
            "Warning: RSP 'r' packet is deprecated (use 'R' "
            "packet instead): ignored\n");
        return;

    case 'R':
        /* Restart the program being debugged. */
        rsp_restart(rspptr);
        return;

    case 's':
        /* Single step (one high level instruction). This could be hard without
           DWARF2 info */
           //printf("case s \n");
        rsp_step(p_buf, rspptr);
        rspptr->current_stat = 1;
        rspptr->just_handle_packet_s = 1;
        if (NON_STOP_MODE == g_option_debug_mode) {
            rspptr->nonstop_dsp_stall_status_reported = 0;
        }
        g_download_program_stage = 0;
        return;

    case 'S':
        /* Single step (one high level instruction) with signal. This could be
           hard without DWARF2 info */
        rsp_step_with_signal(p_buf, rspptr);
        rspptr->current_stat = 1;
        g_download_program_stage = 0;
        return;

    case 't':
        /* Search. This is not well defined in the manual and for now we don't
           support it. No response is defined. */
        fprintf(stderr,
            "Warning: RSP 't' packet not supported: ignored\n");
        return;

    case 'T':
        /* Is the thread alive. We are bare metal, so don't have a thread
           context. The answer is always "OK". */
        put_str_packet("OK", rspptr);
        return;

    case 'v':
        /* Any one of a number of packets to control execution */
        rsp_vpkt(p_buf, rspptr);
        return;

    case 'X':
        /* Write memory (binary) */
        // if(traping)
        // {
        // 	put_str_packet("OK",rspptr);
        // 	return;
        // }
        rsp_write_mem_bin(p_buf, rspptr);
        return;

    case 'z':
        /* Remove a breakpoint/watchpoint. */
        rsp_remove_matchpoint(p_buf, rspptr);
        return;

    case 'Z':
        /* Insert a breakpoint/watchpoint. */
        rsp_insert_matchpoint(p_buf, rspptr);
        return;

    default:
        /* Unknown commands are ignored */
        fprintf(stderr, "Warning: Unknown RSP request %s\n",
            p_buf->data);
        return;
    }
}

/*---------------------------------------------------------------------------*/
/*!Close the connection to the client if it is open                          */
/*---------------------------------------------------------------------------*/
static void rsp_client_close(struct global_rsp* rspptr) {
    if (-1 != rspptr->client_fd) {
        close(rspptr->client_fd);
        rspptr->client_fd = -1;
    }
}				/* rsp_client_close () */

/*---------------------------------------------------------------------------*/
/*!Send a packet to the GDB client

  Modeled on the stub version supplied with GDB. Put out the data preceded by
  a '$', followed by a '#' and a one byte checksum. '$', '#', '*' and '}' are
  escaped by preceding them with '}' and then XORing the character with
  0x20.

  @param[in] p_buf  The data to send                                          */
  /*---------------------------------------------------------------------------*/
static void put_packet(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    unsigned char data[GDB_BUF_MAX * 2];
    int len;
    int ch;			/* Ack char */

    /* Construct $<packet info>#<checksum>. Repeat until the GDB client
       acknowledges satisfactory receipt. */
    do {
        unsigned char checksum = 0;	/* Computed checksum */
        int count = 0;	/* Index into the buffer */
#if 0
        if (DEBUG_GDB_DUMP_DATA) {
            printf("Putting %s\n\n", p_buf->data);
            fflush(stdout);
        }
#endif

        len = 0;
        data[len++] = '$';	/* Start char */

        /* Body of the packet */
        for (count = 0; count < p_buf->len; count++) {
            unsigned char ch = p_buf->data[count];

            /* Check for escaped chars */
            if (('$' == ch) || ('#' == ch) || ('*' == ch)
                || ('}' == ch)) {
                ch ^= 0x20;
                checksum += (unsigned char)'}';
                data[len++] = '}';
            }

            checksum += ch;
            data[len++] = ch;
        }

        data[len++] = '#';	/* End char */

        /* Computed checksum */
        data[len++] = (hexchars[checksum >> 4]);
        data[len++] = (hexchars[checksum % 16]);

        send_rsp_str((unsigned char*)&data, len, rspptr);

        /* Check for ack of connection failure */
        ch = get_rsp_char(rspptr);
        if (0 > ch) {
            return;	/* Fail the put silently. */
        }
    } while ('+' != ch);
#if 0
    if (DEBUG_GDB_DUMP_DATA) {
        printf("packet sent: %s\n", p_buf->data);
    }
#endif
    if (g_show_verbose) {
        printf("[@ %lu us] [GDB <= proxy] %s\n", getRelativeMicroseconds(), 
                (strlen(p_buf->data) != 0) ? p_buf->data : "\"\" (means Not supported GDB packet)");
    }
    fflush(stdout);
}				/* put_packet () */

/*---------------------------------------------------------------------------*/
/*!Convenience to put a constant string packet

  param[in] str  The text of the packet                                     */
  /*---------------------------------------------------------------------------*/
static void put_str_packet(const char* str, struct global_rsp* rspptr) {
    struct rsp_buf buffer;
    int len = strlen(str);

    /* Construct the packet to send, so long as string is not too big,
       otherwise truncate. Add EOS at the end for convenient debug printout */

    if (len >= GDB_BUF_MAX) {
        fprintf(stderr, "Warning: String %s too large for RSP packet: "
            "truncated\n", str);
        len = GDB_BUF_MAX - 1;
    }

    strncpy(buffer.data, str, len);
    buffer.data[len] = 0;
    buffer.len = len;

    put_packet(&buffer, rspptr);

}				/* put_str_packet () */

/*---------------------------------------------------------------------------*/
/*!Get a packet from the GDB client

  Modeled on the stub version supplied with GDB. The data is in a static
  buffer. The data should be copied elsewhere if it is to be preserved across
  a subsequent call to get_packet().

  Unlike the reference implementation, we don't deal with sequence
  numbers. GDB has never used them, and this implementation is only intended
  for use with GDB 6.8 or later. Sequence numbers were removed from the RSP
  standard at GDB 5.0.

  @return  A pointer to the static buffer containing the data                */
  /*---------------------------------------------------------------------------*/
static struct rsp_buf* get_packet(struct global_rsp* rspptr) {
    static struct rsp_buf buf;	/* Survives the return */

    /* Keep getting packets, until one is found with a valid checksum */
    while (1) {
        unsigned char checksum;	/* The checksum we have computed */
        int count;	/* Index into the buffer */
        int ch;		/* Current character */

        /* Wait around for the start character ('$'). Ignore all other
           characters */
        ch = get_rsp_char(rspptr);

        while (ch != '$') {
            if (-1 == ch) {
                return NULL;	/* Connection failed */
            }

            ch = get_rsp_char(rspptr);

            // Potentially handle an interrupt character (0x03) here                
        }

        /* Read until a '#' or end of buffer is found */
        checksum = 0;
        count = 0;
        while (count < GDB_BUF_MAX - 1) {
            ch = get_rsp_char(rspptr);

            if (rspptr->client_waiting && g_show_verbose) {
                printf("%x\n", ch);
            }

            /* Check for connection failure */
            if (0 > ch) {
                return NULL;
            }

            /* If we hit a start of line char begin all over again */
            if ('$' == ch) {
                checksum = 0;
                count = 0;

                continue;
            }

            /* Break out if we get the end of line char */
            if ('#' == ch) {
                break;
            }

            /* Update the checksum and add the char to the buffer */

            checksum = checksum + (unsigned char)ch;
            buf.data[count] = (char)ch;
            count = count + 1;
        }

        /* Mark the end of the buffer with EOS - it's convenient for non-binary
           data to be valid strings. */
        buf.data[count] = 0;
        buf.len = count;

        /* If we have a valid end of packet char, validate the checksum */
        if ('#' == ch) {
            unsigned char xmitcsum;	/* The checksum in the packet */

            ch = get_rsp_char(rspptr);
            if (0 > ch) {
                return NULL;	/* Connection failed */
            }
            xmitcsum = hex(ch) << 4;

            ch = get_rsp_char(rspptr);
            if (0 > ch) {
                return NULL;	/* Connection failed */
            }

            xmitcsum += hex(ch);

            /* If the checksums don't match print a warning, and put the
               negative ack back to the client. Otherwise put a positive ack. */
            if (checksum != xmitcsum) {
                fprintf(stderr,
                    "Warning: Bad RSP checksum: Computed "
                    "0x%02x, received 0x%02x\n", checksum,
                    xmitcsum);

                ch = '-';
                send_rsp_str((unsigned char*)&ch, 1, rspptr);	/* Failed checksum */
            } else {
                ch = '+';
                send_rsp_str((unsigned char*)&ch, 1, rspptr);	/* successful transfer */
                break;
            }
        } else {
            fprintf(stderr, "Warning: RSP packet overran buffer\n");
        }
    }
    return &buf;		/* Success */
}				/* get_packet () */

/*---------------------------------------------------------------------------*/
/*!Put a single character out onto the client socket

  This should only be called if the client is open, but we check for safety.

  @param[in] c  The character to put out                                    */
  /*---------------------------------------------------------------------------*/
static void send_rsp_str(unsigned char* data, int len, struct global_rsp* rspptr) {
    if (-1 == rspptr->client_fd) {
        fprintf(stderr,
            "Warning: Attempt to write '%s' to unopened RSP "
            "client: Ignored\n", data);
        return;
    }

    /* Write until successful (we retry after interrupts) or catastrophic
       failure. */
    while (1) {
        switch (write(rspptr->client_fd, data, len)) {
        case -1:
            /* Error: only allow interrupts or would block */
            if ((EAGAIN != errno) && (EINTR != errno)) {
                fprintf(stderr,
                    "Warning: Failed to write to RSP client: "
                    "Closing client connection: %s\n",
                    strerror(errno));
                rsp_client_close(rspptr);
                return;
            }

            break;

        case 0:
            break;	/* Nothing written! Try again */

        default:
            return;	/* Success, we can return */
        }
    }
}				/* send_rsp_str () */

/*---------------------------------------------------------------------------*/
/*!Get a single character from the client socket

  This should only be called if the client is open, but we check for safety.

  @return  The character read, or -1 on failure                             */
  /*---------------------------------------------------------------------------*/
static int get_rsp_char(struct global_rsp* rspptr) {
    if (-1 == rspptr->client_fd) {
        fprintf(stderr, "Warning: Attempt to read from unopened RSP "
            "client: Ignored\n");
        return -1;
    }

    /* Non-blocking read until successful (we retry after interrupts) or
       catastrophic failure. */
    while (1) {
        unsigned char c;

        switch (read(rspptr->client_fd, &c, sizeof(c))) {
        case -1:
            /* Error: only allow interrupts */
            if ((EAGAIN != errno) && (EINTR != errno)) {
                fprintf(stderr,
                    "Warning: Failed to read from RSP client: "
                    "Closing client connection: %s\n",
                    strerror(errno));
                rsp_client_close(rspptr);
                return -1;
            }

            break;

        case 0:
            // EOF
            rsp_client_close(rspptr);
            return -1;

        default:
            return c & 0xff;	/* Success, we can return (no sign extend!) */
        }
    }
}				/* get_rsp_char () */

/*---------------------------------------------------------------------------*/
/* !Peek at data coming into server from GDB

   Useful for polling for ETX (0x3) chars being sent when GDB wants to
   interrupt

   @return the char we peeked, 0 otherwise                                   */
   /*---------------------------------------------------------------------------*/
static char rsp_peek(struct global_rsp* rspptr) {
    /*
       if (-1 == rspptr->client_fd)
       {
       fprintf (stderr, "Warning: Attempt to read from unopened RSP "
       "client: Ignored\n");
       return  -1;
       }
     */
    char c;
    int n;
    // Using recv here instead of read becuase we can pass the MSG_PEEK
    // flag, which lets us look at what's on the socket, without actually
    // taking it off

    //if (g_show_verbose) 
    //  printf("peeking at GDB socket...\n");

    n = recv(rspptr->client_fd, &c, sizeof(c), MSG_PEEK);

    //if (g_show_verbose) 
    //  printf("peeked, got n=%d, c=0x%x\n",n, c);

    if (n > 0)
        return c;
    else
        return '\0';

}

/*---------------------------------------------------------------------------*/
/*!Handle an interrupt from GDB

  Detect an interrupt from GDB and stall the processor                        */
  /*---------------------------------------------------------------------------*/
static void rsp_interrupt(struct global_rsp* rspptr) {
    unsigned char c;

    if (read(rspptr->client_fd, &c, sizeof(c)) <= 0) {
        // Had issues, just return
        return;
    }
    // Ensure this is a ETX control char (0x3), currently, we only call this
    // function when we've peeked and seen it, otherwise, ignore, return and pray
    // things go back to normal...
    if (c != 0x03) {
        printf
        ("Warning: Interrupt character expected but not found on socket.\n");
        return;
    }
    // Otherwise, it's an interrupt packet, stall the processor, and upon return
    // to the main handle_rsp() loop, it will inform GDB.

    if (g_show_verbose) {
        printf("Interrupt received from GDB.\n");
        printf("core%d: Stalling processor ... ", rspptr->cpu_no);
    }

    set_stall_state(1, rspptr);

    if (g_show_verbose) {
        printf("done.\n");
    }

    // Send a stop reply response, manually set rspptr->sigval to TARGET_SIGNAL_NONE
    rspptr->sigval = TARGET_SIGNAL_NONE;
    rsp_report_exception(rspptr);
    rspptr->client_waiting = 0;	/* No longer waiting */

    return;
}

/*---------------------------------------------------------------------------*/
/*!"Unescape" RSP binary data

  '#', '$' and '}' are escaped by preceding them by '}' and oring with 0x20.

  This function reverses that, modifying the data in place.

  @param[in] data  The array of bytes to convert
  @para[in]  len   The number of bytes to be converted

  @return  The number of bytes AFTER conversion                             */
  /*---------------------------------------------------------------------------*/
static int rsp_unescape(char* data, int len) {
    int from_off = 0;	/* Offset to source char */
    int to_off = 0;		/* Offset to dest char */

    while (from_off < len) {
        /* Is it escaped */
        if ('}' == data[from_off]) {
            from_off++;
            data[to_off] = data[from_off] ^ 0x20;
        } else {
            data[to_off] = data[from_off];
        }

        from_off++;
        to_off++;
    }

    return to_off;

}				/* rsp_unescape () */

/*---------------------------------------------------------------------------*/
/*!Initialize the matchpoint hash table

  This is an open hash table, so this function clears all the links to
  NULL.                                                                     */
  /*---------------------------------------------------------------------------*/
static void mp_hash_init(struct global_rsp* rspptr) {
    int i;

    for (i = 0; i < MP_HASH_SIZE; i++) {
        rspptr->mp_hash[i] = NULL;
    }
}				/* mp_hash_init () */

/*---------------------------------------------------------------------------*/
/*!Add an entry to the matchpoint hash table

  Add the entry if it wasn't already there. If it was there do nothing. The
  match just be on type and addr. The instr need not match, since if this is
  a duplicate insertion (perhaps due to a lost packet) they will be
  different.

  @param[in] type   The type of matchpoint
  @param[in] addr   The address of the matchpoint
  @para[in]  instr  The instruction to associate with the address           */
  /*---------------------------------------------------------------------------*/
static void mp_hash_add(enum mp_type type, uint32_t addr, uint32_t instr, struct global_rsp* rspptr) {
    int hv = addr % MP_HASH_SIZE;
    struct mp_entry* curr;

    /* See if we already have the entry */
    for (curr = rspptr->mp_hash[hv]; NULL != curr; curr = curr->next) {
        if ((type == curr->type) && (addr == curr->addr)) {
            return;	/* We already have the entry */
        }
    }

    /* Insert the new entry at the head of the chain */
    curr = (struct mp_entry*)malloc(sizeof(*curr));

    curr->type = type;
    curr->addr = addr;
    curr->instr = instr;
    curr->next = rspptr->mp_hash[hv];

    rspptr->mp_hash[hv] = curr;

}				/* mp_hash_add () */

/* Called by the RSP server to get any one unused HWP.
 * This will only be called immediately before a 'step'
 * or 'continue,' and the HWP will be disabled as soon
 * as the CPU returns control to the RSP server.
 * Returns -1 if no HWP available.
 */
int hwp_get_available_watchpoint(void) {
    int i;
    int ret = -1;

    for (i = 0; i < HWP_MAX_WP; i++) {
        if (hwp_in_use[i] == 0) {
            ret = i;
            hwp_in_use[i] = 1;

            break;
        }
    }
    if (g_show_verbose)
        printf("HWP granting wp %i to GDB/RSP\n", ret);
    return ret;
}

/* Called by the RSP server to indicate it is no longer
 * using a watchpoint previously granted by
 * hwp_get_available_watchpoint()
 */
void hwp_return_watchpoint(int wp) {
    if (wp >= HWP_MAX_WP) {
        fprintf(stderr, "ERROR! WP value %i out of range in hwp_return_watchpoint()!\n", wp);
    } else {
        if (hwp_in_use[wp] != 0) {
            hwp_in_use[wp] = 0;
            if (g_show_verbose)
                printf("HWP got wp %i back from GDB/RSP\n", wp);
        } else
            fprintf(stderr, "ERROR! hwp_return_watchpoint() returning wp %i, not in use!\n", wp);
    }
}
/*---------------------------------------------------------------------------*/
/*!Look up an entry in the matchpoint hash table

  The match must be on type AND addr.

  @param[in] type   The type of matchpoint
  @param[in] addr   The address of the matchpoint

  @return  The entry deleted, or NULL if the entry was not found            */
  /*---------------------------------------------------------------------------*/
static struct mp_entry* mp_hash_lookup(enum mp_type type, uint32_t addr, struct global_rsp* rspptr) {
    int hv = addr % MP_HASH_SIZE;
    struct mp_entry* curr;

    /* Search */
    for (curr = rspptr->mp_hash[hv]; NULL != curr; curr = curr->next) {
        if ((type == curr->type) && (addr == curr->addr)) {
            return curr;	/* The entry found */
        }
    }

    /* Not found */
    return NULL;

}				/* mp_hash_lookup () */

/*---------------------------------------------------------------------------*/
/*!Delete an entry from the matchpoint hash table

  If it is there the entry is deleted from the hash table. If it is not
  there, no action is taken. The match must be on type AND addr.

  The usual fun and games tracking the previous entry, so we can delete
  things.

  @note  The deletion DOES NOT free the memory associated with the entry,
  since that is returned. The caller should free the memory when they
  have used the information.

  @param[in] type   The type of matchpoint
  @param[in] addr   The address of the matchpoint

  @return  The entry deleted, or NULL if the entry was not found            */
  /*---------------------------------------------------------------------------*/
static struct mp_entry* mp_hash_delete(enum mp_type type, uint32_t addr, struct global_rsp* rspptr) {
    int hv = addr % MP_HASH_SIZE;
    struct mp_entry* prev = NULL;
    struct mp_entry* curr;

    /* Search */
    for (curr = rspptr->mp_hash[hv]; NULL != curr; curr = curr->next) {
        if ((type == curr->type) && (addr == curr->addr)) {
            /* Found - delete. Method depends on whether we are the head of
               chain. */
            if (NULL == prev) {
                rspptr->mp_hash[hv] = curr->next;
            } else {
                prev->next = curr->next;
            }

            return curr;	/* The entry deleted */
        }

        prev = curr;
    }

    /* Not found */
    return NULL;

}				/* mp_hash_delete () */

/*---------------------------------------------------------------------------*/
/*!Utility to give the value of a hex char

  @param[in] ch  A character representing a hexadecimal digit. Done as -1,
  for consistency with other character routines, which can use
  -1 as EOF.

  @return  The value of the hex character, or -1 if the character is
  invalid.                                                         */
  /*---------------------------------------------------------------------------*/
static int hex(int c) {
    return ((c >= 'a') && (c <= 'f')) ? c - 'a' + 10 :
        ((c >= '0') && (c <= '9')) ? c - '0' :
        ((c >= 'A') && (c <= 'F')) ? c - 'A' + 10 : -1;

}				/* hex () */

/*---------------------------------------------------------------------------*/
/*!Convert a register to a hex digit string

  The supplied 32-bit value is converted to an 8 digit hex string according
  the target endianism. It is null terminated for convenient printing.

  @param[in]  val  The value to convert
  @param[out] p_buf  The buffer for the text string                           */
  /*---------------------------------------------------------------------------*/
static void reg2hex(uint32_t val, char* p_buf) {
    int n;			/* Counter for digits */
    int nyb_shift;
    for (n = 0; n < 4; n++) {
#ifdef WORDSBIGENDIAN
        if (n % 2 == 0) {
            nyb_shift = n * 4 + 4;
        } else {
            nyb_shift = n * 4 - 4;
        }
#else
        //		nyb_shift = 28 - (n * 4);
        nyb_shift = n * 8;
#endif
        p_buf[2 * n] = hexchars[(val >> (nyb_shift + 4)) & 0xf];
        p_buf[2 * n + 1] = hexchars[(val >> nyb_shift) & 0xf];
    }

    p_buf[8] = 0;		/* Useful to terminate as string */

}				/* reg2hex () */

/*---------------------------------------------------------------------------*/
/*!Convert a hex digit string to a register value

  The supplied 8 digit hex string is converted to a 32-bit value according
  the target endianism

  @param[in] p_buf  The buffer with the hex string

  @return  The value to convert                                             */
  /*---------------------------------------------------------------------------*/
static uint32_t hex2reg(char* p_buf) {
    int n;			/* Counter for digits */
    uint32_t val = 0;	/* The result */

    for (n = 0; n < 8; n++) {
#ifdef WORDSBIGENDIAN
        int nyb_shift = n * 4;
#else
        int nyb_shift = 28 - (n * 4);
#endif
        val |= hex(p_buf[n]) << nyb_shift;
    }

    return val;

}				/* hex2reg () */

/*---------------------------------------------------------------------------*/
/*!Convert an ASCII character string to pairs of hex digits

  Both source and destination are null terminated.

  @param[out] dest  Buffer to store the hex digit pairs (null terminated)
  @param[in]  src   The ASCII string (null terminated)                      */
  /*---------------------------------------------------------------------------*/
static void ascii2hex(char* dest, char* src) {
    int i;

    /* Step through converting the source string */
    for (i = 0; src[i] != '\0'; i++) {
        char ch = src[i];

        dest[i * 2] = hexchars[ch >> 4 & 0xf];
        dest[i * 2 + 1] = hexchars[ch & 0xf];
    }

    dest[i * 2] = '\0';

}				/* ascii2hex () */

/*---------------------------------------------------------------------------*/
/*!Convert pairs of hex digits to an ASCII character string

  Both source and destination are null terminated.

  @param[out] dest  The ASCII string (null terminated)
  @param[in]  src   Buffer holding the hex digit pairs (null terminated)    */
  /*---------------------------------------------------------------------------*/
static void hex2ascii(char* dest, char* src) {
    int i;

    /* Step through convering the source hex digit pairs */
    for (i = 0; src[i * 2] != '\0' && src[i * 2 + 1] != '\0'; i++) {
        dest[i] =
            ((hex(src[i * 2]) & 0xf) << 4) | (hex(src[i * 2 + 1]) &
                0xf);
    }

    dest[i] = '\0';

}				/* hex2ascii () */

/*---------------------------------------------------------------------------*/
/*!Set the program counter

  This sets the value in the NPC SPR. Not completely trivial, since this is
  actually cached in cpu_state.pc. Any reset of the NPC also involves
  clearing the delay state and setting the pcnext global.

  Only actually do this if the requested address is different to the current
  NPC (avoids clearing the delay pipe).

  @param[in] addr  The address to use                                       */
  /*---------------------------------------------------------------------------*/
static void set_npc(uint32_t addr, struct global_rsp* rspptr) {

    // First set the chain 
    gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */

    if (addr != get_npc(rspptr)) {

        gdb_write_reg(NPC_CPU_REG_ADD, addr);

        if (STALLED == rspptr->stallState) {
            if (g_show_verbose)
                printf
                ("set_npc(): New NPC value (0x%08x) written and locally cached \n",
                    addr);
            rspptr->npcCachedValue = addr;
            rspptr->npcIsCached = 1;
        } else {
            if (g_show_verbose)
                printf
                ("set_npc(): New NPC value (0x%08x) written \n",
                    addr);
            rspptr->npcIsCached = 0;
        }

    } else
        return;

}				/* set_npc () */

//! Read the value of the Next Program Counter (a SPR)

//! Setting the NPC flushes the pipeline, so subsequent reads will return
//! zero until the processor has refilled the pipeline. This will not be
//! happening if the processor is stalled (as it is when GDB had control),
//! so we must cache the NPC. As soon as the processor is unstalled, this
//! cached value becomes invalid.

//! If we are stalled and the value has been cached, use it. If we are stalled
//! and the value has not been cached, cache it (for efficiency) and use
//! it. Otherwise read the corresponding SPR.

//! @return  The value of the NPC
static uint32_t get_npc(struct global_rsp* rspptr) {
    uint32_t current_npc;

    if (STALLED == rspptr->stallState) {
        if (rspptr->npcIsCached == 0) {
            err = gdb_set_chain(SC_RISC_DEBUG);
            err = gdb_read_reg(NPC_CPU_REG_ADD, &(rspptr->npcCachedValue));
            if (err > 0) {
                printf("Error %d reading NPC\n", err);
                rsp_client_close(rspptr);
                return 0;
            }
            if (g_show_verbose)
                printf("get_npc(): caching newly read NPC value (0x%08x)\n", rspptr->npcCachedValue);
            rspptr->npcIsCached = 1;
        } else if (g_show_verbose) {
            printf("get_npc(): reading cached NPC value (0x%08x)\n", rspptr->npcCachedValue);
        }
        return rspptr->npcCachedValue;
    } else {
        err = gdb_read_reg(NPC_CPU_REG_ADD, &current_npc);
        if (err > 0) {
            printf("Error %d reading NPC\n", err);
            rsp_client_close(rspptr);
            return 0;
        }
        return current_npc;
    }
}				// get_npc ()

/*---------------------------------------------------------------------------*/
/*!Send a packet acknowledging an exception has occurred

  This is only called if there is a client FD to talk to                    */
  /*---------------------------------------------------------------------------*/
static void rsp_report_exception(struct global_rsp* rspptr) {
    struct rsp_buf buffer;

    /* Construct a signal received packet */
    buffer.data[0] = 'S';
    buffer.data[1] = hexchars[rspptr->sigval >> 4];
    buffer.data[2] = hexchars[rspptr->sigval % 16];
    buffer.data[3] = 0;
    buffer.len = strlen(buffer.data);
    put_packet(&buffer, rspptr);

    rspptr->nonstop_dsp_stall_status_reported = 1;
}				/* rsp_report_exception () */


/*---------------------------------------------------------------------------*/
static void rsp_step_continue(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    uint32_t addr;		/* Address to continue from, if any */

    // First set the chain 
    err = gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */

    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    if (err > 0) {
        printf
        ("Error %d to set RISC Debug Interface chain in the CONTINUE command 'c'\n",
            err);
        rsp_client_close(rspptr);
        return;
    }

    if (0 == strcmp("c", p_buf->data)) {
        // Arc Sim Code -->   addr = cpu_state.pc;        /* Default uses current NPC */
        /* ---------- NPC ---------- */
        addr = get_npc(rspptr);
    } else if (1 != sscanf(p_buf->data, "c%x", &addr)) {
        fprintf(stderr,
            "Warning: RSP continue address %s not recognized: ignored\n",
            p_buf->data);

        // Arc Sim Code -->   addr = cpu_state.pc;        /* Default uses current NPC */
        /* ---------- NPC ---------- */
        addr = get_npc(rspptr);
    }

    if (g_show_verbose)
        printf("rsp_step_continue() --> Read NPC = 0x%08x\n", addr);

    rsp_step_generic(addr, EXCEPT_NONE, rspptr);

    addr = get_npc(rspptr);
    while (NULL ==
        mp_hash_lookup(BP_MEMORY, addr, rspptr)) {
        rsp_step_generic(addr, EXCEPT_NONE, rspptr);
        addr = get_npc(rspptr);
    }
}

static void rsp_hardware_continue(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    // First set the chain 
    err = gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */

    if (gdb_write_reg(DRR_CPU_REG_ADD, 0))
        printf("Error write to DRR register\n");

    // clear potential ST in DMR1
    uint32_t dmr1_val = 0x0;
    if (gdb_read_reg(DMR1_CPU_REG_ADD, &dmr1_val))
        printf("Error read from DMR1 register\n");
    dmr1_val &= ~SPR_DMR1_ST;
    if (gdb_write_reg(DMR1_CPU_REG_ADD, dmr1_val))
        printf("Error write to DMR1 register\n");

    /* Set traps to be handled by the debug unit in the Debug Stop Register (DSR) Register 0x3014 */
    // Arc sim --> cpu_state.sprs[SPR_DSR]  |= SPR_DSR_TE | SPR_DRR_SCE;
    uint32_t temp_uint32;
    if (gdb_read_reg(DSR_CPU_REG_ADD, &temp_uint32))
        printf("Error read from DSR register\n");
    temp_uint32 |= SPR_DSR_TE;
    temp_uint32 |= SPR_DRR_SCE;
    if (gdb_write_reg(DSR_CPU_REG_ADD, temp_uint32))
        printf("Error write to DSR register\n");
    
    if (g_show_verbose) {
        uint32_t dmr1_val = 0x0;
        if (gdb_read_reg(DMR1_CPU_REG_ADD, &dmr1_val))
            printf("Error read from DMR1 register\n");
        printf("DMR1_CPU_REG = 0x%08x\n", dmr1_val);

        uint32_t dmr2_val = 0x0;
        if (gdb_read_reg(DMR2_CPU_REG_ADD, &dmr2_val))
            printf("Error read from DMR2 register\n");
        printf("DMR2_CPU_REG = 0x%08x\n", dmr2_val);

        uint32_t dsr_val = 0x0;
        if (gdb_read_reg(DSR_CPU_REG_ADD, &dsr_val))
            printf("Error read from DSR register\n");
        printf("DSR_CPU_REG = 0x%08x\n", dsr_val);
    }

    gdb_write_reg(0x0, 0x0);//zsy clear address reg in chip

    /* Unstall the processor */
    set_stall_state(0, rspptr);
    unsigned char stalled;   //annotated by wangwk 17-05-22
    dbg_cpu0_read_ctrl(0, &stalled);  // annotated by wangwk 17-05-22

    /* Note the GDB client is now waiting for a reply. */
    rspptr->client_waiting = 1;   //annotated by wangwk 17-05-22

    if (g_show_verbose) {
        printf("After unstall the processor, stalled = %d\n", stalled);
        printf("waiting = %d\n", rspptr->client_waiting);
        printf("rsp_hardware_continue() done\n");
        fflush(stdout);
    }
}

/*---------------------------------------------------------------------------*/
/*!Handle a RSP continue request

  Parse the command to see if there is an address. Uses the underlying
  generic continue function, with EXCEPT_NONE.

  @param[in] p_buf  The full continue packet                                  */
  /*---------------------------------------------------------------------------*/
static void rsp_continue(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    uint32_t addr;		/* Address to continue from, if any */

    // First set the chain 
    err = gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */

    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    if (err > 0) {
        printf
        ("Error %d to set RISC Debug Interface chain in the CONTINUE command 'c'\n",
            err);
        rsp_client_close(rspptr);
        return;
    }

    if (0 == strcmp("c", p_buf->data)) {
        // Arc Sim Code -->   addr = cpu_state.pc;        /* Default uses current NPC */
        /* ---------- NPC ---------- */
        addr = get_npc(rspptr);
    } else if (1 != sscanf(p_buf->data, "c%x", &addr)) {
        fprintf(stderr,
            "Warning: RSP continue address %s not recognized: ignored\n",
            p_buf->data);

        // Arc Sim Code -->   addr = cpu_state.pc;        /* Default uses current NPC */
        /* ---------- NPC ---------- */
        addr = get_npc(rspptr);
    }

    if (g_show_verbose)
        printf("rsp_continue() --> Read NPC = 0x%08x\n", addr);

    rsp_continue_generic(addr, EXCEPT_NONE, rspptr);
}				/* rsp_continue () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP continue with signal request

  Currently null. Will use the underlying generic continue function.

  @param[in] p_buf  The full continue with signal packet                      */
  /*---------------------------------------------------------------------------*/
static void rsp_continue_with_signal(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    printf("RSP continue with signal '%s' received, signal ignored\n",
        p_buf->data);
    rsp_continue(p_buf, rspptr);

}				/* rsp_continue_with_signal () */

/*---------------------------------------------------------------------------*/
/*!Generic processing of a continue request

  The signal may be EXCEPT_NONE if there is no exception to be
  handled. Currently the exception is ignored.

  The single step flag is cleared in the debug registers and then the
  processor is unstalled.

  @param[in] addr    Address from which to step
  @param[in] except  The exception to use (if any)                          */
  /*---------------------------------------------------------------------------*/
static void rsp_continue_generic(uint32_t addr, uint32_t except, struct global_rsp* rspptr) {
    //	printf("rsp_continue_generic () begin \n");
    uint32_t temp_uint32;
    /* Set the address as the value of the next program counter */
    set_npc(addr, rspptr);
    //	printf("set_npc() addr = %x\n", addr);
        /* Clear Debug Reason Register (DRR) 0x3015 */
        // Arc sim --> cpu_state.sprs[SPR_DRR]   = 0;
    if (gdb_write_reg(DRR_CPU_REG_ADD, 0))
        printf("Error write to DRR register\n");
    //	printf("gdb_write_reg(DRR_CPU_REG_ADD, 0)");

        /* Clear watchpoint break generation in Debug Mode Register 2 (DMR2) 0x3011 */
        // Arc sim --> cpu_state.sprs[SPR_DMR2] &= ~SPR_DMR2_WGB;
    if (gdb_read_reg(DMR2_CPU_REG_ADD, &temp_uint32))
        printf("Error read from DMR2 register\n");
    temp_uint32 &= ~SPR_DMR2_WGB;
    if (gdb_write_reg(DMR2_CPU_REG_ADD, temp_uint32))
        printf("Error write to DMR2 register\n");
    //	printf("rsp_continue_generic () middle");
        /* Clear the single step trigger in Debug Mode Register 1 (DMR1) Register 0x3010 */
        // Arc sim --> cpu_state.sprs[SPR_DMR1] &= ~SPR_DMR1_ST;
    if (gdb_read_reg(DMR1_CPU_REG_ADD, &temp_uint32))
        printf("Error read from DMR1 register\n");
    temp_uint32 &= ~SPR_DMR1_ST;
    //	printf("temp_uint32 = %d", temp_uint32);
    if (gdb_write_reg(DMR1_CPU_REG_ADD, temp_uint32))
        printf("Error write to DMR1 register\n");

    /* Set traps to be handled by the debug unit in the Debug Stop Register (DSR) Register 0x3014 */
    // Arc sim --> cpu_state.sprs[SPR_DSR]  |= SPR_DSR_TE;
    if (gdb_read_reg(DSR_CPU_REG_ADD, &temp_uint32))
        printf("Error read from DSR register\n");
    temp_uint32 |= SPR_DSR_TE;
    temp_uint32 |= SPR_DRR_SCE;
    if (gdb_write_reg(DSR_CPU_REG_ADD, temp_uint32))
        printf("Error write to DSR register\n");

    /* Unstall the processor */
    set_stall_state(0, rspptr);

    /* Note the GDB client is now waiting for a reply. */
    rspptr->client_waiting = 1;
    printf("rsp_continue_generic () done");

}				/* rsp_continue_generic () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP read all registers request

  The registers follow the GDB sequence for OR1K: GPR0 through GPR31, PPC
  (i.e. SPR PPC), NPC (i.e. SPR NPC) and SR (i.e. SPR SR). Each register is
  returned as a sequence of bytes in target endian order.

  Each byte is packed as a pair of hex digits.                              */
  /*---------------------------------------------------------------------------*/
static void rsp_read_all_regs(struct global_rsp* rspptr) {
    struct rsp_buf buffer;	/* Buffer for the reply */
    int r;			/* Register index */
    uint32_t temp_uint32;

    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    // First set the chain 
    gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */

    // To speed up resuming chip when handling k packet
    {
        // clear potential ST in DMR1, set by s packet
        uint32_t dmr1_val = 0x0;
        gdb_read_reg(DMR1_CPU_REG_ADD, &dmr1_val);
        dmr1_val &= ~SPR_DMR1_ST;
        gdb_write_reg(DMR1_CPU_REG_ADD, dmr1_val);
    }

    uint32_t address;
    // Read all GPRs
    for (r = 0; r < MAX_GPRS_all; r++) {
        if (r < GR_NUM)//读GR寄存器
        {
            address = GR_START + r * 4;
            //if(r==29)//为了将29号赋值为30号寄存器，在sp正确后不再需要
            //	address+=4;

            if (DEBUG_GDB_DUMP_DATA)
                printf("address=0x%x,r=%d\n", address, r);
            err = gdb_read_reg(address, &temp_uint32);
            if (err > 0) {
                if (g_show_verbose)
                    printf("Error %d in gdb_read_reg at reg. %d\n",
                        err, r);
                put_str_packet("E01", rspptr);
                return;
            }
        }
        // else if(r - GR_NUM < VR_NUM * (VR_LENGTH/32))//读VR寄存器
        // {
        // 	uint32_t VR_count,VR_count2;
        // 	VR_count=(r-GR_NUM)/(VR_LENGTH/32);//读的是第几个VR,从0开始
        // 	VR_count2 = (r-GR_NUM)%(VR_LENGTH/32);//读的是VR里的第几个32位,从0开始
        // 	//address=VR_START + VR_LENGTH*VR_count + (r-GR_NUM)*4;
        // 	address = VR_CPU_REG_ADD(VR_count) + VR_count2*4;

        // 	if(DEBUG_GDB_DUMP_DATA)
        // 		printf("address=0x%x,r=%d,VR_count=%d,VR_count2=%d\n",address,r,VR_count,VR_count2);
        // 	if(VR_count2<40)
        // 		err = gdb_read_reg(address, &temp_uint32); 
        // 		//temp_uint32=0;
        // 	else
        // 		temp_uint32=0;
        // 	if (err > 0) {
        // 		if (g_show_verbose)
        // 			printf("Error %d in gdb_read_reg at reg. %d\n",
        // 				err, r);
        // 		put_str_packet("E01",rspptr);
        // 		return;
        // 	}
        // }
        else//读MOB类寄存器
        {
            //address=MOB_START + (r - GR_NUM - VR_NUM * (VR_LENGTH/32))*4;
            address = PPC_CPU_REG_ADD;

            if (DEBUG_GDB_DUMP_DATA)
                printf("address = 0x%x, r = %d\n", address, r);
            err = gdb_read_reg(address, &temp_uint32);
            
            if (err > 0) {
                if (g_show_verbose)
                    printf("Error %d in gdb_read_reg at reg. %d\n",
                        err, r);
                put_str_packet("E01", rspptr);
                return;
            }
        }
        reg2hex(temp_uint32, &(buffer.data[r * 8]));
    }

    /* ---------- NPC ---------- */
    temp_uint32 = get_npc(rspptr);

    /* ---------- PPC ---------- */
    err = gdb_read_reg(PPC_CPU_REG_ADD, &temp_uint32);//zsy use expc instead of ppc
    if (err > 0) {
        if (g_show_verbose)
            printf("Error %d in gdb_read_reg read --> PPC\n", err);
        put_str_packet("E01", rspptr);
        return;
    }

    /* ---------- EXPC ---------- */
    err = gdb_read_reg(EXPC_CPU_REG_ADD, &temp_uint32);
    if (err > 0) {
        if (g_show_verbose)
            printf("Error %d in gdb_read_reg read --> EXPC\n", err);
        put_str_packet("E01", rspptr);
        return;
    }

    /* ---------- SR ---------- */
    err = gdb_read_reg(SR_CPU_REG_ADD, &temp_uint32);
    if (err > 0) {
        if (g_show_verbose)
            printf("Error %d in gdb_read_reg read --> SP\n", err);
        put_str_packet("E01", rspptr);
        return;
    }
    if (DEBUG_GDB_DUMP_DATA)
        printf("SR = 0x%08x\n", temp_uint32);

    /* Finalize the packet and send it */
    buffer.data[MAX_GPRS_all * 8] = 0;
    buffer.len = MAX_GPRS_all * 8;
    put_packet(&buffer, rspptr);

    return;
}				/* rsp_read_all_regs () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP write all registers request

  The registers follow the GDB sequence for OR1K: GPR0 through GPR31, PPC
  (i.e. SPR PPC), NPC (i.e. SPR NPC) and SR (i.e. SPR SR). Each register is
  supplied as a sequence of bytes in target endian order.

  Each byte is packed as a pair of hex digits.

  @todo There is no error checking at present. Non-hex chars will generate a
  warning message, but there is no other check that the right amount
  of data is present. The result is always "OK".

  @param[in] p_buf  The original packet request.                              */
  /*---------------------------------------------------------------------------*/
static void rsp_write_all_regs(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    uint32_t regnum;	/* Register index */
    // char          valstr[9];           /* Allow for EOS on the string */

    // /* Check for valid data */
    // if (0 != (strcmp ("G", p_buf->data)) && (GDB_BUF_MAX != strlen(p_buf->data)))
    // {
    //   fprintf (stderr, "Warning: Failed to recognize RSP write register "
    //      "command: %s\n", p_buf->data);
    //   // put_str_packet ("E01",rspptr);
    //   return;
    // }

    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    // First set the chain 
    err = gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */
    if (err > 0) {
        if (g_show_verbose)
            printf("Error %d in gdb_set_chain\n", err);
        put_str_packet("E01", rspptr);
        return;
    }

    /* ---------- GPRS ---------- */
    // for (regnum = 0; regnum < GR_NUM; regnum++) {
    // 	err =
    // 	    gdb_write_reg(0x007f0e00 + regnum * 4,  //0x400
    // 			  hex2reg(&p_buf->data[regnum * 8 + 1]));
    // 	if (err > 0) {
    // 		if (g_show_verbose)
    // 			printf
    // 			    ("Error %d in rsp_write_reg write --> GPRS\n",
    // 			     err);
    // 		put_str_packet("E01",rspptr);
    // 		return;
    // 	}
    // }

    /* ---------- PPC ---------- */
    err =
        gdb_write_reg(PPC_CPU_REG_ADD,
            hex2reg(&p_buf->data[PPC_REGNUM * 8 + 1]));
    if (err > 0) {
        if (g_show_verbose)
            printf("Error %d in rsp_write_reg write --> PPC\n",
                err);
        put_str_packet("E01", rspptr);
        return;
    }
    /* ---------- SR ---------- */
    // err =
    //     gdb_write_reg(SR_CPU_REG_ADD,
    // 		  hex2reg(&p_buf->data[SR_REGNUM * 8 + 1]));
    // if (err > 0) {
    // 	if (g_show_verbose)
    // 		printf("Error %d in rsp_write_reg write --> SR\n", err);
    // 	put_str_packet("E01",rspptr);
    // 	return;
    // }
    /* ---------- NPC ---------- */
    set_npc(hex2reg(&p_buf->data[NPC_REGNUM * 8 + 1]), rspptr);
    /*
       err = gdb_write_reg(NPC_CPU_REG_ADD, hex2reg (&p_buf->data[NPC_REGNUM * 8 + 1]));
       if(err > 0){
       if (g_show_verbose) printf("Error %d in rsp_write_reg write --> NPC\n", err);
       put_str_packet ("E01",rspptr);
       return;
       }
     */
     /* Acknowledge. TODO: We always succeed at present, even if the data was
        defective. */
    put_str_packet("OK", rspptr);
}				/* rsp_write_all_regs () */

static void switchWordEndianness(uint32_t* pWord) {
    uint32_t inByte0 = ((*pWord) >>  0) & 0xff;
    uint32_t inByte1 = ((*pWord) >>  8) & 0xff;
    uint32_t inByte2 = ((*pWord) >> 16) & 0xff;
    uint32_t inByte3 = ((*pWord) >> 24) & 0xff;
    uint32_t switchedWord =   inByte3 << 0
                            | inByte2 << 8
                            | inByte1 << 16
                            | inByte0 << 24;
    *pWord = switchedWord;
}

static uint32_t getMemoryReadAddress(uint32_t addr) {
    uint32_t readAddr = addr;
    /* In C2000 DSP, it seems that the instructions in IRAM are organized by 128-bit groups.
     * Each group is 16-byte aligned and contains 4 C2000 instructions. And the massping of
     * reading address and actual IRAM address in one 128-bit group is as follows, which is
     * the reverse order.
     * read address  -->  IRAM address
     *    0x0               0xc
     *    0x4               0x8
     *    0x8               0x4
     *    0xc               0x0
     */
    if (address_in_bootram_iram(addr)) {
        if (addr % 16 == 0) {
            readAddr = (addr / 16) * 16 + 0xc;
        } else if (addr % 16 == 4) {
            readAddr = (addr / 16) * 16 + 0x8;
        } else if (addr % 16 == 8) {
            readAddr = (addr / 16) * 16 + 0x4;
        } else if (addr % 16 == 0xc) {
            readAddr = (addr / 16) * 16 + 0x0;
        } else {
            assert(("Address must be 4-byte aligned" ,false));
        }
    }
    return readAddr;
}

static int isSPRSAddress(uint32_t address) {
    return (address >= 0x7F0000 && address <= 0x7FFFFF);
}

/*---------------------------------------------------------------------------*/
/* Handle a RSP read memory (symbolic) request

   Syntax is:

   m<addr>,<length>:

   The response is the bytes, lowest address first, encoded as pairs of hex
   digits.

   The length given is the number of bytes to be read.

   @note This function reuses p_buf, so trashes the original command.

   @param[in] p_buf  The command received                                      */
   /*---------------------------------------------------------------------------*/
static void rsp_read_mem(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    unsigned int addr;	/* Where to read the memory */
    int len;		/* Number of bytes to read */
    int off;		/* Offset into the memory */
    uint32_t temp_uint32 = 0;
    char* rec_buf, * rec_buf_ptr;
    /* Current OR implementation is 4-byte words */
    int i;
    int len_cpy;
    /* Couple of temps we might need when doing aligning/leftover accesses */
    uint32_t tmp_word;
    uint8_t tmp_byte;
    char* tmp_word_ptr = (char*)&tmp_word;

    if (2 != sscanf(p_buf->data, "m%x,%x:", &addr, &len)) {
        fprintf(stderr, "Warning: Failed to recognize RSP read memory "
            "command: %s\n", p_buf->data);
        put_str_packet("E01", rspptr);
        return;
    }
    if (g_show_verbose)
        printf("m read %d bytes from 0x%08x\n", len, addr);

    /* Make sure we won't overflow the buffer (2 chars per byte) */
    if ((len * 2) >= GDB_BUF_MAX_TIMES_TWO) {
        fprintf(stderr,
            "Warning: Memory read %s too large for RSP packet: "
            "truncated\n", p_buf->data);
        len = (GDB_BUF_MAX - 1) / 2;
    }

    if (!(rec_buf = (char*)malloc(len))) {
        put_str_packet("E01", rspptr);
        return;
    }
    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    // Set chain 5 --> Wishbone Memory chain
    int isReadSPRS = isSPRSAddress(addr) || isSPRSAddress(addr+len-1);
    enum jtag_chains jchain = isReadSPRS ? SC_RISC_DEBUG : SC_WISHBONE;
    err = gdb_set_chain(jchain);
    if (err) {
        if (g_show_verbose)
            printf("Error %d in gdb_set_chain\n", err);
        put_str_packet("E01", rspptr);
        return;
    }

    len_cpy = len;
    rec_buf_ptr = rec_buf;	// Need to save a copy of pointer

    char word_tem[5] = { '\0' };

    // address not word-aligned, do byte accesses first
    if (addr & 0x3) {
        int num_bytes_to_align = bytes_per_word - (addr & 0x3);
        //printf("num_bytes_to_align:%d\n", num_bytes_to_align);
        int bytes_to_read = (num_bytes_to_align >= len_cpy) ?
            len_cpy : num_bytes_to_align;

        unsigned int addr_tem;
        addr_tem = addr;
        addr = addr - (addr & 0x3);
        err = gdb_read_block(getMemoryReadAddress(addr), (uint32_t*)word_tem, bytes_per_word);
        if (err) {
            put_str_packet("E01", rspptr);
            return;
        }
        if (!isReadSPRS) switchWordEndianness((uint32_t*)word_tem);
        for (int i = 0; i < bytes_to_read; i++) {
            rec_buf[i] = word_tem[(addr_tem & 0x3) + i];
        }

        // Adjust our status
        len_cpy -= bytes_to_read;
        rec_buf_ptr += num_bytes_to_align;
        addr += bytes_per_word;
    }

    // Now perform all full word accesses
    if (len_cpy / bytes_per_word) {
        int words_to_read = len_cpy / bytes_per_word;	// Full words to read
        if (g_show_verbose)
            printf("Full words to read\nrsp_read_mem: reading %d words from 0x%.8x\n",
                words_to_read, addr);
        // Read full data words from Wishbone Memory chain
        for (int i = 0; i < words_to_read; i++) {
            err = gdb_read_block(getMemoryReadAddress(addr), (uint32_t*)rec_buf_ptr, bytes_per_word);
            if (err) {
                put_str_packet("E01", rspptr);
                return;
            }
            if (!isReadSPRS) switchWordEndianness((uint32_t*)rec_buf_ptr);

            addr += bytes_per_word;
            rec_buf_ptr += bytes_per_word;
        }
        len_cpy -= (words_to_read * bytes_per_word);
    }

    // Leftover bytes
    if (len_cpy) {
        //printf("leftover bytes:%d\n", len_cpy);
        err = gdb_read_block(getMemoryReadAddress(addr), (uint32_t*)word_tem, bytes_per_word);
        if (err) {
            put_str_packet("E01", rspptr);
            return;
        }
        if (!isReadSPRS) switchWordEndianness((uint32_t*)word_tem);
        for (int i = 0; i < len_cpy; i++) {
            rec_buf_ptr[i] = word_tem[i];
        }

        addr += len_cpy;
        len_cpy -= len_cpy;
        rec_buf_ptr += len_cpy;

    }

    /* Refill the buffer with the reply */
    for (off = 0; off < len; off++) {
        p_buf->data[(2 * off)] = hexchars[((rec_buf[off] & 0xf0) >> 4)];
        p_buf->data[(2 * off) + 1] = hexchars[(rec_buf[off] & 0x0f)];
    }

    if (g_show_verbose && (err > 0))
        printf("\nError %x\n", err);
    fflush(stdout);
    free(rec_buf);

    p_buf->data[off * 2] = 0;	/* End of string */
    p_buf->len = strlen(p_buf->data);

    if (DEBUG_GDB_BLOCK_DATA) {
        printf("rsp_read_mem: end_adr 0x%.8x data:", addr);
        for (i = 0; i < len * 2; i++)
            printf("str[%d]=%c", i, p_buf->data[i]);
        printf("end\n");
    }

    put_packet(p_buf, rspptr);
}				/* rsp_read_mem () */


static void rsp_read_mem_seprate(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    unsigned int addr;	/* Where to read the memory */
    int len;		/* Number of bytes to read */
    int off;		/* Offset into the memory */
    uint32_t temp_uint32 = 0;
    char* rec_buf, * rec_buf_ptr, p_buf_data[8];
    //int bytes_per_word = 4;	/* Current OR implementation is 4-byte words */
    int i;
    int len_cpy;
    /* Couple of temps we might need when doing aligning/leftover accesses */
    uint32_t tmp_word;
    uint8_t tmp_byte;
    char* tmp_word_ptr = (char*)&tmp_word;

    if (2 != sscanf(p_buf->data, "m%x,%x:", &addr, &len)) {
        fprintf(stderr, "Warning: Failed to recognize RSP read memory "
            "command: %s\n", p_buf->data);
        put_str_packet("E01", rspptr);
        return;
    }
    if (g_show_verbose)
        printf("m read %d byte from 0x%x\n", len, addr);
    if (len == 0x40)
        addr += 0xfffe0;
    int separate_times = len / 4;
    int data_index = 0;
    for (int separate_times_i = 0; separate_times_i < separate_times; separate_times_i++) {
        len = 4;//171221
        /* Make sure we won't overflow the buffer (2 chars per byte) */
        if ((len * 2) >= GDB_BUF_MAX_TIMES_TWO) {
            fprintf(stderr,
                "Warning: Memory read %s too large for RSP packet: "
                "truncated\n", p_buf->data);
            len = (GDB_BUF_MAX - 1) / 2;
        }

        if (!(rec_buf = (char*)malloc(len))) {
            put_str_packet("E01", rspptr);
            return;
        }
        // Make sure the processor is stalled
        gdb_ensure_or1k_stalled();

        // Set chain 5 --> Wishbone Memory chain
        err = gdb_set_chain(SC_WISHBONE);
        if (err) {
            if (g_show_verbose)
                printf("Error %d in gdb_set_chain\n", err);
            put_str_packet("E01", rspptr);
            return;
        }

        if (addr >= 0x100000) {
            addr += 0x400;
            if (addr % 16 == 0)
                addr = (addr / 16) * 16 + 0xc;
            else if (addr % 16 == 4)
                addr = (addr / 16) * 16 + 0x8;
            else if (addr % 16 == 8)
                addr = (addr / 16) * 16 + 0x4;
            else if (addr % 16 == 0xc)
                addr = (addr / 16) * 16 + 0x0;
        }
        len_cpy = len;
        rec_buf_ptr = rec_buf;	// Need to save a copy of pointer

        if (addr & 0x3)		// address not word-aligned, do byte accesses first
        {

            int num_bytes_to_align = bytes_per_word - (addr & 0x3);

            int bytes_to_read = (num_bytes_to_align >= len_cpy) ?
                len_cpy : num_bytes_to_align;

            for (i = 0; i < bytes_to_read; i++) {
                err = gdb_read_byte(addr++, (uint8_t*)&rec_buf[i]);
                if (err) {
                    put_str_packet("E01", rspptr);
                    return;
                }
            }

            // Adjust our status
            len_cpy -= bytes_to_read;
            rec_buf_ptr += num_bytes_to_align;
        }

        if (len_cpy / bytes_per_word)	// Now perform all full word accesses
        {
            int words_to_read = len_cpy / bytes_per_word;	// Full words to read
            if (g_show_verbose)
                printf("rsp_read_mem: reading %d words from 0x%.8x\n",
                    words_to_read, addr);
            // Read full data words from Wishbone Memory chain
            err = gdb_read_block(addr, (uint32_t*)rec_buf_ptr,
                words_to_read * bytes_per_word);

            if (err) {
                put_str_packet("E01", rspptr);
                return;
            }
            // Adjust our status
            len_cpy -= (words_to_read * bytes_per_word);
            addr += (words_to_read * bytes_per_word);
            rec_buf_ptr += (words_to_read * bytes_per_word);
        }
        if (len_cpy)		// Leftover bytes
        {
            for (i = 0; i < len_cpy; i++) {
                err =
                    gdb_read_byte(addr++, (uint8_t*)&rec_buf_ptr[i]);
                if (err) {
                    put_str_packet("E01", rspptr);
                    return;
                }
            }

        }

        /* Refill the buffer with the reply */
        for (off = 0; off < len; off++) {
            ;
            p_buf_data[(2 * off)] = hexchars[((rec_buf[off] & 0xf0) >> 4)];
            p_buf_data[(2 * off) + 1] = hexchars[(rec_buf[off] & 0x0f)];
        }

        char disassemble[2];
        int j;
        for (j = 0; j < 2; j++) {
            disassemble[0] = p_buf_data[j * 2];
            disassemble[1] = p_buf_data[j * 2 + 1];
            p_buf_data[j * 2] = p_buf_data[(3 - j) * 2];
            p_buf_data[j * 2 + 1] = p_buf_data[(3 - j) * 2 + 1];
            p_buf_data[(3 - j) * 2] = disassemble[0];
            p_buf_data[(3 - j) * 2 + 1] = disassemble[1];
        }



        if (g_show_verbose && (err > 0))
            printf("\nError %x\n", err);
        fflush(stdout);
        free(rec_buf);
        //				p_buf->data[off * 2] = 0;	/* End of string */
        //				p_buf->len = strlen(p_buf->data);
        if (DEBUG_GDB_BLOCK_DATA) {
            printf("rsp_read_mem: adr 0x%.8x data: ", addr);
            for (i = 0; i < len * 2; i++)
                printf("%c", p_buf_data[i]);
            printf("\n");
        }
        for (int copy_i = 0; copy_i < 8; copy_i++)
            p_buf->data[data_index++] = p_buf_data[copy_i];

    }
    for (int final_i = 0; final_i < data_index; final_i++) {
        printf("%c", p_buf->data[final_i]);
    }
    printf("\n");
    p_buf->data[data_index] = 0;
    p_buf->len = strlen(p_buf->data);
    put_packet(p_buf, rspptr);

}				/* rsp_read_mem () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP write memory (symbolic) request	 ("M")

  Syntax is:

  M<addr>,<length>:<data>

  Example: M4015cc,2:c320#
  (Write the value 0xc320 to address 0x4015cc.)

  An example target response:
  + $OK#

  The data is the bytes, lowest address first, encoded as pairs of hex
  digits.

  The length given is the number of bytes to be written.

  @note This function reuses p_buf, so trashes the original command.

  @param[in] p_buf  The command received                                      */
  /*---------------------------------------------------------------------------*/
static void rsp_write_mem(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    unsigned int addr;	/* Where to write the memory */
    int len;		/* Number of bytes to write */
    char* symdat;		/* Pointer to the symboli data */
    int datlen;		/* Number of digits in symbolic data */
    int off;		/* Offset into the memory */
    int nibc;		/* Nibbel counter */
    uint32_t val;

    if (2 != sscanf(p_buf->data, "M%x,%x:", &addr, &len)) {
        fprintf(stderr, "Warning: Failed to recognize RSP write memory "
            "command: %s\n", p_buf->data);
        put_str_packet("E01", rspptr);
        return;
    }

    /* Find the start of the data and check there is the amount we expect. */
    symdat =
        (char*)memchr((const void*)p_buf->data, ':', GDB_BUF_MAX) + 1;
    datlen = p_buf->len - (symdat - p_buf->data);

    /* Sanity check */
    if (len * 2 != datlen) {
        fprintf(stderr,
            "Warning: Write of %d digits requested, but %d digits "
            "supplied: packet ignored\n", len * 2, datlen);
        put_str_packet("E01", rspptr);
        return;
    }
    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    // Set chain 5 --> Wishbone Memory chain
    err = gdb_set_chain(SC_WISHBONE);
    if (err) {
        put_str_packet("E01", rspptr);
        return;
    }

    val = 0;
    off = 0;
    //	addr += 0x00100000 - 0x100; //added by wangwk 17-05-09
        /* Write the bytes to memory */
    for (nibc = 0; nibc < datlen; nibc++) {
        val |= 0x0000000f & hex(symdat[nibc]);
        if (nibc % 8 == 7) {
            err = gdb_write_block(addr + off, &val, 4);
            if (g_show_verbose)
                printf("Error %x\n", err);
            fflush(stdout);
            if (err) {
                put_str_packet("E01", rspptr);
                return;
            }
            val = 0;
            off += 4;
        }
        val <<= 4;
    }
    put_str_packet("OK", rspptr);
}				/* rsp_write_mem () */

/*---------------------------------------------------------------------------*/
/*!Read a single register

  The registers follow the GDB sequence for OR1K: GPR0 through GPR31, PC
  (i.e. SPR NPC) and SR (i.e. SPR SR). The register is returned as a
  sequence of bytes in target endian order.

  Each byte is packed as a pair of hex digits.

  @param[in] p_buf  The original packet request. Reused for the reply.        */
  /*---------------------------------------------------------------------------*/
#if 0
static void rsp_read_reg(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    unsigned int regnum;
    uint32_t temp_uint32;
    printf("func rsp_read_reg:p_buf->data=%s", p_buf->data);
    /* Break out the fields from the data */
    if (1 != sscanf(p_buf->data, "p%x", &regnum)) {
        fprintf(stderr,
            "Warning: Failed to recognize RSP read register "
            "command: %s\n", p_buf->data);
        put_str_packet("E01", rspptr);
        return;
    }
    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    // First set the chain 
    err = gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */
    if (err > 0) {
        if (g_show_verbose)
            printf("Error %d in gdb_set_chain\n", err);
        put_str_packet("E01", rspptr);
        return;
    }

    /* Get the relevant register */
    if (regnum < GR_NUM) {
        err = gdb_read_reg(0x007f0e00 + regnum * 4, &temp_uint32);  //0x400
        if (err > 0) {
            if (g_show_verbose)
                printf("Error %d in rsp_read_reg at reg. %d \n",
                    err, regnum);
            put_str_packet("E01", rspptr);
            return;
        }
        printf("%d\n", temp_uint32);
        reg2hex(temp_uint32, p_buf->data);
    } else if (regnum < MAX_GPRS) {
        int32_t VR_count, VR_count2;
        uint32_t address;
        VR_count = (regnum - GR_NUM) / (VR_LENGTH / 32);//读的是第几个VR,从0开始
        VR_count2 = (regnum - GR_NUM) % (VR_LENGTH / 32);//读的是VR里的第几个32位,从0开始
        //address=VR_START + VR_LENGTH*VR_count + (r-GR_NUM)*4;
        address = VR_CPU_REG_ADD(VR_count) + VR_count2 * 4;
        err = gdb_read_reg(address, &temp_uint32);
        if (err > 0) {
            if (g_show_verbose)
                printf("Error %d in gdb_read_reg at reg. %d\n",
                    err, regnum);
            put_str_packet("E01", rspptr);
            return;
        }
        printf("%d\n", temp_uint32);
        reg2hex(temp_uint32, p_buf->data);
        // } else if (PPC_REGNUM == regnum) {	/* ---------- PPC ---------- */
        // 	err = gdb_read_reg(PPC_CPU_REG_ADD, &temp_uint32);
        // 	if (err > 0) {
        // 		if (g_show_verbose)
        // 			printf
        // 			    ("Error %d in rsp_read_reg read --> PPC\n",
        // 			     err);
        // 		put_str_packet("E01",rspptr);
        // 		return;
        // 	}
        
        // 	reg2hex(temp_uint32, p_buf->data);
        // } else if (NPC_REGNUM == regnum) {	/* ---------- NPC ---------- */
        // 	temp_uint32 = get_npc(rspptr);
        // 	/*
        // 	   err = gdb_read_reg(NPC_CPU_REG_ADD, &temp_uint32);
        // 	   if(err > 0){
        // 	   if (g_show_verbose) printf("Error %d in rsp_read_reg read --> PPC\n", err);
        // 	   put_str_packet ("E01",rspptr);
        // 	   return;
        // 	   }
        // 	 */
        // 	reg2hex(temp_uint32, p_buf->data);
        // } else if (SR_REGNUM == regnum) {	/* ---------- SR ---------- */
        // 	err = gdb_read_reg(SR_CPU_REG_ADD, &temp_uint32);
        // 	if (err > 0) {
        // 		if (g_show_verbose)
        // 			printf
        // 			    ("Error %d in rsp_read_reg read --> PPC\n",
        // 			     err);
        // 		put_str_packet("E01",rspptr);
        // 		return;
        // 	}
        // 	reg2hex(temp_uint32, p_buf->data);	
    } else if (regnum < NUM_REGS) {
        //temp_uint32 = get_npc(rspptr);
        gdb_read_reg(PPC_CPU_REG_ADD, &temp_uint32);
        temp_uint32 += 0x00000000;//地址
        reg2hex(temp_uint32, p_buf->data);
    } else {
        /* Error response if we don't know the register */
        fprintf(stderr,
            "Warning: Attempt to read unknown register 0x%x: "
            "ignored\n", regnum);
        put_str_packet("E01", rspptr);
        return;
    }

    p_buf->len = strlen(p_buf->data);
    put_packet(p_buf, rspptr);

}				/* rsp_read_reg () */
#endif

/*---------------------------------------------------------------------------*/
/*!Write a single register

  The registers follow the GDB sequence for OR1K: GPR0 through GPR31, PC
  (i.e. SPR NPC) and SR (i.e. SPR SR). The register is specified as a
  sequence of bytes in target endian order.

  Each byte is packed as a pair of hex digits.

  @param[in] p_buf  The original packet request.                              */
  /*---------------------------------------------------------------------------*/
#if 0
static void rsp_write_reg(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    unsigned int regnum;
    char valstr[9];		/* Allow for EOS on the string */
    // int           err = 0;
//printf("func rsp_write_reg:p_buf->data=%s",p_buf->data);
    /* Break out the fields from the data */
    if (2 != sscanf(p_buf->data, "P%x=%8s", &regnum, valstr)) {
        fprintf(stderr,
            "Warning: Failed to recognize RSP write register "
            "command: %s\n", p_buf->data);
        put_str_packet("E01", rspptr);
        return;
    }
    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    // First set the chain 
    err = gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */
    if (err > 0) {
        if (g_show_verbose)
            printf("Error %d in gdb_set_chain\n", err);
        put_str_packet("E01", rspptr);
        return;
    }

    /* Set the relevant register */
    if (regnum < GR_NUM) {	/* ---------- GPRS ---------- */
        err = gdb_write_reg(0x007f0e00 + regnum * 4, hex2reg(valstr));  //0x400
        if (err > 0) {
            if (g_show_verbose)
                printf
                ("Error %d in rsp_write_reg write --> GPRS\n",
                    err);
            put_str_packet("E01", rspptr);
            return;
        }
    } else if (regnum < MAX_GPRS) {
        int32_t VR_count, VR_count2;
        uint32_t address, temp_uint32;
        VR_count = (regnum - GR_NUM) / (VR_LENGTH / 32);//读的是第几个VR,从0开始
        VR_count2 = (regnum - GR_NUM) % (VR_LENGTH / 32);//读的是VR里的第几个32位,从0开始
        //address=VR_START + VR_LENGTH*VR_count + (r-GR_NUM)*4;
        address = VR_CPU_REG_ADD(VR_count) + VR_count2 * 4;
        err = gdb_write_reg(address, hex2reg(valstr));
        if (err > 0) {
            if (g_show_verbose)
                printf("Error %d in gdb_read_reg at reg. %d\n",
                    err, regnum);
            put_str_packet("E01", rspptr);
            return;
        }
        printf("%d\n", temp_uint32);
        reg2hex(temp_uint32, p_buf->data);
    }
    // 	 else if (PPC_REGNUM == regnum) {	/* ---------- PPC ---------- */
    // /*
    // 		err = gdb_write_reg(PPC_CPU_REG_ADD, hex2reg(valstr));
    // 		if (err > 0) {
    // 			if (g_show_verbose)
    // 				printf
    // 				    ("Error %d in rsp_write_reg write --> PPC\n",
    // 				     err);
    // 			put_str_packet("E01",rspptr);
    // 			return;
    // 		}
    // */
    // 	} else if (NPC_REGNUM == regnum) {	/* ---------- NPC ---------- */
    // 		set_npc(hex2reg(valstr),rspptr);
    // 		/*
    // 		   err = gdb_write_reg(NPC_CPU_REG_ADD, hex2reg (valstr));
    // 		   if(err > 0){
    // 		   if (g_show_verbose) printf("Error %d in rsp_write_reg write --> NPC\n", err);
    // 		   put_str_packet ("E01",rspptr);
    // 		   return;
    // 		   }
    // 		 */
    // 	} else if (SR_REGNUM == regnum) {	/* ---------- SR ---------- */
    // 		err = gdb_write_reg(SR_CPU_REG_ADD, hex2reg(valstr));
    // 		if (err > 0) {
    // 			if (g_show_verbose)
    // 				printf
    // 				    ("Error %d in rsp_write_reg write --> SR\n",
    // 				     err);
    // 			put_str_packet("E01",rspptr);
    // 			return;
    // 		}
    else if (regnum < NUM_REGS) {
        ;
    } else {
        /* Error response if we don't know the register */
        fprintf(stderr,
            "Warning: Attempt to write unknown register 0x%x: "
            "ignored\n", regnum);
        put_str_packet("E01", rspptr);
        return;
    }

    put_str_packet("OK", rspptr);
    // if (regnum == 18) {
    // 	uint32_t readreg;
    // 	gdb_read_reg(0x001f0e00 + regnum*4, &readreg);
    // 	printf("\nreadreg = %08x\n", readreg);
    // }

}				/* rsp_write_reg () */
#endif

/*---------------------------------------------------------------------------*/
/*!Handle a RSP query request

  @param[in] p_buf  The request                                               */
  /*---------------------------------------------------------------------------*/
static void rsp_query(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    /* Temporary fix for gdb executing `monitor reset` meaning to reset DSP.
     * qRcmd,7265736574 will be always received in this case.
     * TODO: to parse command after qRcmd and behave accordingly.
     */
    if (0 == strncmp("qRcmd", p_buf->data, strlen("qRcmd"))) {
        reset_or1k();
        // stall processor after reset it
        dbg_cpu0_write_ctrl(0, 0x01);
        put_str_packet("OK", rspptr);
    } else if (0 == strncmp("qSupported", p_buf->data, strlen("qSupported"))) {
        /* Report a list of the features we support. For now we just ignore any
           supplied specific feature queries, but in the future these may be
           supported as well. Note that the packet size allows for 'G' + all the
           registers sent to us, or a reply to 'g' with all the registers and an
           EOS so the buffer is a well formed string. */
        //setup_or32();
        char reply[GDB_BUF_MAX];
        sprintf(reply, "PacketSize=%x", GDB_BUF_MAX);
        put_str_packet(reply, rspptr);
    } else {
        /* the old behavior, gdb will report operation not supported */
        //printf("Not supported: %s\n", p_buf->data);
        put_str_packet("", rspptr);
    }
    return;
    if (0 == strcmp("qC", p_buf->data)) {
        /* Return the current thread ID (unsigned hex). A null response
           indicates to use the previously selected thread. Since we do not
           support a thread concept, this is the appropriate response. */
        put_str_packet("", rspptr);
    }
    if (0 == strncmp("qAttached", p_buf->data, strlen("qAttached"))) {
        /* Return an indication of whether the remote server attached to an
           existing process or created a new process.
           Reply: '1' - The remote server attached to an existing process.
           '0' - The remote server created a new process. */
        put_str_packet("1", rspptr);
    } else if (0 == strncmp("qCRC", p_buf->data, strlen("qCRC"))) {
        /* Return CRC of memory area */
        fprintf(stderr, "Warning: RSP CRC query not supported\n");
        put_str_packet("E01", rspptr);
    } else if (0 == strcmp("qfThreadInfo", p_buf->data)) {
        /* Return info about active threads. We return just '-1' */
        put_str_packet("m-1", rspptr);
    } else if (0 == strcmp("qsThreadInfo", p_buf->data)) {
        /* Return info about more active threads. We have no more, so return the
           end of list marker, 'l' */
        put_str_packet("l", rspptr);
    } else if (0 ==
        strncmp("qGetTLSAddr:", p_buf->data,
            strlen("qGetTLSAddr:"))) {
        /* We don't support this feature */
        put_str_packet("", rspptr);
    } else if (0 == strncmp("qL", p_buf->data, strlen("qL"))) {
        /* Deprecated and replaced by 'qfThreadInfo' */
        fprintf(stderr,
            "Warning: RSP qL deprecated: no info returned\n");
        put_str_packet("qM001", rspptr);
    } else if (0 == strcmp("qOffsets", p_buf->data)) {
        /* Report any relocation */
        put_str_packet("Text=0;Data=0;Bss=0", rspptr);
    } else if (0 == strncmp("qP", p_buf->data, strlen("qP"))) {
        /* Deprecated and replaced by 'qThreadExtraInfo' */
        fprintf(stderr,
            "Warning: RSP qP deprecated: no info returned\n");
        put_str_packet("", rspptr);
    } else if (0 == strncmp("qRcmd,", p_buf->data, strlen("qRcmd,"))) {
        /* This is used to interface to commands to do "stuff" */
        rsp_command(p_buf, rspptr);
    } else if (0 ==
        strncmp("qSupported", p_buf->data, strlen("qSupported"))) {
        /* Report a list of the features we support. For now we just ignore any
           supplied specific feature queries, but in the future these may be
           supported as well. Note that the packet size allows for 'G' + all the
           registers sent to us, or a reply to 'g' with all the registers and an
           EOS so the buffer is a well formed string. */
        setup_or32();	// setup cpu
        char reply[GDB_BUF_MAX];
        sprintf(reply, "PacketSize=%x", GDB_BUF_MAX);
        put_str_packet(reply, rspptr);
    } else if (0 == strncmp("qSymbol:", p_buf->data, strlen("qSymbol:"))) {
        /* Offer to look up symbols. Nothing we want (for now). TODO. This just
           ignores any replies to symbols we looked up, but we didn't want to
           do that anyway! */
        put_str_packet("OK", rspptr);
    } else if (0 == strncmp("qThreadExtraInfo,", p_buf->data,
        strlen("qThreadExtraInfo,"))) {
        /* Report that we are runnable, but the text must be hex ASCI
           digits. For now do this by steam, reusing the original packet */
        sprintf(p_buf->data, "%02x%02x%02x%02x%02x%02x%02x%02x%02x",
            'R', 'u', 'n', 'n', 'a', 'b', 'l', 'e', 0);
        p_buf->len = strlen(p_buf->data);
        put_packet(p_buf, rspptr);
    } else if (0 == strncmp("qXfer:", p_buf->data, strlen("qXfer:"))) {
        /* For now we support no 'qXfer' requests, but these should not be
           expected, since they were not reported by 'qSupported' */
        fprintf(stderr,
            "Warning: RSP 'qXfer' not supported: ignored\n");
        put_str_packet("", rspptr);
    } else if (0 == strncmp("qTStatus", p_buf->data, strlen("qTStatus"))) {
        /* Ask the stub if there is a trace experiment running right now.
           The reply has the form:
           `Trunning[;field]...'
           running is a single digit 1 if the trace is presently running, or 0
           if not. It is followed by semicolon-separated optional fields that an
           agent may use to report additional status.
         */
         /* Not supported. Return empty packet */
        put_str_packet("", rspptr);
    } else if ((0 == strncmp("qTfV", p_buf->data, strlen("qTfV"))) ||
        (0 == strncmp("qTsV", p_buf->data, strlen("qTsV")))) {
        /*
           These packets request data about trace state variables that are on the
           target. gdb sends qTfV to get the first vari of data, and multiple qTsV
           to get additional variables. Replies to these packets follow the syntax
           of the QTDV packets that define trace state variables.
         */
        put_str_packet("", rspptr);
    } else if (0 == strncmp("qTfP", p_buf->data, strlen("qTfP"))) {
        /*
           These packets request data about tracepoints that are being used by the
           target. gdb sends qTfP to get the first piece of data, and multiple
           qTsP to get additional pieces. Replies to these packets generally take
           the form of the QTDP packets that define tracepoints. (FIXME add
           detailed syntax)
         */
        put_str_packet("", rspptr);
    } else {

        fprintf(stderr, "Unrecognized RSP query: ignored\n");
    }
}				/* rsp_query () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP qRcmd request

  The actual command follows the "qRcmd," in ASCII encoded to hex

  @param[in] p_buf  The request in full                                       */
  /*---------------------------------------------------------------------------*/
static void rsp_command(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    char cmd[GDB_BUF_MAX];
    unsigned int regno;
    uint32_t temp_uint32;

    hex2ascii(cmd, &(p_buf->data[strlen("qRcmd,")]));

    /* Work out which command it is */
    if (0 == strncmp("readspr ", cmd, strlen("readspr"))) {
        /* Parse and return error if we fail */
        if (1 != sscanf(cmd, "readspr %4x", &regno)) {
            fprintf(stderr,
                "Warning: qRcmd %s not recognized: ignored\n",
                cmd);
            put_str_packet("E01", rspptr);
            return;
        }

        /* SPR out of range */
        if (regno > MAX_SPRS) {
            fprintf(stderr,
                "Warning: qRcmd readspr %x too large: ignored\n",
                regno);
            put_str_packet("E01", rspptr);
            return;
        }

        /* Construct the reply */

        // Make sure the processor is stalled
        gdb_ensure_or1k_stalled();

        // First set the chain 
        gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */

        // special case for NPC
        if (regno == NPC_CPU_REG_ADD)
            temp_uint32 = get_npc(rspptr);
        else {
            err = gdb_read_reg(regno, &temp_uint32);
            if (err > 0) {
                if (g_show_verbose)
                    printf
                    ("Error %d in rsp_command at reg. %x \n",
                        err, regno);
            } else {
                reg2hex(temp_uint32, cmd);
                if (g_show_verbose)
                    printf
                    ("Error %d Command readspr Read reg. %x = 0x%08x\n",
                        err, regno, temp_uint32);
            }
        }

        // pack the result into the buffer to send back
        sprintf(cmd, "%8x", (unsigned int)temp_uint32);
        ascii2hex(p_buf->data, cmd);
        p_buf->len = strlen(p_buf->data);
        put_packet(p_buf, rspptr);
    } else if (0 == strncmp("writespr ", cmd, strlen("writespr"))) {
        unsigned int regno;
        uint32_t val;

        /* Parse and return error if we fail */
        if (2 != sscanf(cmd, "writespr %4x %8x", &regno, &val)) {
            fprintf(stderr,
                "Warning: qRcmd %s not recognized: ignored\n",
                cmd);
            put_str_packet("E01", rspptr);
            return;
        }

        /* SPR out of range */
        if (regno > MAX_SPRS) {
            fprintf(stderr,
                "Warning: qRcmd writespr %x too large: ignored\n",
                regno);
            put_str_packet("E01", rspptr);
            return;
        }
        // Make sure the processor is stalled
        gdb_ensure_or1k_stalled();

        // First set the chain 
        gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */

        /* set the relevant register */
        // special case for NPC
        if (regno == NPC_CPU_REG_ADD)
            set_npc(val, rspptr);
        else {

            err = gdb_write_reg(regno, val);
            if (err > 0) {
                if (g_show_verbose)
                    printf
                    ("Error %d in rsp_command write Reg. %x = 0x%08x\n",
                        err, regno, val);
                put_str_packet("E01", rspptr);
                return;
            } else {
                if (g_show_verbose)
                    printf
                    ("Error %d Command writespr Write reg. %x = 0x%08x\n",
                        err, regno, val);
            }
        }
        put_str_packet("OK", rspptr);
    }
}				/* rsp_command () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP set request

  @param[in] p_buf  The request                                               */
  /*---------------------------------------------------------------------------*/
static void rsp_set(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    if (0 == strncmp("QPassSignals:", p_buf->data, strlen("QPassSignals:"))) {
        /* Passing signals not supported */
        put_str_packet("", rspptr);
    } else if ((0 == strncmp("QTDP", p_buf->data, strlen("QTDP"))) ||
        (0 == strncmp("QFrame", p_buf->data, strlen("QFrame"))) ||
        (0 == strcmp("QTStart", p_buf->data)) ||
        (0 == strcmp("QTStop", p_buf->data)) ||
        (0 == strcmp("QTinit", p_buf->data)) ||
        (0 == strncmp("QTro", p_buf->data, strlen("QTro")))) {
        /* All tracepoint features are not supported. This reply is really only
           needed to 'QTDP', since with that the others should not be
           generated. */
        put_str_packet("", rspptr);
    } else {
        fprintf(stderr, "Unrecognized RSP set request: ignored\n");
    }
}				/* rsp_set () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP restart request

  For now we just put the program counter back to the one used with the last
  vRun request. There is no point in unstalling the processor, since we'll
  never get control back.                                                   */
  /*---------------------------------------------------------------------------*/
static void rsp_restart(struct global_rsp* rspptr) {
    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    // First set the chain 
    err = gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */
    if (err > 0) {
        if (g_show_verbose)
            printf("Error %d in gdb_set_chain\n", err);
        put_str_packet("E01", rspptr);
        return;
    }
    // OR32 Arc sim equivalent --> set_npc (rspptr->start_addr,rspptr);
    /* Set NPC to reset vector 0x100 */
    set_npc(rspptr->start_addr, rspptr);
    /*
       err = gdb_write_reg(NPC_CPU_REG_ADD, rspptr->start_addr);
       if(err > 0){
       if (g_show_verbose) printf("Error %d in rsp_restart write Reg. %x = 0x%08x\n", err, NPC_CPU_REG_ADD, rspptr->start_addr);
       put_str_packet ("E01",rspptr);
       return;
       }

       else{
       if (g_show_verbose) printf("Error %d Command Reset. Set NPC to Start vector %x = 0x%08x\n", err, NPC_CPU_REG_ADD, rspptr->start_addr);
       }
     */
}				/* rsp_restart () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP step request

  Parse the command to see if there is an address. Uses the underlying
  generic step function, with EXCEPT_NONE.

  @param[in] p_buf  The full step packet                          */
  /*---------------------------------------------------------------------------*/
static void rsp_step(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    uint32_t addr;		/* The address to step from, if any */

    // Make sure the processor is stalled
    gdb_ensure_or1k_stalled();

    // First set the chain 
    err = gdb_set_chain(SC_RISC_DEBUG);	/* 1 RISC Debug Interface chain */
    if (err > 0) {
        printf
        ("Error %d to set RISC Debug Interface chain in the STEP command 's'\n",
            err);
        rsp_client_close(rspptr);
        return;
    }

    if (0 == strcmp("s", p_buf->data)) {
        // Arc Sim Code -->   addr = cpu_state.pc;        /* Default uses current NPC */
        /* ---------- Npc ---------- */
        //addr = get_npc(rspptr);

        err = gdb_read_reg(NPC_CPU_REG_ADD, &addr);
        if (err > 0) {
            printf("Error %d to read EXPC in the STEP command 's'\n", err);
            rsp_client_close(rspptr);
            return;
        }

    } else if (1 != sscanf(p_buf->data, "s%x", &addr)) {
        fprintf(stderr,
            "Warning: RSP step address %s not recognized: ignored\n",
            p_buf->data);

        // Arc Sim Code -->   addr = cpu_state.pc;        /* Default uses current NPC */
        /* ---------- NPC ---------- */
        //addr = get_npc(rspptr);

        err = gdb_read_reg(NPC_CPU_REG_ADD, &addr);
        if (err > 0) {
            printf("Error %d to read NPC in the STEP command 's'\n", err);
            rsp_client_close(rspptr);
            return;
        }
    }

    if (g_show_verbose)
        printf("[DEBUG_GDB] rsp_step() --> Read NPC = 0x%08x\n", addr);

    rspptr->expected_location_packet_s = addr;

    rsp_step_generic(addr, EXCEPT_NONE, rspptr);
}				/* rsp_step () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP step with signal request

  Currently null. Will use the underlying generic step function.

  @param[in] p_buf  The full step with signal packet              */
  /*---------------------------------------------------------------------------*/
static void rsp_step_with_signal(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    printf("RSP step with signal '%s' received\n", p_buf->data);
    p_buf->data[0] = 's';
    rsp_step(p_buf, rspptr);
}				/* rsp_step_with_signal () */

/*---------------------------------------------------------------------------*/
/*!Generic processing of a step request

  The signal may be EXCEPT_NONE if there is no exception to be
  handled. Currently the exception is ignored.

  The single step flag is set in the debug registers and then the processor
  is unstalled.

  @param[in] addr    Address from which to step
  @param[in] except  The exception to use (if any)                          */
  /*---------------------------------------------------------------------------*/
static void rsp_step_generic(uint32_t addr, uint32_t except, struct global_rsp* rspptr) {
    uint32_t temp_uint32;

    /* Set the address as the value of the next program counter */

    //set_npc(addr,rspptr);	//annotated by wangwk 17-05-22
    /* Clear Debug Reason Register (DRR) 0x3015 */
    // Arc sim --> cpu_state.sprs[SPR_DRR]   = 0;
    if (gdb_write_reg(DRR_CPU_REG_ADD, 0))
        printf("Error write to DRR register\n");
    //printf("write drr = 0x%08x\n",0);
    /* Clear watchpoint break generation in Debug Mode Register 2 (DMR2) 0x3011 */
    // Arc sim --> cpu_state.sprs[SPR_DMR2] &= ~SPR_DMR2_WGB;
    if (gdb_read_reg(DMR2_CPU_REG_ADD, &temp_uint32))
        printf("Error read from DMR2 register\n");
    //printf("read dmr2 = 0x%08x\n", temp_uint32);
    temp_uint32 &= ~SPR_DMR2_WGB;
    //temp_uint32 |= SPR_DMR2_WGB;
    //printf("write dmr2 = 0x%08x\n", temp_uint32);
    if (gdb_write_reg(DMR2_CPU_REG_ADD, temp_uint32))
        printf("Error write to DMR2 register\n");

    /* Set the single step trigger in Debug Mode Register 1 (DMR1) Register 0x3010 */
    // Arc sim --> cpu_state.sprs[SPR_DMR1] |= SPR_DMR1_ST;
    if (gdb_read_reg(DMR1_CPU_REG_ADD, &temp_uint32))
        printf("Error read from DMR1 register\n");
    //printf("read dmr1 = 0x%08x\n", temp_uint32);
    temp_uint32 |= SPR_DMR1_ST;
    //temp_uint32 |= 0xC00000;
    //printf("write dmr1 = 0x%08x\n", temp_uint32);
    if (gdb_write_reg(DMR1_CPU_REG_ADD, temp_uint32))
        printf("Error write to DMR1 register\n");

    /* Set traps to be handled by the debug unit in the Debug Stop Register (DSR) Register 0x3014 */
    // Arc sim --> cpu_state.sprs[SPR_DSR]  |= SPR_DSR_TE;
    if (gdb_read_reg(DSR_CPU_REG_ADD, &temp_uint32))
        printf("Error read from DSR register\n");
    //printf("read dsr = 0x%08x\n", temp_uint32);
    temp_uint32 |= SPR_DSR_TE;
    temp_uint32 |= SPR_DRR_SCE;
    //printf("write dsr = 0x%08x\n", temp_uint32);
    if (gdb_write_reg(DSR_CPU_REG_ADD, temp_uint32))
        printf("Error write to DSR register\n");

    gdb_write_reg(0x0, 0x0);//don't know why-HTT
    //printf("write reg 0\n");

    /* Unstall the processor */
    set_stall_state(0, rspptr);   //annotated by wangwk 17-05-22
    unsigned char stalled;   //annotated by wangwk 17-05-22
    dbg_cpu0_read_ctrl(0, &stalled);  // annotated by wangwk 17-05-22
    /* Note the GDB client is now waiting for a reply. */
    rspptr->client_waiting = 1;   //annotated by wangwk 17-05-22
    //	rspptr->client_waiting = 0;  // added by wangwk 17-05-22
    if (g_show_verbose) {
        printf("After unstall the processor stalled = %d\n", stalled);
        printf("waiting = %d\n", rspptr->client_waiting);
    }
    //	put_str_packet("OK",rspptr);
}				/* rsp_step_generic () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP 'v' packet

  These are commands associated with executing the code on the target

  @param[in] p_buf  The request                                               */
  /*---------------------------------------------------------------------------*/
static void rsp_vpkt(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    if (0 == strncmp("vAttach;", p_buf->data, strlen("vAttach;"))) {
        /* Attaching is a null action, since we have no other process. We just
           return a stop packet (using TRAP) to indicate we are stopped. */
        put_str_packet("S05", rspptr);
        return;
    } else if (0 == strcmp("vCont?", p_buf->data)) {
        /* For now we don't support this. */
        put_str_packet("", rspptr);
        return;
    } else if (0 == strncmp("vCont", p_buf->data, strlen("vCont"))) {
        /* This shouldn't happen, because we've reported non-support via vCont?
           above */
        fprintf(stderr, "Warning: RSP vCont not supported: ignored\n");
        return;
    } else if (0 == strncmp("vFile:", p_buf->data, strlen("vFile:"))) {
        /* For now we don't support this. */
        fprintf(stderr, "Warning: RSP vFile not supported: ignored\n");
        put_str_packet("", rspptr);
        return;
    } else if (0 ==
        strncmp("vFlashErase:", p_buf->data,
            strlen("vFlashErase:"))) {
        /* For now we don't support this. */
        fprintf(stderr,
            "Warning: RSP vFlashErase not supported: ignored\n");
        put_str_packet("E01", rspptr);
        return;
    } else if (0 ==
        strncmp("vFlashWrite:", p_buf->data,
            strlen("vFlashWrite:"))) {
        /* For now we don't support this. */
        fprintf(stderr,
            "Warning: RSP vFlashWrite not supported: ignored\n");
        put_str_packet("E01", rspptr);
        return;
    } else if (0 == strcmp("vFlashDone", p_buf->data)) {
        /* For now we don't support this. */
        fprintf(stderr,
            "Warning: RSP vFlashDone not supported: ignored\n");
        put_str_packet("E01", rspptr);
        return;
    } else if (0 == strncmp("vRun;", p_buf->data, strlen("vRun;"))) {
        /* We shouldn't be given any args, but check for this */
        if (p_buf->len > (int)strlen("vRun;")) {
            fprintf(stderr,
                "Warning: Unexpected arguments to RSP vRun "
                "command: ignored\n");
        }

        /* Restart the current program. However unlike a "R" packet, "vRun"
           should behave as though it has just stopped. We use signal
           5 (TRAP). */
        rsp_restart(rspptr);
        put_str_packet("S05", rspptr);
    } else {
        fprintf(stderr,
            "Warning: Unknown RSP 'v' packet type %s: ignored\n",
            p_buf->data);
        put_str_packet("E01", rspptr);
        return;
    }
}				/* rsp_vpkt () */


/* check whether the chip IRAM range is fixed */
static int is_iram_range_fixed() {
    int isfixed = 0;
    
    if ((g_iram_range.core0_iram_base == g_iram_range.value_invalid)
	    && (g_iram_range.core0_iram_size == g_iram_range.value_invalid)
	    && (g_iram_range.core1_iram_base == g_iram_range.value_invalid)
	    && (g_iram_range.core1_iram_size == g_iram_range.value_invalid)) {
        /* let IRAM range fixed if they are not set */
        isfixed = 1;
    } else if ((g_iram_range.core0_iram_base >= 0x00200000 && g_iram_range.core0_iram_base < 0x00240000)
               && (g_iram_range.core1_iram_base >= 0x00200000 && g_iram_range.core1_iram_base < 0x00240000)) {
        /* let IRAM range fixed if both base in predefined range */
        isfixed = 1;
    }

    return isfixed;
}

/* Check whether addr contains instructions.
 * 
 * Return value:
 *   0 - addr has data
 *   1 - addr has instructioon
 * 
 * Below are possible IRAM ranges for current chips
 * +-----------------+--------------------------+-----------------------------------------------------------------+
 * |   chip type     |          BootRAM         |                              IRAM                               |
 * +-----------------+--------------------------+-----------------------------------------------------------------+
 * | QXS320F280049   | [0x00280000, 0x00282000) | [0x00200000, 0x00240000)                                        |
 * | QXS320F28377L   | [0x00280000, 0x00282000) | [0x00200000, 0x00240000)                                        |
 * | QXS320F28335    | [0x00280000, 0x00282000) | [0x00200000, 0x00240000)                                        |
 * | QXS320F28035    | [0x00280000, 0x00282000) | [0x00200000, 0x00218000)                                        |
 * | QXS320F280049v2 | [0x00280000, 0x00284000) | [0x00000000, 0x00080000) configurable, [0x00080000, 0x00100000) |
 * | QXS320F28P659   | [0x00280000, 0x00284000) | [0x00000000, 0x000C0000) configurable, [0x000C0000, 0x00200000) |
 * +-----------------+--------------------------+-----------------------------------------------------------------+
 */
static int address_in_bootram_iram(unsigned int addr) {
    // return true if bootram
    if (addr >= 0x00280000 && addr < 0x00284000) {
        return 1;
    }

    int isIramAddr = 0;
    if (is_iram_range_fixed()) {
        isIramAddr = (addr >= 0x00200000 && addr < 0x00240000);
    } else {
        isIramAddr = (addr >= g_iram_range.core0_iram_base && addr < g_iram_range.core0_iram_base + g_iram_range.core0_iram_size)
                    || (addr >= g_iram_range.core1_iram_base && addr < g_iram_range.core1_iram_base + g_iram_range.core1_iram_size)
                    || (addr >= g_iram_range.core0cla_iram_base && addr < g_iram_range.core0cla_iram_base + g_iram_range.core0cla_iram_size)
                    || (addr >= g_iram_range.core1cla_iram_base && addr < g_iram_range.core1cla_iram_base + g_iram_range.core1cla_iram_size);
        if ((CHIP_QXS320F28P65x == g_chip_type) || (CHIP_QXS320F2837xD == g_chip_type)) {
            isIramAddr |= (addr >= 0x000C0000 && addr < 0x00200000);
        } else {
            isIramAddr |= (addr >= 0x00080000 && addr < 0x00100000);
        }
    }

    return isIramAddr;
}

static int is_address_xintf_zone(unsigned int addr) {
    return (addr >= 0x20004000 && addr <= 0x204FFFFF);
}

/*---------------------------------------------------------------------------*/
/*!Handle a RSP write memory (binary) request

  Syntax is:

  X<addr>,<length>:

  Followed by the specified number of bytes as raw binary. Response should be
  "OK" if all copied OK, E<nn> if error <nn> has occurred.

  The length given is the number of bytes to be written. However the number
  of data bytes may be greater, since '#', '$' and '}' are escaped by
  preceding them by '}' and oring with 0x20.

  @param[in] p_buf  The command received                                      */
  /*---------------------------------------------------------------------------*/

static void rsp_write_mem_bin(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    unsigned int addr;	/* Where to write the memory */
    int len;		/* Number of bytes to write */
    char* bindat, * bindat_ptr;	/* Pointer to the binary data */
    int off = 0;		/* Offset to start of binary data */
    int newlen;		/* Number of bytes in bin data */
    int i;
    //int bytes_per_word = 4;	/* Current OR implementation is 4-byte words */

    if (2 != sscanf(p_buf->data, "X%x,%x:", &addr, &len)) {
        fprintf(stderr, "Warning: Failed to recognize RSP write memory "
            "command: %s\n", p_buf->data);
        put_str_packet("E01", rspptr);
        return;
    }

    /* Not to write XINTF Zone during program download stage, because XINTF interface is not
     * initialized at this stage.
     */
    if ((1 == g_download_program_stage) && is_address_xintf_zone(addr)) {
        if (g_show_verbose) {
            printf("Attempt writing to XINTF during program download, skip.\n");
            fflush(stdout);
        }
        put_str_packet("OK", rspptr);
        return;
    }

    /* Find the start of the data and "unescape" it */
    bindat = p_buf->data;
    while (off < GDB_BUF_MAX) {
        if (bindat[off] == ':') {
            bindat = bindat + off + 1;
            off++;
            break;
        }
        off++;
    }
    if (off >= GDB_BUF_MAX) {
        put_str_packet("E01", rspptr);
        return;
    }

    newlen = rsp_unescape(bindat, p_buf->len - off);
    //printf("rsp_write_mem_bin newlen = %d \n", newlen);
    /* Sanity check */
    if (newlen != len) {
        int minlen = len < newlen ? len : newlen;

        fprintf(stderr,
            "Warning: Write of %d bytes requested, but %d bytes "
            "supplied. %d will be written\n", len, newlen, minlen);
        len = minlen;
    }
    /*//reset and stall the cpu
    dbg_cpu0_write_ctrl(0, 0x03);
    //unreset the cpu
    dbg_cpu0_write_ctrl(0, 0x01);*/
    gdb_ensure_or1k_stalled();

    // Set chain 5 --> Wishbone Memory chain
    err = gdb_set_chain(SC_WISHBONE);
    if (err) {
        put_str_packet("E01", rspptr);
        return;
    }

    /* Write the bytes to memory */
    if (len) {
        /* IRAM data must be 128-bit aligned, and swap according to HW requirements */
        if (address_in_bootram_iram(addr) != 0) {
            len = swap_buf_128bit(bindat, len);
        }

        if (g_show_verbose) {
            uint32_t temp_uint32;
            for (off = 0; off < len; off++) {
                temp_uint32 = (temp_uint32 << 8) | (0x000000ff & bindat[off]);
                if ((off % 4) == 3) {
                    //temp_uint32 = htonl(temp_uint32);
                }
                switch (off % 16) {
                case 3:
                    printf("Add 0x%08x   Data 0x%08x  ",
                        addr + off - 3, temp_uint32);
                    break;
                case 7:
                case 11:
                    printf("0x%08x  ", temp_uint32);
                    break;
                case 15:
                    printf("0x%08x\n", temp_uint32);
                    break;
                default:
                    break;
                }
                if ((len - off == 1) && (off % 16) < 15)
                    printf("\n");
            }
        }

        bindat_ptr = bindat;	// Copy pointer so we don't trash the original.
        if (addr & 0x3)	// not perfectly aligned at beginning - fix
        {
            if (g_show_verbose) {
                printf("rsp_write_mem_bin: address not word aligned: 0x%.8x\n", addr);
                fflush(stdout);
            }
            // Write enough to align us
            int bytes_to_write = bytes_per_word - (addr & 0x3);
            if (bytes_to_write > len) {
                bytes_to_write = len;	// case of writing 1 byte to adr 0x1
            }
            if (g_show_verbose) {
                printf("rsp_write_mem_bin: writing %d bytes of len (%d)\n", bytes_to_write, len);
                fflush(stdout);
            }

            /* Write by byte looks not working, write by word instead */
            if (2 == bytes_to_write) {
                /* FIXME: I am not sure whether it is related to memory structure, when calling gdb_write_halfword(),
                 *        a write to address[1:0] = 2'b00 actually write to address[1:0] = 2'b10, while 
                 *        a write to address[1:0] = 2'b10 actually write to address[1:0] = 2'b00
                 */
                uint16_t halfword2write = *(uint16_t*)bindat_ptr;
                err = gdb_write_halfword(addr ^ 0x2, halfword2write);
            } else if (1 == bytes_to_write) {
                uint32_t headDataAligned = 0;
                uint32_t wordAlignedAddr = addr & (~0x03);
                // align addr to 32-bit word
                gdb_read_block(getMemoryReadAddress(wordAlignedAddr), &headDataAligned, bytes_per_word);
                switchWordEndianness(&headDataAligned);
                uint8_t * bytePtrOfData = (uint8_t *)(&headDataAligned);
                bytePtrOfData[addr & 0x3] = (uint8_t)(bindat_ptr[0]);
                gdb_write_block(wordAlignedAddr, &headDataAligned, bytes_per_word);
            } else {
                uint32_t headDataAligned = 0;
                uint32_t wordAlignedAddr = addr & (~0x03);
                // align addr to 32-bit word
                gdb_read_block(getMemoryReadAddress(wordAlignedAddr), &headDataAligned, bytes_per_word);
                switchWordEndianness(&headDataAligned);
                uint8_t * bytePtrOfData = (uint8_t *)(&headDataAligned);
                for (int i = 0; i < bytes_to_write; ++i) {
                    bytePtrOfData[bytes_per_word-bytes_to_write+i] = (uint8_t)(bindat_ptr[i]);
                }
                gdb_write_block(wordAlignedAddr, &headDataAligned, bytes_per_word);
            }

            addr += bytes_to_write;
            bindat_ptr += bytes_to_write;
            len -= bytes_to_write;
            if (g_show_verbose) {
                printf("rsp_write_mem_bin: address should now be word aligned: 0x%.8x\n", addr);
                fflush(stdout);
            }
        }

        if ((len > 3) && !err)	// now write full words, if we can
        {
            int words_to_write = len / bytes_per_word;
            if (g_show_verbose)
                printf("rsp_write_mem_bin: writing %d words from 0x%x, len %d bytes\n", words_to_write, addr, len);
            fflush(stdout);

            err = gdb_write_block(addr, (uint32_t*)bindat_ptr, (words_to_write * bytes_per_word));
            addr += (words_to_write * bytes_per_word);
            bindat_ptr += (words_to_write * bytes_per_word);
            len -= (words_to_write * bytes_per_word);
        }

        if (len && !err) {
            if (g_show_verbose) {
                printf("rsp_write_mem_bin: writing remainder %d bytes to 0x%08x\n", len, addr);
                for (i = 0; i < len; i++) {
                    printf("0x%02x ", (uint8_t)bindat_ptr[i]);
                }
                printf("\n");
                fflush(stdout);
            }

            // len must be < bytes_per_word
            if (!(len < bytes_per_word)) {
                printf("ERROR: rsp_write_mem_bin() last several bytes, len >= bytes_per_word. len: %d\n", len);
                assert(len < bytes_per_word);
            }

            // addr must be word aligned
            if (!((addr & 0x3) == 0)) {
                printf("ERROR: rsp_write_mem_bin() last several bytes, addr not 4-byte aligned. addr: 0x%08x\n", addr);
                assert((addr & 0x3) == 0);
            }

            /* writing with gdb_write_byte will fail and leave RAM data unchanged */
            if (2 == len) {
                uint16_t halfword2write = *(uint16_t*)bindat_ptr;
                err = gdb_write_halfword(addr ^ 0x2, halfword2write);
            } else {
                /* align remainder bytes to word and put in leftDataAligned */
                uint32_t leftDataAligned = 0;

                /* read word to write, so that high bytes are not overwritten */
                gdb_read_block(getMemoryReadAddress(addr), &leftDataAligned, bytes_per_word);
                switchWordEndianness(&leftDataAligned);

                /* fill in remainder bytes from low byte to high byte*/
                uint8_t * bytePtrOfData = (uint8_t *)(&leftDataAligned);
                for (int i = 0; i < len; ++i) {
                    bytePtrOfData[i] = (uint8_t)(bindat_ptr[i]);
                }

                gdb_write_block(addr, &leftDataAligned, bytes_per_word);
            }
        }

        if (err) {
            put_str_packet("E01", rspptr);
            return;
        }
#if 0
        if (g_show_verbose)
            printf("Error %x\n", err);
#endif
        fflush(stdout);
    }
    put_str_packet("OK", rspptr);
}				/* rsp_write_mem_bin () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP remove breakpoint or matchpoint request

  For now only memory breakpoints are implemented, which are implemented by
  substituting a breakpoint at the specified address. The implementation must
  cope with the possibility of duplicate packets.

  @todo This doesn't work with icache/immu yet

  @param[in] p_buf  The command received                                      */
  /*---------------------------------------------------------------------------*/
static void rsp_remove_matchpoint(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    enum mp_type type;	/* What sort of matchpoint */
    uint32_t addr;		/* Address specified */
    int len;		/* Matchpoint length (not used) */
    struct mp_entry* mpe;	/* Info about the replaced instr */

    /* Break out the instruction */
    if (3 != sscanf(p_buf->data, "z%1d,%x,%1d", (int*)&type, &addr, &len)) {
        fprintf(stderr, "Warning: RSP matchpoint deletion request not "
            "recognized: ignored\n");
        put_str_packet("E01", rspptr);
        return;
    }

    /* Sanity check that the length is 4 */
    if (4 != len) {
        fprintf(stderr,
            "Warning: RSP matchpoint deletion length %d not "
            "valid: 4 assumed\n", len);
        len = 4;
    }

    /* Sort out the type of matchpoint */
//	switch (type) {
    switch (1) {
    case BP_MEMORY:
        /* Memory breakpoint - replace the original instruction. */
        mpe = mp_hash_delete(type, addr, rspptr);

        /* If the BP hasn't yet been deleted, put the original instruction
           back. Don't forget to free the hash table entry afterwards. */
        if (NULL != mpe) {
            // Arc Sim Code -->   set_program32 (addr, mpe->instr);
            // Make sure the processor is stalled
            gdb_ensure_or1k_stalled();

            // Set chain 5 --> Wishbone Memory chain
            err = gdb_set_chain(SC_WISHBONE);
            if (err) {
                put_str_packet("E01", rspptr);
                return;
            }
            err = gdb_write_block(addr, &mpe->instr, 4);
            if (err) {
                put_str_packet("E01", rspptr);
                return;
            }
            free(mpe);
        }

        put_str_packet("OK", rspptr);
        return;

    case BP_HARDWARE:
        mpe = mp_hash_delete(type, addr, rspptr);

        /* If the BP hasn't yet been deleted, put the original instruction
           back. Don't forget to free the hash table entry afterwards. */
        if (NULL != mpe) {
            uint32_t hwp = mpe->instr;
            // Arc Sim Code -->   set_program32 (addr, mpe->instr);
            // Make sure the processor is stalled
            gdb_ensure_or1k_stalled();

            // Set chain 5 --> Wishbone Memory chain
            err = gdb_set_chain(SC_RISC_DEBUG);
            if (err) {
                put_str_packet("E01", rspptr);
                return;
            }
            uint32_t temp_uint32;
            if (gdb_read_reg(DMR2_CPU_REG_ADD, &temp_uint32))
                printf("Error read from DMR2 register\n");
            //printf("Read from DMR2:0x%x\n", temp_uint32);
            temp_uint32 &= ~((0x1 << hwp) << 12);
            //printf("write 0x%x to DMR2\n", temp_uint32);
            if (gdb_write_reg(DMR2_CPU_REG_ADD, temp_uint32))
                printf("Error write from DMR2 register\n");
            temp_uint32 = 0x01;
            //printf("write 0x%x to DCR%d\n", temp_uint32, hwp);
            if (gdb_write_reg(DCR_CPU_REG_ADD(hwp), temp_uint32))
                printf("Error write to DCR register\n");
            if (gdb_write_reg(DVR_CPU_REG_ADD(hwp), 0))
                printf("Error write to DVR register\n");

            if (gdb_read_reg(DMR1_CPU_REG_ADD, &temp_uint32))
                printf("Error read from DMR1 register\n");
            //printf("Read from DMR1: 0x%x\n", temp_uint32);
            temp_uint32 &= ~SPR_DMR1_ST;
            temp_uint32 |= 0x3 << (2 * hwp);
            //printf("write 0x%x to DMR1\n", temp_uint32);
            if (gdb_write_reg(DMR1_CPU_REG_ADD, temp_uint32))
                printf("Error write from DCR register\n");
            free(mpe);
            hwp_return_watchpoint(hwp); /* mark the wp as unused again */
        }

        put_str_packet("OK", rspptr);
        return;

    case WP_WRITE:
        put_str_packet("", rspptr);	/* Not supported */
        return;

    case WP_READ:
        put_str_packet("", rspptr);	/* Not supported */
        return;

    case WP_ACCESS:
        put_str_packet("", rspptr);	/* Not supported */
        return;

    default:
        fprintf(stderr, "Warning: RSP matchpoint type %d not "
            "recognized: ignored\n", type);
        put_str_packet("E01", rspptr);
        return;

    }
}				/* rsp_remove_matchpoint () */

/*---------------------------------------------------------------------------*/
/*!Handle a RSP insert breakpoint or matchpoint request

  For now only memory breakpoints are implemented, which are implemented by
  substituting a breakpoint at the specified address. The implementation must
  cope with the possibility of duplicate packets.

  @todo This doesn't work with icache/immu yet

  @param[in] p_buf  The command received                                      */
  /*---------------------------------------------------------------------------*/
static void rsp_insert_matchpoint(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    enum mp_type type;	/* What sort of matchpoint */
    uint32_t addr;		/* Address specified */
    int len;		/* Matchpoint length (not used) */
    uint32_t instr;

    /* Break out the instruction */
    if (3 != sscanf(p_buf->data, "Z%1d,%x,%1d", (int*)&type, &addr, &len)) {
        fprintf(stderr, "Warning: RSP matchpoint insertion request not "
            "recognized: ignored\n");
        put_str_packet("E01", rspptr);
        return;
    }
    
    if (g_show_verbose) {
        printf("rsp_insert_matchpoint(): break addr = 0x%x\n", addr);
    }
    /* Sanity check that the length is 4 */
    if (4 != len) {
        fprintf(stderr,
            "Warning: RSP matchpoint insertion length %d not "
            "valid: 4 assumed\n", len);
        len = 4;
    }

    /* Sort out the type of matchpoint */
//	switch (type) {
    switch (1) {
    case BP_MEMORY:	// software-breakpoint Z0  break
        /* Memory breakpoint - substitute a TRAP instruction */
        // Make sure the processor is stalled
        gdb_ensure_or1k_stalled();

        // Set chain 5 --> Wishbone Memory chain
        gdb_set_chain(SC_WISHBONE);

        // Read the data from Wishbone Memory chain
        // Arc Sim Code -->   mp_hash_add (type, addr, eval_direct32 (addr, 0, 0));
        gdb_read_block(addr, &instr, 4);

        mp_hash_add(type, addr, instr, rspptr);

        //instr = OR1K_TRAP_INSTR;
        instr = ntohl(OR1K_TRAP_INSTR);	// Endianess of this needs to be swapped -jb
        err = gdb_write_block(addr, &instr, 4);
        if (err) {
            put_str_packet("E01", rspptr);
            return;
        }
        put_str_packet("OK", rspptr);
        return;

    case BP_HARDWARE:	// hardware-breakpoint Z1  hbreak

        if (NULL == mp_hash_lookup(type, addr, rspptr)) {
            gdb_ensure_or1k_stalled();

            // Set chain 5 --> Wishbone Memory chain
            gdb_set_chain(SC_RISC_DEBUG);

            uint32_t hwp = hwp_get_available_watchpoint();
            if (hwp == -1) /* No HWP available */
            {
                fprintf(stderr, "Warning: no hardware watchpoints available to satisfy GDB request for hardware breakpoint");
                put_str_packet("", rspptr);
            } else {
                mp_hash_add(type, addr, hwp, rspptr);
                //printf("Address of DVR%d = 0x%08x\n", hwp, DVR_CPU_REG_ADD(hwp));
                if (gdb_write_reg(DVR_CPU_REG_ADD(hwp), addr))
                    printf("Error write to DVR%d register\n", hwp);
                if (gdb_write_reg(DCR_CPU_REG_ADD(hwp), 0x23))
                    printf("Error write to DCR%d register\n", hwp);
                uint32_t temp_uint32;
                if (gdb_read_reg(DMR1_CPU_REG_ADD, &temp_uint32))
                    printf("Error read from DMR1 register\n");
                //printf("Read from DMR1: 0x%x\n", temp_uint32);
                temp_uint32 &= ~SPR_DMR1_ST;
                temp_uint32 &= (~(0x3 << (2 * hwp)));
                //printf("Write 0x%x to DMR1\n", temp_uint32);
                //				uint32_t temp_uint32 = 0xff3ffffc;
                if (gdb_write_reg(DMR1_CPU_REG_ADD, temp_uint32))
                    printf("Error write to DMR1 register\n");
                if (gdb_read_reg(DMR2_CPU_REG_ADD, &temp_uint32))
                    printf("Error read from DMR2 register\n");
                //printf("Read from DMR2:0x%x\n", temp_uint32);
                //				temp_uint32 |= (0x1 << hwp) << 12;
                temp_uint32 |= 0x001ff000;	////0x003ff00会被视为单步调试，原因待查
                //printf("Write 0x%x to DMR2\n", temp_uint32);
                if (gdb_write_reg(DMR2_CPU_REG_ADD, temp_uint32))
                    printf("Error write to DMR2 register\n");
            }
        }
        put_str_packet("OK", rspptr);	/* Not supported */
        return;

    case WP_WRITE:		// write-watchpoint    Z2  watch                                                                                                                                                        
        put_str_packet("", rspptr);	/* Not supported */
        return;

    case WP_READ:		// read-watchpoint     Z3  rwatch
        put_str_packet("", rspptr);	/* Not supported */
        return;

    case WP_ACCESS:	// access-watchpoint   Z4  awatch
        put_str_packet("", rspptr);	/* Not supported */
        return;

    default:
        fprintf(stderr, "Warning: RSP matchpoint type %d not "
            "recognized: ignored\n", type);
        put_str_packet("E01", rspptr);
        return;
    }
}				/* rsp_insert_matchpoint () */

/*---------------------------------------------------------------------------
  Setup the or32 to init state

  ---------------------------------------------------------------------------*/
void setup_or32(void) {
    uint32_t temp_uint32;
    // First set the chain 
    err = gdb_set_chain(SC_REGISTER);	/* 4 Register Chain */
    if (err > 0) {
        if (g_show_verbose)
            printf("Error %d in gdb_set_chain\n", err);
    }
    if (gdb_read_reg(0x04, &temp_uint32))
        printf("Error read from register\n");
    if (g_show_verbose)
        printf
        ("Read from chain 4 SC_REGISTER at add 0x00000004 = 0x%08x\n",
            temp_uint32);

    if (gdb_write_reg(0x04, 0x00000001))
        printf("Error write to register\n");
    if (g_show_verbose)
        printf
        ("Write to chain 4 SC_REGISTER at add 0x00000004 = 0x00000001\n");

    //    if(gdb_read_reg(0x04, &temp_uint32)) printf("Error read from register\n");
    //    if (g_show_verbose) printf("Read from chain 4 SC_REGISTER at add 0x00000004 = 0x%08x\n", temp_uint32 );                      

    //    if(gdb_read_reg(0x04, &temp_uint32)) printf("Error read from register\n");
    //    if (g_show_verbose) printf("Read from chain 4 SC_REGISTER at add 0x00000004 = 0x%08x\n", temp_uint32 );                      

    if (gdb_write_reg(0x00, 0x01000001))
        printf("Error write to register\n");
    if (g_show_verbose)
        printf
        ("Write to chain 4 SC_REGISTER at add 0x00000000 = 0x01000001\n");
}

// Function to check if the processor is stalled - if not then stall it.
// this is useful in the event that GDB thinks the processor is stalled, but has, in fact
// been hard reset on the board and is running.
static void gdb_ensure_or1k_stalled() {
    /* no need to check DSP stall status in non-stop mode, return silently. */
    if (NON_STOP_MODE == g_option_debug_mode) {
        return;
    }

    unsigned char stalled;
    dbg_cpu0_read_ctrl(0, &stalled);
    if (g_show_verbose) {
        printf("gdb_ensure_or1k_stalled(): after dbg_cpu0_read_ctrl(), stalled = %d\n", stalled);
    }
    if ((stalled & 0x1) != 0x1) {
        if (g_show_verbose)
            printf("Processor not stalled, like we thought\n");

        // Set the TAP controller to its OR1k chain
        dbg_set_tap_ir(JI_DEBUG);
        gdb_chain = -1;

        // Processor isn't stalled, contrary to what we though, so stall it
        printf("Stalling dsp\n");
        dbg_cpu0_write_ctrl(0, 0x01);	// stall or1k
    }
}

int gdb_write_byte(uint32_t adr, uint8_t data) {

#ifdef OR32_KERNEL_DBG_COMPAT
    if (kernel_debug && IS_VM_ADDR(adr))
        adr = adr & ~OR32_LINUX_VM_MASK;
#endif

    if (DEBUG_CMDS)
        printf("wbyte %d\n", gdb_chain);
    switch (gdb_chain) {
    case SC_WISHBONE:
        return dbg_wb_write8(adr, data) ? ERR_CRC : ERR_NONE;
    default:
        return JTAG_PROXY_INVALID_CHAIN;
    }
}

int gdb_write_halfword(uint32_t adr, uint16_t data) {

#ifdef OR32_KERNEL_DBG_COMPAT
    if (kernel_debug && IS_VM_ADDR(adr))
        adr = adr & ~OR32_LINUX_VM_MASK;
#endif

    if (DEBUG_CMDS)
        printf("whalfword %d\n", gdb_chain);
    switch (gdb_chain) {
    case SC_WISHBONE:
        return dbg_wb_write16(adr, data) ? ERR_CRC : ERR_NONE;
    default:
        return JTAG_PROXY_INVALID_CHAIN;
    }
}

int  gdb_read_byte(uint32_t adr, uint8_t* data) {

#ifdef OR32_KERNEL_DBG_COMPAT
    if (kernel_debug && IS_VM_ADDR(adr))
        adr = adr & ~OR32_LINUX_VM_MASK;
#endif

    switch (gdb_chain) {
    case SC_WISHBONE:
        return dbg_wb_read8(adr, data) ? ERR_CRC : ERR_NONE;
    case SC_TRACE:
        *data = 0;
        return 0;
    default:
        return JTAG_PROXY_INVALID_CHAIN;
    }
}

int gdb_write_reg(uint32_t adr, uint32_t data) {

#ifdef OR32_KERNEL_DBG_COMPAT
    if (kernel_debug && IS_VM_ADDR(adr))
        adr = adr & ~OR32_LINUX_VM_MASK;
#endif

    switch (gdb_chain) {	/* remap registers, to be compatible with jp1 */
    case SC_RISC_DEBUG:
        return dbg_cpu0_write(adr, &data, 4) ? ERR_CRC : ERR_NONE;
    case SC_REGISTER:
        return dbg_cpu0_write_ctrl(adr, data) ? ERR_CRC : ERR_NONE;
    case SC_WISHBONE:
        return dbg_wb_write32(adr, data) ? ERR_CRC : ERR_NONE;
    case SC_TRACE:
        return 0;
    default:
        return JTAG_PROXY_INVALID_CHAIN;
    }
}

int gdb_read_reg(uint32_t adr, uint32_t* data) {
#ifdef OR32_KERNEL_DBG_COMPAT
    if (kernel_debug && IS_VM_ADDR(adr))
        adr = adr & ~OR32_LINUX_VM_MASK;
#endif

    switch (gdb_chain) {
    case SC_RISC_DEBUG:
        return dbg_cpu0_read(adr, data, 4) ? ERR_CRC : ERR_NONE;
    case SC_REGISTER:
        return dbg_cpu0_read_ctrl(adr, (unsigned char*)data) ?
            ERR_CRC : ERR_NONE;
    case SC_WISHBONE:
        return dbg_wb_read32(adr, data) ? ERR_CRC : ERR_NONE;
    case SC_TRACE:
        *data = 0;
        return 0;
    default:
        return JTAG_PROXY_INVALID_CHAIN;
    }
}

int gdb_read_block(uint32_t adr, uint32_t* data, int len) {

#ifdef OR32_KERNEL_DBG_COMPAT
    if (kernel_debug && IS_VM_ADDR(adr))
        adr = adr & ~OR32_LINUX_VM_MASK;
#endif

    if (DEBUG_CMDS)
        printf("[DEBUG_CMDS] gdb_read_block(), gdb_chain = %d\n", gdb_chain);

    switch (gdb_chain) {
    case SC_RISC_DEBUG:
        return dbg_cpu0_read(adr, data, len) ? ERR_CRC : ERR_NONE;
    case SC_WISHBONE:
        return dbg_wb_read_block32(adr, data, len) ? ERR_CRC : ERR_NONE;

    default:
        return JTAG_PROXY_INVALID_CHAIN;
    }
}

int gdb_write_block(uint32_t adr, uint32_t* data, int len) {

#ifdef OR32_KERNEL_DBG_COMPAT
    if (kernel_debug && IS_VM_ADDR(adr))
        adr = adr & ~OR32_LINUX_VM_MASK;
#endif

    if (DEBUG_CMDS)
        printf("[DEBUG_CMDS] gdb_write_block(),  gdb_chain = %d\n", gdb_chain);
    switch (gdb_chain) {
    case SC_RISC_DEBUG:
        return dbg_cpu0_write(adr, data,
            (uint32_t)len) ? ERR_CRC : ERR_NONE;
    case SC_WISHBONE:
        return dbg_wb_write_block32(adr, data,
            len) ? ERR_CRC : ERR_NONE;
    default:
        return JTAG_PROXY_INVALID_CHAIN;
    }
}

int gdb_set_chain(int chain) {
    switch (chain) {
    case SC_RISC_DEBUG:
        if (no_cpu)
            return ERR_NONE;
        else
            gdb_chain = chain;
        return ERR_NONE;
    case SC_REGISTER:
    case SC_TRACE:
    case SC_WISHBONE:
        gdb_chain = chain;
        return ERR_NONE;
    default:
        return JTAG_PROXY_INVALID_CHAIN;
    }
}

/*****************************************************************************
 * Close the connection to the client if it is open
 ******************************************************************************/
static void client_close(char err) {
    if (gdb_fd) {
        perror("gdb socket - " + err);
        close(gdb_fd);
        gdb_fd = 0;
    }
}				/* client_close () */

/*---------------------------------------------------------------------------*/
/* Swap a buffer of 4-byte from 1234 to 4321

   parameter[in]  p_buf and len
   parameter[out] none																											 */
   /*---------------------------------------------------------------------------*/
static void swap_buf(char* p_buf, int len) {
    int temp;
    int n = 0;

    if (len > 2) {
        while (n < len) {
            // swap 0 and 3
            temp = p_buf[n];
            p_buf[n] = p_buf[n + 3];
            p_buf[n + 3] = temp;
            // swap 1 and 2
            temp = p_buf[n + 1];
            p_buf[n + 1] = p_buf[n + 2];
            p_buf[n + 2] = temp;

            n += 4;
        }
    }
}

/*---------------------------------------------------------------------------*/
/* Swap a buffer of 16-byte from 0x0123_4567_89ab_cdef to 0xcdef_89ab_4567_0123

   parameter[in]  p_buf and len
   parameter[out] 128-bit aligned len																											 */
/*---------------------------------------------------------------------------*/

static int swap_buf_128bit(char* p_buf, int len) {
    /* this function is currently called only for instruction data, when handling 'X' packets
     * from gdb. To make sure len and also valid bytes in p_buf is 4-byte aligned and instruction
     * data from gdb is not corrupted.
     */
    assert(len % 4 == 0);

    /* make len and also valid bytes in p_buf 128-bit (16-byte) aligned */
    const int nBytes2Align = 128 / 8;
    if (len % nBytes2Align != 0) {
        int nBytes2Pad = nBytes2Align - (len % nBytes2Align);
        for (int ii = 0; ii < nBytes2Pad; ++ii) {
            p_buf[len+ii] = 0x00; // pad all 0s
        }
        len += nBytes2Pad;
    }
    assert(len % nBytes2Align == 0);

    /* swap foreach 128-bit by 32-bit word */
    const int nWords2Swap  = len / 4;
    int nWordsSwaped = 0;
    uint32_t * wordArray = (uint32_t *)p_buf;
    while (nWordsSwaped < nWords2Swap) {
        uint32_t lWord0 = wordArray[nWordsSwaped + 0];
        uint32_t lWord1 = wordArray[nWordsSwaped + 1];
        uint32_t lWord2 = wordArray[nWordsSwaped + 2];
        uint32_t lWord3 = wordArray[nWordsSwaped + 3];
        wordArray[nWordsSwaped + 0] = lWord3;
        wordArray[nWordsSwaped + 1] = lWord2;
        wordArray[nWordsSwaped + 2] = lWord1;
        wordArray[nWordsSwaped + 3] = lWord0;
        nWordsSwaped += 4;
    }

    return len;
}

//8 slot buffer swap
static void swap_buf_256bit(char* p_buf, int* length) {
    int temp;
    int n = 0;
    int i = 0;
    int len = *length;
    if (len % 32 != 0)//不按8 slot对齐,给后面补上nop,0x80000000
    {
        int left_word;
        left_word = (32 - len % 32) / bytes_per_word;
        for (int i = 0;i < left_word;i++) {
            p_buf[len] = 0x00;
            p_buf[len + 1] = 0x00;
            p_buf[len + 2] = 0x00;
            p_buf[len + 3] = 0x80;
            len += 4;
        }
    }
    *length = len;

    if (len >= 32) {
        while (n < len && len - n >= 32) {
            i = 0;
            while (i < 4) {
                temp = p_buf[n];
                p_buf[n] = p_buf[n + 28];
                p_buf[n + 28] = temp;

                temp = p_buf[n + 4];
                p_buf[n + 4] = p_buf[n + 24];
                p_buf[n + 24] = temp;

                temp = p_buf[n + 8];
                p_buf[n + 8] = p_buf[n + 20];
                p_buf[n + 20] = temp;

                temp = p_buf[n + 12];
                p_buf[n + 12] = p_buf[n + 16];
                p_buf[n + 16] = temp;

                n += 1;
                i += 1;
            }
            n += 28;
        }
        // if (n == len - 12) {
        // 	i = 0;
        // 	while (i < 4) {
        // 		temp = p_buf[n];
        // 		p_buf[n] = p_buf[n + 8];
        // 		p_buf[n + 8] = temp;
        // 		i += 1;
        // 	}
        // }
        // else if (n == 8) {
        // 	i = 0;
        // 	while (i < 4) {
        // 		temp = p_buf[n];
        // 		p_buf[n] = p_buf[n + 4];
        // 		p_buf[n + 4] = temp;
        // 		i += 1;
        // 	}	
        // }		
    }
}
/*---------------------------------------------------------------------------*/
/*!Set the stall state of the processor

  @param[in] state  If non-zero stall the processor.                        */
  /*---------------------------------------------------------------------------*/
static void set_stall_state(int state, struct global_rsp* rspptr) {
    if (no_cpu)
        return;

    if (state == 0) {
        err = dbg_cpu0_write_ctrl(0, 0);	/* unstall or1k */
        rspptr->stallState = UNSTALLED;
        if (g_show_verbose) printf("Core %d: rspptr->stallState == UNSTALLED\n", rspptr->cpu_no);
        rspptr->npcIsCached = 0;
        rspptr->sigval = TARGET_SIGNAL_NONE;
    } else {
        err = dbg_cpu0_write_ctrl(0, 0x01);	/* stall or1k */
        rspptr->stallState = STALLED;
        if (g_show_verbose) printf("Core %d: rspptr->stallState == STALLED\n", rspptr->cpu_no);
    }

    if (err > 0 && g_show_verbose)
        printf("Error %d in set_stall_state Stall state = %d\n", err, state);

}				/* set_stall_state () */

/*---------------------------------------------------------------------------*/
/*!Set the reset bit of the processor's control reg in debug interface
 */
 /*---------------------------------------------------------------------------*/
static void reset_or1k(void) {
    if (g_show_verbose) printf("Reset processor.\n");
    if (g_chip_type == CHIP_QXS320F2833x) {
        err = usb_dbg_cpu0_write_ctrl_nocrc(0, 0x03);
        err = usb_dbg_cpu0_write_ctrl_nocrc(0, 0x01);
        // set JTAG TDO (GPIO 37) in output mode
        uint32_t val2wr = 0x20;
        err = usb_dbg_wb_write_block32_nocrc(0x01003064, &val2wr, 4);
    } else {
        err = dbg_cpu0_write_ctrl(0, 0x02);  /* reset or1k */
    }
    if (err > 0 && g_show_verbose) printf("Error %d in reset_or1k()\n", err);
}				/* reset_or1k () */

/*---------------------------------------------------------------------------*/
/*!Set the reset bit of the processor's control reg in debug interface
 */
 /*---------------------------------------------------------------------------*/
static void unreset_or1k(void) {
    err = dbg_cpu0_write_ctrl(0, 0x00);  /* reset or1k */
    if (err > 0 && g_show_verbose) printf("Error %d in unreset_or1k()\n", err);
}				/* reset_or1k () */

/*---------------------------------------------------------------------------*/
/*!Close down the connection with GDB in the event of a kill signal

 */
 /*---------------------------------------------------------------------------*/
void gdb_close() {
    if (gdb_fd)
        close(gdb_fd);
    // Maybe do other things here!
}

uint32_t reversebytes_uint32t(uint32_t value) {
    return (value & 0x000000FFU) << 24 | (value & 0x0000FF00U) << 8 |
        (value & 0x00FF0000U) >> 8 | (value & 0xFF000000U) >> 24;
}

static int gdb_read_block_end(uint32_t adr, uint32_t* data) {
    uint32_t word_tem;
    err = gdb_set_chain(SC_WISHBONE);
    //printf("addr=0x%08x\n",adr);
    err = gdb_read_block(adr, &word_tem, bytes_per_word);
    if (err)
        return 1;
    //word_tem里是取出的word
    //进行大小端转换  
    (*data) = reversebytes_uint32t(word_tem);
    printf("word_tem=0x%08x\ndata=0x%08x\n", word_tem, *data);
    return 0;
}

static uint32_t convertByteAddress(uint32_t addr) {
    uint32_t retaddr = addr & (~0x3);
    uint32_t last2bits = addr & 0x3;
    switch (last2bits) {
        case 0x0:
            retaddr |= 0x3;
            break;
        case 0x1:
            retaddr |= 0x2;
            break;
        case 0x2:
            retaddr |= 0x1;
            break;
        case 0x3:
            retaddr |= 0x0;
            break;
        default:
            assert(false);
            break;
    }
    return retaddr;
}

static uint32_t getPathnameLength(uint32_t startAddr) {
    uint32_t len = 0; // path len including the last '\0'
    uint8_t ltbyte = 0x0;
    do {
        gdb_read_byte(convertByteAddress(startAddr+len), &ltbyte);
        len++;
    } while (ltbyte != 0);
    len += 1; // to include length of the last '\0'
    return len;
}

static int send_file_request(int trapImm, struct global_rsp* rspptr) {
    const char* Imm2func[] = { "NULL","open1", "open2", "close", "read", "write",
    "lseek", "rename", "unlink", "stat", "fstat", "gettimeofday", "isatty", "system" };

    char* packet = (char*)malloc(1024);
    err = gdb_set_chain(SC_WISHBONE);
    if (err) {
        if (g_show_verbose)
            printf("Error %d in gdb_set_chain\n", err);
        put_str_packet("E01", rspptr);
        return 0;
    }

    gdb_set_chain(SC_RISC_DEBUG);
    switch (trapImm) {
    case open1_func_code: //int open(const char *pathname, int length, int flags)
    {
        //'Fopen,pathptr/len,flags,mode
        current_filefunc = open1_func_code;
        uint32_t path, flags;
        gdb_read_reg(GRN_ADDR(4), &path);
        gdb_read_reg(GRN_ADDR(5), &flags);

        // calculate len of path
        gdb_set_chain(SC_WISHBONE);
        uint32_t len = getPathnameLength(path);
        gdb_set_chain(SC_RISC_DEBUG);

        //协议中必须要有mode，设一个默认的
        mode_t f_attrib = S_IFREG | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH;

        sprintf(packet, "Fopen,%x/%x,%x,%x", path, len, flags, f_attrib);
    }
    break;
    case open2_func_code: //int open2(const char *pathname, int length, int flags, unsigned int mode)
    {
        current_filefunc = open2_func_code;
        uint32_t path, flags;
        mode_t f_attrib;
        gdb_read_reg(GRN_ADDR(4), &path);
        gdb_read_reg(GRN_ADDR(5), &flags);
        gdb_read_reg(GRN_ADDR(6), &f_attrib);

        // calculate len of path
        gdb_set_chain(SC_WISHBONE);
        uint32_t len = getPathnameLength(path);
        gdb_set_chain(SC_RISC_DEBUG);

        sprintf(packet, "Fopen,%x/%x,%x,%x", path, len, flags, f_attrib);
    }
    break;
    case close_func_code: //int close(int fd)
    {
        current_filefunc = close_func_code;
        uint32_t fd;
        gdb_read_reg(GRN_ADDR(4), &fd);
        sprintf(packet, "Fclose,%x", fd);
    }
    break;
    case read_func_code: //int read(int fd, void *buf, unsigned int count);
    {
        current_filefunc = read_func_code;
        uint32_t fd, buf, count;
        gdb_read_reg(GRN_ADDR(4), &fd);
        gdb_read_reg(GRN_ADDR(5), &buf);
        gdb_read_reg(GRN_ADDR(6), &count);
        sprintf(packet, "Fread,%x,%x,%x", fd, buf, count);
    }
    break;
    case write_func_code: //int write(int fd, const void *buf, unsigned int count);
    {
        current_filefunc = write_func_code;
        uint32_t fd, buf, count;
        gdb_read_reg(GRN_ADDR(4), &fd);
        gdb_read_reg(GRN_ADDR(5), &buf);
        gdb_read_reg(GRN_ADDR(6), &count);
        sprintf(packet, "Fwrite,%x,%x,%x", fd, buf, count);
    }
    break;
    default:
        printf("The system call %s is not supported yet\n", Imm2func[trapImm]);
        break;
    }
    printf("send packet:%s\n", packet);

    gdb_set_chain(SC_WISHBONE);

    put_str_packet(packet, rspptr);
    free(packet);

    return 0;
}

static int target_recv_trap_back(struct rsp_buf* p_buf, struct global_rsp* rspptr) {
    traping = 0;

    char retcode[1024];
    uint32_t errorno;
    uint32_t gr30;
    gdb_read_reg(GRN_ADDR(30), &gr30);
    uint32_t errorno_addr;

    if (current_filefunc == open1_func_code)
        gdb_read_block_end(gr30 + (60 << 2), &errorno_addr);
    else if (current_filefunc == open2_func_code)
        gdb_read_block_end(gr30 + (59 << 2), &errorno_addr);
    else if (current_filefunc == close_func_code)
        gdb_read_block_end(gr30 + (62 << 2), &errorno_addr);
    else if (current_filefunc == read_func_code)
        gdb_read_block_end(gr30 + (60 << 2), &errorno_addr);
    else if (current_filefunc == write_func_code)
        gdb_read_block_end(gr30 + (60 << 2), &errorno_addr);
    else
        gdb_read_block_end(gr30 + (60 << 2), &errorno_addr);

    err = gdb_set_chain(SC_WISHBONE);
    if (err) {
        put_str_packet("E01", rspptr);
        return 0;
    }

    sscanf(p_buf->data, "F%[^,]", retcode);
    if (retcode[0] == '-') //fail find errorno
    {
        sscanf(p_buf->data, "F%*[^,],%x[^,]", &errorno);
        printf("system call error:%d\n", errorno);
        gdb_write_block(errorno_addr, &errorno, bytes_per_word);
    } //ignore error, just run
    //write to GR2
    //write to GR30+0

    uint32_t addr;
    if (current_filefunc == open1_func_code)
        addr = gr30 + (59 << 2);
    else if (current_filefunc == open2_func_code)
        addr = gr30 + (58 << 2);
    else if (current_filefunc == close_func_code)
        addr = gr30 + (61 << 2);
    else if (current_filefunc == read_func_code)
        addr = gr30 + (59 << 2);
    else if (current_filefunc == write_func_code)
        addr = gr30 + (59 << 2);
    else
        addr = gr30 + (59 << 2);

    int len = strlen(retcode);
    printf("retcode:%s\n", retcode);
    uint32_t ret = strtoul(retcode, NULL, 16);
    printf("ret=0x%08x\n", ret);
#if 0
    err = gdb_set_chain(SC_WISHBONE);
    if (err) {
        put_str_packet("E01", rspptr);
        return 0;
    }
    gdb_write_block(addr, &ret, bytes_per_word);
#else
    gdb_set_chain(SC_RISC_DEBUG);
    gdb_write_reg(GRN_ADDR(2), (uint32_t)ret);
#endif
    //cpu->mempart->store_word(addr,ret);
    //this->debug_store_region(addr, len, retcode, this);
    //continue to run
    printf("rspptr->current_stat=%s\n", rspptr->current_stat == 1 ? "step" : "continue");

    if (rspptr->current_stat == 1) {
        //rsp_step(p_buf);
        rspptr->sigval = TARGET_SIGNAL_TRAP;
        rsp_report_exception(rspptr);
    } else if (rspptr->current_stat == 2) {
        rsp_hardware_continue(p_buf, rspptr);
    }

    return 1;
}
