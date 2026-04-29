#include "chip_desc.h"
#include <assert.h>

const chipdesc_t g_chipdesc[NUM_KNOWN_CHIPS] = {
    {
        .ctype = CHIP_QXS320F2803x,
        .cname = "qxs320f2803x",
        .corecnt = 1,
        .clacnt = 0,
        .core_jtag_chain_start = 1
    },
    {
        .ctype = CHIP_QXS320F280013x,
        .cname = "qxs320f280013x",
        .corecnt = 1,
        .clacnt = 0,
        .core_jtag_chain_start = 1
    },
    {
        .ctype = CHIP_QXS320F2833x,
        .cname = "qxs320f2833x",
        .corecnt = 1,
        .clacnt = 0,
        .core_jtag_chain_start = 1
    },
    {
        .ctype = CHIP_QXS320F28P65x,
        .cname = "qxs320f28p65x",
        .corecnt = 2,
        .clacnt = 1,
        .core_jtag_chain_start = 2
    },
    {
        .ctype = CHIP_QXS320F2837xS,
        .cname = "qxs320f2837xs",
        .corecnt = 1,
        .clacnt = 1,
        .core_jtag_chain_start = 2
    },
    {
        .ctype = CHIP_QXS320F2837xD,
        .cname = "qxs320f2837xd",
        .corecnt = 2,
        .clacnt = 2,
        .core_jtag_chain_start = 2
    },
    {
        .ctype = CHIP_QXS320F28003x,
        .cname = "qxs320f28003x",
        .corecnt = 1,
        .clacnt = 1,
        .core_jtag_chain_start = 2
    },
    {
        .ctype = CHIP_QXS320F28004x,
        .cname = "qxs320f28004x",
        .corecnt = 2,
        .clacnt = 0,
        .core_jtag_chain_start = 1
    },
};

const chipdesc_t* get_chipdesc(enum CHIP_TYPE chiptype) {
    for (int i = 0; i < NUM_KNOWN_CHIPS; ++i) {
        if (g_chipdesc[i].ctype == chiptype) {
            return &(g_chipdesc[i]);
        }
    }

    assert(0 && "ERROR: unknown chiptype");
    return 0;
}
