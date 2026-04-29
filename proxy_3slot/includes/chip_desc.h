#ifndef CHIP_DESC_H
#define CHIP_DESC_H

enum CHIP_TYPE {
    CHIP_QXS320F2803x = 0,
    CHIP_QXS320F280013x,
    CHIP_QXS320F2833x,
    CHIP_QXS320F28P65x,
    CHIP_QXS320F2837xS,
    CHIP_QXS320F2837xD,
    CHIP_QXS320F28003x,
    CHIP_QXS320F28004x,
    NUM_KNOWN_CHIPS,    // 8
};

// description for a given chip type
typedef struct {
    enum CHIP_TYPE ctype;		// chip type
    const char* cname;			// chip series name
    int corecnt;				// number of cores
    int clacnt;					// number of cla
    int core_jtag_chain_start;	// relative JTA chain start number of core
} chipdesc_t;

extern enum CHIP_TYPE g_chip_type;
extern const chipdesc_t* get_chipdesc(enum CHIP_TYPE chiptype);

#endif // CHIP_DESC_H
