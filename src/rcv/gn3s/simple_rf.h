/*------------------------------------------------------------------------------
* simple_rf.h 

* Copyright (C) 2023 iliasam
*-----------------------------------------------------------------------------*/

#define SIMPLE_RF_BUFFSIZE      (32*512)*4*4 //16*4 kb
#define SIMPLE_RF_FILE_READ_RATIO	16

extern int simple_rf_init(void);
extern void simple_rf_quit(void);

extern int simple_rf_pushtomembuf(void);
extern void simple_rf_getbuf_file(uint64_t buffloc, int n, int dtype, char *expbuf);
extern void simple_rf_getbuf(uint64_t buffloc, int n, int dtype, char *expbuf);

extern void simple_rf_set_rx_buf(uint32_t buf_size);
/* global functions */

