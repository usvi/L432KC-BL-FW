#ifndef _PIC_CONF_H_
#define _PIC_CONF_H_

/* If you want to have GOT in RAM:
 * 1. Use the correct section in linker file
 * 2. Uncomment the define GOT_IN_RAM
 * 3. Use following gcc options:
 *
 * -fPIC
 * -msingle-pic-base
 * -mno-pic-data-is-text-relative
 * -mpic-register=r9
 */

//#define GOT_IN_RAM

#endif /* #define _PIC_CONF_H_ */
