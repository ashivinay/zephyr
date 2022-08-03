/* This function doesn't do anything exciting. It exists to try and
 * reproduce the speed issues seen when moving more complicated files into
 * ITCM
 */

#include <zephyr/zephyr.h>

#define NOP_GEN(i, _) __NOP()

/* The naming here is so we can move where the function is located by LD */
void z_zsimple_function(void) {
	/*
	 * As far as I can tell, this NOP sled affects alignment within the
	 * final binary. Uncommenting on of these lines will insert a given
	 * number of NOPs into this function. Where there are 0 NOPs,
	 * performance is around 59Mbps. However, at 20 NOPs, performance drops
	 * to 45.9 Mbps.
	 * This is reproducible with the overlay-core-files-itcm.conf config
	 * overlay in place.
	 */
	// LISTIFY(0, NOP_GEN,(;)); // 59.2Mbps
	// LISTIFY(20, NOP_GEN,(;)); // 45.9Mbps
}
