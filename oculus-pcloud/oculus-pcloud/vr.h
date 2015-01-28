#ifndef VR_H
#define VR_H

void vr_recenter(void);

void vr_preinit(void);
void vr_init(void);
void vr_deinit(void);
void vr_draw(void(*drawer)(void)); /* draw using the given function per-eye */

#endif
