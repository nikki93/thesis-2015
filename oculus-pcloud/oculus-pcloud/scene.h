#ifndef SCENE_H
#define SCENE_H

#include "array.h"

void scene_init(void);
void scene_deinit(void);
void scene_draw(void);

void scene_set_points(Array *new_points); /* takes ownership of array */

#endif
