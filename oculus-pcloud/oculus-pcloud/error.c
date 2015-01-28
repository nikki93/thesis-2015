#include "error.h"

#include <stdio.h>
#include <stdlib.h>

void error(const char *fmt)
{
    fputs(fmt, stderr);
    exit(1);
}

