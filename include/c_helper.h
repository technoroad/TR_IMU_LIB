#pragma once

#ifndef PRINT_ERR
#define PRINT_ERR(str) printf("\x1b[31m%s\033[m\n", str);
#endif

#ifndef PRINT_WARN
#define PRINT_WARN(str) printf("\x1b[33m%s\033[m\n", str);
#endif

#ifndef PRINT_OK
#define PRINT_OK(str) printf("\x1b[32m%s\033[m\n", str);
#endif
