// project:     followme
// file:        macro.h
// created by bss at 2013-12-22
// last edit: by bss at 2013-12-22
// description: macros.

#ifndef _MACRO_H_
#define _MACRO_H_

#ifndef SAFE_DELETE
#define SAFE_DELETE(p)  \
    if (p != NULL)  \
    {   \
        delete p;   \
        p = NULL;   \
    }
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p)  \
    if (p != NULL)  \
    {   \
        delete[] p;   \
        p = NULL;   \
    }
#endif

#endif

