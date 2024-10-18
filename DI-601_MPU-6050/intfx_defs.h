#ifndef _PCB405_INTFX_DEFS_H
#define _PCB405_INTFX_DEFS_H

#if defined(NO_PACKED_STRUCTS)
  /* Disable struct packing */
  #define PACKED_STRUCT_START
  #define PACKED_STRUCT_END
  #define packed
#elif defined(__GNUC__) || defined(__clang__)
  /* GCC and clang */
  #define PACKED_STRUCT_START
  #define PACKED_STRUCT_END
  #define packed __attribute__((packed))
#elif defined(__ICCARM__) || defined(__CC_ARM)
  /* IAR ARM and Keil MDK-ARM */
  #define PACKED_STRUCT_START _Pragma("pack(push, 1)")
  #define PACKED_STRUCT_END   _Pragma("pack(pop)")
  #define packed
#elif defined(_MSC_VER) && (_MSC_VER >= 1500)
  /* Microsoft Visual Studio */
  #define PACKED_STRUCT_START __pragma(pack(push, 1))
  #define PACKED_STRUCT_END   __pragma(pack(pop))
  #define packed
#else
  /* Unknown compiler */
  #define PACKED_STRUCT_START
  #define PACKED_STRUCT_END
  #define packed
#endif

#endif /* _PCB405_INTFX_DEFS_H */
