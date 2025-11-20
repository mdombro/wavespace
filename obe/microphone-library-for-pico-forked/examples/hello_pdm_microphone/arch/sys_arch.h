#ifndef LWIP_ARCH_SYS_ARCH_H
#define LWIP_ARCH_SYS_ARCH_H

#include <stdint.h>
#include <stddef.h>

#include "lwip/opt.h"

#include "pico/time.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SYS_MBOX_NULL NULL
#define SYS_SEM_NULL  NULL

typedef uint32_t sys_prot_t;

#define SYS_ARCH_DECL_PROTECT(lev) sys_prot_t lev
#define SYS_ARCH_PROTECT(lev)      lev = sys_arch_protect()
#define SYS_ARCH_UNPROTECT(lev)    sys_arch_unprotect(lev)

sys_prot_t sys_arch_protect(void);
void sys_arch_unprotect(sys_prot_t pval);

#if !NO_SYS
#error "This sys_arch implementation supports NO_SYS=1 only. Adjust lwipopts.h."
#endif

#ifdef __cplusplus
}
#endif

#endif /* LWIP_ARCH_SYS_ARCH_H */
