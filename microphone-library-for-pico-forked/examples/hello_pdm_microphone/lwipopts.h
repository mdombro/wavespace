#ifndef _LWIPOPTS_H_
#define _LWIPOPTS_H_

/*
 * Minimal lwIP configuration for Pico W UDP streaming.  This is derived from the
 * Pico SDK Wi-Fi examples (lwipopts_examples_common.h) and tuned for the
 * threadsafe background architecture.
 */

#define NO_SYS                              1
#define LWIP_SOCKET                         0
#define LWIP_NETCONN                        0

#define MEM_ALIGNMENT                       4
#define MEM_SIZE                            8192

#define MEMP_NUM_PBUF                       16
#define MEMP_NUM_UDP_PCB                    6
#define MEMP_NUM_TCP_PCB                    4
#define MEMP_NUM_TCP_PCB_LISTEN             2
#define MEMP_NUM_TCP_SEG                    8
#define MEMP_NUM_SYS_TIMEOUT                8

#define LWIP_ARP                            1
#define LWIP_ETHERNET                       1
#define LWIP_IPV4                           1
#define LWIP_IPV6                           0
#define LWIP_IGMP                           0

#define LWIP_ICMP                           1
#define LWIP_UDP                            1
#define LWIP_TCP                            0
#define LWIP_DNS                            1
#define LWIP_DHCP                           1

#define LWIP_NETIF_STATUS_CALLBACK          1
#define LWIP_NETIF_LINK_CALLBACK            1

#define LWIP_TCPIP_CORE_LOCKING             0
#define LWIP_TCPIP_CORE_LOCKING_INPUT       0
#define SYS_LIGHTWEIGHT_PROT                0

#define ETHARP_SUPPORT_STATIC_ENTRIES       1

#define PBUF_POOL_SIZE                      16
#define PBUF_LINK_HLEN                      16

#define LWIP_SNTP                           0
#define LWIP_STATS                          0
#define LWIP_DEBUG                          0

#define CHECKSUM_GEN_IP                     1
#define CHECKSUM_GEN_UDP                    1
#define CHECKSUM_GEN_TCP                    1
#define CHECKSUM_GEN_ICMP                   1
#define CHECKSUM_CHECK_IP                   1
#define CHECKSUM_CHECK_UDP                  1
#define CHECKSUM_CHECK_TCP                  1

#define LWIP_TIMEVAL_PRIVATE                0

#endif /* _LWIPOPTS_H_ */
