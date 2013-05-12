/*
 * Copyright (C) 2013 Derek Hageman <Derek.C.Hageman@gmail.com> 
 * 
 * This file is part of Fulcrum.
 *
 * Fulcrum is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * Fulcrum is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 * 
 * You should have received a copy of the GNU Affero General Public License
 * along with Fulcrum.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _NETAPI_H
#define _NETAPI_H

#include <stdint.h>
#include <stdbool.h>

#ifndef _API_CALL
#define _API_CALL
#endif

#pragma pack(push,4)
typedef uint32_t socklen_t;
struct in_addr {
    uint32_t s_addr;
};
#pragma pack(2)
struct sockaddr_in {
    uint16_t sin_family;
    uint16_t sin_port;
    struct in_addr sin_addr;
    uint8_t sin_zero[0];
};
#pragma pack(1)
struct sockaddr {
    uint16_t sa_family;
    uint8_t sa_data[sizeof(struct sockaddr_in)];
};
#pragma pack(pop)

inline uint32_t htonl( uint32_t n ) {
    asm ( "REV %0, %0" : "+r" (n) );
    return n;
}
inline uint32_t ntohl( uint32_t n ) { return htonl(n); }

inline uint16_t htons( uint16_t n ) {
    asm ( "REV16 %0, %0" : "+r" (n) );
    return n;
}
inline uint16_t ntohs( uint16_t n ) { return htons(n); }

/**
 * Get the version of the CC3000 service pack.
 * 
 * @return  high order byte is the package number, low order is the build number, 0xFFFF on error
 */
uint16_t ioctl_get_version(void) _API_CALL;

/**
 * Set the connection policy.
 * 
 * @param connectToLast     enable connection to the last AP that the device was connected to
 * @param connectToProfile  enable connection to any AP described in a stored profile
 * @param connectToOpen     enable connection to any open AP
 * @return                  0 on success, -1 on error
 */
int ioctl_set_connection_policy(bool connectToLast, bool connectToProfile, 
                                bool connectToOpen) _API_CALL;

#ifdef NETAPI_FULL
typedef enum {
    Status_Disconnected = 0,
    Status_Scanning = 1,
    Status_Connecting = 2,
    Status_Connected = 3,
    Status_Error = -1,
} NetAPIConnectionStatus;
/**
 * Get the WiFi status.
 * 
 * @return                  the WiFi status
 */
NetAPIConnectionStatus ioctl_get_status(void) _API_CALL;

typedef enum {
    Security_None = 0,
    Security_WEP = 1,
    Security_WPA = 2,
    Security_WPA2 = 3,
} NetAPIScurityType;
/**
 * Connect to an AP.
 * 
 * @param security          the security type
 * @param ssid              the SSID or NULL
 * @param bssid             the BSSID (MAC) of the AP or NULL
 * @param key               the security key or NULL
 * @return                  0 on success, -1 on error
 */
int ioctl_ap_connect(NetAPIScurityType security, const char *ssid,
                     const uint8_t *bssid, const char *key) _API_CALL;
#endif
                     
/**
 * Delete a stored profile.
 * 
 * @param index             the profile index
 * @return                  0 on success, -1 on error
 */
int ioctl_delete_profile(uint32_t index) _API_CALL;

/**
 * Add a profile for an open AP.
 * 
 * @param priority          the priority (higher used first)
 * @param ssid              the SSID of the AP or NULL
 * @param bssid             the BSSID (MAC) of the AP or NULL
 * @return                  the profile index or -1 on error
 */
int ioctl_add_profile_open(uint32_t priority, const char *ssid, 
                           const uint8_t *bssid) _API_CALL;
                           
/**
 * Add a profile for a WPA2 secured AP.
 * 
 * @param priority          the priority (higher used first)
 * @param ssid              the SSID of the AP or NULL
 * @param bssid             the BSSID (MAC) of the AP or NULL
 * @param passphrase        the passphrase
 * @return                  the profile index or -1 on error
 */
int ioctl_add_profile_wpa2(uint32_t priority, const char *ssid, 
                           const uint8_t *bssid, const char *passphrase) 
                           _API_CALL;

/**
 * Add a profile for a WPA secured AP.
 * 
 * @param priority          the priority (higher used first)
 * @param ssid              the SSID of the AP or NULL
 * @param bssid             the BSSID (MAC) of the AP or NULL
 * @param passphrase        the passphrase
 * @return                  the profile index or -1 on error
 */
int ioctl_add_profile_wpa(uint32_t priority, const char *ssid, 
                          const uint8_t *bssid, const char *passphrase) 
                          _API_CALL;

/**
 * Add a profile for a WEP "secured" AP.
 * 
 * @param priority          the priority (higher used first)
 * @param ssid              the SSID of the AP or NULL
 * @param bssid             the BSSID (MAC) of the AP or NULL
 * @param keyLength         the length of all keys (5 or 13)
 * @param key               the key
 */
int ioctl_add_profile_wep(uint32_t priority, const char *ssid, 
                          const uint8_t *bssid, uint8_t keyLength,
                          const uint8_t *key) _API_CALL;


/**
 * Initiate smart config sequence.
 * 
 * @return                  0 on success, -1 on error
 */
int ioctl_smart_config_start(void) _API_CALL;

/**
 * Halt the smart config sequence.
 * 
 * @return                  0 on success, -1 on error
 */
int ioctl_smart_config_stop(void) _API_CALL;

/**
 * Set the smart config prefix.
 * 
 * @param prefix            the three character prefix
 * @return                  0 on success, -1 on error
 */
int ioctl_smart_config_prefix( const char prefix[3] ) _API_CALL;


typedef enum {
    FileID_NVS = 0,
    FileID_NVS_Shadow,
    FileID_WLANConfig,
    FileID_WLANConfig_Shadow,
    FileID_DriverPatches,
    FileID_FirmwarePatches,
    FileID_MAC,
    FileID_FrontendVariables,
    FileID_IPConfig,
    FileID_IPConfig_Shadow,
    FileID_BootloaderPatches,
    FileID_RM,
    FileID_AESKey,
    FileID_SmartConfigShared,
    FileID_User1,
    FileID_User2,
    
    FileID_MAX,
} NetAPINVMemFileID;

/**
 * Read data from the CC3000's EEPROM.
 * 
 * @param file          the file ID
 * @param buffer        the output buffer
 * @param length        the number of bytes to read
 * @param seek          the offset from the start of the file to read at
 * @return              the number of bytes read or negative on error
 */
int nvmem_read(NetAPINVMemFileID file, void *buffer, uint32_t length, 
               uint32_t seek) _API_CALL;

/**
 * Write data to the CC3000's EEPROM.
 * 
 * @param file          the file ID
 * @param buffer        the input buffer
 * @param length        the number of bytes to write
 * @param seek          the offset from the start of the file to write at
 * @return              zero on success
 */
int nvmem_write(NetAPINVMemFileID file, const void *buffer, uint32_t length, 
                uint32_t seek) _API_CALL;
                
/**
 * Create or resize a a file in the CC3000's EEPROM.
 * 
 * @param file          the file ID
 * @param length        the new length (zero to invalidate)
 * @return              zero on success
 */
int nvmem_create(NetAPINVMemFileID file, uint32_t length) _API_CALL;

/**
 * Configure the network interface parameters.
 * 
 * @param ip        the IP address
 * @param subnet    the subnet mask
 * @param gateway   the default gateway
 * @param dns       the DNS server
 * @return          zero on success
 */
int ioctl_network_set(const struct in_addr ip, const struct in_addr subnet,
                      const struct in_addr gateway, const struct in_addr dns) 
                      _API_CALL;
                      
/**
 * Change the network settings to DHCP mode.
 * 
 * @return          zero on success
 */
int ioctl_network_dhcp(void) _API_CALL; 

/**
 * Query the network status.
 * 
 * @param ip        the output IP address or NULL
 * @param subnet    the output usbnet mask or NULL
 * @param gateway   the output gateway or NULL
 * @param dns       the output DNS server or NULL
 * @param dhcp      the output DHCP server
 * @param mac       the output MAC address or NULL
 * @param ssid      the output SSID or NULL
 * @param ssidLength the maximum length of the returned SSID
 * @return          zero on success
 */
int ioctl_network_status(struct in_addr *ip, struct in_addr *subnet,
                          struct in_addr *gateway, struct in_addr *dns,
                          struct in_addr *dhcp, uint8_t *mac, char *ssid,
                          uint16_t ssidLength) _API_CALL;

#ifdef NETAPI_FULL
/**
 * Flush the ARP table.
 * 
 * @return          zero on success
 */
int ioctl_arp_flush(void) _API_CALL;
#endif

/**
 * Enable mDNS advertising of the device.
 * 
 * @param serviceName   the service name to advertise
 * @return              zero on success
 */
int ioctl_mdns_start(const char *serviceName) _API_CALL;

/**
 * Stop mDNS advertising of the device.
 * 
 * @return              zero on success
 */
int ioctl_mdns_stop(void) _API_CALL;

/**
 * Set the network timeouts in seconds.
 * 
 * @param dhcpLease     the DHCP lease time
 * @param arpExpire     the time before expiring an ARP table entry or zero for none
 * @param keepAlive     the keepalive send timer or zero for none
 * @param socketInactivity the time before closing a socket for inactivity or zero for never
 */
int ioctl_set_timeouts(uint32_t dhcpLease, uint32_t arpExpire, 
                       uint32_t keepAlive, uint32_t socketInactivity) _API_CALL;

/**
 * Start an autonomous ping.  The transport handler is called automatically
 * if it succeeds.
 * 
 * @param ip            the IP address
 * @param timeout       the timeout in milliseconds
 * @param attempts      the number of ping attempts
 * @param size          the packet size
 * @return              zero on successful issue
 */
int ioctl_ping_start(const struct in_addr ip, uint32_t timeout,
                     uint32_t attempts, uint32_t size) _API_CALL;
                     
/**
 * Stop any autonomous ping in progress.
 * 
 * @return              zero on success
 */
int ioctl_ping_stop(void) _API_CALL;

/**
 * Request a ping status report.  This will cause the transport handler
 * to be called with the current ping status.
 * 
 * @return              zero on success
 */
int ioctl_ping_status(void) _API_CALL;

/* Only valid constants for the CC3000 */
#define AF_INET         2
#define SOCK_STREAM     1
#define SOCK_DGRAM      2
#define SOCK_RAW        3
#define IPPROTO_TCP     6
#define IPPROTO_UDP     17
#define IPPROTO_RAW     255
#define INADDR_ANY      ((struct in_addr){ .s_addr = 0 })

/** @see socket(2) */
int socket(int domain, int type, int protocol) _API_CALL;

/** @see close(2) */
int closesocket(int fd) _API_CALL;

/** @see accept(2) */
int accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen) _API_CALL;
#define ACCEPT_IN_PROGRESS  -2

/** @see bind(2) */
int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen) _API_CALL;

/** @see connect(2) */
int connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen) 
    _API_CALL;

/** @see listen(2) */
int listen(int sockfd, int backlog) _API_CALL;

/**
 * Resolve a host name to an IP address.
 * 
 * @param hostname      the host name to resolve
 * @param addr          the output address
 * @return              zero on success
 */
int gethostbyname(const char *hostname, struct in_addr *addr) _API_CALL;

typedef uint32_t fd_set;
inline void FD_CLR(int fd, fd_set *set) { *set &= ~(1<<fd); }
inline int FD_ISSET(int fd, const fd_set *set) { return (*set) & (1<<fd); }
inline void FD_SET(int fd, fd_set *set) { *set |= (1<<fd); }
inline void FD_ZERO(fd_set *set) { *set = 0; }

/**
 * Basically POSIX select, with the parameters we don't need removed.  
 * 
 * @param readfds       on input, the sockets to check for reading; on output the sockets available for reading or NULL to disable
 * @param writefds      on input, the sockets to check for writing; on output the sockets available for writing or NULL to disable 
 * @param exceptfds     on input, the sockets to check for exceptions; on output the sockets with exceptions or NULL to disable
 * @param timeout       the maximum time to wait in milliseconds or zero to disable blocking
 * @see select(2)
 */
int select(fd_set *readfds, fd_set *writefds, fd_set *exceptfds, 
           uint32_t timeout) _API_CALL;


/* The valid subset for the CC3000 */
#define SOL_SOCKET              0xFFFF
#define SOCKOPT_RECV_TIMEOUT    1
#define SOCKOPT_NONBLOCK        2
#define SOCK_ON                 0   /* This makes perfect sense! */
#define SOCK_OFF                1

/** @see getsockopt(2) */
int getsockopt(int sockfd, int level, int optname, 
               void *optval, socklen_t optlen) _API_CALL;
               
/** @see setsockopt(2) */
int setsockopt(int sockfd, int level, int optname, 
               const void *optval, socklen_t optlen) _API_CALL;
               
/** @see recv(2) */
int recv(int sockfd, void *buf, uint32_t len) _API_CALL;

/** @see recvfrom(2) */
int recvfrom(int sockfd, void *buf, uint32_t len,
             struct sockaddr *src_addr, socklen_t *addrlen) _API_CALL;
             
/** @see send(2) */
void send(int sockfd, const void *buf, uint32_t len) _API_CALL;

/** @see sendto(2) */
void sendto(int sockfd, const void *buf, uint32_t len,
            const struct sockaddr *dest_addr, socklen_t addrlen) _API_CALL;
        
#endif
