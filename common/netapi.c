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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "netapi.h"
#include "transport.h"
#include "kernel.h"

static bool commandIO(uint16_t opcode, 
                      const void *txData, uint16_t txSize,
                      void *rxData, uint16_t rxSize)
{
    TransmitBuffer tx;
    tx.next = NULL;
    tx.buffer = txData;
    tx.length = txSize;
    
    ReceiveBuffer rx;
    rx.next = NULL;
    rx.buffer = rxData;
    rx.length = rxSize;
    
    if (Transport_command(opcode, &tx, &rx) != 0)
        return false;
    return rx.length == rxSize;
}

static int commandOutput(uint16_t opcode, 
                         const void *txData, uint16_t txSize )
{
    TransmitBuffer tx;
    tx.next = NULL;
    tx.buffer = txData;
    tx.length = txSize;
    
    return Transport_command(opcode, &tx, NULL);
}

static int commandBasic(uint16_t opcode, 
                        const void *txData, uint16_t txSize)
{
    int32_t result = -1;
    if (!commandIO(opcode, txData, txSize, &result, sizeof(result)))
        return -1;
    return result;
}

static int commandBasicChain(uint16_t opcode, TransmitBuffer *tx)
{
    int32_t result = -1;
    ReceiveBuffer rx;
    rx.next = NULL;
    rx.buffer = &result;
    rx.length = sizeof(result);
    
    int rc = Transport_command(opcode, tx, &rx);
    if (rc < 0)
        return rc;
    if (rx.length != sizeof(result))
        return -1;
    return result;
}

static int commandBasicNoParameter(uint16_t opcode)
{
    return commandBasicChain(opcode, NULL);
}

uint16_t ioctl_get_version(void)
{
    #pragma pack(push,1)
    struct {
        uint8_t unknown1;
        uint8_t unknown2;
        uint8_t packageID;
        uint8_t buildNumber;
    } result;
    #pragma pack(pop)
    
    ReceiveBuffer rx;
    rx.next = NULL;
    rx.buffer = &result;
    rx.length = sizeof(result);
    
    if (Transport_command(HCI_CMND_READ_SP_VERSION, NULL, &rx) != 0)
        return 0xFFFF;
    if (rx.length != sizeof(result))
        return 0xFFFF;
    return ((uint16_t)result.packageID << 8) | result.buildNumber;
}

int ioctl_set_connection_policy(bool connectToLast, bool connectToProfile, 
                                bool connectToOpen) 
{
    #pragma pack(push,1)
    struct {
        uint32_t connectToOpen;
        uint32_t connectToLast;
        uint32_t connectToProfile;
    } parameters;
    #pragma pack(pop)
    parameters.connectToOpen = connectToOpen ? 1 : 0;
    parameters.connectToLast = connectToLast ? 1 : 0;
    parameters.connectToProfile = connectToProfile ? 1 : 0;
    
    return commandBasic(HCI_CMND_WLAN_IOCTL_SET_CONNECTION_POLICY,
            &parameters, sizeof(parameters));
}

#ifdef NETAPI_FULL
NetAPIConnectionStatus ioctl_get_status(void)
{
    return commandBasicNoParameter(HCI_CMND_WLAN_IOCTL_STATUSGET);
}

int ioctl_ap_connect(NetAPIScurityType security, const char *ssid,
                     const uint8_t *bssid, const char *key)
{
    #pragma pack(push,1)
    struct {
        uint32_t size1;
        uint32_t ssidLength;
        uint32_t security;
        uint32_t size2;
        uint32_t keyLength;
        uint16_t padding;
        uint8_t bssid[6];
    } parameters;
    #pragma pack(pop)
    
    TransmitBuffer txSSID;
    TransmitBuffer txKey;
    TransmitBuffer tx;
    uint32_t ssidLength;
    uint32_t keyLength;
    if (key != NULL && *key && security != Security_None) {
        keyLength = strlen(key);
        txKey.buffer = key;
        txKey.length = keyLength;
        txKey.next = NULL;
    } else {
        keyLength = 0;
    }
    if (ssid != NULL && *ssid) {
        tx.next = &txSSID;
        
        ssidLength = strlen(ssid);
        if (keyLength != 0) {
            txSSID.next = &txKey;
        } else {
            txSSID.next = NULL;
        }
        txSSID.buffer = ssid;
        txSSID.length = ssidLength;
    } else {
        if (keyLength != 0) {
            tx.next = &txKey;
        } else {
            tx.next = NULL;
        }
        ssidLength = 0;
    }
    
    parameters.size1 = 28;
    parameters.ssidLength = ssidLength;
    parameters.security = security;
    parameters.size2 = 16 + ssidLength;
    parameters.keyLength = keyLength;
    parameters.padding = 0;
    if (bssid != NULL) {
        memcpy(parameters.bssid, bssid, 6);
    } else {
        memset(parameters.bssid, 0, 6);
    }
    
    tx.buffer = &parameters;
    tx.length = sizeof(parameters);    
    return commandBasicChain(HCI_CMND_WLAN_CONNECT, &tx);
}
#endif

int ioctl_delete_profile(uint32_t index)
{
    return commandBasic(HCI_CMND_WLAN_IOCTL_DEL_PROFILE, 
        &index, sizeof(index));
}

int ioctl_add_profile_open(uint32_t priority, const char *ssid, 
                           const uint8_t *bssid)
{
    #pragma pack(push,1)
    struct {
        uint32_t security;
        uint32_t size;
        uint32_t ssidLength;
        uint16_t padding;
        uint8_t bssid[6];
        uint32_t priority;
    } parameters;
    #pragma pack(pop)
    parameters.security = WLAN_SEC_UNSEC;
    parameters.size = 20;
    parameters.padding = 0;
    parameters.priority = priority;
    
    if (bssid != NULL) {
        memcpy(parameters.bssid, bssid, 6);
    } else {
        memset(parameters.bssid, 0, 6);
    }
    
    TransmitBuffer txSSID;
    TransmitBuffer tx;
    if (ssid != NULL && *ssid) {
        tx.next = &txSSID;
        
        uint32_t ssidLength = strlen(ssid);
        txSSID.next = NULL;
        txSSID.buffer = ssid;
        txSSID.length = ssidLength;
        parameters.ssidLength = ssidLength;
    } else {
        tx.next = NULL;
        parameters.ssidLength = 0;
    }
    
    tx.buffer = &parameters;
    tx.length = sizeof(parameters);
    return commandBasicChain(HCI_CMND_WLAN_IOCTL_ADD_PROFILE, &tx);
}

static int addProfileWPA(uint32_t priority, const char *ssid, 
                         const uint8_t *bssid, const char *passphrase,
                         uint32_t securityType)
{
    if (passphrase == NULL || !(*passphrase))
        return -1;
        
    #pragma pack(push,1)
    struct {
        uint32_t security;
        uint32_t size1;
        uint32_t ssidLength;
        uint16_t padding;
        uint8_t bssid[6];
        uint32_t priority;
        uint32_t pairwiseCipher;
        uint32_t groupwiseCipher;
        uint32_t keyManagement;
        uint32_t size2;
        uint32_t passphraseLength;
    } parameters;
    #pragma pack(pop)
    parameters.security = securityType;
    parameters.size1 = 40;
    parameters.padding = 0;
    parameters.priority = priority;
    /* I have no idea what these magic numbers mean! */
    parameters.pairwiseCipher = 0x18;
    parameters.groupwiseCipher = 0x1e;
    parameters.keyManagement = 2;
    
    if (bssid != NULL) {
        memcpy(parameters.bssid, bssid, 6);
    } else {
        memset(parameters.bssid, 0, 6);
    }
    
    uint32_t passLength = strlen(passphrase);
    parameters.passphraseLength = passLength;
    TransmitBuffer txPass;
    txPass.next = NULL;
    txPass.buffer = passphrase;
    txPass.length = passLength;
    
    TransmitBuffer txSSID;
    TransmitBuffer tx;
    if (ssid != NULL && *ssid) {
        tx.next = &txSSID;
        
        uint32_t ssidLength = strlen(ssid);
        txSSID.next = &txPass;
        txSSID.buffer = ssid;
        txSSID.length = ssidLength;
        parameters.ssidLength = ssidLength;
        parameters.size2 = 8 + ssidLength;
    } else {
        tx.next = &txPass;
        parameters.ssidLength = 0;
        parameters.size2 = 8;
    }
    
    tx.buffer = &parameters;
    tx.length = sizeof(parameters);
    return commandBasicChain(HCI_CMND_WLAN_IOCTL_ADD_PROFILE, &tx);
}

int ioctl_add_profile_wpa2(uint32_t priority, const char *ssid, 
                           const uint8_t *bssid, const char *passphrase)
{
    return addProfileWPA(priority, ssid, bssid, passphrase, WLAN_SEC_WPA2);
}

int ioctl_add_profile_wpa(uint32_t priority, const char *ssid, 
                           const uint8_t *bssid, const char *passphrase)
{
    return addProfileWPA(priority, ssid, bssid, passphrase, WLAN_SEC_WPA2);
}

int ioctl_add_profile_wep(uint32_t priority, const char *ssid, 
                          const uint8_t *bssid, uint8_t keyLength,
                          const uint8_t *key)
{
    #pragma pack(push,1)
    struct {
        uint32_t security;
        uint32_t size1;
        uint32_t ssidLength;
        uint16_t padding;
        uint8_t bssid[6];
        uint32_t priority;
        uint32_t size2;
        uint32_t keyLength;
        uint32_t keyIndex;
    } parameters;
    #pragma pack(pop)
    parameters.security = WLAN_SEC_WEP;
    parameters.size1 = 32;
    parameters.padding = 0;
    parameters.priority = priority;
    /* We always only send one key and use it rather than the four supported */
    parameters.keyIndex = 0;
    
    if (bssid != NULL) {
        memcpy(parameters.bssid, bssid, 6);
    } else {
        memset(parameters.bssid, 0, 6);
    }
    
    /* Just send the same key four times */
    TransmitBuffer txKey4;
    txKey4.next = NULL;
    txKey4.buffer = key;
    txKey4.length = keyLength;
    TransmitBuffer txKey3;
    txKey3.next = &txKey4;
    txKey3.buffer = key;
    txKey3.length = keyLength;    
    TransmitBuffer txKey2;
    txKey2.next = &txKey3;
    txKey2.buffer = key;
    txKey2.length = keyLength;
    TransmitBuffer txKey1;
    txKey1.next = &txKey2;
    txKey1.buffer = key;
    txKey1.length = keyLength;
    
    TransmitBuffer txSSID;
    TransmitBuffer tx;
    if (ssid != NULL && *ssid) {
        tx.next = &txSSID;
        
        uint32_t ssidLength = strlen(ssid);
        txSSID.next = &txKey1;
        txSSID.buffer = ssid;
        txSSID.length = ssidLength;
        parameters.ssidLength = ssidLength;
        parameters.size2 = 12 + ssidLength;
    } else {
        tx.next = &txKey1;
        parameters.ssidLength = 0;
        parameters.size2 = 12;
    }
    
    tx.buffer = &parameters;
    tx.length = sizeof(parameters);
    return commandBasicChain(HCI_CMND_WLAN_IOCTL_ADD_PROFILE, &tx);
}


int ioctl_smart_config_start(void)
{
    uint32_t encrypted = 0;
    return commandBasic(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_START, 
        &encrypted, sizeof(encrypted));
}

int ioctl_smart_config_stop(void)
{
    return commandBasicNoParameter(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_STOP);
}

int ioctl_smart_config_prefix( const char prefix[3] )
{
    return commandBasic(HCI_CMND_WLAN_IOCTL_SIMPLE_CONFIG_SET_PREFIX, 
        prefix, 3);
}

int nvmem_read(NetAPINVMemFileID file, void *buffer, uint32_t length, 
               uint32_t seek)
{
    #pragma pack(push,1)
    struct {
        uint32_t fileID;
        uint32_t length;
        uint32_t offset;
    } parameters;
    #pragma pack(pop)
    parameters.fileID = file;
    parameters.length = length;
    parameters.offset = seek;
    
    TransmitBuffer command;
    command.next = NULL;
    command.buffer = &parameters;
    command.length = sizeof(parameters);
    
    ReceiveBuffer rx;
    rx.next = NULL;
    rx.buffer = buffer;
    rx.length = length;
    
    int rc = Transport_read(HCI_CMND_NVMEM_READ, &command, 
            NULL, &rx);
    if (rc < 0)
        return rc;
    return rx.length;
}

int nvmem_write(NetAPINVMemFileID file, const void *buffer, uint32_t length, 
                uint32_t seek)
{
    #pragma pack(push,1)
    struct {
        uint32_t fileID;
        uint32_t size;
        uint32_t length;
        uint32_t offset;
    } parameters;
    #pragma pack(pop)
    parameters.fileID = file;
    parameters.size = 12;
    parameters.length = length;
    parameters.offset = seek;
    
    TransmitBuffer txArguments;
    txArguments.next = NULL;
    txArguments.buffer = &parameters,
    txArguments.length = sizeof(parameters);
    
    TransmitBuffer txData;
    txData.next = NULL;
    txData.buffer = buffer;
    txData.length = length;
    
    int32_t result;
    ReceiveBuffer rx;
    rx.next = NULL;
    rx.buffer = &result;
    rx.length = sizeof(result);
    
    int rc = Transport_write(HCI_CMND_NVMEM_WRITE, &txArguments, &txData, 
            HCI_EVNT_NVMEM_WRITE, &rx);
    if (rc != 0)
        return rc;
    if (rx.length < sizeof(result))
        return -1;
    return result;
}

int nvmem_create(NetAPINVMemFileID file, uint32_t length)
{
    #pragma pack(push,1)
    struct {
        uint32_t fileID;
        uint32_t length;
    } parameters;
    #pragma pack(pop)
    parameters.fileID = file;
    parameters.length = length;
    
    return commandBasic(HCI_CMND_NVMEM_CREATE_ENTRY,
            &parameters, sizeof(parameters));
}


int ioctl_network_set(const struct in_addr ip, const struct in_addr subnet,
                      const struct in_addr gateway, const struct in_addr dns)
{
    #pragma pack(push,1)
    struct {
        uint8_t ip[4];
        uint8_t subnet[4];
        uint8_t gateway[4];
        uint32_t zero;
        uint8_t dns[4];
    } parameters;
    #pragma pack(pop)
    
    memcpy(parameters.ip, &ip.s_addr, 4);
    memcpy(parameters.subnet, &subnet.s_addr, 4);
    memcpy(parameters.gateway, &gateway.s_addr, 4);
    parameters.zero = 0;
    memcpy(parameters.dns, &dns.s_addr, 4);
    return commandOutput(HCI_NETAPP_DHCP, &parameters, sizeof(parameters));
}

int ioctl_network_dhcp(void)
{
    struct in_addr zero;
    zero.s_addr = 0;
    return ioctl_network_set(zero, zero, zero, zero);
}

int ioctl_network_status(struct in_addr *ip, struct in_addr *subnet,
                          struct in_addr *gateway, struct in_addr *dns,
                          struct in_addr *dhcp, uint8_t *mac, char *ssid,
                          uint16_t ssidLength)
{
    #pragma pack(push,1)
    struct {
        uint32_t ip;
        uint32_t subnet;
        uint32_t gateway;
        uint32_t dhcp;
        uint32_t dns;
        uint8_t mac[6];
    } result;
    #pragma pack(pop)
    
    ReceiveBuffer rx;
    rx.buffer = &result;
    rx.length = sizeof(result);
    
    ReceiveBuffer rxSSID;
    if (ssid != NULL && ssidLength != 0) {
        rx.next = &rxSSID;
        rxSSID.next = NULL;
        rxSSID.buffer = ssid;
        rxSSID.length = ssidLength;
    } else {
        rx.next = NULL;
    }
    
    int rc = Transport_command(HCI_NETAPP_IPCONFIG, NULL, &rx);
    if (rc != 0)
        return rc;
    if (rx.length != sizeof(result))
        return -1;
        
    if (ip != NULL)
        ip->s_addr = htonl(result.ip);
    if (subnet != NULL)
        subnet->s_addr = htonl(result.subnet);
    if (gateway != NULL)
        gateway->s_addr = htonl(result.gateway);
    if (dhcp != NULL)
        dhcp->s_addr = htonl(result.dhcp);
    if (dns != NULL)
        dns->s_addr = htonl(result.dns);
    if (mac != NULL)
        memcpy(mac, result.mac, 6);
        
    if (ssid != NULL && rxSSID.length < ssidLength)
        ssid[rxSSID.length] = '\0';
        
    return 0;
}

#ifdef NETAPI_FULL
int ioctl_arp_flush(void)
{
    return Transport_command(HCI_NETAPP_ARP_FLUSH, NULL, NULL);
}
#endif

int ioctl_mdns_start(const char *serviceName)
{
    if (serviceName == NULL || !(*serviceName))
        return -1;
        
    #pragma pack(push,1)
    struct {
        uint32_t enable;
        uint32_t size;
        uint32_t serviceNameLength;
    } parameters;
    #pragma pack(pop)
    
    parameters.enable = 1;
    parameters.size = 8;
    
    uint32_t serviceNameLength = strlen(serviceName);
    if (serviceNameLength > 32)
        return -1;
    parameters.serviceNameLength = serviceNameLength;
    
    TransmitBuffer txName;
    txName.next = NULL;
    txName.buffer = serviceName;
    txName.length = serviceNameLength;
    
    TransmitBuffer tx;
    tx.next = &txName;
    tx.buffer = &parameters;
    tx.length = sizeof(parameters);
    
    return commandBasicChain(HCI_EVNT_MDNS_ADVERTISE, &tx);
}

int ioctl_mdns_stop(void)
{
    #pragma pack(push,1)
    struct {
        uint32_t enable;
        uint32_t size;
        uint32_t serviceNameLength;
    } parameters;
    #pragma pack(pop)
    
    parameters.enable = 0;
    parameters.size = 8;
    parameters.serviceNameLength = 0;
    
    return commandBasic(HCI_EVNT_MDNS_ADVERTISE,
        &parameters, sizeof(parameters));
}

int ioctl_set_timeouts(uint32_t dhcpLease, uint32_t arpExpire, 
                       uint32_t keepAlive, uint32_t socketInactivity) 
{
    #pragma pack(push,1)
    struct {
        uint32_t dhcp;
        uint32_t arp;
        uint32_t keepAlive;
        uint32_t inactivity;
        uint32_t padding;   /* Total length = 20 (netapp.c:50) */
    } parameters;
    #pragma pack(pop)
    
    if (dhcpLease <= 20)
        parameters.dhcp = 20;
    else
        parameters.dhcp = dhcpLease;
        
    if (arpExpire != 0 && arpExpire <= 20)
        parameters.arp = 20;
    else
        parameters.arp = arpExpire;
        
    if (keepAlive != 0 && keepAlive <= 20)
        parameters.keepAlive = 20;
    else
        parameters.keepAlive = keepAlive;
        
    if (socketInactivity != 0 && socketInactivity <= 20)
        parameters.inactivity = 20;
    else
        parameters.inactivity = socketInactivity;
        
    return commandOutput(HCI_NETAPP_SET_TIMERS, 
            &parameters, sizeof(parameters));
}

int ioctl_ping_start(const struct in_addr ip, uint32_t timeout,
                     uint32_t attempts, uint32_t size)
{
    #pragma pack(push,1)
    struct {
        uint32_t ip;
        uint32_t attempts;
        uint32_t size;
        uint32_t timeout;
    } parameters;
    #pragma pack(pop)
    
    parameters.ip = ip.s_addr;
    parameters.attempts = attempts;
    parameters.size = size;
    parameters.timeout = timeout;
    
    return commandOutput(HCI_NETAPP_PING_SEND, 
        &parameters, sizeof(parameters));
}

int ioctl_ping_stop(void)
{
    return Transport_command(HCI_NETAPP_PING_STOP, NULL, NULL);
}

int ioctl_ping_status(void)
{
    return Transport_command(HCI_NETAPP_PING_REPORT, NULL, NULL);
}

static int setRecvTimeout(int sockfd)
{
    uint32_t val = 10;
    if (setsockopt(sockfd, SOL_SOCKET, SOCKOPT_RECV_TIMEOUT,
            &val, sizeof(val)) != 0) {
        return -1;
    }
    return 0;
}

int socket(int domain, int type, int protocol)
{
    #pragma pack(push,1)
    struct {
        uint32_t domain;
        uint32_t type;
        uint32_t protocol;
    } parameters;
    #pragma pack(pop)
    
    parameters.domain = domain;
    parameters.type = type;
    parameters.protocol = protocol;
    
    return commandBasic(HCI_CMND_SOCKET, &parameters, sizeof(parameters));
}

/**
 * Test if the given socket is reserved for the system (and so access to it
 * should return an error).  This is normally a weak alias that always
 * returns false.
 * 
 * @param sockfd        the socket in question
 * @return              true if the socket is reserved
 */
bool netapi_system_socket(int sockfd) __attribute__((weak));
bool netapi_system_socket(int sockfd)
{
    (void)sockfd;
    return false;
}

int closesocket(int fd)
{
    if (netapi_system_socket(fd))
        return -1;
        
    /* It appears that the device does not properly flush like close(2) */
    Transport_flush(TICK_RATE/10);
    
    uint32_t parameters = fd;
    return commandBasic(HCI_CMND_CLOSE_SOCKET, &parameters, sizeof(parameters));
}

int accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen)
{
    uint32_t parameters = sockfd;
    if (netapi_system_socket(sockfd))
        return -1;
    #pragma pack(push,1)
    struct {
        uint32_t sockfd;
        int32_t status;
        uint32_t address;
    } result;
    #pragma pack(pop)
    
    if (!commandIO(HCI_CMND_ACCEPT, &parameters, sizeof(parameters),
            &result, sizeof(result)))
        return -1;
    
    if (addr != NULL && addrlen != NULL && 
            *addrlen >= sizeof(struct sockaddr_in)) {
        struct sockaddr_in in;
        in.sin_family = AF_INET;
        in.sin_port = 0;    /* Don't get this back */
        in.sin_addr.s_addr = result.address;
        memcpy(addr, &in, sizeof(struct sockaddr_in));
        *addrlen = sizeof(struct sockaddr_in);
    } else if (addrlen != NULL) {
        *addrlen = 0;
    }
    
    /* Why result.status is the socket (and not result.sockfd), 
     * I don't know (TI socket.c:349) */
    sockfd = result.status;
    if (sockfd < 0 || sockfd > 7)
        return -1;
    if (setRecvTimeout(sockfd) != 0) {
        closesocket(sockfd);
        return -1;
    }
    return sockfd;
}

static int socketSingleOpAddrIPv4(int sockfd, const struct sockaddr *addr, 
                              socklen_t addrlen, uint16_t opcode)
{
    if (addrlen < sizeof(struct sockaddr_in) || netapi_system_socket(sockfd))
        return -1;
    #pragma pack(push,1)
    struct {
        uint32_t sockfd;
        uint32_t size;
        uint32_t addrlen;
        struct sockaddr_in addr;
    } parameters;
    #pragma pack(pop)
    
    parameters.sockfd = sockfd;
    parameters.size = 8;
    
    /* Only the first 8 bytes are used, so discard anything else even if there's
     * more */
    parameters.addrlen = 8;
    memcpy(&parameters.addr, addr, 8);
    if (parameters.addr.sin_family != AF_INET)
        return -1;    
    return commandBasic(opcode, &parameters, 20);
}

int bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
    return socketSingleOpAddrIPv4(sockfd, addr, addrlen, HCI_CMND_BIND);
}

int connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen)
{
    return socketSingleOpAddrIPv4(sockfd, addr, addrlen, HCI_CMND_CONNECT);
}

int listen(int sockfd, int backlog)
{
    if (netapi_system_socket(sockfd))
        return -1;
    #pragma pack(push,1)
    struct {
        uint32_t sockfd;
        uint32_t backlog;
    } parameters;
    #pragma pack(pop)
    
    parameters.sockfd = sockfd;
    parameters.backlog = backlog;
    return commandBasic(HCI_CMND_LISTEN, &parameters, sizeof(parameters));
}

int gethostbyname(const char *hostname, struct in_addr *addr)
{
    if (hostname == NULL || !(*hostname))
        return -1;
        
    #pragma pack(push,1)
    struct {
        uint32_t size;
        uint32_t hostnameLength;
    } parameters;
    struct {
        int32_t status;
        uint32_t addr;
    } result;
    #pragma pack(pop)
    
    parameters.size = 8;
    uint32_t hostnameLength = strlen(hostname);
    if (hostnameLength > 230)
        return -1;
    parameters.hostnameLength = hostnameLength;
    
    TransmitBuffer txHostname;
    txHostname.next = NULL;
    txHostname.buffer = hostname;
    txHostname.length = hostnameLength;
    
    TransmitBuffer tx;
    tx.next = &txHostname;
    tx.buffer = &parameters;
    tx.length = sizeof(parameters);
    
    ReceiveBuffer rx;
    rx.next = NULL;
    rx.buffer = &result;
    rx.length = sizeof(result);
    
    int rc = Transport_command(HCI_CMND_GETHOSTNAME, &tx, &rx);
    if (rc != 0)
        return -1;
    if (rx.length < sizeof(result))
        return -1;

    /* This apparently returns in host byte order for some random reason */
    if (addr != NULL)
        addr->s_addr = htonl(result.addr);

    if (result.addr == 0)
        return -1;
    if (result.status >= 0)
        return 0;
    return result.status;
}

int select(fd_set *readfds, fd_set *writefds, fd_set *exceptfds, 
           uint32_t timeout) 
{
    #pragma pack(push,1)
    struct {
        uint32_t nfds;
        uint32_t magic1;    /* I'd say these are sizes, but four of them? */
        uint32_t magic2;
        uint32_t magic3;
        uint32_t magic4;
        uint32_t blocking;
        uint32_t read;
        uint32_t write;
        uint32_t except;
        uint32_t to_sec;
        uint32_t to_usec;
    } parameters;
    struct {
        int32_t status;
        uint32_t read;
        uint32_t write;
        uint32_t except;
    } result;
    #pragma pack(pop)
    
    int nfds = 32;
    if (readfds != NULL) {
        asm volatile ( 
            "CLZ %[lz], %[in]" 
            : [lz] "=r" (nfds)
            : [in] "r" (*readfds) );
        parameters.read = *readfds;
    } else {
        parameters.read = 0;
    }
    if (writefds != NULL) {
        int lz;
        asm volatile ( 
            "CLZ %[lz], %[in]" 
            : [lz] "=r" (lz)
            : [in] "r" (*writefds) );
        if (lz < nfds)
            nfds = lz;
        parameters.write = *writefds;
    } else {
        parameters.write = 0;
    }
    /* This doesn't appear to do anything currently: they don't
     * trigger on faults or socket closed. */
    #ifdef NETAPI_FULL
    if (exceptfds != NULL) {
        int lz;
        asm volatile ( 
            "CLZ %[lz], %[in]" 
            : [lz] "=r" (lz)
            : [in] "r" (*exceptfds) );
        if (lz < nfds)
            nfds = lz;
        parameters.except = *exceptfds;
    } else {
        parameters.except = 0;
    }
    #else
    parameters.except = 0;
    #endif
    
    nfds = 32 - nfds;    
    if (nfds == 0 || nfds > 31) {
        Kernel_msleep(timeout);
        return 0;
    }
    
    parameters.nfds = nfds;
    parameters.magic1 = 0x14;
    parameters.magic2 = 0x14;
    parameters.magic3 = 0x14;
    parameters.magic4 = 0x14;
    /* Always block the minimum time and just re-issue commands, so we
     * don't starve the kernel of access to the interface. */
    parameters.blocking = 0;
    parameters.to_sec = 0;
    parameters.to_usec = 16384;
    
    if (timeout == 0) {
        while (true) {
            if (!commandIO(HCI_CMND_BSD_SELECT, &parameters, sizeof(parameters),
                    &result, sizeof(result)))
                return -1;
            if (result.status != 0)
                break;
            Kernel_msleep(64);
        }
    } else {
        uint32_t waitST = (timeout * TICK_RATE) / 1000;
        uint32_t start = systick;
        while (true) {
            if (!commandIO(HCI_CMND_BSD_SELECT, &parameters, sizeof(parameters),
                    &result, sizeof(result)))
                return -1;
            if (result.status != 0)
                break;
            if (Kernel_elapsed(start) >= waitST)
                return 0;
            Kernel_msleep(timeout/8);
        }
    }
    
    if (result.status >= 0) {
        if (readfds != NULL)
            *readfds = result.read;
        if (writefds != NULL)
            *writefds = result.write;
        if (exceptfds != NULL)
            *exceptfds = result.except;
    }
    
    return result.status;
}

int getsockopt(int sockfd, int level, int optname, 
               void *optval, socklen_t optlen)
{
    #pragma pack(push,1)
    struct {
        uint32_t sockfd;
        uint32_t level;
        uint32_t optname;
    } parameters;
    #pragma pack(pop)
    
    parameters.sockfd = sockfd;
    parameters.level = level;
    parameters.optname = optname;
    
    TransmitBuffer tx;
    tx.next = NULL;
    tx.buffer = &parameters;
    tx.length = sizeof(parameters);
    
    ReceiveBuffer rx;
    rx.next = NULL;
    rx.buffer = optval;
    rx.length = optlen;
    
    return Transport_command(HCI_CMND_GETSOCKOPT, &tx, &rx);
}


int setsockopt(int sockfd, int level, int optname, 
               const void *optval, socklen_t optlen)
{
    if (netapi_system_socket(sockfd))
        return -1;
    #pragma pack(push,1)
    struct {
        uint32_t sockfd;
        uint32_t level;
        uint32_t optname;
        uint32_t size;
        uint32_t optlen;
    } parameters;
    #pragma pack(pop)
    
    parameters.sockfd = sockfd;
    parameters.level = level;
    parameters.optname = optname;
    parameters.size = 8;
    parameters.optlen = optlen;
    
    TransmitBuffer txOption;
    txOption.next = NULL;
    txOption.buffer = optval;
    txOption.length = optlen;
    
    TransmitBuffer tx;
    tx.next = &txOption;
    tx.buffer = &parameters;
    tx.length = sizeof(parameters);
    
    return commandBasicChain(HCI_CMND_SETSOCKOPT, &tx) >= 0 ? 0 : -1;
}

static int recvData(int sockfd, void *buf, uint32_t len, uint16_t opcode,
                    ReceiveBuffer *rxArguments)
{
    if (netapi_system_socket(sockfd))
        return -1;
    #pragma pack(push,1)
    struct {
        uint32_t sockfd;
        uint32_t len;
        uint32_t flags;
    } parameters;
    #pragma pack(pop)
    
    parameters.sockfd = sockfd;
    parameters.len = len;
    parameters.flags = 0;
    
    TransmitBuffer tx;
    tx.next = NULL;
    tx.buffer = &parameters;
    tx.length = sizeof(parameters);
    
    ReceiveBuffer rx;
    rx.next = NULL;
    rx.buffer = buf;
    rx.length = len;
    
    int rc = Transport_recv(opcode, &tx, rxArguments, &rx);
    if (rc <= 0)
        return rc;
    return rx.length;
}

int recv(int sockfd, void *buf, uint32_t len)
{
    return recvData(sockfd, buf, len, HCI_CMND_RECV, NULL);
}

int recvfrom(int sockfd, void *buf, uint32_t len,
             struct sockaddr *src_addr, socklen_t *addrlen)
{
    #pragma pack(push,1)
    struct {
        uint8_t padding1[4];
        uint32_t addrLength;
        uint8_t padding2[8];
    } arguments;
    #pragma pack(pop)
    
    ReceiveBuffer rxArguments;
    rxArguments.buffer = &arguments;
    rxArguments.length = sizeof(arguments);
    
    ReceiveBuffer rxFrom;
    if (src_addr != NULL && addrlen != NULL && *addrlen > 0) {
        rxArguments.next = &rxFrom;
        rxFrom.next = NULL;
        rxFrom.buffer = src_addr;
        rxFrom.length = *addrlen;
    } else {
        rxArguments.next = NULL;
    }
    
    int rc = recvData(sockfd, buf, len, HCI_CMND_RECVFROM, &rxArguments);
    if (rc <= 0)
        return rc;
        
    if (addrlen != NULL)
        *addrlen = arguments.addrLength;
        
    return rc;
}


void send(int sockfd, const void *buf, uint32_t len)
{
    if (buf == NULL || len == 0 || netapi_system_socket(sockfd))
        return;
        
    #pragma pack(push,1)
    struct {
        uint32_t sockfd;
        uint32_t size;
        uint32_t len;
        uint32_t flags;
    } parameters;
    #pragma pack(pop)
    
    parameters.sockfd = sockfd;
    parameters.size = 12;
    parameters.len = len;
    parameters.flags = 0;
    
    TransmitBuffer txArguments;
    txArguments.next = NULL;
    txArguments.buffer = &parameters,
    txArguments.length = sizeof(parameters);
    
    TransmitBuffer txData;
    txData.next = NULL;
    txData.buffer = buf;
    txData.length = len;
    
    Transport_send(HCI_CMND_SEND, &txArguments, &txData);
}

void sendto(int sockfd, const void *buf, uint32_t len,
           const struct sockaddr *dest_addr, socklen_t addrlen)
{
    if (buf == NULL || len == 0)
        return;
    if (dest_addr == NULL || addrlen < sizeof(struct sockaddr_in))
        return;
    if (netapi_system_socket(sockfd))
        return;
        
    #pragma pack(push,1)
    struct {
        uint32_t sockfd;
        uint32_t size;
        uint32_t len;
        uint32_t flags;
        uint32_t addrOffset;
        uint32_t addrLength;
    } parameters;
    #pragma pack(pop)
    
    parameters.sockfd = sockfd;
    parameters.size = 20;
    parameters.len = len;
    parameters.flags = 0;
    parameters.addrOffset = 8 + len;
    /* Only the first 8 bytes are used, so discard anything else even if there's
     * more */
    parameters.addrLength = 8;
    
    TransmitBuffer txArguments;
    txArguments.next = NULL;
    txArguments.buffer = &parameters,
    txArguments.length = sizeof(parameters);
    
    TransmitBuffer txAddress;
    txAddress.next = NULL;
    txAddress.buffer = dest_addr;
    txAddress.length = parameters.addrLength;
    
    TransmitBuffer txData;
    txData.next = &txAddress;
    txData.buffer = buf;
    txData.length = len;
    
    Transport_send(HCI_CMND_SENDTO, &txArguments, &txData);
}
