// btstack_config.h — placed in src/ so it overrides the framework default
// Must have ENABLE_HID_HOST for hid_host_init() to be compiled in

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// ENABLE_CLASSIC and ENABLE_BLE are set by the framework build flags — don't redefine
#define ENABLE_HID_HOST

#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

#define HCI_OUTGOING_PRE_BUFFER_SIZE    4
// 1021 = maximum Bluetooth ACL payload. Needed for devices with large SDP/HID
// descriptors (e.g. Wii Balance Board). The WiiMote works with 169 but the
// Balance Board's SDP response exceeds that, causing 0x66 connection failure.
#define HCI_ACL_PAYLOAD_SIZE            (1021)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT    4

#define MAX_NR_HCI_CONNECTIONS          1
#define MAX_NR_L2CAP_CHANNELS           4
#define MAX_NR_L2CAP_SERVICES           3
#define MAX_NR_HID_HOST_CONNECTIONS     1
#define MAX_NR_HIDS_CLIENTS             0
#define MAX_NR_SERVICE_RECORD_ITEMS     4
#define MAX_NR_SM_LOOKUP_ENTRIES        3
#define MAX_NR_WHITELIST_ENTRIES        16
#define MAX_NR_LE_DEVICE_DB_ENTRIES     16
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 3
#define MAX_NR_CONTROLLER_ACL_BUFFERS   3
#define MAX_NR_CONTROLLER_SCO_PACKETS   0

#define MAX_NR_AVDTP_CONNECTIONS        0
#define MAX_NR_AVDTP_STREAM_ENDPOINTS   0
#define MAX_NR_AVRCP_CONNECTIONS        0
#define MAX_NR_BNEP_CHANNELS            0
#define MAX_NR_BNEP_SERVICES            0
#define MAX_NR_RFCOMM_CHANNELS          0
#define MAX_NR_RFCOMM_MULTIPLEXERS      0
#define MAX_NR_RFCOMM_SERVICES          0

#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN         HCI_ACL_PAYLOAD_SIZE
#define HCI_HOST_ACL_PACKET_NUM         3
#define HCI_HOST_SCO_PACKET_LEN         0
#define HCI_HOST_SCO_PACKET_NUM         0

#define NVM_NUM_DEVICE_DB_ENTRIES       16
#define NVM_NUM_LINK_KEYS               16
#define MAX_ATT_DB_SIZE                 512

#define HAVE_EMBEDDED_TIME_MS
#define HAVE_ASSERT
#define HCI_RESET_RESEND_TIMEOUT_MS     1000
#define ENABLE_SOFTWARE_AES128
#define ENABLE_MICRO_ECC_FOR_LE_SECURE_CONNECTIONS

#endif // BTSTACK_CONFIG_H