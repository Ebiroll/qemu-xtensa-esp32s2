#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <errno.h>

#define HCI_COMMAND 0x1
#define HCI_EVENT 0x4
#define HCI_DATA 0x2

#define HCI_COMMAND_COMPLETE_EVENT	0xE
#define HCI_COMMAND_STATUS_EVENT	0xF
#define HCI_COMMAND_COMPLETE_READ	0xc
#define HCI_EVENT_CONNECTION_COMPL	0x3E
#define HCI_NUM_OF_COMP_PKT		0x13
#define HCI_EVENT_DISCONNECTION_COMPL	0x5

#define HCI_COMMAND_SET_EVENT_MASK_OPCODE	0x0c01
#define HCI_COMMAND_RESET_OPCODE		0x0c03
#define HCI_COMMAND_SET_EVENT_MASK_PAGE_OPCODE	0x0c63

#define HCI_COMMAND_LOCAL_VERSION_OPCODE	0x1001
#define HCI_COMMAND_LOCAL_SUP_FEAT_OPCODE	0x1003
#define HCI_COMMAND_READ_BUF_SIZE_OPCODE	0x1005
#define HCI_COMMAND_READ_BD_ADDR_OPCODE		0x1009


#define HCI_COMMAND_LE_SET_EVENT_MASK_OPCODE	0x2001
#define HCI_COMMAND_LE_READ_BUF_SIZE_OPCODE	0x2002
#define HCI_COMMAND_LE_READ_SUP_FEAT_OPCODE	0x2003
#define HCI_COMMAND_LE_SET_ADV_PARAMS_OPCODE	0x2006
#define HCI_COMMAND_LE_SET_ADV_DATA_OPCODE	0x2008
#define HCI_COMMAND_LE_SET_SCAN_RESP_OPCODE	0x2009
#define HCI_COMMAND_LE_SET_ADV_EN_OPCODE	0x200a
#define HCI_COMMAND_LE_READ_R_USED_F_OPCODE	0x2016
#define HCI_COMMAND_LE_ADD_DEV_RES_LIST_OPCODE  0x2027
#define HCI_COMMAND_LE_CLEAR_RES_LIST_OPCODE	0x2029
#define HCI_COMMAND_LE_SET_ADDR_RES_EN_OPCODE	0x202d
#define HCI_COMMAND_LE_SET_PRIVACY_MODE_OPCODE	0x204e

#define HCI_VERSION 0x8 // BT 4.2
#define HCI_REVISION_0 0xE
#define HCI_REVISION_1 0x3
#define HCI_LMP_PAL_VERSION 8
#define HCI_MAN_NAME_0 96
#define HCI_MAN_NAME_1 0x0
#define HCI_LMP_PAL_SUBVERSION_0 0xE
#define HCI_LMP_PAL_SUBVERSION_1 0x3


#define ATT_READ_GROUP_REQ 			0x10
#define ATT_READ_GROUP_RESP			0x11
#define ATT_READ_TYPE_REQ 			0x8
#define ATT_READ_TYPE_RESP			0x9
#define ATT_PREP_WRITE_REQ			0x16
#define ATT_PREP_WRITE_RESP			0x17
#define ATT_EXEC_WRITE_REQ			0x18
#define ATT_EXEC_WRITE_RESP			0x19
#define ATT_READ_REQ				0xa
#define ATT_READ_RESP				0xb

struct att_read_req
{
	uint8_t opcode;
	uint16_t handle;
} __attribute__((packed));

struct att_read_resp
{
	uint8_t opcode;
	uint8_t data[1];
} __attribute__((packed));


struct att_exec_write_req
{
	uint8_t opcode;
	uint8_t flags;
} __attribute__((packed));

struct att_prep_write_req
{
	uint8_t opcode;
	uint16_t handle;
	uint16_t offset;
	uint8_t data[1];
} __attribute__((packed));

struct att_prep_write_resp
{
	uint8_t opcode;
	uint16_t handle;
	uint16_t offset;
	uint8_t data[1];
} __attribute__((packed));

struct att_read_group
{
	uint8_t opcode;
	uint16_t start_handle;
	uint16_t end_handle;
	uint16_t group_type;
} __attribute__((packed));

struct att_data {
	uint16_t handle;
	uint16_t end_handle;
	uint16_t value;
} __attribute__((packed));

struct att_read_group_resp {
	uint8_t opcode;
	uint8_t len;
	struct att_data att_data[1];
} __attribute__((packed));

struct att_read_type
{
	uint8_t opcode;
	uint16_t start_handle;
	uint16_t end_handle;
	uint16_t att_type;
} __attribute__((packed));

struct att_data_r
{
	uint16_t handle;
	uint8_t unknown;
	uint16_t char_handle;
	uint16_t uuid;
} __attribute__((packed));

struct att_read_type_resp {
	uint8_t opcode;
	uint8_t len;
	struct att_data_r att_data[1];
} __attribute__((packed));

struct hci_command
{
	uint16_t opcode;
        uint8_t param_len;
	uint8_t param[1];
} __attribute__((packed));

struct hci_event
{
	uint8_t event_code;
	uint8_t len;
	uint8_t num_pkt;
	uint16_t opcode;
	uint8_t param[1];
} __attribute__((packed));

struct hci_event2
{
	uint8_t event_code;
	uint8_t len;
	uint8_t param[1];
} __attribute__((packed));

struct hci_status
{
	uint8_t event_code;
	uint8_t len;
	uint8_t status;
	uint8_t num_pkt;
	uint16_t opcode;
} __attribute__((packed));

struct acl {
	uint16_t handle;
	uint16_t len;
} __attribute__((packed));

struct l2cap {
	uint16_t len;
	uint16_t cid;
} __attribute__((packed));


uint8_t event_mask[8];
uint8_t le_event_mask[8];
uint8_t event_mask_page[8];
uint8_t bd_addr[6] = { 0x84, 0x0d, 0x8e, 0xe6, 0x6c, 0xd2};
uint8_t peer_addr[6] = { 0xdc, 0, 0xb, 0x15, 0x22, 0x33};
uint8_t adv_data_len = 0;
uint8_t *adv_data[31];
uint8_t addr_res_enable = 0;
uint8_t adv_enable = 0;
uint8_t scan_resp_len = 0;
uint8_t scan_resp_data[31];
uint16_t adv_int_min;
uint16_t adv_int_max;
uint8_t adv_type;
uint8_t adv_own_addr_type;
uint8_t adv_peer_addr_type;
uint8_t adv_peer_addr[6];
uint8_t adv_chan_map;
uint8_t adv_filter_policy;
uint16_t connection = 0;

int nimble_sock = 0;
int host_device_connected = 0;

int esp_bt_host_tx(uint8_t *, int);

int ble_controller_rx_cmd(struct hci_command *);
int ble_controller_rx_evt(struct hci_event *);
int ble_controller_rx_acl(uint8_t *);
int ble_prep_write(uint8_t *, int);
int ble_exec_write(void);
int ble_read_req(void);
int ble_controller_rx(uint8_t *, int);
int ble_controller_read_type(uint16_t *, int);
int ble_controller_read_group(uint16_t *, int);
int ble_controller_feat_send(uint8_t *, int);
void ble_controller_disconnect(void);
int ble_controller_connect(void);
int ble_controller_init(void);
void *bt_socket_thread(void *arg);


int ble_controller_rx_cmd(struct hci_command *cmd) {
	struct hci_event *pkt;
	uint8_t tx_buf[128];

	memset(tx_buf, 0x0, sizeof(tx_buf));
	pkt = (struct hci_event *)&tx_buf[1];


	switch (cmd->opcode) {
	case HCI_COMMAND_RESET_OPCODE:
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->opcode = HCI_COMMAND_RESET_OPCODE;
		pkt->param[0] = 0;
	break;
	case HCI_COMMAND_LOCAL_VERSION_OPCODE:
		pkt->len = 12;
		pkt->num_pkt = 5;
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->opcode = HCI_COMMAND_LOCAL_VERSION_OPCODE;
		pkt->param[0] = 0;
		pkt->param[1] = HCI_VERSION;
		pkt->param[2] = HCI_REVISION_0;
		pkt->param[3] = HCI_REVISION_1;
		pkt->param[4] = HCI_LMP_PAL_VERSION;
		pkt->param[5] = HCI_MAN_NAME_0;
		pkt->param[6] = HCI_MAN_NAME_1;
		pkt->param[7] = HCI_LMP_PAL_SUBVERSION_0;
		pkt->param[8] = HCI_LMP_PAL_SUBVERSION_1;
	break;
	case HCI_COMMAND_LOCAL_SUP_FEAT_OPCODE:
		pkt->len = 12;
		pkt->num_pkt = 5;
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->opcode = HCI_COMMAND_LOCAL_SUP_FEAT_OPCODE;
		pkt->param[0] = 0;
		pkt->param[1] = 0xbf;
		pkt->param[2] = 0xee;
		pkt->param[3] = 0xcd;
		pkt->param[4] = 0xfe;
		pkt->param[5] = 0xdb;
		pkt->param[6] = 0xff;
		pkt->param[7] = 0x7b;
		pkt->param[8] = 0x87;
	break;
	case HCI_COMMAND_SET_EVENT_MASK_OPCODE:
	{
		int i;
		memcpy(event_mask, cmd->param, sizeof(event_mask));
		printf("Event_mask: ");
		for (i = 0; i < 8; i++) {
			printf("%01x ", event_mask[i]);
		} printf("\n");
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->opcode = HCI_COMMAND_SET_EVENT_MASK_OPCODE;
		pkt->param[0] = 0;
	}
	break;
	case HCI_COMMAND_SET_EVENT_MASK_PAGE_OPCODE:
	{
		int i;
		memcpy(event_mask_page, cmd->param, sizeof(event_mask_page));
		printf("Event_mask page: ");
		for (i = 0; i < 8; i++) {
			printf("%01x ", event_mask_page[i]);
		} printf("\n");
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->opcode = HCI_COMMAND_SET_EVENT_MASK_PAGE_OPCODE;
		pkt->param[0] = 0;
	}
	break;
	case HCI_COMMAND_LE_SET_EVENT_MASK_OPCODE:
	{
		int i;
		memcpy(le_event_mask, cmd->param, sizeof(le_event_mask));
		printf("Le Event_mask : ");
		for (i = 0; i < 8; i++) {
			printf("%01x ", le_event_mask[i]);
		} printf("\n");
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->opcode = HCI_COMMAND_LE_SET_EVENT_MASK_OPCODE;
		pkt->param[0] = 0;
	}
	break;
	case HCI_COMMAND_LE_READ_BUF_SIZE_OPCODE:
		pkt->len = 7;
		pkt->num_pkt = 5;
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->opcode = HCI_COMMAND_LE_READ_BUF_SIZE_OPCODE;
		pkt->param[0] = 0; //status
		pkt->param[1] = 0xfb;
		pkt->param[2] = 0;
		pkt->param[3] = 0xa;
	break;
	case HCI_COMMAND_LE_READ_SUP_FEAT_OPCODE:
	{
		int i;
		pkt->len = 12;
		pkt->num_pkt = 5;
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->opcode = HCI_COMMAND_LE_READ_SUP_FEAT_OPCODE;
		for (i = 0; i < 9; i++) {
			pkt->param[i] = 0x0;
		}
	}
	break;
	case HCI_COMMAND_READ_BUF_SIZE_OPCODE:
	{
		int i;
		pkt->len = 11;
		pkt->num_pkt = 5;
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->opcode = HCI_COMMAND_READ_BUF_SIZE_OPCODE;
		for (i = 0; i < 8; i++) {
			pkt->param[i] = 0x0;
		}
	}
	break;
	case HCI_COMMAND_READ_BD_ADDR_OPCODE:
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->len = 10;
		pkt->num_pkt = 5;
		pkt->opcode = HCI_COMMAND_READ_BD_ADDR_OPCODE;
		pkt->param[0] = 0x0;
		memcpy(&pkt->param[1], bd_addr, sizeof(bd_addr));
	break;
	case HCI_COMMAND_LE_SET_ADV_DATA_OPCODE:
		if (cmd->param_len < 32) {
			printf("Set Advertising Data len is invalid %d\n", cmd->param_len);
			return -1;
		}
		if (cmd->param[0] > 31) {
			printf("Set Advertising Data_Len is invalid %d\n", cmd->param[0]);
			return -1;
		}

		adv_data_len = cmd->param[0];
		memcpy(adv_data, &cmd->param[1], adv_data_len);

		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->opcode = HCI_COMMAND_LE_SET_ADV_DATA_OPCODE;
		pkt->param[0] = 0x0;
	break;
	case HCI_COMMAND_LE_SET_ADDR_RES_EN_OPCODE:
		addr_res_enable = cmd->param[0];
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->opcode = HCI_COMMAND_LE_SET_ADDR_RES_EN_OPCODE;
		pkt->param[0] = 0x0;
	break;
	case HCI_COMMAND_LE_CLEAR_RES_LIST_OPCODE:
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->opcode = HCI_COMMAND_LE_CLEAR_RES_LIST_OPCODE;
		pkt->param[0] = 0x0;
	break;
	case HCI_COMMAND_LE_SET_ADV_EN_OPCODE:
		adv_enable = cmd->param[0];
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->opcode = HCI_COMMAND_LE_SET_ADV_EN_OPCODE;
		pkt->param[0] = 0x0;
	break;
	case HCI_COMMAND_LE_ADD_DEV_RES_LIST_OPCODE:
	{
		uint8_t peer_id_type = cmd->param[0];
		uint8_t * peer_id_addr = (uint8_t *) &cmd->param[1];
		uint8_t * peer_irk = (uint8_t *) &cmd->param[7];
		uint8_t * local_irk = (uint8_t *) &cmd->param[23];
		int i;

		printf("peer_id_type %x\n", peer_id_type);
		printf("Peer ID address: ");
		for (i = 0; i < 6; i++) {
			printf(" %01x ", peer_id_addr[i]);
		} printf("\n");

		printf("Peer IRK: ");
		for (i = 0; i < 16; i++) {
			printf(" %01x ", peer_irk[i]);
		} printf("\n");

		printf("Local IRK: ");
		for (i = 0; i < 16; i++) {
			printf(" %01x ", local_irk[i]);
		} printf("\n");

		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->opcode = HCI_COMMAND_LE_ADD_DEV_RES_LIST_OPCODE;
		pkt->param[0] = 0x0;
	}
	break;
	case HCI_COMMAND_LE_SET_PRIVACY_MODE_OPCODE:
		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->opcode = HCI_COMMAND_LE_SET_PRIVACY_MODE_OPCODE;
		pkt->param[0] = 0x1;
	break;
	case HCI_COMMAND_LE_SET_SCAN_RESP_OPCODE:
		if (cmd->param_len < 32) {
			printf("SET Scan Resp Data len is invalid %d\n", cmd->param_len);
			return -1;
		}
		if (cmd->param[0] > 31) {
			printf("Set Scan Resp Data_Len is invalid %d\n", cmd->param[0]);
			return -1;
		}

		scan_resp_len = cmd->param[0];
		memcpy(scan_resp_data, &cmd->param[1], scan_resp_len);

		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->opcode = HCI_COMMAND_LE_SET_SCAN_RESP_OPCODE;
		pkt->param[0] = 0x0;
	break;
	case HCI_COMMAND_LE_SET_ADV_PARAMS_OPCODE:
		adv_int_min = *((uint16_t *) &cmd->param[0]);
		adv_int_max = *((uint16_t *) &cmd->param[2]);
		adv_type = cmd->param[4];
		adv_own_addr_type = cmd->param[5];
		adv_peer_addr_type = cmd->param[6];
		memcpy(adv_peer_addr, &cmd->param[7], sizeof(adv_peer_addr));
		adv_chan_map = cmd->param[13];
		adv_filter_policy = cmd->param[14];

		pkt->event_code = HCI_COMMAND_COMPLETE_EVENT;
		pkt->len = 4;
		pkt->num_pkt = 5;
		pkt->opcode = HCI_COMMAND_LE_SET_ADV_PARAMS_OPCODE;
		pkt->param[0] = 0x0;
	break;
	case HCI_COMMAND_LE_READ_R_USED_F_OPCODE:
	{
		struct hci_status *s = (struct hci_status *)&tx_buf[1];;
		uint16_t *c = (uint16_t*)&cmd->param[0];
		if (*c != connection) {
			printf("Connection handle is invalid %x\n", *c);
			return -1;
		}

		tx_buf[0] = HCI_EVENT;

		s->event_code = HCI_COMMAND_STATUS_EVENT;
		s->len = 4;
		s->num_pkt = 5;
		s->opcode = HCI_COMMAND_LE_READ_R_USED_F_OPCODE;
		s->status = 0x0;

		if (esp_bt_host_tx(tx_buf, s->len+2+1) < 0) {
			printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
                        return -1;
		}

		if (!nimble_sock) {
			printf("Nimble_sock is closed! Abort!\n");
			return -1;
		}

		if (send(nimble_sock, "R_USED_FEAT", 11, 0) < 0) {
			printf("Nimble_sock is closed! Abort!\n");
			return -1;
		}

		return 0;
	}
	break;
	default:
		printf("Received unknown command opcode %06x. Doesn't support; skipping\n",cmd->opcode);
		return -1;
	}


	tx_buf[0] = HCI_EVENT;

	if (esp_bt_host_tx(tx_buf, pkt->len+2+1) < 0) {
		printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
		return -1;
	}

	return 0;
}

int ble_controller_rx_evt(struct hci_event *evt)
{
	printf("Received HCI event. Doesn't support; skipping\n");
	return 0;
}

// Settings UUID to read all the CHARS
#define GENERIC_SERVICES 0x1704
struct att_data services_att = { 0, };

// Settings UUID to read TAP Result.
#define BLE_GATTS_CHAR_UUID_READ 0x1706
struct att_data read_att = { 0, };

// Settings UUID to write TAP Result.
#define BLE_GATTS_CHAR_UUID_WRITE 0x1705
struct att_data write_att = { 0, };

int data_sent = 0;
int data_sent_len = 0;
int data_sent_done = 0;
int data_sent_resp = 0;

int ble_controller_rx_acl(uint8_t *data)
{
	uint8_t tx_buf[128];

	memset(tx_buf, 0x0, sizeof(tx_buf));
	struct hci_event *pkt = (struct hci_event *)&tx_buf[1];

	struct acl *acl = (struct acl *) &data[0];
	if (acl->handle != 0x0)
	{
		printf("ERROR: ACL TX invalid handle %x\n", acl->handle);
		return -1;
	}

	printf("acl->len = %d\n", acl->len);


	struct l2cap *l2cap = (struct l2cap *)&data[4];
	if (l2cap->cid != 0x0004) {
		printf("ERROR: L2CAP TX invalid CID %x\n", l2cap->cid);
		return -1;
	}

	switch(data[8]) {
	case ATT_READ_GROUP_RESP:
	{
		struct att_read_group_resp *att= (struct att_read_group_resp *)&data[8];
		if (att->len != 0x06) {
			printf("ERROR: TX ATT invalid LEN %x\n", att->len);
			return -1;
		}

		int num = (l2cap->len - 2) / att->len;
		printf("->>> ATT_READ_GROUP_RESP:  num %d\n",num);

		struct att_data * atd = ( struct att_data * ) &att->att_data[0];
		while ( num != 0) {
			if (atd->value == GENERIC_SERVICES) {
				services_att.handle = atd->handle;
			}
			atd++; num--;
		}

		if (!services_att.handle) {
			printf("ERROR: GENERIC_SERVICES: handle  not found %x\n", atd->handle);
			if (send(nimble_sock, "NOTFND", 6, 0) < 0) {
				printf("ATT_READ_GROUP_RESP: send() failed\n");
				return -1;
			}
		} else {
			if (send(nimble_sock, &services_att.handle, sizeof(services_att.handle), 0) < 0) {
				printf("ATT_READ_GROUP_RESP: send() failed\n");
				return -1;
			}
		}
		pkt->event_code = HCI_NUM_OF_COMP_PKT;
		pkt->len = 5;
		pkt->num_pkt = 1;
		pkt->opcode = 0x0000; //handle
		pkt->param[0] = 0x1;
		pkt->param[1] = 0x0;
	}
	break;
	case ATT_READ_TYPE_RESP:
	{
		struct att_read_type_resp *att= (struct att_read_type_resp *)&data[8];
		if (att->len != 0x07) {
			printf("ERROR: TX ATT invalid LEN %x\n", att->len);
			return -1;
		}

		int num = (l2cap->len - 2) / att->len;
                printf("->>> ATT_READ_TYPE_RESP:  num %d\n",num);

                struct att_data_r * atd = ( struct att_data_r * ) &att->att_data[0];
                while ( num != 0) {
                        if (atd->uuid == BLE_GATTS_CHAR_UUID_READ) {
                                read_att.handle = atd->char_handle;
				printf("READ_SERVICES: handle found %x\n", read_att.handle);
				if (send(nimble_sock, &atd->char_handle, sizeof(atd->char_handle), 0) < 0) {
					printf("ATT_READ_TYPE_RESP: send() failed\n");
					return -1;
				}
                        } else if (atd->uuid == BLE_GATTS_CHAR_UUID_WRITE) {
                                write_att.handle = atd->char_handle;
				printf("WRITE_SERVICES: handle found %x\n", write_att.handle);
				if (send(nimble_sock, &atd->char_handle, sizeof(atd->char_handle), 0) < 0) {
					printf("ATT_READ_TYPE_RESP: send() failed\n");
					return -1;
				}
			}
                        atd++; num--;
                }

                pkt->event_code = HCI_NUM_OF_COMP_PKT;
                pkt->len = 5;
                pkt->num_pkt = 1;
                pkt->opcode = 0x0000; //handle
                pkt->param[0] = 0x1;
                pkt->param[1] = 0x0;
	}
	break;
	case ATT_PREP_WRITE_RESP:
	{
		struct att_prep_write_resp *atw= (struct att_prep_write_resp *)&data[8];
		printf("->> ATT_PREP_WRITE_RESP: %x handle %x offset data: %s\n", atw->handle, atw->offset, atw->data);
		if (send(nimble_sock, &atw->data[0], l2cap->len-5, 0) < 0) {
			printf("ATT_PREP_WRITE_RESP: send() failed\n");
			return -1;
		}
                pkt->event_code = HCI_NUM_OF_COMP_PKT;
                pkt->len = 5;
                pkt->num_pkt = 1;
                pkt->opcode = 0x0000; //handle
                pkt->param[0] = 0x1;
                pkt->param[1] = 0x0;
	}
	break;
	case ATT_EXEC_WRITE_RESP:
		data_sent_resp = 1;
		printf("->> ATT_EXEC_WRITE_RESP\n");
		if (send(nimble_sock, "OK", 2, 0) < 0) {
			printf("ATT_EXEC_WRITE_RESP: send() failed\n");
			return -1;
		}
                pkt->event_code = HCI_NUM_OF_COMP_PKT;
                pkt->len = 5;
                pkt->num_pkt = 1;
                pkt->opcode = 0x0000; //handle
                pkt->param[0] = 0x1;
                pkt->param[1] = 0x0;
	break;
	case ATT_READ_RESP:
	{
		if (nimble_sock == 0) {
			printf("ARR_READ_RESP: nimble_sock closed!\n");
			return -1;
		}

		struct att_read_resp *att = (struct att_read_resp *)&data[8];

		if (send(nimble_sock, &att->data[0], l2cap->len-1, 0) < 0) {
			printf("ARR_READ_RESP: send() failed\n");
			return -1;
		}
                pkt->event_code = HCI_NUM_OF_COMP_PKT;
                pkt->len = 5;
                pkt->num_pkt = 1;
                pkt->opcode = 0x0000; //handle
                pkt->param[0] = 0x1;
                pkt->param[1] = 0x0;
	}
	break;
	default:
		printf("Unknown ATT opcode %x\n", data[0]);
		return -1;
	}

	tx_buf[0] = HCI_EVENT;

	if (esp_bt_host_tx(tx_buf, pkt->len+2+1) < 0) {
		printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
		return -1;
	}

	return 0;
}

int ble_prep_write(uint8_t *data, int len) {
	uint8_t tx_buf[128];
	struct acl *acl;
	struct l2cap *l2cap;

	if (!host_device_connected) {
		return -1;
	}

	data_sent_resp = 0;

	memset(tx_buf, 0x0, sizeof(tx_buf));

	if (read_att.handle && write_att.handle) {
		// Prepare Write Request
		tx_buf[0] = HCI_DATA;
		acl = (struct acl *) &tx_buf[1];
		acl->handle = 0x2000;

		l2cap = (struct l2cap *)&tx_buf[5];
		l2cap->cid = 0x0004; //ATT

		struct att_prep_write_req *atw = (struct att_prep_write_req *) &tx_buf[9];
		atw->opcode = ATT_PREP_WRITE_REQ;
		atw->handle = write_att.handle;
		atw->offset = data_sent;

		acl->len = sizeof(struct l2cap) + sizeof(struct att_prep_write_req) - 1 + len;
		l2cap->len = sizeof(struct att_prep_write_req) - 1 + len;

		memcpy(atw->data, data, len);
		data_sent+=len;
		printf("acl->len %x Sending %01x %01x %01x\n",  acl->len,atw->data[0], atw->data[1], atw->data[2]);


		if (esp_bt_host_tx(tx_buf, acl->len+sizeof(struct acl)+1) < 0) {
			printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
                        return -1;
                }
	}

	return 0;
}


int ble_exec_write(void) {
	uint8_t tx_buf[128];
	struct acl *acl;
	struct l2cap *l2cap;

	if (!host_device_connected || !write_att.handle) {
		return -1;
	}

	memset(tx_buf, 0x0, sizeof(tx_buf));
	tx_buf[0] = HCI_DATA;

	acl = (struct acl *) &tx_buf[1];
	acl->handle = 0x2000;
	acl->len = sizeof(struct l2cap) + sizeof(struct att_exec_write_req);

	l2cap = (struct l2cap *)&tx_buf[5];
	l2cap->len = sizeof(struct att_exec_write_req);
	l2cap->cid = 0x0004; //ATT

	struct att_exec_write_req *atx = (struct att_exec_write_req *) &tx_buf[9];
	atx->opcode = ATT_EXEC_WRITE_REQ;
	atx->flags = 0x01;

	data_sent = 0;

	if (esp_bt_host_tx(tx_buf, acl->len+sizeof(struct acl)+1) < 0) {
		printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
		return -1;
	}
	return 0;
}


int ble_read_req(void)
{
	uint8_t tx_buf[128];
	struct acl *acl;
	struct l2cap *l2cap;

	if (!host_device_connected || !read_att.handle || !data_sent_resp) {
                return -1;
        }

	memset(tx_buf, 0x0, sizeof(tx_buf));

	// Prepare Read Request
	tx_buf[0] = HCI_DATA;
	acl = (struct acl *) &tx_buf[1];
	acl->handle = 0x2000;
	acl->len = sizeof(struct l2cap) + sizeof(struct att_read_req);

	l2cap = (struct l2cap *)&tx_buf[5];
	l2cap->len = sizeof(struct att_read_req);
	l2cap->cid = 0x0004; //ATT

	struct att_read_req *atw = (struct att_read_req *) &tx_buf[9];
	atw->opcode = ATT_READ_REQ;
	atw->handle = read_att.handle;

	if (esp_bt_host_tx(tx_buf, acl->len+sizeof(struct acl)+1) < 0) {
		printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
		return -1;
	}
	printf("ATT_READ_REQ was sent!\n");

	return 0;
}

int ble_controller_rx(uint8_t *data, int len)
{
	int ret = -1;

	printf("OK. Data received.\n");
	printf("Packet type: %03x\n", data[0]);
	switch(data[0]) {
	case HCI_COMMAND:
		ret = ble_controller_rx_cmd((struct hci_command *) &data[1]);
		break;
	case HCI_EVENT:
		ret = ble_controller_rx_evt((struct hci_event *) &data[1]);
		break;
	case HCI_DATA:
		ret = ble_controller_rx_acl(&data[1]);
		break;
	default:
		printf("Received unknown packet type %03x. Doesn't support; skipping\n",data[0]);
		break;
	}

	return ret;
}

int ble_controller_read_type(uint16_t *data, int len)
{
	uint8_t tx_buf[128];
	//Send ACL

	if (len != 6) {
		printf("%s: failed. Length is invalid (%d)!\n", __FUNCTION__, len);
		return -EINVAL;
	}

	memset(tx_buf, 0x0, sizeof(tx_buf));

        tx_buf[0] = HCI_DATA;
        struct acl *acl = (struct acl *) &tx_buf[1];
        acl->handle = 0x2000;
        acl->len = sizeof(struct l2cap) + sizeof(struct att_read_type);

        struct l2cap *l2cap = (struct l2cap *)&tx_buf[5];
	l2cap->len = sizeof(struct att_read_type);
	l2cap->cid = 0x0004; //ATT

	struct att_read_type *att= (struct att_read_type *)&tx_buf[9];
	att->opcode = ATT_READ_TYPE_REQ;
/*
	if (!read_att.handle) {
		att->start_handle = 0x001f;
	} else if (!write_att.handle ) {
		att->start_handle = 0x0026;
	}
	att->end_handle = 0xffff;
	att->att_type = 0x2803;
*/
	att->start_handle = data[0];
	att->end_handle = data[1];
	att->att_type = data[2];
	printf("start_handle %02x end_handle %02x group_type %02x\n", att->start_handle, att->end_handle, att->att_type);

	printf("->>> ACL: start_handle %x\n", att->start_handle);

	if (esp_bt_host_tx(tx_buf, acl->len+sizeof(struct acl)+1) < 0) {
		printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
		return -1;
	}

	return 0;
}

int ble_controller_read_group(uint16_t *data, int len)
{
	uint8_t tx_buf[128];
	//Send ACL

	if (len != 6) {
		printf("%s: failed. Length is invalid (%d)!\n", __FUNCTION__, len);
		return -EINVAL;
	}

	memset(tx_buf, 0x0, sizeof(tx_buf));

	tx_buf[0] = HCI_DATA;
	struct acl *acl = (struct acl *) &tx_buf[1];
	acl->handle = 0x2000; // set PB bit; handle is 0
	acl->len = sizeof(struct l2cap) + sizeof(struct att_read_group);

	struct l2cap *l2cap = (struct l2cap *)&tx_buf[5];
	l2cap->len = sizeof(struct att_read_group);
	l2cap->cid = 0x0004; //ATT

	struct att_read_group *att= (struct att_read_group *)&tx_buf[9];
	att->opcode = ATT_READ_GROUP_REQ;
	att->start_handle = data[0];//0x0001;
	att->end_handle = data[1];//0xffff;
	att->group_type = data[2];//0x2800;
	printf("start_handle %02x end_handle %02x group_type %02x\n", att->start_handle, att->end_handle, att->group_type);

	if (esp_bt_host_tx(tx_buf, acl->len+sizeof(struct acl)+1) < 0) {
		printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
		return -1;
	}

	return 0;
}

#define BT_LE_SUPP_FEAT_LEN	8

int ble_controller_feat_send(uint8_t *data, int len)
{
	uint8_t tx_buf[128];

	memset(tx_buf, 0x0, sizeof(tx_buf));

	tx_buf[0] = HCI_EVENT;

	if (len != BT_LE_SUPP_FEAT_LEN) {
		return -EINVAL;
	}

	struct hci_event2 *evt = (struct hci_event2 *)&tx_buf[1];

	evt->event_code = HCI_EVENT_CONNECTION_COMPL;
	evt->len = 12; // code + status + handle + features
        evt->param[0] = 0x4; // subevent code
	evt->param[1] = 0x0; // status
	uint16_t *c = ((uint16_t*) &evt->param[2]);
	*c = connection;
	memcpy(&evt->param[4], data, len);
	int i;
	for (i = 0; i < BT_LE_SUPP_FEAT_LEN; i++) {
		printf("Feature: data[%d] %01x\n",i, data[i]);
	}

	if (esp_bt_host_tx(tx_buf, evt->len+2+1) < 0) {
		printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
		return -1;
	}

	return 0;
}

void ble_controller_disconnect(void)
{
	uint8_t tx_buf[128];

	if (!host_device_connected)
		return;

	memset(tx_buf, 0x0, sizeof(tx_buf));

	tx_buf[0] = HCI_EVENT;
	struct hci_event2 *evt  = (struct hci_event2 *)&tx_buf[1];
	evt->event_code = HCI_EVENT_DISCONNECTION_COMPL;
	evt->len = 4;
	evt->param[0] = 0x0; // status
	evt->param[1] = 0x0; // handle
	evt->param[2] = 0x0; // handle
	evt->param[3] = 0x16; // terminated by host

	if (esp_bt_host_tx(tx_buf, evt->len+2+1) < 0) {
		printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
		return;
        }

	host_device_connected = 0;
}

int ble_controller_connect(void)
{
	uint8_t tx_buf[128];

	if (!adv_enable) {
		return -EAGAIN;
	}

	memset(tx_buf, 0x0, sizeof(tx_buf));

	tx_buf[0] = HCI_EVENT;
	struct hci_event2 *evt  = (struct hci_event2 *)&tx_buf[1];
	int i;

	evt->event_code = HCI_EVENT_CONNECTION_COMPL;
	evt->len = 31;
	evt->param[0] = 0xA; // subevent code
	evt->param[1] = 0x0; // status
	evt->param[2] = 0x0; // connection handle
	evt->param[3] = 0x0; // connection handle

	connection = 0;

	evt->param[4] = 0x1; // slave
	evt->param[5] = 0x1; // random address type
	memcpy(&evt->param[6], peer_addr, sizeof(peer_addr));
	// local res addr
	for (i = 12; i < 18; i++) {
		evt->param[i] = 0x0;
	}
	// peer res addr
	for (i = 18; i < 24; i++) {
		evt->param[i] = 0x0;
	}

	evt->param[24] = 0x18; // con int
	evt->param[25] = 0x0; //con interval
	evt->param[26] = 0x0; // latency
	evt->param[27] = 0x0; // latency
	evt->param[28] = 0x48; //timeout
	evt->param[29] = 0x0; //timeout
	evt->param[30] = 0x1; // clock

	if (esp_bt_host_tx(tx_buf, evt->len+2+1) < 0) {
		printf("send() failed: %s %d\n", __FUNCTION__, __LINE__);
		return -1;
	}

	host_device_connected = 1;
	printf("Send Connection Event OK\n");

	return 0;
}

int ble_controller_init(void)
{
	memset(event_mask, 0x0, sizeof(event_mask));
	memset(le_event_mask, 0x0, sizeof(le_event_mask));
	memset(event_mask_page, 0x0, sizeof(event_mask_page));

	return 0;
}


#define BT_LE_SEND_R_FEAT 0x10
#define BT_LE_READ_GROUP  0x20
#define BT_LE_READ_TYPE	  0x30

#define BT_LE_PREP_WRITE 0x55
#define BT_LE_EXEC_WRITE 0x56
#define BT_LE_READ_REQ	0x57

#define CLIENT_NIMBLE "NIMBLE"

#define MAX_EVENTS 10

#define BT_PORT		8883

void *bt_socket_thread(void *arg)
{
	struct sockaddr_in server;
	struct epoll_event ev, events[MAX_EVENTS];
	int listen_sock, nfds, epollfd;
	int enable = 1;

	listen_sock = socket(AF_INET, SOCK_STREAM|SOCK_NONBLOCK, 0);
	if (listen_sock < 0) {
		printf("Cannot open socket\n");
		return (void*)-1;
	}

	if (setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
	{
		printf("setsockopt(SO_REUSEADDR) failed");
		return (void*)-1;
	}

	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_family = AF_INET;
	server.sin_port = htons(BT_PORT);

	if (bind(listen_sock, (struct sockaddr *) &server, sizeof(server)) < 0) {
		printf("bind() failed\n");
		return (void*)-1;
	}

	listen(listen_sock, 1);

        epollfd = epoll_create1(0);
        if (epollfd < 0) {
               perror("epoll_create failed");
               return (void*)-1;
        }

        ev.events = EPOLLIN;
        ev.data.fd = listen_sock;
        if (epoll_ctl(epollfd, EPOLL_CTL_ADD, listen_sock, &ev) < 0) {
               perror("epoll_ctl: listen_sock");
               return (void*)-1;
        }

	nimble_sock = 0;

	printf("Server setup success on port %d. Starting listenning for incomming requests.\n", BT_PORT);

	while ( 1 )
	{
		int n;

		nfds = epoll_wait(epollfd, events, MAX_EVENTS, -1);
                if (nfds == -1) {
			perror("epoll_wait");
			return (void*)-1;
                }

		for (n = 0; n < nfds; ++n) {
			if (events[n].data.fd == listen_sock) {
				struct sockaddr_in client;
				unsigned int len = sizeof(client);
				int sock = accept4(listen_sock, (struct sockaddr *) &client, &len, SOCK_NONBLOCK);
				if (sock < 0) {
					printf("accept() failed\n");
					return (void*)-1;
				}
				printf("New connection: %s\n", inet_ntoa(client.sin_addr));

				if (nimble_sock) {
					printf("Nimble and test was already connected. Skipping.");
					close(nimble_sock);
					continue;
				}

				ev.events = EPOLLIN | EPOLLET;
				ev.data.fd = sock;
				if (epoll_ctl(epollfd, EPOLL_CTL_ADD, sock, &ev) == -1) {
					perror("epoll_ctl: nimble_sock");
					return (void*)-1;
				}
				printf("Wait for ID packet...\n");
			} else if (events[n].data.fd == nimble_sock) {
				uint8_t buf[128];
				memset(buf, 0x0, sizeof(buf));

				if (events[n].events & EPOLLERR) {
					printf("test sock: disconnected.\n");
					if (epoll_ctl(epollfd, EPOLL_CTL_DEL, nimble_sock, &ev) == -1) {
                                                perror("epoll_ctl: nimble_sock");
                                                return (void*)-1;
                                        }
					close(nimble_sock);
					nimble_sock = 0;
					continue;
				}

				int size = recv(nimble_sock, buf, sizeof(buf), MSG_DONTWAIT);
				if (size <= 0) {
					printf("nimble_sock: recv() failed\n");
					printf("nimble sock: disconnected.\n");
					if (epoll_ctl(epollfd, EPOLL_CTL_DEL, nimble_sock, &ev) == -1) {
                                                perror("epoll_ctl: nimble_sock");
                                                return (void*)-1;
                                        }
					close(nimble_sock);
					nimble_sock = 0;
					ble_controller_disconnect();
					continue;
				}
				printf("OK. Data received %d.\n", size);
				printf("Packet type: %03x\n", buf[0]);
				switch(buf[0]) {
				case BT_LE_SEND_R_FEAT:
					if (ble_controller_feat_send(&buf[1], size-1) < 0) {
						if (send(nimble_sock, "EINVAL", 6, 0) < 0) {
							printf("Nimble_Sock: Send() failed.\n");
							continue;
						}
					}
					if (send(nimble_sock, "OK", 2, 0) < 0) {
						printf("Nimble_Sock: Send() failed.\n");
						continue;
					}
				break;
				case BT_LE_READ_GROUP:
					if (ble_controller_read_group((uint16_t *)&buf[1], size-1) < 0) {
						if (send(nimble_sock, "EINVAL", 6, 0) < 0) {
							printf("Nimble_Sock: Send() failed.\n");
							continue;
						}
					}
				break;
				case BT_LE_READ_TYPE:
					if (ble_controller_read_type((uint16_t *)&buf[1], size-1) < 0) {
						if (send(nimble_sock, "EINVAL", 6, 0) < 0) {
							printf("Nimble_Sock: Send() failed.\n");
							continue;
						}
					}
				break;
				case BT_LE_PREP_WRITE:
					printf("nimble_sock: New data to send to host\n");
					if (ble_prep_write(&buf[1], size-1) < 0) {
						if (send(nimble_sock, "EINVAL", 6, 0) < 0) {
							printf("Nimble_Sock: Send() failed.\n");
							continue;
						}
					}
					// response will be send back when host respond
				break;
				case BT_LE_EXEC_WRITE:
					printf("nimble_sock: New data to send to host\n");
					if (ble_exec_write() < 0) {
						if (send(nimble_sock, "EINVAL", 6, 0) < 0) {
							printf("Nimble_Sock: Send() failed.\n");
							continue;
						}
					}
					// response will be send back when host respond
				break;
				case BT_LE_READ_REQ:
					printf("nimble_sock: Read result from host\n");

					if (ble_read_req() < 0) {
						if (send(nimble_sock, "EAGAIN", 6, 0) < 0) {
							printf("Nimble_Sock: Send() failed.\n");
							continue;
						}
					}
					// response will be send back when host respond
				break;
				default:
					printf("Test: Received unknown packet type %03x. Doesn't support; skipping\n",buf[0]);
                                        continue;
				}
			} else {
				uint8_t buf[32];
				int size = 0;
				int sock = events[n].data.fd;

				size = recv(sock, buf, sizeof(buf), MSG_DONTWAIT);
				if (size <= 0) {
					printf("No data recevied. Closing socket...\n");
					close(events[n].data.fd);
					if (epoll_ctl(epollfd, EPOLL_CTL_DEL, sock, &ev) == -1) {
						perror("epoll_ctl: sock");
						return (void*)-1;
					}
					continue;
				}
				if (size > strlen(CLIENT_NIMBLE)) {
					printf("Too many bytes received in ID packet! nbytes %d data: %s\n",
							size, buf);
					printf("Closing socket...\n");
					if (epoll_ctl(epollfd, EPOLL_CTL_DEL, sock, &ev) == -1) {
						perror("epoll_ctl: sock");
						return (void*)-1;
					}
					close(sock);
					continue;
				}
				if (!strncmp(CLIENT_NIMBLE, (char *)buf, 6)) {
					nimble_sock = sock;

					if (host_device_connected) {
						if (send(nimble_sock, "ALREADY_PAIRED", 14, 0) < 0) {
                                                        printf("Nimble_Sock: Send() failed.\n");
                                                        continue;
                                                }
						continue;
					}

					if (ble_controller_connect() < 0) {
						printf("ble_controller_connect() failed. Closing socket...\n");
						if (epoll_ctl(epollfd, EPOLL_CTL_DEL, sock, &ev) == -1) {
							perror("epoll_ctl: sock");
							return (void*)-1;
						}
						if (send(nimble_sock, "EAGAIN", 6, 0) < 0) {
							printf("Nimble_Sock: Send() failed.\n");
							continue;
						}
						close(nimble_sock);
						nimble_sock = 0;
						continue;
					}
					if (send(nimble_sock, "OK", 2, 0) < 0) {
						printf("Nimble_sock: Send() failed.\n");
						continue;
					}
					printf("Connected with BLE HOST\n");
				} else {
					printf("Unknown client ID: %s\n", buf);
					printf("Closing socket...\n");
		                        if (epoll_ctl(epollfd, EPOLL_CTL_DEL, sock, &ev) == -1) {
						perror("epoll_ctl: sock");
						return (void*)-1;
					}
					close(sock);
		                        continue;
				}
			}
		}
	}

	if (nimble_sock)
		close(nimble_sock);
	close (listen_sock);

	return (void *)NULL;
}
