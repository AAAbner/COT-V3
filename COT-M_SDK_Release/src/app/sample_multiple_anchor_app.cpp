/**
* @file         sample_app.cpp
* @brief        This is a brief description.
* @details      This is the detail description.
* @author       author
* @date     	date
* @version  	A001
* @par Copyright (c):  Copyright (c) 2017 by COTiot Instruments, Inc.
* @par History:
*   version: author, date, desc\n
*/

/*********************************************************************
 * INCLUDE
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "am_hal_systick.h"
#include "am_hal_gpio.h"
#include "am_util_delay.h"

#include "error.h"
#include "osal.h"
#include "osal_time.h"
#include "osal_tasks.h"
#include "osal_pwrmgr.h"
#include "gpio.h"
#include "log.h"
#include "data_package.h"
#include "rf_param_type_define.h"
#include "cot_mac.h"
#include "RfParamConfig.h"
#include "af.h"
#include "cot_mt.h"
#include "cot_mt_task.h"
#include "cot_ranging.h"
#include "sample_app.h"
/*********************************************************************
 * MACROS
 */
#define MSG(FORMAT, ARG...) /* message that is destined to the user */
#define MSG_DEBUG(FLAG, fmt, ...)                                                                         \
            do  {                                                                                         \
                if (FLAG)                                                                                 \
                {\
                    log_printf("%s:%d:%s(): " ,__FILE__, __LINE__, __FUNCTION__);\
					log_printf(fmt,##__VA_ARGS__);\
                }\
            } while (0)

#define MASTER_ROLE   //节点代码
//#define SLAVE_ROLE  //锚点代码

/*********************************************************************
 * CONSTANTS
 */
// sample msg  Events
#define SMAPLE_MSG_TX_EVENT                              			0x0002								//发送事件
#define SMAPLE_MSG_TX_TIMEOUT_EVENT                      			0x0004
#define SMAPLE_MSG_RX_TIMEOUT_EVENT                           		0x0008
#define SMAPLE_MSG_RX_ERROR_EVENT                       			0x0010
//#define SMAPLE_MSG_RNG_EVENT                          				0x0020
//#define SMAPLE_MSG_RANGING_DONE_EVENT                         		0x0040
//#define SMAPLE_MSG_RANGING_TIMEOUT_EVENT                            0x0080
#define SMAPLE_MSG_CONFIG_EVENT                                 	0x0100						//配置事件
#define SMAPLE_MSG_TX_DONE_EVENT                                 	0x0200
#define SMAPLE_MSG_RX_DONE_EVENT                                 	0x0400
#define SMAPLE_MSG_INIT_EVENT                                 		0x0800

#define SMAPLE_MSG_MASTER_DECE_TIMER_EVENT                          0x1000					//主机DECE_TIMER事件
#define SMAPLE_MSG_START_RNG_EVENT									0x2000													//开始测距事件
#define SMAPLE_MSG_HANDSHAKE_EVENT									0x4000													//处理握手事件


#define SMAPLE_MSG_HANDDATA_EVENT									0x0020			//自己加的事件，发送数据事件
#define SMAPLE_MSG_START_DATA_EVENT									0x0040			//自己加的事件，发送数据事件

#define SX1280_CHIP 0x00
#define SX1281_CHIP 0x01


/*!
 * \brief sample application cmd
 */
#define COT_SYSTEM_CMD_H                    0x00

#define SAMPLE_APP_CMD_HANDLE               0x01
#define SAMPLE_APP_CMD_BROADCAST            0x02					//0x02如果是进入广播帧
#define SAMPLE_APP_CMD_SET_MULTICAST_ID     0x03
#define SMAPLE_APP_CMD_SET_WORK_PARRAM      0x04

#define SMAPLE_APP_CMD_START_RANG           0x20					//0x20如果是开始测距帧
#define SMAPLE_APP_CMD_SEND_PKG             0x21
#define SMAPLE_APP_CMD_SEND_PKG_LOWPOWER    0x22

#define SMAPLE_APP_CMD_DISTANCE_DATA        0x40					//0x40如果是数据帧（自定义）


/*!
 * \brief cot_phy_payload_head_t;
 * it's len is 10 Byte((4byte src_dev_id)+(4byte dst_dev_id)+(1byte commad_h)+(1byte commad_l))
 */
#define COT_PHY_PAYLOAD_HEAD    10

#define STATUS_SUCCESS          0
#define STATUS_FAIL             1

#define COMFIRM_FRAME           1
#define UNCOMFIRM_FRAME         0

#define MASTER_REVE_ANCHOR_COUNT		5
#define MASTER_RNG_DEVICE_COUNT			3
/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
    //uint8_t                     src_dev_id[4];              //the device id of sender device id
    //uint8_t                     dst_dev_id[4];              //the device id of receiver device id
    uint8_t                     commad_h;                   //the high byte of commad				命令的高字节
    uint8_t                     commad_l;                   //the low byte of commad				命令的低字节
    uint8_t                     *data;                      //the payload data field				数据域
}cot_application_msg_t;

typedef struct
{
    osal_event_hdr_t            hdr;                    //osal event
    uint8_t  								src_task_id;			//the osal task id of the sendder
	RF_RadioPacketTypes_t       ModulationType;      	//modulation type
	int8_t                      rssi;                   //only effect when recevie
	cot_rf_hdr_t                rf_hdr;                 //only effect when send
    uint16_t                    arq_num;                //the num of retransmission
    uint8_t                     tx_window_time;         //tx window time
    uint16_t                    cot_application_msg_len;
    cot_application_msg_t       cot_application_msg;
}cot_sample_msg_t;

typedef struct device_node {
	uint32_t device_id;
	int8_t rssi; 
}BroadCast_Data;
/*********************************************************************
 * GLOBAL VARIABLES
 */



/*********************************************************************
 * EXTERNAL VARIABLES
 */



/*********************************************************************
 * EXTERNAL FUNCTIONS
 */



/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t s_sample_app_id = 0; ///<  sample application id for osal task id

static  uint8_t s_soft_version[4]={2,0,0,0};

static  uint8_t s_hard_ware_version[2]={1,0};

uint8_t  read_distance[2];						//两个字节存放读取的距离值

int8_t  read_rssi;				//两个字节存放读取的rssi

uint16_t uint16_distance;										//uint16_t型距离值

static ConfigData_t s_config_data;

//static bool g_start_rng_flag_master = false;
static bool g_start_rng_flag_slave = false;

static uint16_t g_start_rng_count = 0;

static uint16_t g_start_data_count = 0;		//自定义的 开始发送数据帧计数

static uint16_t g_device_id_count = 0;		//回复的锚点数量

BroadCast_Data g_device_id[5];						//数组长度为5，其中每一个代表广播帧数据 的ID的4个字节 和 RSSI值

static cot_mac_msg_t *s_cot_sample_msg_tx;
bool g_flag_send_frist = false;				// 新一轮开始标志

pkg_type_t g_pkg_type = UNCONFIRM;
bool g_start_rng_flag = false;				//	启动测距标志

bool g_start_data_flag = false;        //  自己加的 ，启动数据帧标志
static uint8_t g_send_count_t = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
/**
* @name: sample_rf_param_set
* This funtion is to initialization rf param
* @param[in]   task_id -- phy task id in osal.
* @param[out]  none
* @retval  ERR_SUCCESS  0
* @retval  do not deal the event >0
* @par identifier
*      reserve
* @par other
*      none
* @par ModifyBlog
*      create by zhangjh on 2017-05-10
*/
//static int32_t sample_rf_param_get(ConfigData_t ConfigData, PHYParamSettings_t *phy_param_setting );



static void mac_layer_system_message_process( cot_mac_msg_t *p_msg );							//mac层处理多任务系统消息函

/*********************************************************************
 * PROFILE CALLBACKS
 */



/*********************************************************************
 * PUBLIC FUNCTIONS
 */


void master_broadcast_data()																							//节点广播数据
{
	s_cot_sample_msg_tx->dst_addr = 0xFFFFFFFF;															//广播目的地址ID
	s_cot_sample_msg_tx->data = (uint8_t*)(s_cot_sample_msg_tx + 1);
	s_cot_sample_msg_tx->data[0] = 0x00;
	s_cot_sample_msg_tx->data[1] = 0x02;
	uint8_t temp[64];
	memset(temp,0x5a,64);						//将temp中每个值设为0x5a；
	
	//memcpy(&s_cot_sample_msg_tx->data[2],temp,64);
	s_cot_sample_msg_tx->data_len = 2;

}

void slave_broadcast_data(uint32_t device_id)						//锚点回复广播数据
{
	s_cot_sample_msg_tx->dst_addr = device_id;			//device_id为目的地址
	s_cot_sample_msg_tx->data = (uint8_t*)(s_cot_sample_msg_tx + 1);
	s_cot_sample_msg_tx->data[0] = 0x00;
	s_cot_sample_msg_tx->data[1] = 0x02;
	uint8_t temp[64];
	memset(temp,0x5a,64);
	
	//memcpy(&s_cot_sample_msg_tx->data[2],temp,64);
	s_cot_sample_msg_tx->data_len = 2;
	
}
void master_handshake_data(uint8_t count)									//节点处理握手数据？
{
	log_printf("\n%x\n",g_device_id[count].device_id);			//打印锚点ID
	s_cot_sample_msg_tx->dst_addr = g_device_id[count].device_id;
	s_cot_sample_msg_tx->data = (uint8_t*)(s_cot_sample_msg_tx + 1);
	s_cot_sample_msg_tx->data[0] = 0x00;
	s_cot_sample_msg_tx->data[1] = 0x20;				//0x20代表协商 通知某锚点开始测距
	uint8_t temp[64];
	memset(temp,0x5a,64);
	
	memcpy(&s_cot_sample_msg_tx->data[2],temp,2);
	s_cot_sample_msg_tx->data_len = 4;
}
void slave_handshake_data(uint32_t device_id)							//锚点处理握手数据？   技术支持说这个函数没有用到，因为是协商，不用交互
{
	s_cot_sample_msg_tx->dst_addr = device_id;
	s_cot_sample_msg_tx->data = (uint8_t*)(s_cot_sample_msg_tx + 1);
	s_cot_sample_msg_tx->data[0] = 0x00;
	s_cot_sample_msg_tx->data[1] = 0x20;
	
	uint8_t temp[64];
	memset(temp,0x5a,64);
	
	memcpy(&s_cot_sample_msg_tx->data[2],temp,2);
	s_cot_sample_msg_tx->data_len = 4;	
}


//自己新增的函数
void master_handdisance_data(uint8_t count)												//节点处理数据帧
{
	log_printf("\n%x\n",g_device_id[count].device_id);
	s_cot_sample_msg_tx->dst_addr = g_device_id[count].device_id;
	s_cot_sample_msg_tx->data = (uint8_t*)(s_cot_sample_msg_tx + 1);
	s_cot_sample_msg_tx->data[0] = 0x00;
	s_cot_sample_msg_tx->data[1] = 0x40;				//0x20代表协商 通知某锚点开始测距
	s_cot_sample_msg_tx->data[2] = read_distance[0];
	s_cot_sample_msg_tx->data[3] = read_distance[1];
	s_cot_sample_msg_tx->data[4] = read_rssi;
//	uint8_t temp[64];
//	memset(temp,0x5a,64);
//	memcpy(&s_cot_sample_msg_tx->data[2],temp,2);
	s_cot_sample_msg_tx->data_len = 5;
	
}
void slave_handdisance_data(uint32_t device_id)										  //锚点处理数据帧
{

	
}

/**
* @name: sample_app_init
* This funtion deal with the sample app init
* @param[in]   task_id -- phy task id in osal.
* @param[out]  none
* @retval  ERR_SUCCESS  0
* @retval  do not deal the event   >0
* @par identifier
*      reserve
* @par other
*      none
* @par ModifyBlog
*      create by zhangjh on 2017-04-29
*/
void sample_app_init( uint8_t taskId )
{
    s_sample_app_id = taskId ;

    osal_set_event( s_sample_app_id, SMAPLE_MSG_INIT_EVENT );
}


uint8_t get_sample_task_id( void )
{
    return s_sample_app_id;
}

/**
* @name: cot_sample_msg_handle
* This
* @param[in]   sample_msg -- sample message.
* @param[out]  sample_msg -- sample message.
* @retval  none
* @retval  none
* @par identifier
*      reserve
* @par other
*      none
* @par ModifyBlog
*      create by zhangjh on 2017-05-18
*/
static void cot_sample_msg_handle( cot_mac_msg_t *sample_msg_packet )						//消息处理函数
{
	cot_application_msg_t *sample_msg;
	sample_msg = (cot_application_msg_t *)sample_msg_packet->data;				//sample_msg代表了cot_mac_msg_t型变量的data区域
	uint8_t cmd_h;
	cmd_h = sample_msg->commad_h;											//sample_msg 其第二个字节（命令的低字节）代表了帧 的类型？   
	switch( cmd_h )
	{
		case COT_SYSTEM_CMD_H:
			{
				uint8_t cmd_l = sample_msg->commad_l;
				switch(cmd_l)
				{
					  case SAMPLE_APP_CMD_BROADCAST:							//如果是广播帧 0x02
						{
              #ifdef MASTER_ROLE		//主（处理锚点回复的广播帧）
								g_device_id[g_device_id_count].device_id = sample_msg_packet->src_addr;			//源ID（这个device_id是锚点的ID）
								g_device_id[g_device_id_count].rssi = sample_msg_packet->rssi;							//接收到数据的RSSI值
								g_device_id_count++;				//收到一条回复帧，自加1
							
								if(g_device_id_count >= MASTER_RNG_DEVICE_COUNT)					//如果回复的锚点数量达到了3
								{
									osal_stop_timerEx(get_sample_task_id(),SMAPLE_MSG_MASTER_DECE_TIMER_EVENT);				//停止时间窗事件
									osal_set_event(get_sample_task_id(),SMAPLE_MSG_MASTER_DECE_TIMER_EVENT);					//设置进入时间窗事件
								}
								else																		
								{
									osal_set_event(get_sample_task_id(),SMAPLE_MSG_RX_EVENT);		//少于三个，设置进入接收事件
								}
							#else		//从（处理的是节点广播帧）
								
								uint32_t dev_id = s_config_data.RFParamSettings.cot_dev_id;//get_dev_id();
								srand(dev_id);
								int16_t delayms = rand()%10;								
								slave_broadcast_data(sample_msg_packet->src_addr);							//源ID，这个参数（是广播节点的ID）
								osal_start_timerEx(s_sample_app_id,SMAPLE_MSG_TX_EVENT,delayms*2);   //随机时延之后进入  消息发送事件
								log_printf("delayms = %d\n",delayms*2);
							#endif
						}
						break;
            case SMAPLE_APP_CMD_START_RANG:  					//如果协商帧  0x20
            {
							#ifdef MASTER_ROLE
							
								//osal_set_event(s_sample_app_id,SMAPLE_MSG_START_RNG_EVENT);
								//osal_start_timerEx(s_sample_app_id,SMAPLE_MSG_START_RNG_EVENT,7);
							#else																			
								
								//g_start_rng_flag_slave = true;
								//slave_handshake_data(sample_msg_packet->src_addr);
								osal_set_event(s_sample_app_id,SMAPLE_MSG_START_RNG_EVENT);					//锚点进入开始测距事件
							#endif
                            
            }
            break;
						case SMAPLE_APP_CMD_DISTANCE_DATA:						//如果是数据帧 0x40， 自己加
						{
							#ifdef MASTER_ROLE			//节点收到的锚点回复的数据帧
							
								//osal_set_event(s_sample_app_id,SMAPLE_MSG_START_RNG_EVENT);			//节点进入
								//osal_start_timerEx(s_sample_app_id,SMAPLE_MSG_START_RNG_EVENT,7);
							#else										//锚点收到数据帧									
								//g_start_rng_flag_slave = true;
								//slave_handshake_data(sample_msg_packet->src_addr);			
								//uint8_t data_buff[128];
								//memset(data_buff, 0x0, sizeof(data_buff) );
								//							
								//sample_msg->data[0];						//具体打印格式可以参考800多行的地方，sprintf字符串格式化命令
							  log_printf("%d %d %d\n",sample_msg->data[0],sample_msg->data[1],sample_msg->data[2]);       //串口打印距离值和rssi值
								osal_set_event(s_sample_app_id,SMAPLE_MSG_START_DATA_EVENT);					//锚点进入发送事件，发送回复数据帧？
							#endif							
						}
						break;
						
					default:
						break;
				}
			}
			break;		
		default:
			break;
	}
}


static void phy_button_process_osal_msg( mt_osal_msg_data_t *p_msg )
{
    switch( p_msg->event_cmd )
    {
        case KEY_EVENT_CMD_KEY1_PRESSED:
        {   
            #ifdef MASTER_ROLE
			g_start_rng_count = 0;
			g_device_id_count = 0;
			g_pkg_type = UNCONFIRM;
			memset(g_device_id,0x0,sizeof(g_device_id)/sizeof(BroadCast_Data));
			master_broadcast_data();						//按键一下 表示  发送广播帧
			osal_start_timerEx(get_sample_task_id(),SMAPLE_MSG_MASTER_DECE_TIMER_EVENT,1000);				//1秒后进入  时间窗口事件
			osal_set_event(get_sample_task_id(), SMAPLE_MSG_CONFIG_EVENT | SMAPLE_MSG_TX_EVENT);		//进入发送事件
			#endif
        }
        break;
        case KEY_EVENT_CMD_KEY2_PRESSED:
        {   
        }
        break;
        case KEY_EVENT_CMD_KEY3_PRESSED:
        {
		
        }
        break;
        case KEY_EVENT_CMD_KEY4_PRESSED:
        {
			
        }
        break;
        case KEY_EVENT_CMD_KEY5_PRESSED:
        {
        }
        break;
        default:
            break;
    }
}
/**
* @name: phy_layer_process_osal_msg
* This
* @param[in]   inArgName input argument description.
* @param[out]  outArgName output argument description.
* @retval  ERR_SUCCESS 0
* @retval  ERR_ERROR   < 0
* @par identifier
*      reserve
* @par other
*      none
* @par ModifyBlog
*      create by zhangjh on 2017-04-29
*/
static void mac_layer_system_message_process( cot_mac_msg_t *p_msg )
{
    switch (p_msg->event_cmd)
    {
        case MAC_EVENT_RX_DONE_CMD:												//接收到数据会进入这个函数接口
        {
						cot_sample_msg_handle(p_msg);
				}
        break;

        case MAC_EVENT_TX_DONE_CMD:												//TX_DONE发送完会进入这里
        {
            osal_set_event(s_sample_app_id,SMAPLE_MSG_TX_DONE_EVENT);
        }
        break;

        case MAC_EVENT_TX_TIMEOUT_CMD:										//
        {
            osal_set_event(s_sample_app_id,SMAPLE_MSG_TX_TIMEOUT_EVENT);
        }
        break;

        case MAC_EVENT_RX_TIMEOUT_CMD:										//接收超时事件
        {
            osal_set_event(s_sample_app_id,SMAPLE_MSG_RX_TIMEOUT_EVENT);
        }
        break;

		case MAC_EVENT_RX_ERROR_CMD:													//接收错误事件
        {
            osal_set_event(s_sample_app_id,SMAPLE_MSG_RX_ERROR_EVENT);
        }
        break;

        default:
            // do nothing
            break;
    }
}


/**
 * @name: sample_app_processevent
 * This function provide simple useing of the rf lib apis.
 * @param[in]   task_id -- the osal task id.
 *				 events -- the event of the task_id.
 * @param[out]  none
 * @retval  ERR_SUCCESS  0
 * @retval  do not deal the event   >0
 * @par identifier
 *      reserve
 * @par other
 *      none
 * @par ModifyBlog
 *      create by zhangjh on 2017-04-29
 */
#include "hal_timers.h"
uint16_t sample_app_processevent(uint8_t task_id, uint16_t events)							//这个功能提供了对rf lib apis的简单使用。
{
    uint16_t ret = ERR_SUCCESS ;
	
	if( ( events & SMAPLE_MSG_MASTER_DECE_TIMER_EVENT ) )				//新增的时间检测事件（时间窗），节点发送广播帧后，在该时间内接收周边锚点回复
	{
		
		if( g_device_id_count > 0 )							//如果回复的锚点数量>0 ?
		{
			//gpio_config(13,0);
			uint8_t i = 0,j = 0;
			
			if(g_device_id_count > 3)							//如果回复的锚点数量>4 ?
			{
				//根据信号强度大到小排序		
				BroadCast_Data temp_data;
				for(i = 0; i < g_device_id_count; i++)
				{
					for(j = i+1; j< g_device_id_count; j++)
					{
						if( g_device_id[i].rssi < g_device_id[j].rssi)
						{
							temp_data = g_device_id[i];
							g_device_id[i] = g_device_id[j];
							g_device_id[j] = temp_data;
						}
					}
				}
			}
			for(int i = 0;i< g_device_id_count;i++)
			{
				log_printf("device_id = %x\n",g_device_id[i].device_id);
			}
			log_printf("\n");
			//gpio_config(12,1);
			//gpio_config(16,1);
			osal_set_event(task_id,SMAPLE_MSG_HANDSHAKE_EVENT);
		}
		else																//如果没有锚点回复广播帧
		{
			g_start_rng_count = 0;
			g_device_id_count = 0;
			g_flag_send_frist = true;						//新一轮开始标志？
			g_pkg_type = UNCONFIRM;
			memset(g_device_id,0x0,sizeof(g_device_id)/sizeof(BroadCast_Data));
			master_broadcast_data();								//再次发送一次广播帧
			
			osal_set_event(get_sample_task_id(), SMAPLE_MSG_CONFIG_EVENT | SMAPLE_MSG_TX_EVENT);
		}
		return (events ^ SMAPLE_MSG_MASTER_DECE_TIMER_EVENT);
	}

	if ( events & SMAPLE_MSG_HANDSHAKE_EVENT )													//新增的握手事件
	{
		log_printf(" -> SMAPLE_MSG_HANDSHAKE_EVENT");
		master_handshake_data(g_start_rng_count);								//节点处理握手数据（通知锚点开始测距）
		g_pkg_type = CONFIRM;
		g_start_rng_flag = true;
		osal_set_event(task_id,SMAPLE_MSG_CONFIG_EVENT|SMAPLE_MSG_TX_EVENT);					//消息发送事件 | 消息配置事件
		return (events ^ SMAPLE_MSG_HANDSHAKE_EVENT);
	}
	
	
	if( events & SMAPLE_MSG_HANDDATA_EVENT )												//自己新增的数据事件？？
	{
		log_printf(" -> SMAPLE_MSG_HANDDATA_EVENT");
		master_handdisance_data(g_start_data_count);						//节点处理数据帧（发送数据帧给锚点）
		g_pkg_type = CONFIRM;
		g_start_data_flag = true;
		osal_set_event(task_id,SMAPLE_MSG_CONFIG_EVENT|SMAPLE_MSG_TX_EVENT);	
		return (events ^ SMAPLE_MSG_HANDDATA_EVENT);		
	}																																	
	
	
	
	
    if ( events & SYS_EVENT_MSG )
    {
        cot_sample_msg_t *samle_msg_ptr;

        while ( (samle_msg_ptr = (cot_sample_msg_t*)osal_msg_receive( task_id )) != NULL )
        {
            switch(samle_msg_ptr->hdr.event)
            {
                case CMD_MAC_LAYER:
                    {
                        mac_layer_system_message_process((cot_mac_msg_t *)samle_msg_ptr);
                    }
                    break;
				case CMD_KEY_MSG:
					{
						phy_button_process_osal_msg((mt_osal_msg_data_t *)samle_msg_ptr);
					}
					break; 
            }
            // Release the OSAL message
            osal_msg_deallocate( (uint8_t *)samle_msg_ptr );
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SMAPLE_MSG_INIT_EVENT )									//消息初始化事件
    {
        DataFlashInit(&s_config_data);
		
				s_cot_sample_msg_tx = (cot_mac_msg_t*)osal_msg_allocate( sizeof(cot_mac_msg_t)+ 239 );

        cot_mac_msg_t *p_msg = (cot_mac_msg_t*)osal_msg_allocate( sizeof(cot_mac_msg_t)+sizeof(PHYParamSettings_t) );
        if( p_msg )
        {
            PHYParamSettings_t phy_param_setting ;
            sample_rf_param_get(s_config_data, &phy_param_setting);

            p_msg->hdr.event = CMD_MAC_LAYER;
            p_msg->event_cmd = MAC_EVENT_INIT_CMD;
            p_msg->src_task_id = task_id;
            p_msg->ModulationType = phy_param_setting.RF_RadioPacketTypes;
            p_msg->data = (uint8_t*)(p_msg+1);

            memcpy((char *)p_msg->data,(char *)&phy_param_setting,sizeof(phy_param_setting));
            p_msg->data_len = sizeof(phy_param_setting);
            osal_msg_send(get_mac_layer_task_id(),(uint8_t*)p_msg);
        }
		
		mt_osal_msg_data_t *p_mt_msg = (mt_osal_msg_data_t*)osal_msg_allocate( sizeof(mt_osal_msg_data_t) );
        if( p_mt_msg )
        {
            p_mt_msg->hdr.event = CMD_SERIAL_MSG;
			p_mt_msg->event_cmd = SERIAL_EVENT_INIT_CMD;
            p_mt_msg->task_id = s_sample_app_id;
            p_mt_msg->msg_len = 0;
            osal_msg_send(get_mt_task_id(),(uint8_t*)p_mt_msg);
        }
        
		
		osal_start_timerEx(task_id,SMAPLE_MSG_CONFIG_EVENT,2);						//2ms之后 进入  消息配置事件
		return (events ^ SMAPLE_MSG_INIT_EVENT);
	}

    if ( events & SMAPLE_MSG_CONFIG_EVENT )								//消息配置事件
    {
		
        cot_mac_msg_t *p_msg = (cot_mac_msg_t*)osal_msg_allocate( sizeof(cot_mac_msg_t)+sizeof(PHYParamSettings_t) );
		
		s_config_data.RFParamSettings.ModulationType = RF_PACKET_TYPE_LORA;
		s_config_data.RFParamSettings.ModulationParam1 = RF_LORA_SF6;
		s_config_data.RFParamSettings.ModulationParam2 = RF_LORA_BW_1600;
		s_config_data.RFParamSettings.ModulationParam3 = RF_LORA_CR_4_5;

		s_config_data.RFParamSettings.PacketParam1 = 12 ; // PreambleLength
		s_config_data.RFParamSettings.PacketParam2 = RF_LORA_PACKET_VARIABLE_LENGTH;
		s_config_data.RFParamSettings.PacketParam3 = DEMO_GFS_LORA_MAX_PAYLOAD;
		s_config_data.RFParamSettings.PacketParam4 = RF_LORA_CRC_ON;
		s_config_data.RFParamSettings.PacketParam5 = RF_LORA_IQ_NORMAL;
		
        if( p_msg )
        {
            PHYParamSettings_t phy_param_setting ;
            sample_rf_param_get(s_config_data, &phy_param_setting);

            p_msg->hdr.event = CMD_MAC_LAYER;
            p_msg->event_cmd = MAC_EVENT_CONFIG_CMD;
            p_msg->src_task_id = task_id;
            p_msg->ModulationType = phy_param_setting.RF_RadioPacketTypes;
            p_msg->data = (uint8_t*)(p_msg+1);

            memcpy((char *)p_msg->data,(char *)&phy_param_setting,sizeof(phy_param_setting));
            p_msg->data_len = sizeof(phy_param_setting);
            osal_msg_send(get_mac_layer_task_id(),(uint8_t*)p_msg);

            osal_pwrmgr_task_state( task_id, PWRMGR_HOLD);
        }
		
		#ifdef SLAVE_ROLE					//如果是从机
			g_start_rng_flag_slave = false;
			osal_set_event(task_id, SMAPLE_MSG_RX_EVENT);					//一开始，进入RX事件
		#endif
		
        return (events ^ SMAPLE_MSG_CONFIG_EVENT);
    }

	
	if ( events & SMAPLE_MSG_START_RNG_EVENT )							//开始测距事件
	{
		
		s_config_data.RFParamSettings.ModulationType = RF_PACKET_TYPE_RANGING;
		s_config_data.RFParamSettings.ModulationParam1 = RF_LORA_SF6;
		s_config_data.RFParamSettings.ModulationParam2 = RF_LORA_BW_1600;
		s_config_data.RFParamSettings.ModulationParam3 = RF_LORA_CR_4_5;

		s_config_data.RFParamSettings.PacketParam1 = 12 ; // PreambleLength
		s_config_data.RFParamSettings.PacketParam2 = RF_LORA_PACKET_VARIABLE_LENGTH;
		s_config_data.RFParamSettings.PacketParam3 = 13;
		s_config_data.RFParamSettings.PacketParam4 = RF_LORA_CRC_ON;
		s_config_data.RFParamSettings.PacketParam5 = RF_LORA_IQ_NORMAL;
        s_config_data.RFParamSettings.RngRequestCount = 15;
        s_config_data.RFParamSettings.RngFullScale = 30;
		#ifdef MASTER_ROLE					//测距主机
        s_config_data.RFParamSettings.RngAddress = g_device_id[g_start_rng_count].device_id;    
		#else												//测距从机
		s_config_data.RFParamSettings.RngAddress = get_dev_id();
		#endif
        s_config_data.RFParamSettings.RngAntenna = DEMO_RNG_ANT_1;
        s_config_data.RFParamSettings.RngUnit = DEMO_RNG_UNIT_SEL_M;

log_printf("start rng\n");
        PHYParamSettings_t phy_param_setting ;
        sample_rf_param_get(s_config_data, &phy_param_setting);

        #ifdef MASTER_ROLE
        phy_param_setting.Entity = RF_MASTER;
        #endif

        #ifdef SLAVE_ROLE
        phy_param_setting.Entity = RF_SLAVE;
        #endif

        cot_ranging_result_t cot_ranging_res;							//测距结果的存放
        memset( (char *)&cot_ranging_res, 0x0, sizeof(cot_ranging_result_t) );
        cot_ranging(&phy_param_setting, &cot_ranging_res);						//测距接口
				g_start_rng_flag = false;														//启动测距标志 清0
		
        if( phy_param_setting.Entity == RF_MASTER )				//如果是主机的话
        {
			//gpio_config(12,0);
            uint8_t printf_buff[128];				//要打印的缓存区？
            uint8_t index = 0;
						g_start_rng_count++;							//开始测距标志加一， 和其中一个锚点进行测距，他就加一
            memset(printf_buff, 0x0, sizeof(printf_buff) );
            sprintf( (char *)printf_buff, "Range: %5.2f mi", cot_ranging_res.distance );					//printf_buff先存储距离值并打印
					
						uint16_distance  = (uint8_t)(cot_ranging_res.distance*10);		//这四行自己加
						read_distance[0] = (uint8_t)((uint16_distance>>8)&0xFF);
						read_distance[1] = (uint8_t)(uint16_distance&0xFF);  			 
						read_rssi = cot_ranging_res.rssi;
					
            index = strlen((char *)printf_buff);
            sprintf( (char *)printf_buff+index, ",cnt_pkg_rx_ok=: %d",cot_ranging_res.cnt_packet_rx_ok );	//printf_buff+index处先存储rssi并打印
            log_printf("%s\n",printf_buff);
					
						if(g_device_id_count > MASTER_RNG_DEVICE_COUNT)									//如果回复的锚点数量>3   其中MASTER_RNG_DEVICE_COUNT = 3
						{
							//这里需要新增一个事件，用于测距完成之后的事件，发送数据帧
				
				
									if(g_start_rng_count < MASTER_RNG_DEVICE_COUNT)								//如果开始测距标志位<3，表明所有测距还没完成
											osal_set_event(task_id,SMAPLE_MSG_HANDDATA_EVENT);				//进入处理数据帧事件
									
//										osal_set_event(task_id,SMAPLE_MSG_HANDSHAKE_EVENT);					
//									else												//如果开始测距标志位>=3,代表和三个锚点都进行了测距。就够了！
//									{
//										//这里需要插入一个事件，用于测距完成之后的事件，发送数据帧给每个锚点

						 }
						else				//如果回复的锚点数量<=3
						{
							if(g_start_rng_count < g_device_id_count)					//如果开始测距标志位< 回复的锚点数量 ， 
								osal_set_event(task_id,SMAPLE_MSG_HANDDATA_EVENT);//继续进入握手事件
//							else													//如果开始测距标志位>=回复的锚点数量,代表和所有回复锚点都进行了测距。就够了！
						}
        }
				
        else					//如果是锚点的话
        {
//            uint8_t printf_buff[128];
//            uint8_t index = 0;

//            memset(printf_buff, 0x0, sizeof(printf_buff) );
//            sprintf( (char *)printf_buff+index, "cnt_pkg_rx_ok=: %d",cot_ranging_res.cnt_packet_rx_ok );			//cnt_packet_rx_ok啥意思？？
//            log_printf("%s\n",printf_buff);
						osal_set_event(task_id,SMAPLE_MSG_CONFIG_EVENT);					//进入消息配置事件
						osal_set_event(task_id,SMAPLE_MSG_RX_EVENT);					//进入接收事件
        }
				
//		osal_set_event(task_id,SMAPLE_MSG_RNG_EVENT);					//进入下一个使能测距事件
		return (events ^ SMAPLE_MSG_START_RNG_EVENT);
	}
	
	
	
	if( events & SMAPLE_MSG_START_DATA_EVENT )											//自己新增的开始数据帧事件
    {
				
				s_config_data.RFParamSettings.ModulationType = RF_PACKET_TYPE_LORA;
				s_config_data.RFParamSettings.ModulationParam1 = RF_LORA_SF6;
				s_config_data.RFParamSettings.ModulationParam2 = RF_LORA_BW_1600;
				s_config_data.RFParamSettings.ModulationParam3 = RF_LORA_CR_4_5;

				s_config_data.RFParamSettings.PacketParam1 = 12 ; // PreambleLength
				s_config_data.RFParamSettings.PacketParam2 = RF_LORA_PACKET_VARIABLE_LENGTH;
				s_config_data.RFParamSettings.PacketParam3 = DEMO_GFS_LORA_MAX_PAYLOAD;
				s_config_data.RFParamSettings.PacketParam4 = RF_LORA_CRC_ON;
				s_config_data.RFParamSettings.PacketParam5 = RF_LORA_IQ_NORMAL;
log_printf("start data\n");
				PHYParamSettings_t phy_param_setting ;
        sample_rf_param_get(s_config_data, &phy_param_setting);
			
				#ifdef MASTER_ROLE
        phy_param_setting.Entity = RF_MASTER;
        #endif

        #ifdef SLAVE_ROLE
        phy_param_setting.Entity = RF_SLAVE;
        #endif
				
				g_start_data_flag = false;
			
				if( phy_param_setting.Entity == RF_MASTER )				//如果是主机的话
        {
						g_start_data_count++;
						if(g_device_id_count > MASTER_RNG_DEVICE_COUNT)									//如果回复的锚点数量>3   其中MASTER_RNG_DEVICE_COUNT = 3
						{
								if(g_start_data_count < MASTER_RNG_DEVICE_COUNT)								//如果开始数据标志位<3，表明所有数据帧传输还没完成
										osal_set_event(task_id,SMAPLE_MSG_HANDSHAKE_EVENT);					//继续进入握手事件
									else												//如果开始数据标志位>=3,代表和三个锚点都进行了数据传输。就够了！
									{
										//这里需要插入一个事件，用于测距完成之后的事件，发送数据帧给每个锚点
											wait_ms(2000);
											g_pkg_type = UNCONFIRM;
											g_start_data_count = 0;
											g_start_rng_count = 0;
											g_device_id_count = 0;
											memset(g_device_id,0x0,sizeof(g_device_id)/sizeof(BroadCast_Data));
											master_broadcast_data();						//第二轮发送广播帧
											g_flag_send_frist = true;
											osal_set_event(get_sample_task_id(), SMAPLE_MSG_CONFIG_EVENT | SMAPLE_MSG_TX_EVENT);
									}
						}
						else				//如果回复的锚点数量<=3
						{
							if(g_start_data_count < g_device_id_count)						//如果开始数据标志位<3，表明所有数据帧传输还没完成
									osal_set_event(task_id,SMAPLE_MSG_HANDSHAKE_EVENT);//继续进入握手事件
							else													//如果开始数据标志位>=3,代表和三个锚点都进行了数据传输。就够了！
							{

										wait_ms(2000);
										g_pkg_type = UNCONFIRM;
										g_start_rng_count = 0;
										g_start_data_count = 0;
										g_device_id_count = 0;
										memset(g_device_id,0x0,sizeof(g_device_id)/sizeof(BroadCast_Data));
										master_broadcast_data();
										g_flag_send_frist = true;
										osal_set_event(get_sample_task_id(), SMAPLE_MSG_CONFIG_EVENT | SMAPLE_MSG_TX_EVENT);
							}
						}
        }
				else					//如果是锚点的话
        {
					osal_set_event(task_id,SMAPLE_MSG_CONFIG_EVENT);					//进入消息配置事件
        }
        return (events ^ SMAPLE_MSG_START_DATA_EVENT);
    }

		
		
		
    if(events & SMAPLE_MSG_TX_EVENT) 																// 发送数据
    {
        cot_mac_msg_t *p_msg = (cot_mac_msg_t*)osal_msg_allocate( sizeof(cot_mac_msg_t)+ 239 );
        if( p_msg )
        {
            p_msg->hdr.event = CMD_MAC_LAYER;
            p_msg->event_cmd = MAC_EVENT_TX_CMD;
            p_msg->src_task_id = task_id;
						p_msg->pkg_hdr.Bits.frame_pending = 0;
						p_msg->dst_addr = s_cot_sample_msg_tx->dst_addr;
						p_msg->pkg_hdr.Bits.MType = g_pkg_type;
						p_msg->pkg_hdr.Bits.ack_request = 1;
            p_msg->data = (uint8_t *)(p_msg+1);

            memcpy((char *)p_msg->data,s_cot_sample_msg_tx->data,s_cot_sample_msg_tx->data_len);
            p_msg->data_len = s_cot_sample_msg_tx->data_len;
            osal_msg_send(get_mac_layer_task_id(),(uint8_t*)p_msg);
        }

        return (events ^ SMAPLE_MSG_TX_EVENT);
    }

    if(events & SMAPLE_MSG_RX_EVENT) 																	// 使能接收数据
    {
        cot_mac_msg_t *p_msg = (cot_mac_msg_t*)osal_msg_allocate( sizeof(cot_mac_msg_t) );
        if( p_msg )
        {
            p_msg->hdr.event = CMD_MAC_LAYER;
            p_msg->event_cmd = MAC_EVENT_RX_CMD;
            p_msg->src_task_id = task_id;
            p_msg->irq_type.Value = RF_IRQ_RX_DONE | RF_IRQ_CRC_ERROR | RF_IRQ_RX_TX_TIMEOUT;
            p_msg->tick_timeout.Step = RF_RADIO_TICK_SIZE_1000_US;
            p_msg->tick_timeout.NbSteps = 0xFFFF;
            p_msg->data_len = 0;
            p_msg->data = NULL;

            osal_msg_send(get_mac_layer_task_id(),(uint8_t*)p_msg);
        }

        osal_pwrmgr_task_state(task_id, PWRMGR_HOLD);

        return (events ^ SMAPLE_MSG_RX_EVENT);
    }


    if ( events & SMAPLE_MSG_TX_DONE_EVENT )							//消息发送完成事件
		{
				log_printf("SMAPLE_MSG_TX_DONE_EVENT\n");
				#ifdef MASTER_ROLE
					if(g_start_rng_flag)//启动测距标志
					{	
						log_printf("g_start_rng_flag = true\n");
						osal_set_event(task_id,SMAPLE_MSG_START_RNG_EVENT);
						
					}
//					else
//					{
//						log_printf("g_start_rng_flag = false\n");
//						osal_set_event(task_id,SMAPLE_MSG_RX_EVENT);
//					}
					/////////////////////////////////////////////////自己加的,这里的if else 应该要放到上面那个if else中去
					else if(g_start_data_flag)//启动数据标志
					{
						log_printf("g_start_data_flag = true\n");
						osal_set_event(task_id,SMAPLE_MSG_START_DATA_EVENT);
					}
//					else								//这一段移到最下面？？
//					{
//						log_printf("g_start_data_flag = false\n");
//						osal_set_event(task_id,SMAPLE_MSG_RX_EVENT);
//					}

				  else if(g_flag_send_frist)//新一轮开始标志
					{
						g_flag_send_frist = false;
						
						osal_start_timerEx(get_sample_task_id(),SMAPLE_MSG_MASTER_DECE_TIMER_EVENT,1000);
					}
					
					else
					{
						log_printf("g_start_data_flag = false\n");
						osal_set_event(task_id,SMAPLE_MSG_RX_EVENT);
					}
				#else
					//if(g_start_rng_flag_slave)
						//osal_set_event(task_id,SMAPLE_MSG_START_RNG_EVENT);
					//else
					{
						osal_set_event(task_id,SMAPLE_MSG_RX_EVENT);
					}
				#endif
					return (events ^ SMAPLE_MSG_TX_DONE_EVENT);
    }

    if ( events & SMAPLE_MSG_TX_TIMEOUT_EVENT )						//消息发送超时事件
    {

				g_send_count_t++;
				if(g_send_count_t < 3)
				{
					osal_start_timerEx(task_id, SMAPLE_MSG_TX_EVENT,10);
				}
				else
				{
					g_send_count_t = 0;
					g_start_rng_count++;
					osal_set_event(task_id,SMAPLE_MSG_HANDSHAKE_EVENT);
				}
				log_printf("SMAPLE_MSG_TX_TIMEOUT_EVENT\n");
				return (events ^ SMAPLE_MSG_TX_TIMEOUT_EVENT);
    }

    if ( events & SMAPLE_MSG_RX_TIMEOUT_EVENT )						//消息接收超时事件
    {
				//osal_start_timerEx(task_id,SMAPLE_MSG_RX_EVENT,2);
				log_printf("SMAPLE_MSG_RX_TIMEOUT_EVENT\n");
				osal_set_event(task_id,SMAPLE_MSG_RX_EVENT);						//重新进入消息接收事件
				return (events ^ SMAPLE_MSG_RX_TIMEOUT_EVENT);
    }

    if ( events & SMAPLE_MSG_RX_ERROR_EVENT )
    {
		log_printf("SMAPLE_MSG_RX_ERROR_EVENT\n");
		//osal_start_timerEx(task_id,SMAPLE_MSG_RX_EVENT,2);
				osal_set_event(task_id,SMAPLE_MSG_RX_EVENT);
        return (events ^ SMAPLE_MSG_RX_ERROR_EVENT);
    }

    return ret ;
}

