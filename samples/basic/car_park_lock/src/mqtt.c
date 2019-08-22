#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <mosquitto.h>
#include <string.h>
#include <json-c/json.h>
#include <errno.h>
#include <pthread.h>

#define HOST "39.100.123.34"
#define PORT  1883

#define KEEP_ALIVE 60
#define MSG_MAX_SIZE  512
#define SOCKET_RCV_BUFFER_SIZE	(1024 * 5)
#define MAX_NODE_NUM	6
#define MAX_RADIO_PKG_SIZE	256
typedef struct{
    uint8_t strDevEUI[8 * 2 + 1];
    uint8_t strmacaddr[6 * 2 + 1];
    uint32_t iDevAddr;
    bool isClassC;    
}st_ServerNodeDatabase,*pst_ServerNodeDatabase;

typedef union{
    uint8_t value;
    uint8_t motor:1;
    uint8_t sensor1:1;
    uint8_t sensor2:1;
}un_LockStatus,*pun_LockStatus;

st_ServerNodeDatabase stServerNodeDatabase;
bool session = true;
uint8_t globaltopic[100] = {0};
uint8_t globaldatabuf[1024] = {0};
bool rx1stpkgflag = false;
struct mosquitto *globalmosq = NULL;
bool isconfirmrequest = false;
uint32_t downcmdcnt = 0,upcmdcnt = 0,downcnt = 0,upcnt = 0;

uint8_t DataUpFrameCnt = 0;
uint8_t DataDownFrameCnt = 0;

static const int8_t hexval[] = {
    /*00-1F*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*20-3F*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,-1,-1,-1,-1,-1,-1,
    /*40-5F*/ -1,10,11,12,13,14,15,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*60-7F*/ -1,10,11,12,13,14,15,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*80-9F*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*A0-BF*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*C0-DF*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    /*E0-FF*/ -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
};

uint8_t gethex (uint8_t* dst, const uint8_t* src, uint16_t len) {
    uint8_t n = 0;
    if(len & 1) { // odd number of digits
    return 0;
    }
    while(len--) {
    int8_t v = hexval[*src++];
    if(v < 0) { // bad hex digit
        return 0;
    }
    *dst = (*dst << 4) | v; // shift nibble
    if((len & 1) == 0) { // advance at every second digit
        dst++;
        n++;
    }
    }
    return n;
}

uint8_t tolower (uint8_t c) {
    if(c >= 'A' && c <= 'Z') {
    c += 'a' - 'A'; // make lower case
    }
    return c;
}

uint8_t puthex (uint8_t* dst, const uint8_t* src, uint8_t len) {
    uint8_t l = len;
    while(len--) {
    *dst++ = "0123456789ABCDEF"[*src >> 4];
    *dst++ = "0123456789ABCDEF"[*src & 0xF];
    src++;
    }
    return 2*l;
}

void my_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
	int len;
	struct json_object *pragma = NULL;
    struct json_object *obj = NULL;
	uint8_t *pstrchr;
	uint32_t sendlen;
	uint8_t stringformat[256 * 2];
	uint8_t datatosend[1024];
	uint8_t echodata[MAX_RADIO_PKG_SIZE * 2];
    uint8_t iPort = 0;
    uint8_t strgatewaymacaddr[6 * 2 + 1] = {0};
    if(message->payloadlen > 0)
	{
	    printf("%s\r\n%s\r\n", message->topic, message->payload);
        pstrchr = message->payload;
        if(message->payload)
        {
            pragma = json_tokener_parse((const char *)pstrchr);
            if(pragma == NULL)
            {
                return;
            }
			json_object_object_get_ex(pragma,"NetAddr",&obj);
            if(obj == NULL)
            {
                printf("Format error %d\r\n",__LINE__);
                json_object_put(pragma);
                return;
            }
            stServerNodeDatabase.iDevAddr = json_object_get_int(obj);

                
            json_object_object_get_ex(pragma,"Port",&obj);
            if(obj == NULL)
            {
                printf("Format error %d\r\n",__LINE__);
                json_object_put(pragma);
                return;
            }
            iPort = json_object_get_int(obj);
                                
            memcpy(stServerNodeDatabase.strDevEUI,message->topic + strlen("LoRaWAN/Up/") + 6 * 2 + 1,8 * 2);
            memcpy(stServerNodeDatabase.strmacaddr,message->topic + strlen("LoRaWAN/Up/"),6 * 2);
            json_object_object_get_ex(pragma,"NodeType",&obj);
            if(obj == NULL)
            {
                printf("Format error %d\r\n",__LINE__);
                json_object_put(pragma);
                return;
            }
            if(strcmp("Class C",json_object_get_string(obj)))
            {
                stServerNodeDatabase.isClassC = true;
            }
            else
            {
                stServerNodeDatabase.isClassC = false;
            }
            json_object_object_get_ex(pragma,"ConfirmRequest",&obj);
	        if(obj == NULL)
            {
                printf("Format error %d\r\n",__LINE__);
                json_object_put(pragma);
                return;
            }
            isconfirmrequest = json_object_get_boolean(obj);
				
			json_object_object_get_ex(pragma,"Data",&obj);
			if(obj == NULL)
			{
				printf("Format error %d\r\n",__LINE__);
				json_object_put(pragma);
				return;
			}
			memset(echodata,0,sizeof(echodata));
			strcpy((char *)echodata,(const char*)json_object_get_string(obj));
            json_object_put(pragma);
            pragma = json_object_new_object();
	        json_object_object_add(pragma,"NetAddr",json_object_new_int(stServerNodeDatabase.iDevAddr));
	        json_object_object_add(pragma,"Port",json_object_new_int(1));
	        json_object_object_add(pragma,"ConfirmRequest",json_object_new_boolean(1));
			json_object_object_add(pragma,"Confirm",json_object_new_boolean(isconfirmrequest));
            //if(iPort == 1)
            {
                //if(strlen(echodata) == 2)
                {
                    //strcmp("00",echodata)?strcpy(echodata,"00"):strcpy(echodata,"01");
                    //json_object_object_add(pragma,"Data",json_object_new_string(echodata));
                }
                //else
                {
                    json_object_object_add(pragma,"Data",json_object_new_string(echodata));
                }
            }
            /*else
            {
                json_object_object_add(pragma,"Data",json_object_new_string(echodata));
            }*/
	        //json_object_object_add(pragma,"Data",json_object_new_string(""));
            unsigned char topic[8 + 1 + 6 * 2 + 1 + 8 * 2 + 1 + 2 + 1 + 10 + 10] = {0};
			memset(datatosend,0,sizeof(datatosend));
            memset(topic,0,sizeof(topic));
			strcpy(datatosend,json_object_to_json_string(pragma));
	        sendlen = strlen(datatosend);
			/*发布消息*/
			//sprintf(topic,"%s,%s,%s,%s","LoRaWAN/",strmacaddr,"/","0123456789ABCDEF");
			strcpy(topic,"LoRaWAN/Down/");
            //printf("gateway = %s\r\n",stServerNodeDatabase.strmacaddr);
			strcat(topic,stServerNodeDatabase.strmacaddr);
			strcat(topic,"/");
			strcat(topic,stServerNodeDatabase.strDevEUI);
			//printf("topic = %s\r\n",topic);
            //printf("data = %s\r\n",datatosend);
            uint8_t tempdata[5];
            int16_t sensor1,sensor2;
            uint8_t lockstatus;
            printf("%s\n",echodata );
            for(uint8_t myloop = 0;myloop < 10;myloop ++)
            {
                echodata[myloop] = tolower(echodata[myloop]);
            }
            gethex (tempdata, echodata, 10);
            //StringToHex(echodata, tempdata, 5);

            printf("%02X\n", tempdata[0]);
            printf("%02X\n", tempdata[1]);
            printf("%02X\n", tempdata[2]);
            printf("%02X\n", tempdata[3]);
            printf("%02X\n", tempdata[4]);
            //printf("%02X\n", tempdata[5]);
            un_LockStatus unLockStatus;
            //DataUpFrameCnt = tempdata[0];
            unLockStatus.value = tempdata[0];
            if(unLockStatus.value & 0x01)
            {
                upcnt ++;
            }
            else
            {
                downcnt ++;
            }
            if(upcmdcnt && downcmdcnt)
            {
                printf("upcmdcnt = %d, downcmdcnt = %d, upcnt = %d, downcnt = %d, upokpercent = %d\%, downokpercent = %d\%\r\n",upcmdcnt,downcmdcnt,upcnt,downcnt,upcnt * 100 / upcmdcnt,downcnt * 100 / downcmdcnt);
            }
            sensor1 =  tempdata[1] | (tempdata[2] << 8);
            sensor2 =  tempdata[3] | (tempdata[4] << 8);
            printf("\rmotor status = %d, sensor1 status = %d, sensor2 status = %d; sensor1 = %d, sensor2 = %d\r\n", unLockStatus.motor,unLockStatus.sensor1,unLockStatus.sensor2,sensor1,sensor2);
	//while(1)
	{
		strcpy(globaltopic,topic);
		strcpy(globaldatabuf,datatosend);
		rx1stpkgflag = 1;
		globalmosq = mosq;
        uint8_t databyte[3];
        //indication = !indication;
        /*if(isconfirmrequest)
        {
            databyte[0] = DataUpFrameCnt;
            databyte[1] = 2;
            databyte[2] = 0;
            uint8_t stingdata[256 * 2] = {0};

            puthex (stingdata, databyte, 3);
            pragma = json_object_new_object();
            json_object_object_add(pragma,"NetAddr",json_object_new_int(stServerNodeDatabase.iDevAddr));
            json_object_object_add(pragma,"Port",json_object_new_int(1));
            json_object_object_add(pragma,"ConfirmRequest",json_object_new_boolean(1));
            json_object_object_add(pragma,"Confirm",json_object_new_boolean(isconfirmrequest));

            json_object_object_add(pragma,"Data",json_object_new_string(stingdata));
            memset(datatosend,0,sizeof(datatosend));
            strcpy(datatosend,json_object_to_json_string(pragma));
            mosquitto_publish(globalmosq,NULL,globaltopic,strlen(datatosend),datatosend,0,0);
            printf("\r\n%s\r\n%s\r\n", globaltopic,datatosend);
        }*/
        //mosquitto_publish(mosq,NULL,topic,strlen(datatosend)+1,datatosend,0,0);
		//sleep(2);
	}
			//mosquitto_publish(mosq,NULL,topic,strlen(datatosend)+1,sendlen,0,0);
	        //json_object_put(pragma);
            //memset(buffer,0,1024 * 100);
        }
    }else{
        printf("%s (null)\n", message->topic);
    }
    fflush(stdout);
}

void my_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
    int i;
    unsigned char topic[8 + 1 + 6 * 2 + 2 + 1 + 10] = {0};
    printf("%s, %d\r\n",__func__,__LINE__);
    if(!result){
        /* Subscribe to broker information topics on successful connect. */
        strcpy(topic,"LoRaWAN/Up");
        strcat(topic,"/0CEFAFD2DB12/#");
        mosquitto_subscribe(mosq, NULL, topic, 2);
        printf("topic = %s\r\n",topic);
    }else{
        fprintf(stderr, "Connect failed\n");
    }
    printf("%s, %d\n", __func__,__LINE__);
}
void my_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
    int i;
    printf("Subscribed (mid: %d): %d", mid, granted_qos[0]);
    for(i=1; i<qos_count; i++){
        printf(", %d", granted_qos[i]);
    }
    printf("\n");
}

void my_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    /* Pring all log messages regardless of level. */
    printf("%s\n", str);
}

static void *pthread(void* arg)
{
	int count = 1;
    struct json_object *pragma = NULL;
    uint8_t databyte[3];
    uint8_t stingdata[7] = {0};
    uint8_t cmd;
    uint8_t datatosend[1024] = {0};
    int indication = 0;
    while(1)
    {
    	if(rx1stpkgflag)
    	{
            //scanf("%c",&cmd);
            cmd = 'i';
            memset(stingdata,0,sizeof(stingdata));
            switch(cmd)
            {
                case 'g':
                    databyte[0] = 0;
                    puthex (stingdata, databyte, 1);
                    break;
                case 's':
                    databyte[0] = 1;
                    databyte[1] = 1;
                    databyte[2] = 0xff;
                    puthex (stingdata, databyte, 3);
                    break;
                case 'i':
                    
                    indication = !indication;
                    /*if(isconfirmrequest)
                    {
                        DataDownFrameCnt = DataUpFrameCnt;
                    }
                    else
                    {
                        DataDownFrameCnt ++;
                    }*/
                    //databyte[0] = DataDownFrameCnt;
                    databyte[0] = 2;
                    printf("direct?\n");
                    
                    //scanf("%d",&indication);
                    
                    if(indication)
                    {
                        printf("up\n");
                        databyte[1] = 1;
                    }
                    else
                    {
                        printf("down\n");
                        databyte[1] = 0;
                    }
                    puthex (stingdata, databyte, 2);
                    pragma = json_object_new_object();
                    json_object_object_add(pragma,"NetAddr",json_object_new_int(stServerNodeDatabase.iDevAddr));
                    json_object_object_add(pragma,"Port",json_object_new_int(1));
                    json_object_object_add(pragma,"ConfirmRequest",json_object_new_boolean(1));
                    json_object_object_add(pragma,"Confirm",json_object_new_boolean(isconfirmrequest));

                    json_object_object_add(pragma,"Data",json_object_new_string(stingdata));
                    memset(datatosend,0,sizeof(datatosend));
                    strcpy(datatosend,json_object_to_json_string(pragma));
                    mosquitto_publish(globalmosq,NULL,globaltopic,strlen(datatosend),datatosend,0,0);
                    printf("%s\n%s\n", globaltopic,datatosend);
                    json_object_put(pragma);
                    if(indication)
                    {
                        upcmdcnt ++;
                    }
                    else
                    {
                        downcmdcnt ++;
                    }
                    /*if(upcmdcnt && downcmdcnt)
                    {
                        printf("upcmdcnt = %d, downcmdcnt = %d, upcnt = %d, downcnt = %d, upokpercent = %d\%, downokpercent = %d\%\r\n",upcmdcnt,downcmdcnt,upcnt,downcnt,upcnt * 100 / upcmdcnt,downcnt * 100 / downcmdcnt);
                    }*/
                    sleep(7);
                    break;
                default:
                    break;
            }
    	}
    }       

    return NULL;
}

int main(void )
{
	uint8_t stringformat[256 * 2];
	int len;
    struct json_object *pragma = NULL;
    struct mosquitto *mosq = NULL;
    int err,i;
	uint8_t deveui[8 * 2 + 1] = {0};
	uint8_t senddata[1024] = {0};
	pthread_t tidp;


    if ((pthread_create(&tidp, NULL, pthread, NULL)) == -1)
    {
        printf("create error!\n");
        return 1;
    }
    
    sleep(1);
    
    printf("mian continue!\n");

    memset(&stServerNodeDatabase,0,sizeof(stServerNodeDatabase));
	init_mqtt:
	//libmosquitto 库初始化
	mosquitto_lib_init();
	//创建mosquitto客户端
	mosq = mosquitto_new(NULL,session,NULL);
	if(!mosq){
		printf("create client failed..\n");
		mosquitto_lib_cleanup();
		sleep(1);
		goto init_mqtt;
		//return 0;
	}
	//mosquitto_log_callback_set(mosq, my_log_callback);
	mosquitto_connect_callback_set(mosq, my_connect_callback);
	mosquitto_message_callback_set(mosq, my_message_callback);
	//mosquitto_subscribe_callback_set(mosq, my_subscribe_callback);
	mosquitto_username_pw_set(mosq,"MJ-Modem","www.colt.xin");
	printf("%s, %d\r\n",__func__,__LINE__);
	//连接服务器
	if(mosquitto_connect(mosq, HOST, PORT, KEEP_ALIVE)){
        printf("%s, %d\r\n",__func__,__LINE__);
		fprintf(stderr, "Unable to connect.\n");
		mosquitto_destroy(mosq);
        	mosquitto_lib_cleanup();
        	sleep(1);
        	goto init_mqtt;
		//return 0;
	}
	printf("%s, %d\r\n",__func__,__LINE__);
	//循环处理网络消息
	mosquitto_loop_forever(mosq, -1, 1);
    printf("%s, %d\r\n",__func__,__LINE__);
	mosquitto_destroy(mosq);
	mosquitto_lib_cleanup();
	mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        sleep(1);
        goto init_mqtt;
	return 0;
}


