#include "HI229.h"
#include "cmsis_os.h"


static Packet_t RxPkt; /* used for data receive */
static int16_t acc[3];
static int16_t gyo[3];
static int16_t mag[3];
static float eular[3];
static float quat[4];
static uint8_t id;

IMU_GET_DATA_t IMU_Get_Data;

void IMU_get(void)
{
	//ÍÓÂÝÒÇÅ·À­½Ç
	get_eular(IMU_Get_Data.IMU_Eular);
	//ÍÓÂÝÒÇ½ÇËÙ¶È
	get_raw_gyo(IMU_Get_Data.IMU_Gyo);
}

void IMU_TASK(void const * argument)
{
  /* USER CODE BEGIN IMU_TASK */
  /* Infinite loop */
  for(;;)
  {
		IMU_get();
    osDelay(1);
  }
  /* USER CODE END IMU_TASK */
}


int get_raw_acc(int16_t* a)
{
    memcpy(a, acc, sizeof(acc));
    return 0;
}

int get_raw_gyo(int16_t* g)
{
    memcpy(g, gyo, sizeof(gyo));
    return 0;
}

int get_raw_mag(int16_t* m)
{
    memcpy(m, mag, sizeof(mag));
    return 0;
}

int get_eular(float* e)
{
    memcpy(e, eular, sizeof(eular));
    return 0;
}

int get_quat(float* q)
{
    memcpy(q, quat, sizeof(quat));
    return 0;
}

int get_id(uint8_t *user_id)
{
    *user_id = id;
    return 0;
}

static void OnDataReceived(Packet_t *pkt)
{
    int offset = 0;
    uint8_t *p = pkt->buf;
    while(offset < pkt->payload_len)
    {
        switch(p[offset])
        {
            case kItemID:
                id = p[1];
                offset += 2;
                break;
            case kItemAccRaw:
            case kItemAccCalibrated:
            case kItemAccFiltered:
            case kItemAccLinear:
                memcpy(acc, p + offset + 1, sizeof(acc));
                offset += 7;
                break;
            case kItemGyoRaw:
            case kItemGyoCalibrated:
            case kItemGyoFiltered:
                memcpy(gyo, p + offset + 1, sizeof(gyo));
                offset += 7;
                break;
            case kItemMagRaw:
            case kItemMagCalibrated:
            case kItemMagFiltered:
                memcpy(mag, p + offset + 1, sizeof(mag));
                offset += 7;
                break;
            case kItemRotationEular:
                eular[0] = ((float)(int16_t)(p[offset+1] + (p[offset+2]<<8)))/100;
                eular[1] = ((float)(int16_t)(p[offset+3] + (p[offset+4]<<8)))/100;
                eular[2] = ((float)(int16_t)(p[offset+5] + (p[offset+6]<<8)))/10;
                offset += 7;
                break;
            case kItemRotationEular2:
                memcpy(eular, p + offset + 1, sizeof(eular));
                offset += 13;
                break;
            case kItemRotationQuat:
                memcpy(quat, p + offset + 1, sizeof(quat));
                offset += 17;
                break;
            case kItemPressure:
                offset += 5;
                break;
            case kItemTemperature:
                offset += 5;
                break;
            default:
				offset++;
                break;
        }
    }
    
}

int IMU_Init(void)
{
    Packet_DecodeInit(&RxPkt, OnDataReceived);
    return 0;
}

