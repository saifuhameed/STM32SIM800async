/*
 * sim800.h
 *
 *  Created on: Jun 15, 2023
 *      Author: saif
 */

#ifndef INC_SIM800_H_
#define INC_SIM800_H_
/*
********************************************************************
* AT+COPS? 	TA returns the current mode and the currently selected operator.
* AT+COPS=? returns a list of quadruplets, each representing an operator present in the network.
* AT+CCLK? 	returns time in "yy/MM/dd,hh:mm:ss±zz "
* AT+CMGF=1 set SMS system into text mode
* AT+CNMI=0,0,0,0,0 <mode>,<mt>,<bm>,<ds>,<bfr> TA selects the procedure for how the receiving of new messages from the network is indicated to the TE when TE is active, e.g. DTR signal is ON
* AT+CUSD=1 enable USSD result to TE (uart)
* AT+CMGD=1 deletes SMS at index 1
* AT+CMGR=1 reads SMS at index 1
* AT+CPMS? read SMS status  <mem1>,<used1>,<total1>,<mem2>,<used2>,<total2>, <mem3>,<used3>,<total3>
* AT+CSQ SIgnal quality report 0 = -115 dBm or less, 1 = -111 dBm, 2...30 = -110... -54 dBm, 31 -52 dBm or greater, 99 not known or not detectable
* AT+CSMINS? SIM Inserted Status Reporting
**********************************************************************
*/
typedef  int8_t(*myFunc)(uint8_t);

typedef enum {
	STATUS_NOT_STARTED,
	STATUS_STARTED,
	STATUS_EXECUTING,
	STATUS_COMPLETED
} Step_Status;

typedef struct
{
	myFunc mfunc;
	uint8_t step;
	uint8_t step_status;
	uint8_t function_status;
	uint16_t timeout;
	uint32_t started_time;
	uint8_t *ret;
	uint8_t param1;
}FuncStatus;



typedef struct
{
	uint8_t index;
	uint8_t valid;
	uint8_t processed;
	char oa[30];
	char scts[30];
	char data[281];
}SMSData;

typedef struct
{
	uint8_t sim_configured;
	uint8_t sim_inserted;
	uint8_t sim_network_registered;
	uint8_t sim_network_quality;
	uint8_t sms_used1;
	uint8_t sms_total1;
	uint8_t sms_used2;
	uint8_t sms_total2;
	uint8_t sms_used3;
	uint8_t sms_total3;
}SIM800Status;


void HAL_UART_RxCpltCallback_SIM800(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback_SIM800( UART_HandleTypeDef *huart,uint16_t Size);
void HAL_UART_TxCpltCallback_SIM800(UART_HandleTypeDef *huart);

void QueueFunction_SIM800(myFunc f, uint8_t param1 );

int8_t SendSMS_SIM800(uint8_t step);
int8_t CheckNetworkRegistration_SIM800(uint8_t step);
int8_t CheckNetworkSignalStrength_SIM800(uint8_t step);
int8_t CheckSIMCardInserted_SIM800(uint8_t);
int8_t _CheckSMS(uint8_t step);
int8_t SendUSSD_SIM800(uint8_t step);
int8_t ShowUSSD_SIM800(uint8_t step);
int8_t CallAdmin_SIM800(uint8_t step);
int8_t Hangup_SIM800(uint8_t step);
int8_t Configure_SIM800(uint8_t step);
int8_t DeleteSMS_SIM800(uint8_t step);
int8_t _ReadSMS(uint8_t step);


void ProcessQueFunction_SIM800();
SMSData GetReceivedSMSData_SIM800();
void SetDataForSMSSending_SIM800(char * num, char * msg);
void Initialize_SIM800(void);
SIM800Status GetStatus_SIM800();
void _SendCommandNonblocking(uint8_t* command);

void _ClearSMSStatus();

void Debug(const char *info);


#endif /* INC_SIM800_H_ */
