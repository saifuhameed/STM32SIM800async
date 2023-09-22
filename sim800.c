/*
 * sim800.c
 *
 *  Created on: Jun 15, 2023
 *      Author: saif
 */
#include "main.h"
#include <stdio.h>
#include "string.h"
#include <stdlib.h>
#include "sim800.h"
#include <ctype.h>



extern char adminnumber[];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
FuncStatus funcStatus[];
int8_t current_func_tail;
int8_t current_func_head;
SMSData smsDataReceived;
SMSData smsDataForSend;

UART_HandleTypeDef *_sim800_uart;
UART_HandleTypeDef *_debug_uart;

SIM800Status _sim800_status={0,0,0,0,0,0,0,0,0,0};

#define SIM800_BUFFER_SIZE 512
uint8_t sim800_uart_rx_buffer[SIM800_BUFFER_SIZE] = {'\0'};

#define SIM800_MainBuf_SIZE 2048
uint8_t SIM800_MainBuf[SIM800_MainBuf_SIZE];
uint16_t oldPos = 0;
uint16_t newPos = 0;
uint8_t rx_event_fired=0; //1 means rx_event fired
uint8_t tx_complete_fired=0; //1 means rx_event fired


char oa[30], scts[30], data[281];

char ussd_command[12];

#define MAX_FUNC 10
FuncStatus funcStatus[MAX_FUNC]={};
int8_t current_func_tail=-1;
int8_t current_func_head=-1;

#define AT_OK "\r\nOK\r\n"

SMSData GetReceivedSMSData_SIM800(){
	return smsDataReceived;
}

void SetDataForSMSSending_SIM800(char * num, char * msg){
	strcpy(smsDataForSend.oa,num);
	sprintf(smsDataForSend.data, msg  );
}

uint32_t sim_prev_ticks_250=0;
uint32_t sim_prev_ticks_1000=0;
uint32_t sim_prev_ticks_5000=0;
uint32_t sim_prev_ticks_10000=0;

void ProcessQueFunction_SIM800(uint32_t current_tick){
	if((current_tick-sim_prev_ticks_250)>250){
		sim_prev_ticks_250=current_tick;
		if(_sim800_status.sim_configured && !_sim800_status.sim_inserted)QueueFunction_SIM800(CheckSIMCardInserted_SIM800,0);
		if(_sim800_status.sim_inserted && !_sim800_status.sim_network_registered)QueueFunction_SIM800(CheckNetworkRegistration_SIM800,0);
		if(_sim800_status.sim_inserted && !_sim800_status.sim_network_registered)QueueFunction_SIM800(CheckNetworkSignalStrength_SIM800,0);
		if(_sim800_status.sim_inserted && !_sim800_status.sim_network_registered)QueueFunction_SIM800(_CheckSMS,0);
		if(!_sim800_status.sim_inserted){
			_sim800_status.sim_network_registered=0;
			_sim800_status.sim_network_quality=0;
		}
	}
	if((current_tick-sim_prev_ticks_5000)>5000){
		sim_prev_ticks_5000=current_tick;
		if(!_sim800_status.sim_configured)QueueFunction_SIM800(Configure_SIM800,0);
	}
	if((current_tick-sim_prev_ticks_10000)>10000){
		sim_prev_ticks_10000=current_tick;

		if(_sim800_status.sim_configured)QueueFunction_SIM800(CheckSIMCardInserted_SIM800,0);
		if(_sim800_status.sim_inserted)QueueFunction_SIM800(CheckNetworkRegistration_SIM800,0);
		if(_sim800_status.sim_inserted)QueueFunction_SIM800(CheckNetworkSignalStrength_SIM800,0);
		if(_sim800_status.sim_network_registered)QueueFunction_SIM800(_CheckSMS,0);
		if (_sim800_status.sms_used1>0){
			QueueFunction_SIM800(_ReadSMS,0);
		}
	}

	if(funcStatus[current_func_head].mfunc!=NULL){
			if(funcStatus[current_func_head].function_status!=STATUS_COMPLETED){
				int8_t ret=funcStatus[current_func_head].mfunc(funcStatus[current_func_head].step);
				if(ret!=-1)*funcStatus[current_func_head].ret=ret;
			}else{
				funcStatus[current_func_head].mfunc=NULL;
				current_func_head++;
				if(current_func_head>=MAX_FUNC){
					current_func_head=0;
				}
			}
	}
}


void QueueFunction_SIM800(myFunc f, uint8_t param1 ){
	if(current_func_tail==-1){
		current_func_tail=0;
		current_func_head=0;
	}
	funcStatus[current_func_tail].step=1;
	funcStatus[current_func_tail].mfunc=f;
	funcStatus[current_func_tail].step_status=STATUS_NOT_STARTED;
	funcStatus[current_func_tail].function_status=STATUS_NOT_STARTED;
	funcStatus[current_func_tail].started_time=0;
	funcStatus[current_func_tail].timeout=1000;
	funcStatus[current_func_tail].param1=param1;
	current_func_tail++;
	if(current_func_tail>=MAX_FUNC)current_func_tail=0;
}


SIM800Status GetStatus_SIM800(){
	return _sim800_status;
}

void _ClearSMSStatus(){
	_sim800_status.sms_used1=0;
	_sim800_status.sms_used2=0;
	_sim800_status.sms_used3=0;
	_sim800_status.sms_total1=0;
	_sim800_status.sms_total2=0;
	_sim800_status.sms_total3=0;
}




void Debug(const char *info) {
  char CRLF[3] = "\r\n";
  HAL_UART_Transmit(_debug_uart, (uint8_t *)CRLF, strlen(CRLF), 100);
  HAL_UART_Transmit(_debug_uart, (uint8_t *)info, strlen(info), 100);
  HAL_UART_Transmit(_debug_uart, (uint8_t *)CRLF, strlen(CRLF), 100);
}



uint8_t _GetCSQStrength(int8_t csq){
	uint8_t nq[6]={0,2,9,14,19,27};
	for(uint8_t i=5;i>=0;i--){
		if(csq>=nq[i]) return i;
	}
	return 0;

}

void HAL_UART_TxCpltCallback_SIM800(UART_HandleTypeDef *huart)
{
	if (huart->Instance == _sim800_uart->Instance) {
		funcStatus[current_func_head].step_status=STATUS_COMPLETED;
		tx_complete_fired =1;
		HAL_UARTEx_ReceiveToIdle_DMA(_sim800_uart, sim800_uart_rx_buffer, SIM800_BUFFER_SIZE);
	}
}


void HAL_UARTEx_RxEventCallback_SIM800( UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance == _sim800_uart->Instance) {

		oldPos = newPos;  // Update the last position before copying new data
		if (oldPos+Size > SIM800_MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = SIM800_MainBuf_SIZE-oldPos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)SIM800_MainBuf+oldPos, sim800_uart_rx_buffer, datatocopy);  // copy data in that remaining space
			oldPos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)SIM800_MainBuf, (uint8_t *)sim800_uart_rx_buffer+datatocopy, (Size-datatocopy));  // copy the remaining data
			newPos = (Size-datatocopy);  // update the position
		}/* if the current position + new data size is less than the main buffer we will simply copy the data into the buffer and update the position */
		else
		{
			memcpy ((uint8_t *)SIM800_MainBuf+oldPos, sim800_uart_rx_buffer, Size);
			newPos = Size+oldPos;
		}
		rx_event_fired=1;
		HAL_UARTEx_ReceiveToIdle_DMA(_sim800_uart, sim800_uart_rx_buffer, SIM800_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}

}


uint8_t sim800command[300];


void _BeginBuffersForUartTransmit(){
	funcStatus[current_func_head].step_status=STATUS_STARTED;
	memset( sim800_uart_rx_buffer , '\0' , SIM800_BUFFER_SIZE );
	memset( SIM800_MainBuf , '\0' , SIM800_MainBuf_SIZE );
	newPos=0;
	funcStatus[current_func_head].started_time=HAL_GetTick();
	funcStatus[current_func_head].timeout=100;
	//sim800_uart_rx_index = 0;
	funcStatus[current_func_head].step++;
	tx_complete_fired=0;
}



char *_CheckAck(uint8_t* ack){
	char *pch1 = strstr((const char*)&SIM800_MainBuf[0],(const char*) ack);
	return pch1;
}


void Initialize_SIM800(){
	_sim800_uart=&huart1;
	_debug_uart=&huart2;
}

void _SendCommandNonblocking(uint8_t* command){
	uint8_t sz=strlen((const char*)command);
	HAL_UART_Transmit_IT(_sim800_uart,command, sz);
	HAL_UARTEx_ReceiveToIdle_DMA(_sim800_uart, sim800_uart_rx_buffer, SIM800_BUFFER_SIZE);
}

int8_t Configure_SIM800(uint8_t step){
	uint32_t tick=HAL_GetTick();
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){
		_BeginBuffersForUartTransmit();
		funcStatus[current_func_head].timeout=1000;
		_SendCommandNonblocking((uint8_t *)"AT\r\n");
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		char *pch1=_CheckAck((uint8_t*) AT_OK);
		if(pch1){
			_BeginBuffersForUartTransmit();
			funcStatus[current_func_head].timeout=1000;
			_SendCommandNonblocking((uint8_t *)"AT+CMGF=1\r");
		}else{
			if(timeout_fired){
				funcStatus[current_func_head].step=1;
				funcStatus[current_func_head].step_status=STATUS_NOT_STARTED;
			}
		}
		return -1;
	}
	if(step==3 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		char *pch1=_CheckAck((uint8_t*)  "AT+CMGF=1\r" AT_OK);
		if(pch1){
			_BeginBuffersForUartTransmit();
			funcStatus[current_func_head].timeout=1000;
			_SendCommandNonblocking((uint8_t *)"AT+CNMI=0,0,0,0,0\r");
		}else{
			if(timeout_fired){
				funcStatus[current_func_head].step=2;
			}
		}
		return -1;
	}
	if(step==4 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		char *pch1=_CheckAck((uint8_t*)  AT_OK);
		if(pch1){
			funcStatus[current_func_head].function_status=STATUS_COMPLETED;
			_sim800_status.sim_configured=1;
			return 1;
		}else{
			if(timeout_fired){
				_sim800_status.sim_configured=0;
				funcStatus[current_func_head].step=3;
			}
		}
	}
	return -1;
}

int8_t SendUSSD_SIM800(uint8_t step){
	uint32_t tick=HAL_GetTick();
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){
		_BeginBuffersForUartTransmit();
		_SendCommandNonblocking((uint8_t *)"AT+CUSD=1\r");
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		_BeginBuffersForUartTransmit();
		sprintf((char *)sim800command, "AT+CUSD=1,\"%s\"\r", ussd_command);
		funcStatus[current_func_head].timeout=5000;
		_SendCommandNonblocking(sim800command);
		return -1;
	}
	if(step==3 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		char *pch1=_CheckAck((uint8_t*)  "+CUSD:");
		if(pch1){
			uint8_t param_start=10; // +CUSD:1, "USSD reply from operator"
			uint8_t len=strlen(pch1);
			strcpy(smsDataForSend.oa,adminnumber);
			memset( smsDataForSend.data , '\0' , sizeof(smsDataForSend.data) );
			memcpy( smsDataForSend.data,pch1+10,len-param_start);
			funcStatus[current_func_head].mfunc=SendSMS_SIM800; //Change pointer to this function to sendSMS_SIM800
			funcStatus[current_func_head].step=1; //Start from Step 1 of sendSMS_SIM800
			funcStatus[current_func_head].step_status=STATUS_NOT_STARTED;
			funcStatus[current_func_head].function_status=STATUS_NOT_STARTED;
		}
	}

	return -1;
}

int8_t ShowUSSD_SIM800(uint8_t step){
	uint32_t tick=HAL_GetTick();
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){
		_BeginBuffersForUartTransmit();
		_SendCommandNonblocking((uint8_t *)"AT+CUSD=1\r");
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		_BeginBuffersForUartTransmit();
		sprintf((char *)sim800command, "AT+CUSD=1,\"%s\"\r", ussd_command);
		funcStatus[current_func_head].timeout=5000;
		_SendCommandNonblocking(sim800command);
		return -1;
	}
	if(step==3 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		char *pch1=_CheckAck((uint8_t*)  "+CUSD:");
		if(pch1){
			uint8_t param_start=10; // +CUSD:1, "USSD reply from operator"
			uint8_t len=strlen(pch1);
			memset( smsDataReceived.data , '\0' , sizeof(smsDataReceived.data) );
			memcpy( smsDataReceived.data,pch1+10,len-param_start);
			funcStatus[current_func_head].function_status=STATUS_COMPLETED;
			return 1;
		}
	}

	return -1;
}

int8_t CallAdmin_SIM800(uint8_t step){
	uint32_t tick=HAL_GetTick();
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){
		_BeginBuffersForUartTransmit();
		Debug("Calling ADMIN");
		sprintf((char *)sim800command, "ATD%s;\r", adminnumber);
		_SendCommandNonblocking(sim800command);
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		funcStatus[current_func_head].function_status=STATUS_COMPLETED;
		return -1;
	}
	return -1;
}

int8_t Hangup_SIM800(uint8_t step){
	uint32_t tick=HAL_GetTick();
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){
		_BeginBuffersForUartTransmit();
		_SendCommandNonblocking((uint8_t*)"ATH\r\n");
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		funcStatus[current_func_head].function_status=STATUS_COMPLETED;
		return -1;
	}
	return -1;
}

int8_t DeleteSMS_SIM800(uint8_t step){
	uint32_t tick=HAL_GetTick();
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){
		_BeginBuffersForUartTransmit();
		sprintf((char *)sim800command, "Deleting SMS %d", funcStatus[current_func_head].param1);
		Debug((const char *)sim800command);
		sprintf((char *)sim800command, "AT+CMGD=%d\r", funcStatus[current_func_head].param1);
		_SendCommandNonblocking(sim800command);
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		funcStatus[current_func_head].function_status=STATUS_COMPLETED;
		return -1;
	}
	return -1;
}

int8_t _ReadSMS(uint8_t step){

	uint32_t tick=HAL_GetTick();
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED   ) {
		rx_event_fired=0;
		smsDataReceived.index++;
		if(smsDataReceived.index>_sim800_status.sms_total1){
			funcStatus[current_func_head].function_status=STATUS_COMPLETED;
			smsDataReceived.index=0;
			return 1;
		}
		smsDataReceived.valid=0;
		_BeginBuffersForUartTransmit();
		sprintf((char *)sim800command, "AT+CMGR=%d\r", smsDataReceived.index);
		_SendCommandNonblocking(sim800command);
		return -1;
	}
	if(step==2 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;

		char * pch1=_CheckAck( (uint8_t*)"+CMGR:");
		if(pch1){
			funcStatus[current_func_head].step=1;
			funcStatus[current_func_head].step_status=STATUS_NOT_STARTED;
			int res = sscanf(pch1, "%*[^,],\"%[^\"]\",%*[^,],\"%[^\"]\"\r\n%[^\r]", oa, scts, data);
			if(res){
				smsDataReceived.valid=1;
				smsDataReceived.processed=0;
				strcpy(smsDataReceived.oa, oa);
				strcpy(smsDataReceived.scts, scts);
				strcpy(smsDataReceived.data, data);
			}
		}else{
			char * pch1=_CheckAck((uint8_t*) AT_OK);
			char * pch2=_CheckAck( (uint8_t*)"\r\nERROR\r\n");
			if(pch1 || pch2){
				funcStatus[current_func_head].step=1;
				funcStatus[current_func_head].step_status=STATUS_NOT_STARTED;
			}
		}


		if(pch1){
			funcStatus[current_func_head].step=1;
			funcStatus[current_func_head].step_status=STATUS_NOT_STARTED;
			return smsDataReceived.index;
		}else{
			return -1;
		}

	}
	return -1;
}

int8_t _CheckSMS(uint8_t step){
	uint32_t tick=HAL_GetTick();

	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){

		_BeginBuffersForUartTransmit();
		_SendCommandNonblocking((uint8_t*)"AT+CPMS?\r\n");
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		char * pch1=_CheckAck( (uint8_t*)"+CPMS:");
		if(pch1){
			funcStatus[current_func_head].function_status=STATUS_COMPLETED;
			sscanf(pch1, "%*[^,],%d,%d,%*[^,],%d,%d,%*[^,],%d,%d",(int*) &_sim800_status.sms_used1, (int*)&_sim800_status.sms_total1,
					(int*)&_sim800_status.sms_used2, (int*)&_sim800_status.sms_total2, (int*)&_sim800_status.sms_used3,(int*) &_sim800_status.sms_total3);

			return _sim800_status.sms_used1;
		}
		if(timeout_fired){
			funcStatus[current_func_head].function_status=STATUS_COMPLETED;
			_ClearSMSStatus();
			return 0;
		}
		return -1;

	}
	return -1;
}


int8_t CheckNetworkRegistration_SIM800(uint8_t step){
	uint32_t tick=HAL_GetTick();
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){
		_BeginBuffersForUartTransmit();
		_SendCommandNonblocking((uint8_t*)"AT+COPS?\r");
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired)){
		rx_event_fired=0;
		char *pch1=_CheckAck((uint8_t*)  "+COPS:");
		if(pch1){
			funcStatus[current_func_head].function_status=STATUS_COMPLETED;
			char *pch3 = strstr(pch1, ",");
			if(pch3){
				_sim800_status.sim_network_registered=1;
				return 1;
			}
		}
		if(timeout_fired){
			funcStatus[current_func_head].function_status=STATUS_COMPLETED;
			_sim800_status.sim_network_registered=0;
			return 0;
		}
		return -1;
	}
	return -1;

}

int8_t CheckNetworkSignalStrength_SIM800(uint8_t step){
	uint32_t tick=HAL_GetTick();
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){
		_BeginBuffersForUartTransmit();
		_SendCommandNonblocking((uint8_t*)"AT+CSQ\r");
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired)){
		rx_event_fired=0;
		if(timeout_fired)funcStatus[current_func_head].function_status=STATUS_COMPLETED;
		char *pch1=_CheckAck((uint8_t*) "+CSQ:");
		if(pch1 ){
			funcStatus[current_func_head].function_status=STATUS_COMPLETED;
			char *pch3 = strstr(pch1, ",");
			if(pch3){
				char subtext[4];
				memset( subtext , '\0' , 4 );
				memcpy(subtext,pch1+5,(pch3-pch1)-5);
				int x = atoi(subtext);
				uint8_t ret=_GetCSQStrength(x);
				_sim800_status.sim_network_quality=ret;
				return ret;
			}
		}else{
			if(timeout_fired){
				_sim800_status.sim_network_quality=0;
				return 0;
			}
		}
		return 0;
	}
	return -1;
}


int8_t CheckSIMCardInserted_SIM800(uint8_t step){

	uint32_t tick=HAL_GetTick();
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED){
		_BeginBuffersForUartTransmit();
		_SendCommandNonblocking((uint8_t*)"AT+CSMINS?\r");
		return -1;
	}
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==2 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		if(timeout_fired)funcStatus[current_func_head].function_status=STATUS_COMPLETED;
		char * pch1=_CheckAck((uint8_t*) "+CSMINS:");
		if(pch1){
			funcStatus[current_func_head].function_status=STATUS_COMPLETED;
			char *pch3 = strstr(pch1, ",");
			if(pch3){
				uint8_t sim=*++pch3;
				if(sim=='1'){
					_sim800_status.sim_inserted=1;
					return 1;
				}
			}
			_ClearSMSStatus();
			_sim800_status.sim_inserted=0;
			return 0;
		}else{
			if(timeout_fired){
				_sim800_status.sim_inserted=0;
				return 0;
			}
		}
		return -1;
	}
	return -1;
}

int8_t SendSMS_SIM800(uint8_t step){
	uint32_t tick=HAL_GetTick();
	uint8_t timeout_fired=(tick-funcStatus[current_func_head].started_time)>funcStatus[current_func_head].timeout;
	if(step==1 && funcStatus[current_func_head].step_status!=STATUS_STARTED   ) {
		rx_event_fired=0;
		_BeginBuffersForUartTransmit();
		_SendCommandNonblocking((uint8_t*)"AT+CMGF=1\r");
		return -1;
	}
	if(step==2 && tx_complete_fired==1 && (timeout_fired || rx_event_fired) ){
		rx_event_fired=0;
		char * pch1=_CheckAck((uint8_t*) "AT+CMGF=1\r\r\nOK\r\n");
		if(pch1){
			_BeginBuffersForUartTransmit();
			sprintf((char*)sim800command, "AT+CMGS=\"%s\"\r",smsDataForSend.oa);
			_SendCommandNonblocking(sim800command);
		}else if(timeout_fired  ){
			funcStatus[current_func_head].step=1;
			return -1;
		}

		return -1;
	}
	if(step==3 && tx_complete_fired==1 && (timeout_fired || rx_event_fired)){
		sprintf((char*)sim800command, "AT+CMGS=\"%s\"\r\r\n>",smsDataForSend.oa);
		char * pch1=_CheckAck((uint8_t*) sim800command);
		if(pch1){
			_BeginBuffersForUartTransmit();
			sprintf((char*)sim800command, "%s%c",smsDataForSend.data,26);//(char)26 Special termination character for SIM800
			_SendCommandNonblocking((uint8_t*)sim800command );//smsDataForSend.data
		}

		return -1;
	}
	if(step==4 && tx_complete_fired==1 && (timeout_fired || rx_event_fired)  ){
		rx_event_fired=0;
		funcStatus[current_func_head].function_status=STATUS_COMPLETED;
		return -1;
	}
	return -1;
}
