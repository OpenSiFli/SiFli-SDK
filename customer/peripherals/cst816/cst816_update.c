#include <rtthread.h>
#include <string.h>
#include <stdbool.h>
#include "drv_io.h"
#include "capacitive_hynitron_cst816d_update.h"


#define  DBG_LEVEL           DBG_INFO  //DBG_LOG //
#define LOG_TAG              "update.cst918"
#include <drv_log.h>



#define REG_LEN_1B   1
#define REG_LEN_2B   2
#define UP_ADDR      0x6A

#define delay_ms(ms) rt_thread_mdelay(ms)



extern uint32_t cst816_i2c_write(uint8_t deive_addr ,uint16_t reg, uint8_t *p_data, uint16_t len);
extern uint32_t cst816_i2c_read(uint8_t deive_addr, const uint16_t reg, uint8_t *p_data, uint16_t len);




bool TP_HRS_WriteBytes_updata(uint8_t device_addr,uint16_t reg, uint8_t *data,uint16_t len,uint8_t lenth)
{
    uint32_t ret;
    
    ret = cst816_i2c_write(device_addr, reg, data, len);

    if(ret < 0)
    {
        return 0;
    }
    return 1;
}

bool TP_HRS_read_updata(uint8_t device_addr,uint16_t reg, uint8_t *data,uint16_t len,uint8_t lenth)
{   
    uint32_t ret;
    
    ret = cst816_i2c_read(device_addr, reg, data,  len);

    if(ret < 0)
    {
        return 0;
    }
    return 1;
}


/*****************************************************************/
// For CSK0xx update
 /*
  *
  */	
static int cst816s_enter_bootmode(void)
{
     char retryCnt = 50;

     BSP_TP_Reset(0);
     delay_ms(10);
     BSP_TP_Reset(1);
     delay_ms(10);
     
     while(retryCnt--)
     {
            uint8_t cmd[3];
            cmd[0] = 0xAB;
            if ( TP_HRS_WriteBytes_updata(UP_ADDR ,0xA001,cmd,1,REG_LEN_2B))
            {  // enter program mode
               delay_ms(2); // 4ms
            //   continue;                   
            }
            delay_ms(2); // 4ms
            TP_HRS_read_updata(UP_ADDR ,0xA003,cmd,1,REG_LEN_2B);
            		
            	 
            if (cmd[0] == 0xC1){
                delay_ms(2); // 4ms
                return 0;
            }
            delay_ms(2); // 4ms
     }
	 return -1;
}


/*
  *
  */	
#define PER_LEN	512
uint8_t  data_send[PER_LEN];

static int cst816s_update(uint16_t startAddr,uint16_t len,const unsigned char *src)
{
   
    uint32_t sum_len;
    uint8_t cmd[10];


    sum_len = 0;
    uint32_t  k_data=0,b_data=0;
    k_data=len/PER_LEN;
    
    if (cst816s_enter_bootmode() == 0)
    {
       // return -1;
    }	
    for(uint32_t i=0;i<k_data;i++)
    {
         cmd[0] = startAddr&0xFF;
         cmd[1] = startAddr>>8;
         TP_HRS_WriteBytes_updata(UP_ADDR ,0xA014,cmd,2,REG_LEN_2B);
         memcpy(data_send,src,PER_LEN);
         TP_HRS_WriteBytes_updata(UP_ADDR ,0xA018,data_send,PER_LEN,REG_LEN_2B);
         delay_ms(10);
         cmd[0] = 0xEE;
         TP_HRS_WriteBytes_updata(UP_ADDR ,0xA004,cmd,1,REG_LEN_2B);
         delay_ms(50);

        // {
             uint8_t retrycnt = 50;
    		 uint16_t  kcnt=0;
             while(retrycnt--)
    		 {
                 cmd[0] = 0;
                 TP_HRS_read_updata(UP_ADDR ,0xA005,cmd,1,REG_LEN_2B);
                 LOG_I("cmd=%x \r\n",cmd[0]);
                 if (cmd[0] == 0x55)
				 {
					 kcnt=0;
					 cmd[0] = 0;
					 // success
					 break;
                 }
				 else
				 {
					 kcnt++;
					//LOG_I("error \r\n");
				 }
                 delay_ms(10);
               }
			   if(kcnt>=40)
			   LOG_I("error \r\n");
        // }
           startAddr += PER_LEN;
           src += PER_LEN;
           sum_len += PER_LEN;

     }

	 k_data=len%PER_LEN;
	 if(k_data>0)
	 {
		cmd[0] = startAddr&0xFF;
		cmd[1] = startAddr>>8;
		TP_HRS_WriteBytes_updata(UP_ADDR ,0xA014,cmd,2,REG_LEN_2B);
	    memcpy(data_send,src,k_data);
		TP_HRS_WriteBytes_updata(UP_ADDR ,0xA018,data_send,k_data,REG_LEN_2B);
		  
		cmd[0] = 0xEE;
		TP_HRS_WriteBytes_updata(UP_ADDR ,0xA004,cmd,1,REG_LEN_2B);
		 
		delay_ms(100);
		 
		 
		{
            uint8_t retrycnt = 50;
            while(retrycnt--){
             cmd[0] = 0;
             TP_HRS_read_updata(UP_ADDR ,0xA005,cmd,1,REG_LEN_2B);
             if (cmd[0] == 0x55){
                 // success
                 break;
             }
             delay_ms(10);
             }
        }
		startAddr += k_data;
        src += k_data;
        sum_len += k_data;
	 }
    		 
    		  		
    		 
     // exit program mode
    cmd[0] = 0x00;
    TP_HRS_WriteBytes_updata(UP_ADDR ,0xA003,cmd,1,REG_LEN_2B);
    /*
    delay_ms(10);
    BSP_TP_Reset(0);
    delay_ms(100);
    BSP_TP_Reset(1);
    delay_ms(10);  
    */
    return 0;
     
 }


  /*
   *
   */
 static uint32_t cst816s_read_checksum(uint16_t startAddr,uint16_t len)
 {
      union{
          uint32_t sum;
          uint8_t buf[4];
      }checksum;
      uint8_t cmd[3];
      char readback[4] = {0};

      if (cst816s_enter_bootmode() == 0){
        // return -1;
      }
      
      cmd[0] = 0;
      TP_HRS_WriteBytes_updata(UP_ADDR ,0xA003,cmd,1,REG_LEN_2B);
      delay_ms(500);

      checksum.sum = 0;
      TP_HRS_read_updata(UP_ADDR ,0xA008,checksum.buf,2,REG_LEN_2B);
       //   return -1;
      LOG_I("checksum.sum=%x \r\n",checksum.sum);
      return checksum.sum;
 }

 
 uint16_t checksum=0;
 bool ctp_hynitron_update(void)
  {
      uint8_t lvalue;
      uint8_t write_data[2];
      bool temp_result = true;
 
 //  TP_init();
     // test_data=cst816s_enter_bootmode();
     //hctp_i2c_init(UP_ADDR ,50);     
    if (cst816s_enter_bootmode() == 0)
    {
         LOG_I("cst816s_enter_bootmode()=%x \r\n",cst816s_enter_bootmode());
         if(sizeof(app_bin) > 10)
         {
             uint16_t startAddr = app_bin[1];
             uint16_t length = app_bin[3];
                         
             checksum = app_bin[5];
             startAddr <<= 8; startAddr |= app_bin[0];
             length <<= 8; length |= app_bin[2];
             checksum <<= 8; checksum |= app_bin[4];   
             if(cst816s_read_checksum(startAddr, length)!= checksum)
             {
                LOG_I("startAddrO=%d \r\n",startAddr);
                LOG_I("checksum=%x \r\n",checksum);
                cst816s_update(startAddr, length, &app_bin[6]);
                LOG_I("startAddrT=%d \r\n",startAddr);
                LOG_I("checksum_new=%x \r\n",checksum);
                cst816s_read_checksum(startAddr, length);        
             }
         }
				 BSP_TP_Reset(0);
				 delay_ms(20);
				 BSP_TP_Reset(1);
				 delay_ms(30);
         return true;
     }

 
     return false;
 }



