#include "Screen.h"
//static QUEUE que = {0,0,0};                                            //指令队列
static uint32_t cmd_state = 0;                                           //队列帧尾检测状态
static unsigned short cmd_pos = 0;                                              //当前指令指针位置
#define CMD_HEAD 0XEE                                                  //帧头
#define CMD_TAIL 0XFFFCFFFF                                            //帧尾
#define CMD_MAX_SIZE 64                                            //帧尾

uint8_t  cmd_buffer[CMD_MAX_SIZE];                                       //指令缓存







unsigned short queue_find_cmd(unsigned char *buffer,unsigned short buf_len)
//qsize=unsigned short
//buf_len=64 .CMD_MAX_SIZE 64 
{
    unsigned short cmd_size = 0;
    unsigned char _data = 0;

//     while(queue_size()>0)
//     {
//         //取一个数据
//         queue_pop(&_data);

//         if(cmd_pos==0&&_data!=CMD_HEAD)                               //指令第一个字节必须是帧头，否则跳过
//         {
//             continue;
//         }
//         //    LED2_ON;
//         if(cmd_pos<buf_len)                                           //防止缓冲区溢出
//             buffer[cmd_pos++] = _data;

//         cmd_state = ((cmd_state<<8)|_data);                           //拼接最后4个字节，组成一个32位整数

//         //最后4个字节与帧尾匹配，得到完整帧
//         if(cmd_state==CMD_TAIL)
//         {
//             //LED2_ON;
//             cmd_size = cmd_pos;                                       //指令字节长度
//             cmd_state = 0;                                            //重新检测帧尾巴
//             cmd_pos = 0;                                              //复位指令指针

// #if(CRC16_ENABLE)
//             //去掉指令头尾EE，尾FFFCFFFF共计5个字节，只计算数据部分CRC
//             if(!CheckCRC16(buffer+1,cmd_size-5))                      //CRC校验
//                 return 0;

//             cmd_size -= 2;                                            //去掉CRC16（2字节）
// #endif
//             return cmd_size;
//         }
//     }
//     return 0;                                                         //没有形成完整的一帧
}





void processData(PCTRL_MSG msg, uint16_t size){
    //HAL_UART_Transmit(&huart1, (uint8_t *)&buffer, sizeof(buffer), 0xFFFF);

 uint8_t cmd_type = msg->cmd_type;                                                  //指令类型
 HAL_UART_Transmit(&huart1, (uint8_t *)&cmd_type, sizeof(cmd_type), 0xFFFF);
    // uint8_t ctrl_msg = msg->ctrl_msg;                                                  //消息的类型
    // uint8_t control_type = msg->control_type;                                          //控件类型
    // uint16_t screen_id = PTR2U16(&msg->screen_id);                                     //画面ID
    // uint16_t control_id = PTR2U16(&msg->control_id);                                   //控件ID
    // uint32_t value = PTR2U32(msg->param);                                              //数值


    switch(cmd_type)
    {  
    // case NOTIFY_TOUCH_PRESS:  //触摸屏按下                                                      
    // case NOTIFY_TOUCH_RELEASE: //触摸屏松开                                                     
    //     NotifyTouchXY(cmd_buffer[1],PTR2U16(cmd_buffer+2),PTR2U16(cmd_buffer+4)); 
    //     break;                                                                    
    // case NOTIFY_WRITE_FLASH_OK: //写FLASH成功                                                    
    //     NotifyWriteFlash(1);                                                      
    //     break;                                                                    
    // case NOTIFY_WRITE_FLASH_FAILD://写FLASH失败                                                  
    //     NotifyWriteFlash(0);                                                      
    //     break;                                                                    
    // case NOTIFY_READ_FLASH_OK: //读取FLASH成功                                                     
    //     NotifyReadFlash(1,cmd_buffer+2,size-6); //去除帧头帧尾                                    
    //     break;                                                                    
    // case NOTIFY_READ_FLASH_FAILD: //读取FLASH失败                                                  
    //     NotifyReadFlash(0,0,0);                                                   
    //     break;                                                                    
    // case NOTIFY_READ_RTC: //读取RTC时间                                                          
    //     NotifyReadRTC(cmd_buffer[2],cmd_buffer[3],cmd_buffer[4],cmd_buffer[5],cmd_buffer[6],cmd_buffer[7],cmd_buffer[8]);
    //     break;
    // case NOTIFY_CONTROL:
    //     {
    //         if(ctrl_msg==MSG_GET_CURRENT_SCREEN) //画面ID变化通知                                   
    //         {
    //             NotifyScreen(screen_id);//画面切换调动的函数                                            
    //         }
    //         else
    //         {
    //             switch(control_type)
    //             {
    //             case kCtrlButton: //按钮控件                                                  
    //                 NotifyButton(screen_id,control_id,msg->param[1]);                  
    //                 break;                                                             
    //             case kCtrlText: //文本控件                                                    
    //                 NotifyText(screen_id,control_id,msg->param);                       
    //                 break;                                                             
    //             case kCtrlProgress: //进度条控件                                                
    //                 NotifyProgress(screen_id,control_id,value);                        
    //                 break;                                                             
    //             case kCtrlSlider: //滑动条控件                                                  
    //                 NotifySlider(screen_id,control_id,value);                          
    //                 break;                                                             
    //             case kCtrlMeter:  //仪表控件                                                  
    //                 NotifyMeter(screen_id,control_id,value);                           
    //                 break;                                                             
    //             case kCtrlMenu: //菜单控件                                                    
    //                 NotifyMenu(screen_id,control_id,msg->param[0],msg->param[1]);      
    //                 break;                                                              
    //             case kCtrlSelector:   //选择控件                                              
    //                 NotifySelector(screen_id,control_id,msg->param[0]);                
    //                 break;                                                              
    //             case kCtrlRTC: //倒计时控件                                                     
    //                 NotifyTimer(screen_id,control_id);
    //                 break;
    //             default:
    //                 break;
    //             }
    //         } 
    //         break;  
    //     } 
    // case NOTIFY_HandShake:   //握手通知                                                                                                            
    //     NOTIFYHandShake();
    //     break;
    default:
        break;
    }

}

