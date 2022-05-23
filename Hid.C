
/********************************** (C) COPYRIGHT *******************************
 * File Name          :CompatibilityHID.C
 * Author             : WCH
 * Version            : V1.2
 * Date               : 2018/02/28
 * Description        : CH554模拟HID兼容设备，支持中断上下传，支持控制端点上下传，支持全速传输
 *******************************************************************************/

#include "./Public/CH552.H"
#include "./Public/Debug.H"
#include <stdio.h>
#include <string.h>

#define THIS_ENDP0_SIZE 64
#define ENDP2_IN_SIZE 64
#define ENDP2_OUT_SIZE 64

UINT8X Ep0Buffer[MIN(64, THIS_ENDP0_SIZE + 2)] _at_ 0x0000;                                                   //端点0 OUT&IN缓冲区，必须是偶地址
UINT8X Ep2Buffer[MIN(64, ENDP2_IN_SIZE + 2) + MIN(64, ENDP2_OUT_SIZE + 2)] _at_ MIN(64, THIS_ENDP0_SIZE + 2); //端点2 IN&OUT缓冲区,必须是偶地址

UINT8 SetupReq, SetupLen, Ready, Count, UsbConfig;
PUINT8 pDescr;             // USB配置标志
USB_SETUP_REQ SetupReqBuf; //暂存Setup包

#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)

sbit Ep2InKey = P1 ^ 5; // K1按键
#pragma NOAREGS

/*设备描述符*/
UINT8C DevDesc[18] = {
    0x12,            // bLength
    0x01,            // bDescriptorType
    0x00, 0x02,      // bcdUSB (2.0)
    0x00,            // bDeviceClass
    0x00,            // bDeviceSubClass
    0x00,            // bDeviceProtocol
    THIS_ENDP0_SIZE, // 0x08,       // bMaxPacketSize0
    0xFF, 0xFF,      // idEVendor (Microsoft Corp.)
    0x00, 0x01,      // idProduct (Xbox360 Controller)
    0x00, 0x01,      // bcdDevice
    0x00,            // 0x01,        // iManufacturer
    0x00,            // 0x02,        // iProduct
    0x00,            // 0x03,        // iSerialNumber
    0x01             // bNumConfigurations
};
UINT8C CfgDesc[34] = {
    // Configuration Descriptor
    0x09,       // bLength
    0x02,       // bDescriptorType (CONFIGURATION)
    0x22, 0x00, // wTotalLength (34)
    0x01,       // bNumInterfaces
    0x01,       // bConfigurationValue
    0x00,       // iConfiguration
    0xA0,       // bmAttributes
    0x32,       // bMaxPower

    /* ---------------------------------------------------- */
    // Interface 0:
    0x09, // bLength
    0x04, // bDescriptorType (INTERFACE)
    0x00, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints
    0x03, // bInterfaceClass
    0x00, // bInterfaceSubClass
    0x00, // bInterfaceProtocol
    0x00, // iInterface

    // HID Descriptor (If0)
    0x09,       // bLength
    0x21,       // bDescriptorType
    0x00, 0x01, // bcdHid
    0x00,       // bCountryCode
    0x01,       // bNumDescription
    0x22,       // bDescriptionType (Report descripiton type)
    0x1A, 0x00, // Total length of Report description

    // Endpoint 1:
    0x07,       // bLength
    0x05,       // bDescriptorType (ENDPOINT)
    0x82,       // bEndpointAddress (IN, 1)
    0x03,       // bmAttributes (Interrupt)
    0x01, 0x00, // wMaxPacketSize
    0x04        // bInterval ms
};
/*字符串描述符 略*/

/*HID类报表描述符*/
UINT8C HIDRepDesc[] =
    {
        0x05, 0x01, // USAGE_PAGE (Generic Desktop)
        0x09, 0x05, // USAGE (Game Pad)
        0xa1, 0x01, // COLLECTION (Application)
        // 0xa1, 0x00, //   COLLECTION (Physical)
        0xa1, 0x02, //   COLLECTION (Logical)
        // ReportID - 8 bits
        // 0x85, 0x01,                    //     REPORT_ID (1)
        // Buttons - 8 bits
        0x05, 0x09, //     USAGE_PAGE (Button)
        0x19, 0x01, //     USAGE_MINIMUM (Button 1)
        0x29, 0x08, //     USAGE_MAXIMUM (Button 8)
        0x15, 0x00, //     LOGICAL_MINIMUM (0)
        0x25, 0x01, //     LOGICAL_MAXIMUM (1)
        0x75, 0x01, //     REPORT_SIZE (1)
        0x95, 0x08, //     REPORT_COUNT (8)
        0x81, 0x02, //     INPUT (Data,Var,Abs)
        0xc0,       //     END_COLLECTION
        0xc0        // END_COLLECTION
};

UINT8 UserEp2Buf[1] = {0x00}; //用户数据定义
UINT8 Endp2Busy = 0;

/*******************************************************************************
 * Function Name  : USBDeviceInit()
 * Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USBDeviceInit()
{
    IE_USB = 0;
    USB_CTRL = 0x00;             // 先设定USB设备模式
    UDEV_CTRL = bUD_PD_DIS;      // 禁止DP/DM下拉电阻
    UDEV_CTRL &= ~bUD_LOW_SPEED; //选择全速12M模式，默认方式
    USB_CTRL &= ~bUC_LOW_SPEED;
    UEP2_DMA = Ep2Buffer;                       //端点2数据传输地址
    UEP2_3_MOD |= bUEP2_TX_EN | bUEP2_RX_EN;    //端点2发送接收使能
    UEP2_3_MOD &= ~bUEP2_BUF_MOD;               //端点2收发各64字节缓冲区
    UEP0_DMA = Ep0Buffer;                       //端点0数据传输地址
    UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN); //端点0单64字节收发缓冲区
    USB_DEV_AD = 0x00;
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
    UDEV_CTRL |= bUD_PORT_EN;                              // 允许USB端口
    USB_INT_FG = 0xFF;                                     // 清中断标志
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;
}

/*******************************************************************************
 * Function Name  : Enp2BlukIn()
 * Description    : USB设备模式端点2的批量上传
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void Enp2BlukIn()
{
    memcpy(Ep2Buffer + MAX_PACKET_SIZE, UserEp2Buf, sizeof(UserEp2Buf)); //加载上传数据
    UEP2_T_LEN = sizeof(UserEp2Buf);                                     //上传最大包长度
    UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;             //有数据时上传数据并应答ACK
}

/*******************************************************************************
 * Function Name  : DeviceInterrupt()
 * Description    : CH559USB中断处理函数
 *******************************************************************************/
void DeviceInterrupt(void) interrupt INT_NO_USB using 1 // USB中断服务程序,使用寄存器组1
{
    UINT8 len, i;
    if (UIF_TRANSFER) // USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 2:       // endpoint 2# 端点上传
            UEP2_T_LEN = 0;          //预使用发送长度一定要清空
            UEP2_CTRL ^= bUEP_T_TOG; //手动翻转
            Endp2Busy = 0;
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //默认应答NAK
            break;
        // case UIS_TOKEN_OUT | 2: // endpoint 2# 端点下传
        //     if (U_TOG_OK)       // 不同步的数据包将丢弃
        //     {
        //         len = USB_RX_LEN;        //接收数据长度，数据从Ep2Buffer首地址开始存放
        //         UEP2_CTRL ^= bUEP_R_TOG; //手动翻转
        //         for (i = 0; i < len; i++)
        //         {
        //             Ep2Buffer[MAX_PACKET_SIZE + i] = Ep2Buffer[i] ^ 0xFF; // OUT数据取反到IN由计算机验证
        //         }
        //         UEP2_T_LEN = len;
        //         UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // 允许上传
        //     }
        //     break;
        case UIS_TOKEN_SETUP | 0: // SETUP事务
            UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
            len = USB_RX_LEN;
            if (len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                len = 0; // 默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;
                if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) /*HID类命令*/
                {
                    switch (SetupReq)
                    {
                    case 0x01:                           // GetReport
                        pDescr = UserEp2Buf;             //控制端点上传输据
                        if (SetupLen >= THIS_ENDP0_SIZE) //大于端点0大小，需要特殊处理
                        {
                            len = THIS_ENDP0_SIZE;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    case 0x02: // GetIdle
                        break;
                    case 0x03: // GetProtocol
                        break;
                    case 0x09: // SetReport
                        break;
                    case 0x0A: // SetIdle
                        break;
                    case 0x0B: // SetProtocol
                        break;
                    default:
                        len = 0xFF; /*命令不支持*/
                        break;
                    }
                    if (SetupLen > len)
                    {
                        SetupLen = len; //限制总长度
                    }
                    len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
                    memcpy(Ep0Buffer, pDescr, len);                                 //加载上传数据
                    SetupLen -= len;
                    pDescr += len;
                }
                else //标准请求
                {
                    switch (SetupReq) //请求码
                    {
                    case USB_GET_DESCRIPTOR:
                        switch (UsbSetupBuf->wValueH)
                        {
                        case 1:               //设备描述符
                            pDescr = DevDesc; //把设备描述符送到要发送的缓冲区
                            len = sizeof(DevDesc);
                            break;
                        case 2:               //配置描述符
                            pDescr = CfgDesc; //把设备描述符送到要发送的缓冲区
                            len = sizeof(CfgDesc);
                            break;
                        case 0x22:               //报表描述符
                            pDescr = HIDRepDesc; //数据准备上传
                            len = sizeof(HIDRepDesc);
                            break;
                        default:
                            len = 0xff; //不支持的命令或者出错
                            break;
                        }
                        if (SetupLen > len)
                        {
                            SetupLen = len; //限制总长度
                        }
                        len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
                        memcpy(Ep0Buffer, pDescr, len);                                 //加载上传数据
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL; //暂存USB设备地址
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if (SetupLen >= 1)
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        if (UsbConfig)
                        {
                            Ready = 1; // set config命令一般代表usb枚举完成的标志
                        }
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                                         // Clear Feature
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // 端点
                        {
                            switch (UsbSetupBuf->wIndexL)
                            {
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF; // 不支持的端点
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF; // 不是端点不支持
                        }
                        break;
                    case USB_SET_FEATURE:                               /* Set Feature */
                        if ((UsbSetupBuf->bRequestType & 0x1F) == 0x00) /* 设置设备 */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
                            {
                                if (CfgDesc[7] & 0x20)
                                {
                                    /* 设置唤醒使能标志 */
                                }
                                else
                                {
                                    len = 0xFF; /* 操作失败 */
                                }
                            }
                            else
                            {
                                len = 0xFF; /* 操作失败 */
                            }
                        }
                        else if ((UsbSetupBuf->bRequestType & 0x1F) == 0x02) /* 设置端点 */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
                            {
                                switch (((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
                                {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
                                    break;
                                default:
                                    len = 0xFF; /* 操作失败 */
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF; /* 操作失败 */
                            }
                        }
                        else
                        {
                            len = 0xFF; /* 操作失败 */
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if (SetupLen >= 2)
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff; //操作失败
                        break;
                    }
                }
            }
            else
            {
                len = 0xff; //包长度错误
            }
            if (len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
            }
            else if (len <= THIS_ENDP0_SIZE) //上传数据或者状态阶段返回0长度包
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1，返回应答ACK
            }
            else
            {
                UEP0_T_LEN = 0;                                                      //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1,返回应答ACK
            }
            break;
        case UIS_TOKEN_IN | 0: // endpoint0 IN
            switch (SetupReq)
            {
            case USB_GET_DESCRIPTOR:
            case HID_GET_REPORT:
                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
                memcpy(Ep0Buffer, pDescr, len);                                 //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG; //同步标志位翻转
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0; //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        // case UIS_TOKEN_OUT | 0: // endpoint0 OUT
        //     len = USB_RX_LEN;
        //     if (SetupReq == 0x09)
        //     {
        //         if (Ep0Buffer[0])
        //         {
        //             printf("Light on Num Lock LED!\n");
        //         }
        //         else if (Ep0Buffer[0] == 0)
        //         {
        //             printf("Light off Num Lock LED!\n");
        //         }
        //     }
        //     UEP0_CTRL ^= bUEP_R_TOG; //同步标志位翻转
        //     break;
        default:
            break;
        }
        UIF_TRANSFER = 0; //写0清空中断
    }
    if (UIF_BUS_RST) //设备模式USB总线复位中断
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        Endp2Busy = 0;
        UIF_BUS_RST = 0; //清中断标志
    }
    if (UIF_SUSPEND) // USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) //挂起
        {
#ifdef DE_PRINTF
            printf("zz\n"); //睡眠状态
#endif
            //             while ( XBUS_AUX & bUART0_TX )
            //             {
            //                 ;    //等待发送完成
            //             }
            //             SAFE_MOD = 0x55;
            //             SAFE_MOD = 0xAA;
            //             WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO;                                   //USB或者RXD0有信号时可被唤醒
            //             PCON |= PD;                                                               //睡眠
            //             SAFE_MOD = 0x55;
            //             SAFE_MOD = 0xAA;
            //             WAKE_CTRL = 0x00;
        }
    }
    else
    {                      //意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF; //清中断标志
                           //      printf("UnknownInt  N");
    }
}

void initGPIO()
{
    P1_MOD_OC = P1_MOD_OC | 0x4F;
    P1_DIR_PU = P1_DIR_PU | 0x4F;
    P3_MOD_OC = P3_MOD_OC | 0x38;
    P3_DIR_PU = P3_DIR_PU | 0x38;
    // P1_MOD_OC = P1_MOD_OC & ~0x4F;
    // P1_DIR_PU = P1_DIR_PU & ~0x4F;
    // P3_MOD_OC = P3_MOD_OC & ~0x38;
    // P3_DIR_PU = P3_DIR_PU & ~0x38;
}

main()
{
    UINT8 keyStatus, port1, port3;
    CfgFsys();    // CH559时钟选择配置
    mDelaymS(5);  //修改主频等待内部晶振稳定,必加
    mInitSTDIO(); //串口0初始化
#ifdef DE_PRINTF
    printf("start ...\n");
#endif
    initGPIO();
    USBDeviceInit(); // USB设备模式初始化
    EA = 1;          //允许单片机中断
    UEP1_T_LEN = 0;  //预使用发送长度一定要清空
    UEP2_T_LEN = 0;  //预使用发送长度一定要清空
    Ready = 0;
#ifdef DE_PRINTF
    printf("while begin ...\n");
#endif
    while (1)
    {
        if (Ready)
        {
            port1 = ~P1 & 0xF2; // 获取P1口 1,4,5,6,7引脚
            port3 = ~P3 & 0x1C; // 获取P3口 2,3,4引脚
            keyStatus = 0x01;
            keyStatus &= (port1 >> 1);   // P1.1 -> bit 0
            keyStatus |= (port1 & 0xF0); // P1.4567 -> bit 4567
            keyStatus |= (port3 >> 1);   // P3.234 -> bit 123
            UserEp2Buf[0] = keyStatus;
            Enp2BlukIn();
            Endp2Busy = 1;
            while (Endp2Busy)
            {
                /* code */
            }
        }
        mDelaymS(3);
    }
}
