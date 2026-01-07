#include "RC522.h"

/*
 * Function Name: RC522_SPI_Transfer
 * Description: A common function used by Write_MFRC522 and Read_MFRC522
 * Input Parameters: data - the value to be written
 * Returns: a byte of data read from the module
 */
uint8_t RC522_SPI_Transfer(uchar data)
{
	uchar rx_data;
	HAL_SPI_TransmitReceive(HSPI_INSTANCE,&data,&rx_data,1,100);
	return rx_data;
}
//설명: Write_MFRC522와 Read_MFRC522에서 공통적으로 사용하는 SPI 전송 함수
//  입력 매개변수: data - 전송할 값
// 반환값: 모듈에서 읽은 바이트 데이터


void Write_MFRC522(uchar addr, uchar val)
{
	/* CS LOW */
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_RESET);


	RC522_SPI_Transfer((addr<<1)&0x7E); //주소
	RC522_SPI_Transfer(val); //데이터

	/* CS HIGH */
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_SET);
}
//SPI로 주소와 데이터를 연속 전송하여 레지스터에 값 저장
// 좌측으로 1비트 시프트 + LSB=0 (쓰기)


uchar Read_MFRC522(uchar addr)
{
	uchar val;

	/* CS LOW */
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_RESET);

	  // even though we are calling transfer frame once, we are really sending
	  // two 8-bit frames smooshed together-- sending two 8 bit frames back to back
	  // results in a spike in the select line which will jack with transactions
	  // - top 8 bits are the address. Per the spec, we shift the address left
	  //   1 bit, clear the LSb, and set the MSb to indicate a read
	  // - bottom 8 bits are all 0s on a read per 8.1.2.1 Table 6
	RC522_SPI_Transfer(((addr<<1)&0x7E) | 0x80);
	val = RC522_SPI_Transfer(0x00);

	/* CS HIGH */
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_SET);
	return val;
}


//주소 전송 후, 1바이트 수신
//주소는 좌측으로 1비트 시프트 + MSB=1 (읽기)

void SetBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);  // set bit mask
}

//레지스터의 특정 비트를 OR 연산으로 설정


void ClearBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
}
//레지스터의 특정 비트를 AND(~mask)로 클리어

void AntennaOn(void)
{

	Read_MFRC522(TxControlReg);
	SetBitMask(TxControlReg, 0x03);
}
//안테나(전송기) 켬


void AntennaOff(void)
{
	ClearBitMask(TxControlReg, 0x03);
}
//안테나 끔


void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}

//RC522 내부 리셋 (CommandReg에 리셋 명령어)

void MFRC522_Init(void)
{
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(MFRC522_RST_PORT,MFRC522_RST_PIN,GPIO_PIN_SET);
	MFRC522_Reset();

	//Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
	Write_MFRC522(TModeReg, 0x8D);		//Tauto=1; f(Timer) = 6.78MHz/TPreScaler
	Write_MFRC522(TPrescalerReg, 0x3E);	//TModeReg[3..0] + TPrescalerReg
	Write_MFRC522(TReloadRegL, 30);
	Write_MFRC522(TReloadRegH, 0);

	Write_MFRC522(TxAutoReg, 0x40);		// force 100% ASK modulation
	Write_MFRC522(ModeReg, 0x3D);		// CRC Initial value 0x6363

	AntennaOn();
}


//SPI 핀, RC522 초기 설정, 안테나 켜기
//CRC, 타이머, 전송 모드 등을 설정

uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen)
{
    uchar status = MI_ERR;
    uchar irqEn = 0x00;        // 인터럽트 허용 마스크
    uchar waitIRq = 0x00;      // 대기할 인터럽트 비트
    uchar lastBits;
    uchar n;
    uint i;

    // 명령 종류에 따라 인터럽트 설정값 다르게 설정
    switch (command)
    {
        case PCD_AUTHENT:       // 카드 인증 명령일 때
        {
            irqEn = 0x12;       // 인증 시 허용할 인터럽트 비트
            waitIRq = 0x10;     // 완료 대기할 인터럽트 비트
            break;
        }
        case PCD_TRANSCEIVE:    // 데이터 송수신 명령일 때
        {
            irqEn = 0x77;       // 송수신 시 허용할 인터럽트 비트
            waitIRq = 0x30;     // 완료 대기할 인터럽트 비트
            break;
        }
        default:
            break;
    }

    Write_MFRC522(CommIEnReg, irqEn|0x80);     // 인터럽트 허용 설정 (IRQ 핀 활성화 포함)
    ClearBitMask(CommIrqReg, 0x80);            // 인터럽트 요청 비트 초기화
    SetBitMask(FIFOLevelReg, 0x80);            // FIFO 버퍼 비우기

    Write_MFRC522(CommandReg, PCD_IDLE);       // IDLE 명령으로 현재 명령 취소

    // FIFO 버퍼에 데이터 쓰기
    for (i = 0; i < sendLen; i++)
    {
        Write_MFRC522(FIFODataReg, sendData[i]);
    }

    // 명령 실행
    Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80);       // StartSend = 1, 데이터 전송 시작
    }

    // 응답 수신 대기 (최대 약 25ms)
    i = 2000;
    do
    {
        n = Read_MFRC522(CommIrqReg);          // 인터럽트 상태 레지스터 읽기
        i--;
    }
    while ((i != 0) && !(n & 0x01) && !(n & waitIRq)); // Idle 또는 원하는 IRQ 비트가 켜질 때까지 대기

    ClearBitMask(BitFramingReg, 0x80);         // StartSend = 0

    if (i != 0)
    {
        // 에러 레지스터 확인 (버퍼 오버플로우, 충돌, CRC 오류, 프로토콜 오류 등)
        if (!(Read_MFRC522(ErrorReg) & 0x1B))
        {
            status = MI_OK;
            if (n & irqEn & 0x01)              // 카드가 존재하지 않는 경우
            {
                status = MI_NOTAGERR;
            }

            if (command == PCD_TRANSCEIVE)
            {
                n = Read_MFRC522(FIFOLevelReg);        // 수신된 바이트 수
                lastBits = Read_MFRC522(ControlReg) & 0x07;  // 마지막 바이트의 유효 비트 수
                if (lastBits)
                {
                    *backLen = (n - 1) * 8 + lastBits; // 전체 비트 수 계산
                }
                else
                {
                    *backLen = n * 8;
                }

                if (n == 0)
                    n = 1;
                if (n > MAX_LEN)
                    n = MAX_LEN;

                // 수신된 데이터 FIFO에서 읽기
                for (i = 0; i < n; i++)
                {
                    backData[i] = Read_MFRC522(FIFODataReg);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }

    return status;
}


//RC522 ↔ RFID 카드 양방향 통신 함수
//
//명령어(PCD_TRANSCEIVE, PCD_AUTHENT)를 RC522에 보내고 응답을 받음
//
//FIFO에 명령과 데이터를 쓰고, 명령 실행 → 응답 기다리고 읽음
//이 함수는 RC522와 명령 또는 데이터를 송수신할 때 사용하는 핵심 함수입니다.
//sendData를 FIFO에 쓰고,
//명령을 실행하고,
//인터럽트가 뜰 때까지 대기하고,
//결과를 backData로 받아옵니다.
//RFID 카드 인증, 요청, 읽기/쓰기 등에서 모두 이 함수를 통해 통신합니다.


uchar MFRC522_Request(uchar reqMode, uchar *TagType)
{
    uchar status;
    uint backBits;           // 수신된 데이터의 비트 수 (예: 16비트)

    // 마지막 바이트의 유효 비트 수를 7로 설정 (TxLastBits = 7)
    // 카드에게 보낼 요청 명령은 7비트만 필요하므로 이 설정을 통해 마지막 바이트에서 7비트만 전송되도록 함
    Write_MFRC522(BitFramingReg, 0x07);

    TagType[0] = reqMode;    // 요청 모드 설정 (예: 0x26 - 카드 감지 요청)

    // 카드로 명령 전송하고 응답을 기다림
    // TagType 배열을 그대로 사용하여 응답도 여기에 저장
    status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    // 응답이 정상적으로 오지 않거나, 응답 비트 수가 0x10 (16비트)가 아닌 경우 에러 처리
    if ((status != MI_OK) || (backBits != 0x10))
    {
        status = MI_ERR;
    }
    return status;
}
//카드가 있는지 확인 (탐색)
//PICC_REQIDL: 대기 상태 카드 탐색
//성공 시 TagType[0..1]에 카드 타입 저장

uchar MFRC522_Anticoll(uchar *serNum)
{
    uchar status;          // 함수 수행 결과 상태 저장 변수 (성공/실패)
    uchar i;               // 반복문용 변수
    uchar serNumCheck=0;   // 카드 시리얼 번호 체크용 XOR 변수
    uint unLen;            // 응답 비트 길이 저장 변수

    // 비트 프레이밍 레지스터 설정 (전송할 마지막 비트 수 설정)
    // 여기서는 0으로 설정하여 8비트 단위로 전송하도록 함
    Write_MFRC522(BitFramingReg, 0x00);    // TxLastBits = BitFramingReg[2..0]

    // anticollision (충돌 방지) 명령과 관련 데이터 세팅
    serNum[0] = PICC_ANTICOLL;  // anticollision 명령 코드 (0x93)
    serNum[1] = 0x20;           // NVB (Number of Valid Bits) = 0x20 (32비트 유효 데이터 의미)

    // 카드와 데이터 송수신 실행 (anticollision 명령 전송 및 응답 받음)
    // PCD_TRANSCEIVE: 송신 후 수신 명령
    // serNum: 보내는 데이터 버퍼, 길이 2바이트
    // serNum: 수신 데이터 저장 버퍼 (덮어쓰기)
    // unLen: 수신 비트 수 저장
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)  // 명령 성공 시
    {
        // 카드 시리얼 번호 검증 - XOR 검사
        // serNum[0] ~ serNum[3]까지 4바이트가 카드 고유 번호
        for (i = 0; i < 4; i++)
        {
            serNumCheck ^= serNum[i];  // 4바이트를 XOR 누적
        }
        // 마지막 바이트는 시리얼 번호 체크 값(검증용)
        if (serNumCheck != serNum[i])  // serNum[4]와 XOR 결과가 다르면
        {
            status = MI_ERR;  // 오류 상태로 설정 (시리얼 번호 오류)
        }
    }

    return status;  // 명령 수행 결과 반환 (MI_OK 또는 MI_ERR)
}


//카드 충돌 방지: 여러 카드 중 UID(고유 번호) 읽기
//UID는 5바이트, 마지막 바이트는 체크섬


void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData)
{
    uchar i, n;

    ClearBitMask(DivIrqReg, 0x04);			//CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);			//Clear the FIFO pointer

    //Writing data to the FIFO
    for (i=0; i<len; i++)
    {
		Write_MFRC522(FIFODataReg, *(pIndata+i));
	}
    Write_MFRC522(CommandReg, PCD_CALCCRC);

    //Wait CRC calculation is complete
    i = 0xFF;
    do
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));			//CRCIrq = 1

    //Read CRC calculation result
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegH);
}

//CRC 계산을 RC522에 시킴
//FIFO에 데이터 넣고 CRC 명령 실행 → 결과를 읽음


uchar MFRC522_SelectTag(uchar *serNum)
{
	uchar i;
	uchar status;
	uchar size;
	uint recvBits;
	uchar buffer[9];

	//ClearBitMask(Status2Reg, 0x08);			//MFCrypto1On=0

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
    	buffer[i+2] = *(serNum+i);
    }
	CalulateCRC(buffer, 7, &buffer[7]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

    if ((status == MI_OK) && (recvBits == 0x18))
    {
		size = buffer[0];
	}
    else
    {
		size = 0;
	}

    return size;
}

/*
 * Function Name: MFRC522_Auth
 * Description: Verify card password
 * Input parameters: authMode - Password Authentication Mode
                 0x60 = A key authentication
                 0x61 = Authentication Key B
             BlockAddr--Block address
             Sectorkey--Sector password
             serNum--Card serial number, 4-byte
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey, uchar *serNum)
{
    uchar status;
    uint recvBits;
    uchar i;
	uchar buff[12];

	//Verify the command block address + sector + password + card serial number
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i=0; i<6; i++)
    {
		buff[i+2] = *(Sectorkey+i);
	}
    for (i=0; i<4; i++)
    {
		buff[i+8] = *(serNum+i);
	}
    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
    {
		status = MI_ERR;
	}

    return status;
}

/*
 * Function Name: MFRC522_Read
 * Description: Read block data
 * Input parameters: blockAddr - block address; recvData - read block data
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Read(uchar blockAddr, uchar *recvData)
{
    uchar status;
    uint unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }

    return status;
}

/*
 * Function Name: MFRC522_Write
 * Description: Write block data
 * Input parameters: blockAddr - block address; writeData - to 16-byte data block write
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Write(uchar blockAddr, uchar *writeData)
{
    uchar status;
    uint recvBits;
    uchar i;
	uchar buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
		status = MI_ERR;
	}

    if (status == MI_OK)
    {
        for (i=0; i<16; i++)		//Data to the FIFO write 16Byte
        {
        	buff[i] = *(writeData+i);
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {
			status = MI_ERR;
		}
    }

    return status;
}

/*
 * Function Name: MFRC522_Halt
 * Description: Command card into hibernation
 * Input: None
 * Return value: None
 */
void MFRC522_Halt(void)
{
	uint unLen;
	uchar buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	CalulateCRC(buff, 2, &buff[2]);

	MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}
