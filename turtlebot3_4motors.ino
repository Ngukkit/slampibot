/*******************************************************************************
  Modified for assigning Dynamixel IDs 1/2/3/4 via OpenCR + Arduino IDE
  All baudrates unified to 9600 bps (USB Serial & Dynamixel bus)
*******************************************************************************/

#include <DynamixelSDK.h>
#include <stdarg.h>
#include <vector>

// ================= User Configuration =================
// Protocol 2.0 (XL430 / TurtleBot3)
static const uint32_t BAUD_TARGET       = 9600;   // Host <-> Dynamixel bus baud
static const uint8_t  BAUD_INDEX_TARGET = 0;      // DXL Baud Index: 0=9600, 1=57600, 2=115200, 3=1Mbps ...
// ======================================================

// Protocol version (for reference)
#define PROTOCOL_VERSION2               2.0

#if defined(__OPENCR__) 
#define DEVICENAME                      "/dev/OpenCR"
#elif defined(__OPENCM904__)
#define DEVICENAME                      "3"
#endif

#define CMD_SERIAL                      Serial   // USB Serial

typedef union
{
  uint8_t  u8Data[4];
  uint16_t u16Data[2];
  uint32_t u32Data;

  int8_t   s8Data[4];
  int16_t  s16Data[2];
  int32_t  s32Data;
} dxl_ret_t;

char *dev_name = (char*)DEVICENAME;

dynamixel::PacketHandler *packetHandler2;
dynamixel::PortHandler   *portHandler;

int tb3_id   = -1;
int tb3_baud = -1;

bool requestConfirm(void);
void flushCmd(void);
bool findOneMotorAnyBaud(uint8_t prefer_id_if_exists);
bool assignMotorToID(uint8_t target_id);
void testMotor(uint8_t id);

void      writeDXL(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                   uint8_t id, uint16_t addr, uint16_t length, uint32_t value);
dxl_ret_t readDXL (dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                   uint8_t id, uint16_t addr, uint16_t length);

void drawTitle(void)
{
  CMD_SERIAL.println();
  CMD_SERIAL.println("===== OpenCR Dynamixel Setup (9600bps) =====");
  CMD_SERIAL.println("1) Assign connected motor to ID = 1 (Left-Front)");
  CMD_SERIAL.println("2) Assign connected motor to ID = 2 (Right-Front)");
  CMD_SERIAL.println("3) Assign connected motor to ID = 3 (Left-Rear)");
  CMD_SERIAL.println("4) Assign connected motor to ID = 4 (Right-Rear)");
  CMD_SERIAL.println("5) Test motor ID=1");
  CMD_SERIAL.println("6) Test motor ID=2");
  CMD_SERIAL.println("7) Test motor ID=3");
  CMD_SERIAL.println("8) Test motor ID=4");
  CMD_SERIAL.println("---------------------------------------------");
  CMD_SERIAL.println("Tip: 반드시 모터 1개만 연결해서 ID 부여하세요!");
  CMD_SERIAL.print(">> ");
}

void setup()
{
  
  CMD_SERIAL.begin(15600);
  while (!CMD_SERIAL) {;}

  packetHandler2 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION2);
  portHandler    = dynamixel::PortHandler::getPortHandler(dev_name);

  if (portHandler->openPort())
  {
    CMD_SERIAL.println("Succeeded to open the port!");
    CMD_SERIAL.printf(" - Device Name : %s\r\n", dev_name);
  }
  else
  {
    CMD_SERIAL.printf("Failed to open the port! [%s]\n", dev_name);
    while (1);
  }

  // 호스트-버스 보레이트 9600으로 맞춤
  portHandler->setBaudRate(BAUD_TARGET);
  tb3_baud = BAUD_TARGET;

  CMD_SERIAL.println("\r\nStart Dynamixel ID/BAUD setup (9600bps)");
  drawTitle();
}

void loop()
{
  if (!CMD_SERIAL.available()) return;
  uint8_t ch = CMD_SERIAL.read();

  switch (ch)
  {
    case '1': flushCmd(); if (requestConfirm()) assignMotorToID(1); break;
    case '2': flushCmd(); if (requestConfirm()) assignMotorToID(2); break;
    case '3': flushCmd(); if (requestConfirm()) assignMotorToID(3); break;
    case '4': flushCmd(); if (requestConfirm()) assignMotorToID(4); break;

    case '5': flushCmd(); CMD_SERIAL.println("Test ID=1"); testMotor(1); break;
    case '6': flushCmd(); CMD_SERIAL.println("Test ID=2"); testMotor(2); break;
    case '7': flushCmd(); CMD_SERIAL.println("Test ID=3"); testMotor(3); break;
    case '8': flushCmd(); CMD_SERIAL.println("Test ID=4"); testMotor(4); break;

    default:  break;
  }

  drawTitle();
}

bool requestConfirm(void)
{
  CMD_SERIAL.print("Proceed? y/n : ");
  while (1)
  {
    if (CMD_SERIAL.available())
    {
      uint8_t c = CMD_SERIAL.read();
      if (c == 'y' || c == 'Y') { CMD_SERIAL.println("yes"); flushCmd(); return true; }
      break;
    }
  }
  CMD_SERIAL.println("no");
  return false;
}

void flushCmd(void)
{
  while (CMD_SERIAL.available()) (void)CMD_SERIAL.read();
}

// 단일 모터만 연결되어 있다는 전제 하에, 원하는 ID가 이미 그 모터에 잡혀있는지 먼저 체크.
// 없으면 baud 후보를 돌며 broadcastPing으로 "그 한 개"를 탐색한다.
bool findOneMotorAnyBaud(uint8_t prefer_id_if_exists)
{
  // 9600 포함하여 스캔
  uint32_t baud_tbl[] = {9600, 57600, 115200, 1000000, 2000000, 3000000};
  const size_t COUNT_BAUD = sizeof(baud_tbl)/sizeof(baud_tbl[0]);

  int baud_pre = portHandler->getBaudRate();
  tb3_id   = -1;
  tb3_baud = -1;

  CMD_SERIAL.println("Find Motor (single connected)...");

  // 1) 원하는 ID가 이미 있는지 먼저 확인
  for (size_t i=0; i<COUNT_BAUD; i++)
  {
    portHandler->setBaudRate(baud_tbl[i]);
    uint16_t model_number=0;
    int res = packetHandler2->ping(portHandler, prefer_id_if_exists, &model_number);
    if (res == COMM_SUCCESS)
    {
      tb3_id   = prefer_id_if_exists;
      tb3_baud = baud_tbl[i];
      CMD_SERIAL.printf("  Found target ID %d at baud %d (model %u)\n",
                        tb3_id, tb3_baud, model_number);
      portHandler->setBaudRate(tb3_baud);
      return true;
    }
  }

  // 2) 없으면 broadcastPing으로 "연결된 한 개" 찾기
  for (size_t i=0; i<COUNT_BAUD; i++)
  {
    portHandler->setBaudRate(baud_tbl[i]);
    std::vector<unsigned char> vec;
    int res = packetHandler2->broadcastPing(portHandler, vec);
    if (res != COMM_SUCCESS) continue;

    if (vec.size() == 1)
    {
      tb3_id   = vec[0];
      tb3_baud = baud_tbl[i];
      CMD_SERIAL.printf("  Found one motor: ID %d at baud %d\n", tb3_id, tb3_baud);
      portHandler->setBaudRate(tb3_baud);
      return true;
    }
    else if (vec.size() > 1)
    {
      CMD_SERIAL.println("  WARNING: multiple motors detected! 1개만 연결해서 다시 시도하세요.");
      for (auto id : vec) CMD_SERIAL.printf("    - ID %d\n", id);
      portHandler->setBaudRate(baud_pre);
      return false;
    }
  }

  portHandler->setBaudRate(baud_pre);
  CMD_SERIAL.println("  No motor found.");
  return false;
}

// 연결된 '그 한 개' 모터를 target_id로 부여하고, 보레이트도 9600으로 통일
bool assignMotorToID(uint8_t target_id)
{
  CMD_SERIAL.printf("Assign connected motor to ID=%d, Baud=%lu(idx=%u)\n",
                    target_id, (unsigned long)BAUD_TARGET, (unsigned)BAUD_INDEX_TARGET);

  if (!findOneMotorAnyBaud(target_id))
  {
    CMD_SERIAL.println("  -> Motor not found or multiple detected. Abort.");
    return false;
  }

  // 1) Torque OFF (Addr 64)
  writeDXL(portHandler, packetHandler2, tb3_id, 64, 1, 0);

  // 2) Baud 변경 (Addr 8) -> 9600(Idx 0)
  if (BAUD_INDEX_TARGET != 0xFF)
  {
    writeDXL(portHandler, packetHandler2, tb3_id, 8, 1, BAUD_INDEX_TARGET);
    // 호스트 포트도 동일 보레이트로 변경
    portHandler->setBaudRate(BAUD_TARGET);
  }

  // 3) ID 변경 (Addr 7)
  if (tb3_id != target_id)
  {
    writeDXL(portHandler, packetHandler2, tb3_id, 7, 1, target_id);
    tb3_id = target_id;
  }

  // 4) Verify
  {
    uint16_t model=0;
    int res = packetHandler2->ping(portHandler, tb3_id, &model);
    if (res == COMM_SUCCESS)
    {
      CMD_SERIAL.printf("  -> OK. Now ID=%d at %lu bps (model %u)\n",
                        tb3_id, (unsigned long)BAUD_TARGET, model);
      return true;
    }
    else
    {
      CMD_SERIAL.println("  -> Verify failed. Power-cycle and retry.");
      return false;
    }
  }
}

void testMotor(uint8_t id)
{
  CMD_SERIAL.printf("Test Motor (ID=%u)\n", id);

  // 테스트도 9600bps에서 수행
  portHandler->setBaudRate(BAUD_TARGET);

  uint16_t model_number=0;
  int dxl_comm_result = packetHandler2->ping(portHandler, id, &model_number);
  if (dxl_comm_result == COMM_SUCCESS)
  {
    CMD_SERIAL.printf("  Found Model: %u\n", model_number);
    writeDXL(portHandler, packetHandler2, id, 64, 1, 1); // Torque ON

    // 살짝 움직였다 멈추기 (Goal Velocity = 100 → 0)
    writeDXL(portHandler, packetHandler2, id, 104, 4, 100);
    delay(800);
    writeDXL(portHandler, packetHandler2, id, 104, 4, 0);
    CMD_SERIAL.println("  Move test complete.");
  }
  else
  {
    CMD_SERIAL.println("  Not found. Check ID/baud/power/wiring.");
  }
}

void writeDXL(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
              uint8_t id, uint16_t addr, uint16_t length, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, (uint8_t)value, &dxl_error);
  else if (length == 2)
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, (uint16_t)value, &dxl_error);
  else if (length == 4)
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, (uint32_t)value, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS)
  {
    CMD_SERIAL.println(packetHandler->getTxRxResult(dxl_comm_result));
    CMD_SERIAL.println("Fail to write!");
  }
  else if (dxl_error != 0)
  {
    CMD_SERIAL.println(packetHandler->getRxPacketError(dxl_error));
  }
}

dxl_ret_t readDXL(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler,
                  uint8_t id, uint16_t addr, uint16_t length)
{
  uint8_t dxl_error = 0;
  int     dxl_comm_result = COMM_TX_FAIL;
  dxl_ret_t ret; ret.u32Data = 0;

  int8_t  value8  = 0;
  int16_t value16 = 0;
  int32_t value32 = 0;

  if (length == 1)
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, (uint8_t*)&value8, &dxl_error);
  else if (length == 2)
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, (uint16_t*)&value16, &dxl_error);
  else if (length == 4)
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, addr, (uint32_t*)&value32, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0) CMD_SERIAL.println(packetHandler->getRxPacketError(dxl_error));
    if (length == 1)      ret.u32Data = (uint8_t)value8;
    else if (length == 2) ret.u32Data = (uint16_t)value16;
    else if (length == 4) ret.u32Data = (uint32_t)value32;
  }
  else
  {
    CMD_SERIAL.println(packetHandler->getTxRxResult(dxl_comm_result));
    CMD_SERIAL.println("Fail to read!");
  }
  return ret;
}
