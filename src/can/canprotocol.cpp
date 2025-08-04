#include "canprotocol.h"

namespace edu
{

void makeCanStdID(int32_t sysID, int32_t nodeID, int32_t* inputAddress, int32_t* outputAddress, int32_t* broadcastAddress)
{
	int32_t sID  =    sysID << 8;
	int32_t iBit =    INPUTBIT  << 7;
	int32_t oBit =    OUTPUTBIT << 7;
	int32_t nID  =    nodeID;

	*inputAddress     = sID | iBit  | nID;
	*outputAddress    = sID | oBit  | nID;
	*broadcastAddress = sID | iBit  | 0b0000000;
}

} // namespace