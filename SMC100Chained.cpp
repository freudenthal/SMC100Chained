#include "SMC100Chained.h"

const char SMC100Chained::CarriageReturnCharacter = '\r';
const char SMC100Chained::NewLineCharacter = '\n';
const char SMC100Chained::GetCharacter = '?';
const char SMC100Chained::NoErrorCharacter = '@';
const uint32_t SMC100Chained::WipeInputEvery = 100000;
const uint32_t SMC100Chained::CommandReplyTimeMax = 500000;
const uint32_t SMC100Chained::WaitAfterSendingTimeMax = 20000;
const uint32_t SMC100Chained::PollStatusTimeInterval = 100000;
const uint32_t SMC100Chained::PollPositionTimeInterval = 100000;

const SMC100Chained::CommandStruct SMC100Chained::CommandLibrary[] =
{
	{CommandType::None,"  ",CommandParameterType::None,CommandGetSetType::None},
	{CommandType::Enable,"MM",CommandParameterType::Int,CommandGetSetType::GetSet},
	{CommandType::Home,"OR",CommandParameterType::None,CommandGetSetType::None},
	{CommandType::MoveAbs,"PA",CommandParameterType::Float,CommandGetSetType::GetSet},
	{CommandType::MoveRel,"PR",CommandParameterType::Float,CommandGetSetType::GetSet},
	{CommandType::MoveEstimate,"PT",CommandParameterType::Float,CommandGetSetType::GetAlways},
	{CommandType::Configure,"PW",CommandParameterType::Int,CommandGetSetType::GetSet},
	{CommandType::Analogue,"RA",CommandParameterType::None,CommandGetSetType::GetAlways},
	{CommandType::GPIOInput,"RB",CommandParameterType::None,CommandGetSetType::GetAlways},
	{CommandType::Reset,"RS",CommandParameterType::None,CommandGetSetType::None},
	{CommandType::GPIOOutput,"SB",CommandParameterType::Int,CommandGetSetType::GetSet},
	{CommandType::LimitPositive,"SR",CommandParameterType::Float,CommandGetSetType::GetSet},
	{CommandType::LimitNegative,"SL",CommandParameterType::Float,CommandGetSetType::GetSet},
	{CommandType::PositionAsSet,"TH",CommandParameterType::None,CommandGetSetType::GetAlways},
	{CommandType::PositionReal,"TP",CommandParameterType::None,CommandGetSetType::GetAlways},
	{CommandType::KeypadEnable,"JM",CommandParameterType::Int,CommandGetSetType::GetSet},
	{CommandType::ErrorCommands,"TE",CommandParameterType::None,CommandGetSetType::GetAlways},
	{CommandType::ErrorStatus,"TS",CommandParameterType::None,CommandGetSetType::GetAlways}
};

const SMC100Chained::StatusCharSet SMC100Chained::StatusLibrary[] =
{
	{"0A",StatusType::NoReference},
	{"0B",StatusType::NoReference},
	{"0C",StatusType::NoReference},
	{"0D",StatusType::NoReference},
	{"0E",StatusType::NoReference},
	{"0F",StatusType::NoReference},
	{"10",StatusType::NoReference},
	{"11",StatusType::NoReference},
	{"14",StatusType::NoReference},
	{"1E",StatusType::Homing},
	{"1F",StatusType::Homing},
	{"28",StatusType::Moving},
	{"32",StatusType::Ready},
	{"33",StatusType::Ready},
	{"34",StatusType::Ready},
	{"35",StatusType::Ready},
	{"3C",StatusType::Disabled},
	{"3D",StatusType::Disabled},
	{"3E",StatusType::Disabled},
	{"46",StatusType::Jogging},
	{"47",StatusType::Jogging},
};

SMC100Chained::SMC100Chained(HardwareSerial *serial, const uint8_t* addresses, const uint8_t addresscount)
{
	SerialPort = serial;
	MotorCount = addresscount;
	CurrentCommand = NULL;
	CurrentCommandParameter = 0.0;
	ReplyBufferIndex = 0;
	for (uint8_t Index = 0; Index < SMC100ChainedReplyBufferSize; ++Index)
	{
		ReplyBuffer[Index] = 0;
	}
	for (uint8_t Index = 0; Index < SMC100ChainedMaxMotors; ++Index)
	{
		MotorState[Index].Address = 0;
		MotorState[Index].Status = StatusType::Unknown;
		MotorState[Index].HasBeenHomed = false;
		MotorState[Index].Position = 0.0;
		MotorState[Index].GPIOInput = 0;
		MotorState[Index].GPIOOutput = 0;
		MotorState[Index].AnalogueReading = 0.0;
		MotorState[Index].PositionLimitNegative = 0.0;
		MotorState[Index].PositionLimitPositive = 0.0;
		MotorState[Index].PollStatus = false;
		MotorState[Index].PollPosition = false;
		MotorState[Index].NeedToPollPosition = false;
		MotorState[Index].FinishedCallback = NULL;
	}
	for (uint8_t Index = 0; Index < MotorCount; ++Index)
	{
		MotorState[Index].Address = addresses[Index];
	}
	ClearCommandQueue();
	AllCompleteCallback = NULL;
	MoveCompleteCallback = NULL;
	HomeCompleteCallback = NULL;
	GPIOReturnCallback = NULL;
	NeedToFireMoveComplete = false;
	NeedToFireHomeComplete = false;
	CurrentCommand = NULL;
	LastWipeTime = 0;
	TransmitTime = 0;
	Verbose = false;
	PollStatus = false;
	PollPosition = false;
	PollPositionTimeLast = 0;
	PollStatusTimeLast = 0;
	Mode = ModeType::Inactive;
}

void SMC100Chained::Begin()
{
	for (uint8_t Index = 0; Index < MotorCount; ++Index)
	{
		CommandEnqueue(Index, CommandType::ErrorStatus, 0.0, CommandGetSetType::None);
		CommandEnqueue(Index, CommandType::LimitPositive, 0.0, CommandGetSetType::Get);
		CommandEnqueue(Index, CommandType::LimitNegative, 0.0, CommandGetSetType::Get);
		CommandEnqueue(Index, CommandType::GPIOInput, 0.0, CommandGetSetType::None);
	}
	Mode = ModeType::Idle;
}

bool SMC100Chained::IsHomed(uint8_t MotorIndex)
{
	if (MotorIndex < MotorCount)
	{
		return MotorState[MotorIndex].HasBeenHomed;
	}
	return false;
}

bool SMC100Chained::IsReady(uint8_t MotorIndex)
{
	if (MotorIndex < MotorCount)
	{
		if (MotorState[MotorIndex].Status == StatusType::Ready)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	return false;
}

bool SMC100Chained::IsMoving(uint8_t MotorIndex)
{
	if (MotorIndex < MotorCount)
	{
		if (MotorState[MotorIndex].Status == StatusType::Moving)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	return false;
}

bool SMC100Chained::IsEnabled(uint8_t MotorIndex)
{
	if (MotorState[MotorIndex].Status != StatusType::Disabled)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void SMC100Chained::Enable(uint8_t MotorIndex,bool Setting)
{
	if (MotorIndex >= MotorCount)
	{
		Serial.print("<SMCERROR>(Motor index too large)");
		return;
	}
	float ParamterValue = 0.0;
	if (Setting)
	{
		ParamterValue = 1.0;
	}
	CommandEnqueue(MotorIndex, CommandType::Enable, ParamterValue, CommandGetSetType::Set);
}

bool SMC100Chained::IsBusy()
{
	return Busy;
}

void SMC100Chained::Home(uint8_t MotorIndex)
{
	CommandEnqueue(MotorIndex, CommandType::Home, 0.0, CommandGetSetType::None);
}

void SMC100Chained::MoveAbsolute(uint8_t MotorIndex, float Target)
{
	if (MotorIndex >= MotorCount)
	{
		Serial.print("<SMCERROR>(Motor index too large)");
		return;
	}
	if (Target < MotorState[MotorIndex].PositionLimitNegative)
	{
		Target = MotorState[MotorIndex].PositionLimitNegative;
	}
	if (Target > MotorState[MotorIndex].PositionLimitPositive)
	{
		Target = MotorState[MotorIndex].PositionLimitPositive;
	}
	CommandEnqueue(MotorIndex, CommandType::MoveAbs, Target, CommandGetSetType::Set);
}

float SMC100Chained::GetPosition(uint8_t MotorIndex)
{
	if (MotorIndex >= MotorCount)
	{
		Serial.print("<SMCERROR>(Motor index too large)");
		return NAN;
	}
	return MotorState[MotorIndex].Position;
}

void SMC100Chained::SendGetGPIOInput(uint8_t MotorIndex)
{
	if (MotorIndex >= MotorCount)
	{
		Serial.print("<SMCERROR>(Motor index too large)");
		return;
	}
	CommandEnqueue(MotorIndex, CommandType::GPIOInput, 0.0, CommandGetSetType::None);
}

bool SMC100Chained::GetGPIOInput(uint8_t MotorIndex, uint8_t Pin)
{
	if (MotorIndex >= MotorCount)
	{
		Serial.print("<SMCERROR>(Motor index too large)");
		return false;
	}
	if (Pin > 3)
	{
		Serial.print("<SMCERROR>(Get GPIO input index too large.)");
		return false;
	}
	return bitRead(MotorState[MotorIndex].GPIOInput, Pin);
}

void SMC100Chained::SetGPIOOutput(uint8_t MotorIndex, uint8_t Pin, bool Output)
{
	if (MotorIndex >= MotorCount)
	{
		Serial.print("<SMCERROR>(Motor index too large)");
		return;
	}
	if (Pin > 3)
	{
		Serial.print("<SMCERROR>(Get GPIO input index too large.)");
		return;
	}
	bitWrite(MotorState[MotorIndex].GPIOOutput, Pin, Output);
	CommandEnqueue(MotorIndex, CommandType::GPIOOutput, (float)(MotorState[MotorIndex].GPIOOutput), CommandGetSetType::Set);
}

void SMC100Chained::SetGPIOOutputAll(uint8_t MotorIndex, uint8_t Code)
{
	if (MotorIndex >= MotorCount)
	{
		Serial.print("<SMCERROR>(Motor index too large)");
		return;
	}
	MotorState[MotorIndex].GPIOOutput = Code;
	CommandEnqueue(MotorIndex, CommandType::GPIOOutput, (float)(MotorState[MotorIndex].GPIOOutput), CommandGetSetType::Set);
}

void SMC100Chained::SetAxesCompleteCallback(uint8_t MotorIndex, FinishedListener Callback)
{
	if (MotorIndex < MotorCount)
	{
		MotorState[MotorIndex].FinishedCallback = Callback;
	}
	else
	{
		Serial.print("<SMCERROR>(Can not assign callback. Motor index too high.)");
	}
}

void SMC100Chained::SetAllCompleteCallback(FinishedListener Callback)
{
	AllCompleteCallback = Callback;
}

void SMC100Chained::SetHomeCompleteCallback(FinishedListener Callback)
{
	HomeCompleteCallback = Callback;
}

void SMC100Chained::SetMoveCompleteCallback(FinishedListener Callback)
{
	MoveCompleteCallback = Callback;
}

void SMC100Chained::SetGPIOReturnCallback(FinishedListener Callback)
{
	GPIOReturnCallback = Callback;
}

void SMC100Chained::SetVerbose(bool VerboseToSet)
{
	Verbose = VerboseToSet;
}

void SMC100Chained::Check()
{
	if ( (Mode == ModeType::Idle) && (CommandQueueEmpty()) )
	{
		if (PollStatus)
		{
			CheckErrorStatusPoll();
		}
		if (PollPosition)
		{
			CheckPositionPoll();
		}
	}
	switch (Mode)
	{
		case ModeType::Idle:
			CheckCommandQueue();
			break;
		case ModeType::WaitForCommandReply:
			CheckForCommandReply();
			break;
		default:
			break;
	}
}

void SMC100Chained::PrepareErrorStatusPolling(uint8_t MotorIndex)
{
	PrepareErrorStatusPolling(MotorIndex, true);
}

void SMC100Chained::PrepareErrorStatusPolling(uint8_t MotorIndex, bool Enable)
{
	if (MotorIndex < MotorCount)
	{
		if (MotorState[MotorIndex].PollStatus != Enable)
		{
			EnqueueErrorStatusRequest(MotorIndex);
			PollStatus = Enable;
			MotorState[MotorIndex].PollStatus = Enable;
			if (Enable)
			{
				MotorState[MotorIndex].NeedToPollPosition = true;
			}
			PollStatus = Enable;
			PollStatusTimeLast = micros();
			if (Verbose)
			{
				Serial.print("<SMCV>(EPoll ");
				Serial.print(MotorIndex);
				Serial.print(")\n");
			}
		}
	}
	else
	{
		Serial.print("<SMCERROR>(Motor index too large for error polling)");
	}
}

void SMC100Chained::CheckErrorStatusPoll()
{
	if ( (micros() - PollStatusTimeLast) > PollStatusTimeInterval)
	{
		for (int Index = 0; Index < MotorCount; ++Index)
		{
			EnqueueErrorStatusRequest(Index);
			PollStatusTimeLast = micros();
		}
	}
}


void SMC100Chained::PreparePositionPolling(uint8_t MotorIndex)
{
	PreparePositionPolling(MotorIndex, true);
}

void SMC100Chained::PreparePositionPolling(uint8_t MotorIndex, bool Enable)
{
	if (MotorIndex < MotorCount)
	{
		if (MotorState[MotorIndex].PollStatus != Enable)
		{
			EnqueuePositionRequest(MotorIndex);
			MotorState[MotorIndex].PollPosition = Enable;
			PollPosition = Enable;
			PollPositionTimeLast = micros();
			if (Verbose)
			{
				Serial.print("<SMCV>(PPoll ");
				Serial.print(MotorIndex);
				Serial.print(")\n");
			}
		}
	}
	else
	{
		Serial.print("<SMCERROR>(Motor index too large for positon polling)");
	}
}

void SMC100Chained::PollPositionRealNeededMotors()
{
	for (uint8_t Index = 0; Index < MotorCount; ++Index)
	{
		if (MotorState[Index].NeedToPollPosition)
		{
			MotorState[Index].NeedToPollPosition = false;
			PreparePositionPolling(Index);
		}
	}
}

void SMC100Chained::CheckPositionPoll()
{
	if ( (micros() - PollPositionTimeLast) > PollPositionTimeInterval)
	{
		PollPositionRealNeededMotors();
		PollStatusTimeLast = micros();
	}
}

void SMC100Chained::ModeTransitionToIdle()
{
	if (CommandQueueEmpty())
	{
		Busy = false;
		if (AllCompleteCallback != NULL)
		{
			AllCompleteCallback();
		}
	}
	Mode = ModeType::Idle;
}

void SMC100Chained::ModeTransitionToWaitForReply()
{
	ReplyBufferIndex = 0;
	ReplyBuffer[ReplyBufferIndex] = '\0';
	Mode = ModeType::WaitForCommandReply;
}

void SMC100Chained::CheckCommandQueue()
{
	bool NewCommandPulled = CommandQueuePullToCurrentCommand();
	if (NewCommandPulled)
	{
		if (CurrentCommand != NULL)
		{
			Busy = true;
			SendCurrentCommand();
		}
		else
		{
			Serial.print("<SMC100Chained>(Command in queue is null.)\n");
		}
	}
	else
	{
		if (Busy)
		{
			Busy = false;
			if (AllCompleteCallback != NULL)
			{
				AllCompleteCallback();
			}
		}
		if ( (micros() - LastWipeTime) > WipeInputEvery )
		{
			LastWipeTime = micros();
			if (SerialPort->available())
			{
				SerialPort->read();
			}
		}
	}
}

void SMC100Chained::CheckForCommandReply()
{
	if (SerialPort->available())
	{
		char NewChar = SerialPort->read();
		if (NewChar == CarriageReturnCharacter)
		{

		}
		else if (NewChar == NewLineCharacter)
		{
			ReplyBuffer[ReplyBufferIndex] = '\0';
			if (Verbose)
			{
				Serial.print("<SMCV>(Reply: ");
				Serial.print(ReplyBuffer);
				Serial.print(" )\n");
			}
			ParseReply();
		}
		else
		{
			ReplyBuffer[ReplyBufferIndex] = NewChar;
			ReplyBufferIndex++;
			if (ReplyBufferIndex > SMC100ChainedReplyBufferSize)
			{
				ReplyBuffer[SMC100ChainedReplyBufferSize-1] = '\0';
				Serial.print("<SMC100Chained>(Error: Buffer overflow with ");
				Serial.print(ReplyBuffer);
				Serial.print(")\n");
				Mode = ModeType::Idle;
			}
		}
	}
	if ( (micros() - TransmitTime) > CommandReplyTimeMax )
	{
		Mode = ModeType::Idle;
		Serial.print("<SMC200>(Time out detected.)\n");
	}
}

void SMC100Chained::ParseReply()
{
	char* EndOfAddress;
	char* ParameterAddress;
	uint8_t AddressOfReply = strtol(ReplyBuffer, &EndOfAddress, 10);
	if (AddressOfReply != CurrentCommandAddress)
	{
		Serial.print("<SMC100Chained>(Address does not match return for ");
		Serial.print(ReplyBuffer);
		Serial.print(")\n");
	}
	else if ( (CurrentCommand->CommandChar[0] != *EndOfAddress) || (CurrentCommand->CommandChar[1] != *(EndOfAddress + 1)) )
	{
		Serial.print("<SMC100Chained>(Return string expected ");
		Serial.print(CurrentCommand->CommandChar[0]);
		Serial.print(CurrentCommand->CommandChar[1]);
		Serial.print(" but received ");
		Serial.print(*EndOfAddress);
		Serial.print(*(EndOfAddress + 1));
		Serial.print(")\n");
	}
	else
	{
		ParameterAddress = EndOfAddress + 2;
		//char* EndOfReplyData = ReplyData + ReplyBufferIndex;
		if (CurrentCommand->Command == CommandType::PositionReal)
		{
			float Position = atof(ParameterAddress);
			UpdatePosition(AddressOfReply, Position);
		}
		else if (CurrentCommand->Command == CommandType::ErrorCommands)
		{
			if (*ParameterAddress != NoErrorCharacter)
			{
				Serial.print("<SMC100Chained>(Error code: ");
				Serial.print(*ParameterAddress);
				Serial.print(" motor: ");
				Serial.print(AddressOfReply);
				Serial.print(")\n");
			}
		}
		else if (CurrentCommand->Command == CommandType::ErrorStatus)
		{
			bool ErrorStatus = false;
			char ErrorCode[4];
			for (uint8_t Index = 0; Index < 4; Index++)
			{
				ErrorCode[Index] = *(ParameterAddress + Index);
				if (ErrorCode[Index] != '0')
				{
					ErrorStatus = true;
				}
			}
			if (ErrorStatus)
			{
				Serial.print("<SMC100Chained>(Error hardware code: ");
				Serial.print(ErrorCode);
				Serial.print(" motor: ");
				Serial.print(AddressOfReply);
				Serial.print(")\n");
				Mode = ModeType::Idle;
			}
			char StatusChar[3];
			StatusChar[0] = *(ParameterAddress + 4);
			StatusChar[1] = *(ParameterAddress + 5);
			StatusChar[2] = '\0';
			StatusType Status = ConvertStatus(StatusChar);
			UpdateStatus(AddressOfReply, Status);
		}
		else if (CurrentCommand->Command == CommandType::GPIOInput)
		{
			uint8_t GPIOInput = (uint8_t)atoi(ParameterAddress);
			UpdateGPIOInput(AddressOfReply, GPIOInput);
			if (GPIOReturnCallback != NULL)
			{
				GPIOReturnCallback();
			}
		}
		else if (CurrentCommand->Command == CommandType::Analogue)
		{
			float AnalogueReading = atof(ParameterAddress);
			UpdateAnalogue(AddressOfReply, AnalogueReading);
		}
		else if ( (CurrentCommand->Command == CommandType::LimitNegative) )
		{
			if (CurrentCommandGetOrSet == CommandGetSetType::Get)
			{
				float PositionLimitNegative = atof(ParameterAddress);
				UpdatePositionLimitNegative(AddressOfReply, PositionLimitNegative);
			}
		}
		else if (CurrentCommand->Command == CommandType::LimitPositive)
		{
			if (CurrentCommandGetOrSet == CommandGetSetType::Get)
			{
				float PositionLimitPositive = atof(ParameterAddress);
				UpdatePositionLimitNegative(AddressOfReply, PositionLimitPositive);
			}
		}
	}
}

bool SMC100Chained::ConvertMotorAddressToIndex(uint8_t Address, uint8_t* MotorIndexReturn)
{
	for (uint8_t Index = 0; Index < MotorCount; ++Index)
	{
		if (MotorState[Index].Address == Address)
		{
			*MotorIndexReturn = Index;
			return true;
		}
	}
	return false;
}

void SMC100Chained::UpdateGPIOInput(uint8_t MotorAddress, uint8_t GPIOInputToSet)
{
	uint8_t MotorIndex = 0;
	bool FoundMotor = ConvertMotorAddressToIndex(MotorAddress, &MotorIndex);
	if (FoundMotor)
	{
		MotorState[MotorIndex].GPIOInput = GPIOInputToSet;
	}
}

void SMC100Chained::UpdatePositionLimitPositive(uint8_t MotorAddress, float PositionLimitPositiveToSet)
{
	uint8_t MotorIndex = 0;
	bool FoundMotor = ConvertMotorAddressToIndex(MotorAddress, &MotorIndex);
	if (FoundMotor)
	{
		MotorState[MotorIndex].PositionLimitPositive = PositionLimitPositiveToSet;
	}
}

void SMC100Chained::UpdatePositionLimitNegative(uint8_t MotorAddress, float PositionLimitNegativeToSet)
{
	uint8_t MotorIndex = 0;
	bool FoundMotor = ConvertMotorAddressToIndex(MotorAddress, &MotorIndex);
	if (FoundMotor)
	{
		MotorState[MotorIndex].PositionLimitNegative = PositionLimitNegativeToSet;
	}
}

void SMC100Chained::UpdateAnalogue(uint8_t MotorAddress, float AnalogueToSet)
{
	uint8_t MotorIndex = 0;
	bool FoundMotor = ConvertMotorAddressToIndex(MotorAddress, &MotorIndex);
	if (FoundMotor)
	{
		MotorState[MotorIndex].AnalogueReading = AnalogueToSet;
	}
}

void SMC100Chained::UpdatePosition(uint8_t MotorAddress, float PositionToSet)
{
	uint8_t MotorIndex = 0;
	bool FoundMotor = ConvertMotorAddressToIndex(MotorAddress, &MotorIndex);
	if (FoundMotor)
	{
		MotorState[MotorIndex].Position = PositionToSet;
		MotorState[MotorIndex].NeedToPollPosition = false;
		MotorState[MotorIndex].PollPosition = false;
		if (MotorState[MotorIndex].FinishedCallback != NULL)
		{
			MotorState[MotorIndex].FinishedCallback();
		}
		CheckAllPollPosition();
	}
}

void SMC100Chained::CheckAllPollPosition()
{
	bool AllMotorsPolled = true;
	for (uint8_t Index = 0; Index < MotorCount; ++Index)
	{
		if (MotorState[Index].PollPosition)
		{
			AllMotorsPolled = false;
			break;
		}
	}
	if (AllMotorsPolled)
	{
		PollPosition = false;
		if ( (NeedToFireHomeComplete) && (HomeCompleteCallback != NULL) )
		{
			NeedToFireHomeComplete = false;
			HomeCompleteCallback();
		}
		if ( (NeedToFireMoveComplete) && (MoveCompleteCallback != NULL) )
		{
			NeedToFireMoveComplete = false;
			MoveCompleteCallback();
		}
	}
}

void SMC100Chained::UpdateStatus(uint8_t MotorAddress, StatusType Status)
{
	uint8_t MotorIndex = 0;
	bool FoundMotor = ConvertMotorAddressToIndex(MotorAddress, &MotorIndex);
	if (FoundMotor)
	{
		MotorState[MotorIndex].Status = Status;
		if (Status == StatusType::Unknown)
		{
			Serial.print("<SMC100Chained>(Error status code not recognized)\n");
		}
		else if ( Status == StatusType::NoReference )
		{
			MotorState[MotorIndex].HasBeenHomed = false;
			MotorState[MotorIndex].PollStatus = false;
		}
		else if ( Status == StatusType::Homing )
		{
			MotorState[MotorIndex].HasBeenHomed = false;
		}
		else if ( Status == StatusType::Moving )
		{
			MotorState[MotorIndex].HasBeenHomed = true;
		}
		else if ( Status == StatusType::Ready )
		{
			if (MotorState[MotorIndex].PollStatus)
			{
				MotorState[MotorIndex].PollStatus = false;
				CheckAllPollStatus();
			}
			MotorState[MotorIndex].HasBeenHomed = true;
		}
	}
}

void SMC100Chained::CheckAllPollStatus()
{
	bool AllMotorsPolled = true;
	for (uint8_t Index = 0; Index < MotorCount; ++Index)
	{
		if (MotorState[Index].PollStatus)
		{
			AllMotorsPolled = false;
			break;
		}
	}
	if (AllMotorsPolled)
	{
		PollStatus = false;
		for (uint8_t Index = 0; Index < MotorCount; ++Index)
		{
			if (MotorState[Index].NeedToPollPosition)
			{
				MotorState[Index].NeedToPollPosition = false;
				PreparePositionPolling(Index);
			}
		}
	}
}

SMC100Chained::StatusType SMC100Chained::ConvertStatus(char* StatusChar)
{
	for (uint8_t Index = 0; Index < 21; ++Index)
	{
		if ( strcmp( StatusChar, StatusLibrary[Index].Code ) == 0)
		{
			return StatusLibrary[Index].Type;
		}
	}
	Serial.print("<SMCError>(Unknown status code : ");
	Serial.print(StatusChar);
	Serial.print(")\n");
	return StatusType::Unknown;
}

bool SMC100Chained::SendCurrentCommand()
{
	bool Status = true;
	if (CurrentCommand->Command == CommandType::None)
	{
		Mode = ModeType::Idle;
		Serial.print("<SMC100Chained>(Empty command requested.)");
		return false;
	}
	SerialPort->print(CurrentCommandAddress);
	SerialPort->write(CurrentCommand->CommandChar[0]);
	SerialPort->write(CurrentCommand->CommandChar[1]);
	if (Verbose)
	{
		Serial.print("<SMC100>(Send: ");
		Serial.print(CurrentCommandAddress);
		Serial.write(CurrentCommand->CommandChar[0]);
		Serial.write(CurrentCommand->CommandChar[1]);
	}
	if (CurrentCommandGetOrSet == CommandGetSetType::Get)
	{
		//Serial.write(GetCharacter);
		SerialPort->write(GetCharacter);
		if (Verbose)
		{
			Serial.write(GetCharacter);
		}
	}
	else if (CurrentCommandGetOrSet == CommandGetSetType::Set)
	{
		if (CurrentCommand->SendType == CommandParameterType::Int)
		{
			//Serial.print((int)(CurrentCommandParameter));
			SerialPort->print((int)(CurrentCommandParameter));
			if (Verbose)
			{
				Serial.print((int)(CurrentCommandParameter));
			}
		}
		else if (CurrentCommand->SendType == CommandParameterType::Float)
		{
			//Serial.print(CurrentCommandParameter,6);
			SerialPort->print(CurrentCommandParameter,6);
			if (Verbose)
			{
				Serial.print(CurrentCommandParameter,6);
			}
		}
		else
		{
			Status = false;
		}
	}
	else if ( (CurrentCommand->GetSetType == CommandGetSetType::None) || (CurrentCommand->GetSetType == CommandGetSetType::GetAlways) )
	{

	}
	else
	{
		Status = false;
		Serial.print("<SMC11Error>(Command type not recognized.)");
	}
	//Serial.print(NewLineCharacter);
	SerialPort->write(CarriageReturnCharacter);
	SerialPort->write(NewLineCharacter);
	if (Verbose)
	{
		Serial.print(" )\n");
	}
	ReplyBufferIndex = 0;
	TransmitTime = micros();
	UpdateStateOnSending();
	return Status;
}

void SMC100Chained::UpdateStateOnSending()
{
	if ( (CurrentCommand->Command == CommandType::MoveAbs) || (CurrentCommand->Command == CommandType::MoveRel) )
	{
		if (CurrentCommandGetOrSet == CommandGetSetType::Set)
		{
			NeedToFireMoveComplete = true;
			MotorState[CurrentCommandMotorIndex].NeedToPollPosition = true;
			PrepareErrorStatusPolling(CurrentCommandMotorIndex);
		}
	}
	if ( (CurrentCommand->Command == CommandType::Home) )
	{
		if (CurrentCommandGetOrSet == CommandGetSetType::Set)
		{
			NeedToFireHomeComplete = true;
			MotorState[CurrentCommandMotorIndex].NeedToPollPosition = true;
			PrepareErrorStatusPolling(CurrentCommandMotorIndex);
		}
	}
	if ( (CurrentCommandGetOrSet == CommandGetSetType::Get) || (CurrentCommand->GetSetType == CommandGetSetType::GetAlways) )
	{
		ModeTransitionToWaitForReply();
		Mode = ModeType::WaitForCommandReply;
	}
	else
	{
		ModeTransitionToIdle();
	}
}

void SMC100Chained::ClearCommandQueue()
{
	for (uint8_t Index = 0; Index < SMC100ChainedQueueCount; ++Index)
	{
		CommandQueue[Index].Command = NULL;
		CommandQueue[Index].Parameter = 0.0;
		CommandQueue[Index].MotorIndex = 0;
		CommandQueue[Index].GetOrSet = CommandGetSetType::None;
	}
	CommandQueueHead = 0;
	CommandQueueTail = 0;
	CommandQueueFullFlag = false;
}
bool SMC100Chained::CommandQueueFull()
{
	return CommandQueueFullFlag;
}
bool SMC100Chained::CommandQueueEmpty()
{
	return ( !CommandQueueFullFlag && (CommandQueueHead == CommandQueueTail) );
}
uint8_t SMC100Chained::CommandQueueCount()
{
	uint8_t Count = SMC100ChainedQueueCount;
	if(!CommandQueueFullFlag)
	{
		if(CommandQueueHead >= CommandQueueTail)
		{
			Count = (CommandQueueHead - CommandQueueTail);
		}
		else
		{
			Count = (SMC100ChainedQueueCount + CommandQueueHead - CommandQueueTail);
		}
	}
	return Count;
}
void SMC100Chained::CommandQueueAdvance()
{
	if(CommandQueueFullFlag)
	{
		CommandQueueTail = (CommandQueueTail + 1) % SMC100ChainedQueueCount;
	}
	CommandQueueHead = (CommandQueueHead + 1) % SMC100ChainedQueueCount;
	CommandQueueFullFlag = (CommandQueueHead == CommandQueueTail);
}
void SMC100Chained::CommandQueueRetreat()
{
	CommandQueueFullFlag = false;
	CommandQueueTail = (CommandQueueTail + 1) % SMC100ChainedQueueCount;
}
void SMC100Chained::EnqueueGetLimitNegative(uint8_t MotorIndex)
{
	CommandCurrentPut(MotorIndex, CommandType::LimitNegative, 0.0, CommandGetSetType::Get);
}
void SMC100Chained::EnqueueGetLimitPositive(uint8_t MotorIndex)
{
	CommandCurrentPut(MotorIndex, CommandType::LimitPositive, 0.0, CommandGetSetType::Get);
}
void SMC100Chained::EnqueueErrorCommandRequest(uint8_t MotorIndex)
{
	CommandCurrentPut(MotorIndex, CommandType::ErrorCommands, 0.0, CommandGetSetType::None);
}
void SMC100Chained::EnqueueErrorStatusRequest(uint8_t MotorIndex)
{
	CommandCurrentPut(MotorIndex, CommandType::ErrorStatus, 0.0, CommandGetSetType::None);
}
void SMC100Chained::EnqueuePositionRequest(uint8_t MotorIndex)
{
	CommandCurrentPut(MotorIndex, CommandType::PositionReal, 0.0, CommandGetSetType::None);
}
void SMC100Chained::CommandCurrentPut(uint8_t MotorIndex, CommandType Type, float Parameter, CommandGetSetType GetOrSet)
{
	//CurrentCommand = const_cast<CommandStruct*>(&CommandLibrary[static_cast<uint8_t>(Type)]);
	CurrentCommand = &CommandLibrary[static_cast<uint8_t>(Type)];
	CurrentCommandParameter = Parameter;
	CurrentCommandAddress = MotorState[MotorIndex].Address;
	CurrentCommandMotorIndex = MotorIndex;
	CurrentCommandGetOrSet = GetOrSet;
}
void SMC100Chained::CommandEnqueue(uint8_t MotorIndex, CommandType Type, float Parameter, CommandGetSetType GetOrSet)
{
	//CommandStruct* CommandPointer = const_cast<CommandStruct*>(&CommandLibrary[static_cast<uint8_t>(Type)]);
	const CommandStruct* CommandPointer = &CommandLibrary[static_cast<uint8_t>(Type)];
	CommandEnqueue(MotorIndex, CommandPointer, Parameter, GetOrSet);
}
void SMC100Chained::CommandEnqueue(uint8_t MotorIndex, const CommandStruct* CommandPointer, float Parameter, CommandGetSetType GetOrSet)
{
	CommandQueue[CommandQueueHead].Command = CommandPointer;
	CommandQueue[CommandQueueHead].Parameter = Parameter;
	CommandQueue[CommandQueueHead].MotorIndex = MotorIndex;
	CommandQueue[CommandQueueHead].GetOrSet = GetOrSet;
	CommandQueueAdvance();
}
bool SMC100Chained::CommandQueuePullToCurrentCommand()
{
	bool Status = false;
	if (!CommandQueueEmpty())
	{
		CurrentCommand = CommandQueue[CommandQueueTail].Command;
		CurrentCommandParameter = CommandQueue[CommandQueueTail].Parameter;
		CurrentCommandGetOrSet = CommandQueue[CommandQueueTail].GetOrSet;
		CurrentCommandMotorIndex = CommandQueue[CommandQueueTail].MotorIndex;
		CurrentCommandAddress = MotorState[CurrentCommandMotorIndex].Address;
		CommandQueueRetreat();
		Status = true;
		//Serial.print("NP");
		//Serial.print(CurrentCommandParameter);
		//Serial.print("\n");
	}
	return Status;
}
