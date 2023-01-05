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
const uint8_t SMC100Chained::ErrorCountMaxLimit = 32;

const SMC100Chained::CommandStruct SMC100Chained::CommandLibrary[] =
{
	{CommandType::None,"  ",CommandParameterType::None,CommandGetSetType::None,0},
	{CommandType::Enable,"MM",CommandParameterType::Int,CommandGetSetType::GetSet,0},
	{CommandType::Home,"OR",CommandParameterType::None,CommandGetSetType::None,0},
	{CommandType::MoveAbs,"PA",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::MoveRel,"PR",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::MoveEstimate,"PT",CommandParameterType::Float,CommandGetSetType::GetAlways,0},
	{CommandType::Configure,"PW",CommandParameterType::Int,CommandGetSetType::GetSet,10000},
	{CommandType::Analogue,"RA",CommandParameterType::None,CommandGetSetType::GetAlways,0},
	{CommandType::GPIOInput,"RB",CommandParameterType::None,CommandGetSetType::GetAlways,0},
	{CommandType::Reset,"RS",CommandParameterType::None,CommandGetSetType::None,5000},
	{CommandType::GPIOOutput,"SB",CommandParameterType::Int,CommandGetSetType::GetSet,0},
	{CommandType::LimitPositive,"SR",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::LimitNegative,"SL",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::PositionAsSet,"TH",CommandParameterType::None,CommandGetSetType::GetAlways,0},
	{CommandType::PositionReal,"TP",CommandParameterType::None,CommandGetSetType::GetAlways,0},
	{CommandType::Velocity,"VA",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::Acceleration,"AC",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::Backlash,"BA",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::Hysteresis,"BH",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::FilterKd,"FD",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::ErrorLimit,"FE",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::FrictionCompensation,"FF",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::JerkTime,"JR",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::Kd,"KD",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::Ki,"KI",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::Kp,"KP",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::Kv,"KV",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::HomeVelocity,"OH",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::HomeTime,"OT",CommandParameterType::Float,CommandGetSetType::GetSet,0},
	{CommandType::KeypadEnable,"JM",CommandParameterType::Int,CommandGetSetType::GetSet,0},
	{CommandType::ErrorCommands,"TE",CommandParameterType::None,CommandGetSetType::GetAlways,0},
	{CommandType::ErrorStatus,"TS",CommandParameterType::None,CommandGetSetType::GetAlways,0},
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
	SerialPort->begin(57600);
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
		MotorState[Index].PositionLimitNegative = -1000.0;
		MotorState[Index].PositionLimitPositive = 1000.0;
		MotorState[Index].SoftLimitNegative = -1000.0;
		MotorState[Index].SoftLimitPositive = 1000.0;
		MotorState[Index].Velocity = 0.0;
		MotorState[Index].Acceleration = 0.0;
		MotorState[Index].PollStatus = false;
		MotorState[Index].PollPosition = false;
		MotorState[Index].ReverseDirection = false;
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
	AutomaticallyPoll = true;
	Mode = ModeType::Inactive;
}

void SMC100Chained::Begin()
{
	Mode = ModeType::Idle;
	SerialPort->write(17); //0x11 Xon
	SerialPort->write(CarriageReturnCharacter);
	SerialPort->write(NewLineCharacter);
	for (uint8_t Index = 0; Index < MotorCount; ++Index)
	{
		CommandEnqueue(Index, CommandType::ErrorStatus, 0.0, CommandGetSetType::Get);
		CommandEnqueue(Index, CommandType::LimitPositive, 0.0, CommandGetSetType::Get);
		CommandEnqueue(Index, CommandType::LimitNegative, 0.0, CommandGetSetType::Get);
		CommandEnqueue(Index, CommandType::Velocity, 0.0, CommandGetSetType::Get);
		CommandEnqueue(Index, CommandType::Acceleration, 0.0, CommandGetSetType::Get);
		CommandEnqueue(Index, CommandType::GPIOInput, 0.0, CommandGetSetType::None);
	}
}

void SMC100Chained::UpdateAfterHoming()
{
	for (uint8_t MotorIndex = 0; MotorIndex < MotorCount; ++MotorIndex)
	{
		UpdateMotorParameters(MotorIndex);
	}
}

void SMC100Chained::AutomaticPolling(bool EnableAutomaticPolling)
{
	AutomaticallyPoll = EnableAutomaticPolling;
}

void SMC100Chained::UpdateMotorParameters(uint8_t MotorIndex)
{
	CommandEnqueue(MotorIndex, CommandType::LimitPositive, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::LimitNegative, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::Velocity, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::Acceleration, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::GPIOInput, 0.0, CommandGetSetType::None);
}

void SMC100Chained::ClearMotorSettings()
{
	CurrentSettings.MotorIndex = -1;
	CurrentSettings.Address = -1;
	CurrentSettings.Complete = false;
	CurrentSettings.Backlash = NAN;
	CurrentSettings.Hysteresis = NAN;
	CurrentSettings.FilterKd = NAN;
	CurrentSettings.ErrorLimit = NAN;
	CurrentSettings.FrictionCompensation = NAN;
	CurrentSettings.JerkTime = NAN;
	CurrentSettings.Kd = NAN;
	CurrentSettings.Ki = NAN;
	CurrentSettings.Kp = NAN;
	CurrentSettings.Kv = NAN;
	CurrentSettings.HomeVelocity = NAN;
	CurrentSettings.HomeTime = NAN;
}

void SMC100Chained::SendGetMotorSettings(uint8_t MotorIndex)
{
	ClearMotorSettings();
	CurrentSettings.MotorIndex = MotorIndex;
	CurrentSettings.Address = MotorState[MotorIndex].Address;
	CommandEnqueue(MotorIndex, CommandType::Backlash, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::Hysteresis, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::FilterKd, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::ErrorLimit, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::FrictionCompensation, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::JerkTime, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::Kd, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::Ki, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::Kp, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::Kv, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::HomeVelocity, 0.0, CommandGetSetType::Get);
	CommandEnqueue(MotorIndex, CommandType::HomeTime, 0.0, CommandGetSetType::Get);
}

SMC100Chained::MotorSettings SMC100Chained::GetMotorSettings()
{
	return CurrentSettings;
}

void SMC100Chained::UpdateMotorSetting(MotorSettings NewSettings)
{
	CommandEnqueue(NewSettings.MotorIndex, CommandType::Configure, 1.0, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::Backlash, NewSettings.Backlash, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::Hysteresis, NewSettings.Hysteresis, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::FilterKd, NewSettings.FilterKd, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::ErrorLimit, NewSettings.ErrorLimit, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::FrictionCompensation, NewSettings.FrictionCompensation, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::JerkTime, NewSettings.JerkTime, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::Kd, NewSettings.Kd, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::Ki, NewSettings.Ki, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::Kp, NewSettings.Kp, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::Kv, NewSettings.Kv, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::HomeVelocity, NewSettings.HomeVelocity, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::HomeTime, NewSettings.HomeTime, CommandGetSetType::Set);
	CommandEnqueue(NewSettings.MotorIndex, CommandType::Configure, 0.0, CommandGetSetType::Set);
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

void SMC100Chained::PrintMotorIndexError()
{
	Serial.print("<SMCERROR>(Motor index too large)");
}

void SMC100Chained::Enable(uint8_t MotorIndex,bool Setting)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return;
	}
	float ParamterValue = 0.0;
	if (Setting)
	{
		ParamterValue = 1.0;
	}
	CommandEnqueue(MotorIndex, CommandType::Enable, ParamterValue, CommandGetSetType::Set);
}

bool SMC100Chained::GetReversed(uint8_t MotorIndex)
{
	if (MotorIndex < MotorCount)
	{
		return MotorState[MotorIndex].ReverseDirection;
	}
	return false;
}

void SMC100Chained::SetReversed(uint8_t MotorIndex, bool Setting)
{
	if (MotorIndex < MotorCount)
	{
		MotorState[MotorIndex].ReverseDirection = Setting;
	}
}

void SMC100Chained::SetSoftLimitRange(uint8_t MotorIndex, float NegativeLimit, float PositiveLimit)
{
	if (MotorIndex < MotorCount)
	{
		if (NegativeLimit > PositiveLimit)
		{
			float Temp = PositiveLimit;
			PositiveLimit = NegativeLimit;
			NegativeLimit = Temp;
		}
		MotorState[MotorIndex].SoftLimitNegative = NegativeLimit;
		MotorState[MotorIndex].SoftLimitPositive = PositiveLimit;
	}
}

bool SMC100Chained::IsBusy()
{
	return Busy;
}

void SMC100Chained::Home(uint8_t MotorIndex)
{
	if (MotorState[MotorIndex].HasBeenHomed)
	{
		if ( (HomeCompleteCallback != NULL) )
		{
			NeedToFireHomeComplete = false;
			HomeCompleteCallback();
		}
		UpdateMotorParameters(MotorIndex);
	}
	else
	{
		CommandEnqueue(MotorIndex, CommandType::Home, 0.0, CommandGetSetType::None);
	}
}

void SMC100Chained::SendGetVelocity(uint8_t MotorIndex, FinishedListener Callback)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return;
	}
	CommandEnqueue(MotorIndex, CommandType::Velocity, 0.0, CommandGetSetType::Get, Callback);
}

void SMC100Chained::SendGetAcceleration(uint8_t MotorIndex, FinishedListener Callback)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return;
	}
	CommandEnqueue(MotorIndex, CommandType::Acceleration, 0.0, CommandGetSetType::Get, Callback);
}

void SMC100Chained::SendSetVelocity(uint8_t MotorIndex, float VelocityToSet, FinishedListener Callback)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return;
	}
	CommandEnqueue(MotorIndex, CommandType::Velocity, VelocityToSet, CommandGetSetType::Set, Callback);
}

void SMC100Chained::SendSetAcceleration(uint8_t MotorIndex, float AccelerationToSet, FinishedListener Callback)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return;
	}
	CommandEnqueue(MotorIndex, CommandType::Acceleration, AccelerationToSet, CommandGetSetType::Set, Callback);
}

void SMC100Chained::MoveAbsolute(uint8_t MotorIndex, float Target)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return;
	}
	if (Target > MotorState[MotorIndex].SoftLimitPositive)
	{
		Target = MotorState[MotorIndex].SoftLimitPositive;
	}
	if (Target < MotorState[MotorIndex].SoftLimitNegative)
	{
		Target = MotorState[MotorIndex].SoftLimitNegative;
	}
	if (MotorState[MotorIndex].ReverseDirection)
	{
		Target = -1.0*Target;
	}
	if (Target < MotorState[MotorIndex].PositionLimitNegative)
	{
		Target = MotorState[MotorIndex].PositionLimitNegative;
		Serial.print("<SMCERROR>(Position for motor ");
		Serial.print(MotorIndex);
		Serial.print(" is under limit.)\n");
	}
	if (Target > MotorState[MotorIndex].PositionLimitPositive)
	{
		Target = MotorState[MotorIndex].PositionLimitPositive;
		Serial.print("<SMCERROR>(Position for motor ");
		Serial.print(MotorIndex);
		Serial.print(" is over limit.)\n");
	}
	CommandEnqueue(MotorIndex, CommandType::MoveAbs, Target, CommandGetSetType::Set);
}

void SMC100Chained::SendGetErrorStatus(uint8_t MotorIndex, FinishedListener Callback)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return;
	}
	CommandEnqueue(MotorIndex, CommandType::ErrorStatus, 0, CommandGetSetType::Get, Callback);
}

void SMC100Chained::SendGetPosition(uint8_t MotorIndex, FinishedListener Callback)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return;
	}
	CommandEnqueue(MotorIndex, CommandType::PositionReal, 0, CommandGetSetType::Get, Callback);
}

float SMC100Chained::GetVelocity(uint8_t MotorIndex)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return NAN;
	}
	return MotorState[MotorIndex].Velocity;
}

float SMC100Chained::GetAcceleration(uint8_t MotorIndex)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return NAN;
	}
	return MotorState[MotorIndex].Acceleration;
}

float SMC100Chained::GetPosition(uint8_t MotorIndex)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return NAN;
	}
	if (MotorState[MotorIndex].ReverseDirection)
	{
		return -1.0*MotorState[MotorIndex].Position;
	}
	else
	{
		return MotorState[MotorIndex].Position;
	}
}

void SMC100Chained::SendGetGPIOInput(uint8_t MotorIndex)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
		return;
	}
	CommandEnqueue(MotorIndex, CommandType::GPIOInput, 0.0, CommandGetSetType::None);
}

bool SMC100Chained::GetGPIOInput(uint8_t MotorIndex, uint8_t Pin)
{
	if (MotorIndex >= MotorCount)
	{
		PrintMotorIndexError();
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
		PrintMotorIndexError();
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
		PrintMotorIndexError();
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
	bool CheckIsIdle = true;
	if ( (Mode == ModeType::Idle) && CommandQueueEmpty() && AutomaticallyPoll)
	{
		if (PollStatus)
		{
			CheckErrorStatusPoll();
			CheckIsIdle = false;
		}
		if (PollPosition)
		{
			CheckPositionPoll();
			CheckIsIdle = false;
		}
	}
	switch (Mode)
	{
		case ModeType::Idle:
			if (ErrorRecovery)
			{
				if (Verbose)
				{
					Serial.print("<SMCERROR>(Sending current recovery command.)");
				}
				ErrorRecovery = false;
				SendCurrentCommand();
			}
			else
			{
				CheckCommandQueue();
			}
			break;
		case ModeType::WaitForCommandReply:
			CheckForCommandReply();
			break;
		case ModeType::WaitOnSentCommand:
			if ( (millis() - SentTime) > SentDelay)
			{
				SentTime = 0;
				SentDelay = 0;
				ModeTransitionToIdle();
			}
			break;
		default:
			break;
	}
	if ( Busy && CheckIsIdle && (Mode == ModeType::Idle) && CommandQueueEmpty() )
	{
		Busy = false;
		if (AllCompleteCallback != NULL)
		{
			AllCompleteCallback();
		}
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
			PollStatusTimeLast = micros();
			if (Verbose)
			{
				Serial.print("<SMCV>(Status poll : ");
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
		for (int MotorIndex = 0; MotorIndex < MotorCount; ++MotorIndex)
		{
			if (MotorState[MotorIndex].PollStatus)
			{
				EnqueueErrorStatusRequest(MotorIndex);
			}
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
		if (MotorState[MotorIndex].PollPosition != Enable)
		{
			EnqueuePositionRequest(MotorIndex);
			MotorState[MotorIndex].PollPosition = Enable;
			MotorState[MotorIndex].NeedToPollPosition = false;
			PollPosition = Enable;
			PollPositionTimeLast = micros();
			if (Verbose)
			{
				Serial.print("<SMCV>(Position poll ");
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

void SMC100Chained::ModeTransitionToWaitOnSentCommand()
{
	Mode = ModeType::WaitOnSentCommand;
}

void SMC100Chained::ModeTransitionToIdle()
{
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
	bool KeepGoing = SerialPort->available() > 0;
	while (KeepGoing)
	{
		char NewChar = SerialPort->read();
		if (NewChar == CarriageReturnCharacter)
		{
			KeepGoing = SerialPort->available() > 0;
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
			KeepGoing = false;
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
				ModeTransitionToIdle();
				KeepGoing = false;
			}
			else
			{
				KeepGoing = SerialPort->available() > 0;
			}
		}
	}
	if ( (micros() - TransmitTime) > CommandReplyTimeMax )
	{
		ModeTransitionToIdle();
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
			bool ErrorHandled = false;
			if (*ParameterAddress != NoErrorCharacter)
			{
				ErrorHandled = UpdateCommandErrors(AddressOfReply, *ParameterAddress);
				if (!ErrorHandled)
				{
					Serial.print("<SMC100Chained>(Error code: ");
					Serial.print(ConvertToErrorString(*ParameterAddress));
					Serial.print(" motor: ");
					Serial.print(AddressOfReply);
					Serial.print(")\n");
				}
			}
			else
			{
				ErrorRecovery = false;
				ErrorCount = 0;
				if ((ErrorBackupCompleteCallback != NULL) && (!ErrorBackupCompleteCallbackFired))
				{
					ErrorBackupCompleteCallbackFired = true;
					ErrorBackupCompleteCallback();
				}
			}
		}
		else if (CurrentCommand->Command == CommandType::ErrorStatus)
		{
			bool ErrorStatusFlag = false;
			char ErrorCode[4];
			for (uint8_t Index = 0; Index < 4; Index++)
			{
				ErrorCode[Index] = *(ParameterAddress + Index);
				if (ErrorCode[Index] != '0')
				{
					ErrorStatusFlag = true;
				}
			}
			if (ErrorStatusFlag)
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
				UpdatePositionLimitPositive(AddressOfReply, PositionLimitPositive);
			}
		}
		else if (CurrentCommand->Command == CommandType::Velocity)
		{
			if (CurrentCommandGetOrSet == CommandGetSetType::Get)
			{
				float Velocity = atof(ParameterAddress);
				UpdateVelocity(AddressOfReply, Velocity);
			}
		}
		else if (CurrentCommand->Command == CommandType::Acceleration)
		{
			if (CurrentCommandGetOrSet == CommandGetSetType::Get)
			{
				float Acceleration = atof(ParameterAddress);
				UpdateAcceleration(AddressOfReply, Acceleration);
			}
		}
		else if ( IsMotorSetting(CurrentCommand->Command) )
		{
			if (CurrentCommandGetOrSet == CommandGetSetType::Get)
			{
				float SettingValue = atof(ParameterAddress);
				UpdateMotorSetting(AddressOfReply, CurrentCommand->Command, SettingValue);
			}
		}
		if (
			(CurrentCommand->Command != CommandType::ErrorCommands) &&
			(CurrentCommand->Command != CommandType::ErrorStatus) &&
			(CurrentCommand->Command != CommandType::PositionReal) &&
			AutomaticallyPoll )
		{
			SendErrorCommands(CurrentCommandMotorIndex);
		}
		else
		{
			if ((CurrentCommandCompleteCallback != NULL) && (!CurrentCommandCompleteCallbackFired))
			{
				CurrentCommandCompleteCallbackFired = true;
				CurrentCommandCompleteCallback();
			}
			//if (ErrorRecovery)
			//{
			//	ModeTransitionToWaitOnSentCommand();
			//}
			//else
			//{
			ModeTransitionToIdle();
			//}
		}
	}
}

void SMC100Chained::UpdateMotorSetting(uint8_t MotorAddress, CommandType SettingType, float SettingValue)
{
	if (CurrentSettings.Address == -1)
	{
		CurrentSettings.Address = MotorAddress;
	}
	if (MotorAddress == CurrentSettings.Address)
	{
		switch(SettingType)
		{
			case CommandType::Backlash :
				CurrentSettings.Backlash = SettingValue;
				break;
			case CommandType::Hysteresis :
				CurrentSettings.Hysteresis = SettingValue;
				break;
			case CommandType::FilterKd :
				CurrentSettings.FilterKd = SettingValue;
				break;
			case CommandType::ErrorLimit :
				CurrentSettings.ErrorLimit = SettingValue;
				break;
			case CommandType::FrictionCompensation :
				CurrentSettings.FrictionCompensation = SettingValue;
				break;
			case CommandType::JerkTime :
				CurrentSettings.JerkTime = SettingValue;
				break;
			case CommandType::Kd :
				CurrentSettings.Kd = SettingValue;
				break;
			case CommandType::Ki :
				CurrentSettings.Ki = SettingValue;
				break;
			case CommandType::Kp :
				CurrentSettings.Kp = SettingValue;
				break;
			case CommandType::Kv :
				CurrentSettings.Kv = SettingValue;
				break;
			case CommandType::HomeVelocity :
				CurrentSettings.HomeVelocity = SettingValue;
				break;
			case CommandType::HomeTime :
				CurrentSettings.HomeTime = SettingValue;
				break;
			default:
				break;
		}
		bool NowComplete = CheckSettingsComplete();
		if (!CurrentSettings.Complete && NowComplete)
		{
			PrintMotorSettings();
		}
		CurrentSettings.Complete = NowComplete;
		if (Verbose)
		{
			Serial.print("<SMCV>(Setting check ");
			Serial.print(NowComplete);
			Serial.print(")\n");
		}
	}
	else
	{
		Serial.print("<SMC100Chained>(Error motor setting address of ");
		Serial.print(MotorAddress);
		Serial.print(" seen when settting ");
		Serial.print(CurrentSettings.Address);
		Serial.print(")\n");
	}
}

void SMC100Chained::PrintMotorSettings()
{
	Serial.print("[SMCSET](");
	Serial.print(CurrentSettings.MotorIndex);
	Serial.print(",Address,");
	Serial.print(CurrentSettings.Address);
	Serial.print(",Backlash,");
	Serial.print(CurrentSettings.Backlash);
	Serial.print(",Hysteresis,");
	Serial.print(CurrentSettings.Hysteresis);
	Serial.print(",FilterKd,");
	Serial.print(CurrentSettings.FilterKd);
	Serial.print(",ErrorLimit,");
	Serial.print(CurrentSettings.ErrorLimit);
	Serial.print(",FrictionCompensation,");
	Serial.print(CurrentSettings.FrictionCompensation);
	Serial.print(",JerkTime,");
	Serial.print(CurrentSettings.JerkTime);
	Serial.print(",Kd,");
	Serial.print(CurrentSettings.Kd);
	Serial.print(",Ki,");
	Serial.print(CurrentSettings.Ki);
	Serial.print(",Kp,");
	Serial.print(CurrentSettings.Kp);
	Serial.print(",Kv,");
	Serial.print(CurrentSettings.Kv);
	Serial.print(",HomeVelocity,");
	Serial.print(CurrentSettings.HomeVelocity);
	Serial.print(",HomeTime,");
	Serial.print(CurrentSettings.HomeTime);
	Serial.print(")\n");
}

bool SMC100Chained::CheckSettingsComplete()
{
	if (isnan(CurrentSettings.Backlash))
	{
		return false;
	}
	if (isnan(CurrentSettings.Hysteresis))
	{
		return false;
	}
	if (isnan(CurrentSettings.FilterKd))
	{
		return false;
	}
	if (isnan(CurrentSettings.ErrorLimit))
	{
		return false;
	}
	if (isnan(CurrentSettings.FrictionCompensation))
	{
		return false;
	}
	if (isnan(CurrentSettings.JerkTime))
	{
		return false;
	}
	if (isnan(CurrentSettings.Kd))
	{
		return false;
	}
	if (isnan(CurrentSettings.Ki))
	{
		return false;
	}
	if (isnan(CurrentSettings.Kp))
	{
		return false;
	}
	if (isnan(CurrentSettings.Kv))
	{
		return false;
	}
	if (isnan(CurrentSettings.HomeVelocity))
	{
		return false;
	}
	if (isnan(CurrentSettings.HomeTime))
	{
		return false;
	}
	return true;
}

bool SMC100Chained::IsMotorSetting(CommandType TypeToCheck)
{
	switch(TypeToCheck)
	{
		case CommandType::Backlash :
		case CommandType::Hysteresis :
		case CommandType::FilterKd :
		case CommandType::ErrorLimit :
		case CommandType::FrictionCompensation :
		case CommandType::JerkTime :
		case CommandType::Kd :
		case CommandType::Ki :
		case CommandType::Kp :
		case CommandType::Kv :
		case CommandType::HomeVelocity :
		case CommandType::HomeTime :
			return true;
		default:
			return false;
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

void SMC100Chained::UpdateVelocity(uint8_t MotorAddress, float VelocityToSet)
{
	uint8_t MotorIndex = 0;
	bool FoundMotor = ConvertMotorAddressToIndex(MotorAddress, &MotorIndex);
	if (FoundMotor)
	{
		MotorState[MotorIndex].Velocity = VelocityToSet;
	}
}

void SMC100Chained::UpdateAcceleration(uint8_t MotorAddress, float AccelerationToSet)
{
	uint8_t MotorIndex = 0;
	bool FoundMotor = ConvertMotorAddressToIndex(MotorAddress, &MotorIndex);
	if (FoundMotor)
	{
		MotorState[MotorIndex].Acceleration = AccelerationToSet;
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
		if (MotorState[MotorIndex].Status == StatusType::Ready)
		{
			if (MotorState[MotorIndex].FinishedCallback != NULL)
			{
				MotorState[MotorIndex].FinishedCallback();
			}
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
		if (Verbose)
		{
			Serial.print("<SMCV>(All motors position polled)\n");
		}
		if (NeedToFireHomeComplete)
		{
			NeedToFireHomeComplete = false;
			if (HomeCompleteCallback != NULL)
			{
				HomeCompleteCallback();
			}
			UpdateAfterHoming();
		}
		if ( (NeedToFireMoveComplete) && (MoveCompleteCallback != NULL) )
		{
			NeedToFireMoveComplete = false;
			MoveCompleteCallback();
		}
	}
}

bool SMC100Chained::UpdateCommandErrors(uint8_t MotorAddress, char ErrorChar)
{
	bool Handled = false;
	if (ErrorChar == 'H')
	{
		uint8_t MotorIndex = 0;
		bool FoundMotor = ConvertMotorAddressToIndex(MotorAddress, &MotorIndex);
		if (FoundMotor)
		{
			MotorState[MotorIndex].HasBeenHomed = false;
		}
		Handled = false;
	}
	else if (ErrorChar == 'M')
	{
		if (ErrorCount < ErrorCountMaxLimit)
		{
			RestoreErrorBackup();
			//SentDelay = 50000;
			//SentTime = millis();
			ErrorRecovery = true;
			ErrorCount++;
			if (Verbose)
			{
				Serial.print("<SMCV>(Command not allowed during movement. Resending.)\n");
			}
			Handled = true;
		}
		else
		{
			Serial.print("<SMC100Chained>(Over error count. Aborting command.)\n");
			ErrorCount = 0;
			ErrorRecovery = false;
			Handled = false;
		}
	}
	return Handled;
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

const char* SMC100Chained::ConvertToErrorString(char ErrorChar)
{
	switch (ErrorChar)
	{
		case 'A':
			return "Unknown message";
		case 'B':
			return "Incorrect address";
		case 'C':
			return "Parameter missing";
		case 'D':
			return "Command not allowed";
		case 'E':
			return "Already homing";
		case 'F':
			return "ESP stage unknown";
		case 'G':
			return "Displacement out of limits";
		case 'H':
			return "Not allowed in NOT REFERENCED";
		case 'I':
			return "Not allowed in CONFIGURATION";
		case 'J':
			return "Not allowed in DISABLED";
		case 'K':
			return "Not allowed in READY";
		case 'L':
			return "Not allowed in HOMING";
		case 'M':
			return "Not allowed in MOVING";
		case 'N':
			return "Out of soft limits";
		case 'S':
			return "Communication time out";
		case 'U':
			return "EEPROM error";
		case 'V':
			return "Error during command execution";
		case 'W':
			return "Command not allowed for PP";
		case 'X':
			return "Command not allowed for CC";
		default:
			return "Unknown error character";
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
		}
	}
	if (AllMotorsPolled && AutomaticallyPoll)
	{
		PollStatus = false;
		for (uint8_t Index = 0; Index < MotorCount; ++Index)
		{
			if (MotorState[Index].NeedToPollPosition)
			{
				MotorState[Index].NeedToPollPosition = false;
				PreparePositionPolling(Index);
				if (Verbose)
				{
					Serial.print("<SMCV>(Poll status done ");
					Serial.print(Index);
					Serial.print(" polling position.)\n");
				}
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
		Serial.print("<SMCV>(Send: ");
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
			MotorState[CurrentCommandMotorIndex].Status = StatusType::Moving;
			if (AutomaticallyPoll)
			{
				PrepareErrorStatusPolling(CurrentCommandMotorIndex);
			}
		}
	}
	if ( (CurrentCommand->Command == CommandType::Home) )
	{
		NeedToFireHomeComplete = true;
		AutomaticallyPoll = true;
		MotorState[CurrentCommandMotorIndex].NeedToPollPosition = true;
		PrepareErrorStatusPolling(CurrentCommandMotorIndex);
	}
	if (CurrentCommandGetOrSet == CommandGetSetType::Set)
	{
		if (CurrentCommand->Command == CommandType::Velocity)
		{
			MotorState[CurrentCommandMotorIndex].Velocity = CurrentCommandParameter;
		}
		else if (CurrentCommand->Command == CommandType::Acceleration)
		{
			MotorState[CurrentCommandMotorIndex].Acceleration = CurrentCommandParameter;
		}
		else if (CurrentCommand->Command == CommandType::LimitPositive)
		{
			MotorState[CurrentCommandMotorIndex].PositionLimitPositive = CurrentCommandParameter;
		}
		else if (CurrentCommand->Command == CommandType::LimitNegative)
		{
			MotorState[CurrentCommandMotorIndex].PositionLimitNegative = CurrentCommandParameter;
		}
	}
	if ((CurrentCommand->SentDelay != 0) && (CurrentCommandGetOrSet != CommandGetSetType::Get) && ((int)CurrentCommandParameter == 0))
	{
		SentDelay = CurrentCommand->SentDelay;
		SentTime = millis();
		ModeTransitionToWaitOnSentCommand();
		if (Verbose)
		{
			Serial.print("<SMCV>(Waiting on configure commit to memory.)\n");
		}
	}
	else if ( (CurrentCommandGetOrSet == CommandGetSetType::Get) || (CurrentCommand->GetSetType == CommandGetSetType::GetAlways) )
	{
		ModeTransitionToWaitForReply();
	}
	else if (
		(CurrentCommand->Command != CommandType::ErrorCommands) &&
		(CurrentCommand->Command != CommandType::ErrorStatus) &&
		(CurrentCommand->Command != CommandType::PositionReal) &&
		AutomaticallyPoll)
	{
		SendErrorCommands(CurrentCommandMotorIndex);
	}
	else
	{
		if ( (CurrentCommandCompleteCallback != NULL) && (CurrentCommandGetOrSet == CommandGetSetType::Set) && (!CurrentCommandCompleteCallbackFired) )
		{
			CurrentCommandCompleteCallbackFired = true;
			CurrentCommandCompleteCallback();
		}
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
	CommandEnqueue(MotorIndex, CommandType::LimitNegative, 0.0, CommandGetSetType::Get);
}
void SMC100Chained::EnqueueGetLimitPositive(uint8_t MotorIndex)
{
	CommandEnqueue(MotorIndex, CommandType::LimitPositive, 0.0, CommandGetSetType::Get);
}
void SMC100Chained::EnqueueErrorCommandRequest(uint8_t MotorIndex)
{
	CommandEnqueue(MotorIndex, CommandType::ErrorCommands, 0.0, CommandGetSetType::Get);
}
void SMC100Chained::EnqueueErrorStatusRequest(uint8_t MotorIndex)
{
	CommandEnqueue(MotorIndex, CommandType::ErrorStatus, 0.0, CommandGetSetType::Get);
}
void SMC100Chained::EnqueuePositionRequest(uint8_t MotorIndex)
{
	CommandEnqueue(MotorIndex, CommandType::PositionReal, 0.0, CommandGetSetType::Get);
}
void SMC100Chained::CommandEnqueue(uint8_t MotorIndex, CommandType Type, float Parameter, CommandGetSetType GetOrSet)
{
	const CommandStruct* CommandPointer = &CommandLibrary[static_cast<uint8_t>(Type)];
	CommandEnqueue(MotorIndex, CommandPointer, Parameter, GetOrSet, NULL);
}
void SMC100Chained::CommandEnqueue(uint8_t MotorIndex, CommandType Type, float Parameter, CommandGetSetType GetOrSet, FinishedListener CommandCompleteCallback)
{
	const CommandStruct* CommandPointer = &CommandLibrary[static_cast<uint8_t>(Type)];
	CommandEnqueue(MotorIndex, CommandPointer, Parameter, GetOrSet, CommandCompleteCallback);
}
void SMC100Chained::CommandEnqueue(uint8_t MotorIndex, const CommandStruct* CommandPointer, float Parameter, CommandGetSetType GetOrSet, FinishedListener CommandCompleteCallback)
{
	CommandQueue[CommandQueueHead].Command = CommandPointer;
	CommandQueue[CommandQueueHead].Parameter = Parameter;
	CommandQueue[CommandQueueHead].MotorIndex = MotorIndex;
	CommandQueue[CommandQueueHead].GetOrSet = GetOrSet;
	CommandQueue[CommandQueueHead].CompleteCallback = CommandCompleteCallback;
	if (Verbose)
	{
		Serial.print("<SMCV>(Enqueue ");
		Serial.print(CommandQueue[CommandQueueHead].Command->CommandChar);
		Serial.print(",");
		Serial.print(MotorIndex);
		Serial.print(",");
		Serial.print(static_cast<uint8_t>(GetOrSet));
		Serial.print(",");
		Serial.print(Parameter);
		Serial.print(")\n");
	}
	CommandQueueAdvance();
}
void SMC100Chained::RestoreErrorBackup()
{
	if (Verbose)
	{
		Serial.print("<SMCV>(Recover error backup.)\n");
	}
	CommandEnqueue(ErrorBackupMotorIndex, ErrorBackupCommand, ErrorBackupParameter, ErrorBackupGetOrSet, ErrorBackupCompleteCallback);
	//CurrentCommand = ErrorBackupCommand;
	//CurrentCommandParameter = ErrorBackupParameter;
	//CurrentCommandAddress = ErrorBackupAddress;
	//CurrentCommandGetOrSet = ErrorBackupGetOrSet;
	//CurrentCommandMotorIndex = ErrorBackupMotorIndex;
	//CurrentCommandCompleteCallback = ErrorBackupCompleteCallback;
	//CurrentCommandCompleteCallbackFired = ErrorBackupCompleteCallbackFired;
}
void SMC100Chained::SendErrorCommands(uint8_t MotorIndex)
{
	ErrorBackupCommand = CurrentCommand;
	ErrorBackupParameter = CurrentCommandParameter;
	ErrorBackupAddress = CurrentCommandAddress;
	ErrorBackupGetOrSet = CurrentCommandGetOrSet;
	ErrorBackupMotorIndex = CurrentCommandMotorIndex;
	ErrorBackupCompleteCallback = CurrentCommandCompleteCallback;
	ErrorBackupCompleteCallbackFired = CurrentCommandCompleteCallbackFired;
	CurrentCommand = &CommandLibrary[static_cast<uint8_t>(CommandType::ErrorCommands)];
	CurrentCommandParameter = 0.0;
	CurrentCommandGetOrSet = CommandGetSetType::Get;
	CurrentCommandMotorIndex = MotorIndex;
	CurrentCommandAddress = MotorState[CurrentCommandMotorIndex].Address;
	CurrentCommandCompleteCallback = NULL;
	CurrentCommandCompleteCallbackFired = false;
	SendCurrentCommand();
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
		CurrentCommandCompleteCallback = CommandQueue[CommandQueueTail].CompleteCallback;
		if (CurrentCommandCompleteCallback != NULL)
		{
			CurrentCommandCompleteCallbackFired = false;
		}
		else
		{
			CurrentCommandCompleteCallbackFired = true;
		}
		CommandQueueRetreat();
		Status = true;
		if (Verbose)
		{
			Serial.print("<SMCV>(Dequeue ");
			Serial.print(CurrentCommand->CommandChar);
			Serial.print(",");
			Serial.print(CurrentCommandMotorIndex);
			Serial.print(",");
			Serial.print(CurrentCommandAddress);
			Serial.print(",");
			Serial.print(static_cast<uint8_t>(CurrentCommandGetOrSet));
			Serial.print(",");
			Serial.print(CurrentCommandParameter,5);
			Serial.print(",");
			Serial.print( ( (CurrentCommandCompleteCallback != NULL) ? 'Y' : 'N' ) );
			Serial.print(",");
			Serial.print( ( (CurrentCommandCompleteCallbackFired) ? 'Y' : 'N' ) );
			Serial.print(")\n");
		}
		//Serial.print("NP");
		//Serial.print(CurrentCommandParameter);
		//Serial.print("\n");
	}
	return Status;
}
