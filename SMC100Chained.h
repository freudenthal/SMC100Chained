#ifndef SMC100Chained_h	//check for multiple inclusions
#define SMC100Chained_h

#include "Arduino.h"

#define SMC100ChainedQueueCount 32
#define SMC100ChainedMaxMotors 3
#define SMC100ChainedReplyBufferSize 32

class SMC100Chained
{
	public:
		typedef void ( *FinishedListener )();
		enum class CommandType : uint8_t
		{
			None,
			Enable,
			Home,
			MoveAbs,
			MoveRel,
			MoveEstimate,
			Configure,
			Analogue,
			GPIOInput,
			Reset,
			GPIOOutput,
			LimitPositive,
			LimitNegative,
			PositionAsSet,
			PositionReal,
			Velocity,
			Acceleration,
			Backlash,
			Hysteresis,
			FilterKd,
			ErrorLimit,
			FrictionCompensation,
			JerkTime,
			Kd,
			Ki,
			Kp,
			Kv,
			HomeVelocity,
			HomeTime,
			KeypadEnable,
			ErrorCommands,
			ErrorStatus,
		};
		enum class CommandParameterType : uint8_t
		{
			None,
			Int,
			Float,
			ErrorCommand,
			ErrorStatus
		};
		enum class StatusType : uint8_t
		{
			Unknown,
			Error,
			NoReference,
			Homing,
			Moving,
			Ready,
			Disabled,
			Jogging,
		};
		enum class CommandGetSetType : uint8_t
		{
			None,
			Get,
			Set,
			GetSet,
			GetAlways,
		};
		enum class ModeType : uint8_t
		{
			Inactive,
			Idle,
			WaitForCommandReply,
			WaitOnSentCommand,
		};
		struct CommandStruct
		{
			CommandType Command;
			const char* CommandChar;
			CommandParameterType SendType;
			CommandGetSetType GetSetType;
			uint32_t SentDelay;
		};
		struct CommandQueueEntry
		{
			const CommandStruct* Command;
			CommandGetSetType GetOrSet;
			uint8_t MotorIndex;
			float Parameter;
			FinishedListener CompleteCallback;
		};
		struct StatusCharSet
		{
			const char* Code;
			StatusType Type;
		};
		struct MotorSettings
		{
			int MotorIndex;
			uint8_t Address;
			bool Complete;
			float Backlash;
			float Hysteresis;
			float FilterKd;
			float ErrorLimit;
			float FrictionCompensation;
			float JerkTime;
			float Kd;
			float Ki;
			float Kp;
			float Kv;
			float HomeVelocity;
			float HomeTime;
		};
		struct MotorStatus
		{
			uint8_t Address;
			StatusType Status;
			bool HasBeenHomed;
			float Position;
			uint8_t GPIOInput;
			uint8_t GPIOOutput;
			float AnalogueReading;
			float PositionLimitNegative;
			float PositionLimitPositive;
			float SoftLimitNegative;
			float SoftLimitPositive;
			float Velocity;
			float Acceleration;
			bool ReverseDirection;
			bool PollStatus;
			bool PollPosition;
			bool NeedToPollPosition;
			FinishedListener FinishedCallback;
		};
		SMC100Chained(HardwareSerial* serial, const uint8_t* addresses, const uint8_t addresscount);
		void Check();
		void Begin();
		void AutomaticPolling(bool EnableAutomaticPolling);
		bool IsHomed(uint8_t MotorIndex);
		bool IsReady(uint8_t MotorIndex);
		bool IsMoving(uint8_t MotorIndex);
		bool IsEnabled(uint8_t MotorIndex);
		void Enable(uint8_t MotorIndex, bool Setting);
		bool GetReversed(uint8_t MotorIndex);
		void SetReversed(uint8_t MotorIndex, bool Setting);
		void SetSoftLimitRange(uint8_t MotorIndex, float NegativeLimit, float PositiveLimit);
		bool IsBusy();
		void Home(uint8_t MotorIndex);
		void MoveAbsolute(uint8_t MotorIndex, float Target);
		void SetGPIOOutput(uint8_t MotorIndex, uint8_t Pin, bool Output);
		void SetGPIOOutputAll(uint8_t MotorIndex, uint8_t Code);
		void SendGetGPIOInput(uint8_t MotorIndex);
		bool GetGPIOInput(uint8_t MotorIndex, uint8_t Pin);
		void SetAxesCompleteCallback(uint8_t MotorIndex, FinishedListener Callback);
		void SetAllCompleteCallback(FinishedListener Callback);
		void SetHomeCompleteCallback(FinishedListener Callback);
		void SetMoveCompleteCallback(FinishedListener Callback);
		void SetGPIOReturnCallback(FinishedListener Callback);
		float GetPosition(uint8_t MotorIndex);
		void SetVerbose(bool VerboseToSet);
		void SendGetVelocity(uint8_t MotorIndex, FinishedListener Callback = NULL);
		void SendGetAcceleration(uint8_t MotorIndex, FinishedListener Callback = NULL);
		void SendSetVelocity(uint8_t MotorIndex, float VelocityToSet, FinishedListener Callback = NULL);
		void SendSetAcceleration(uint8_t MotorIndex, float AccelerationToSet, FinishedListener Callback = NULL);
		float GetVelocity(uint8_t MotorIndex);
		float GetAcceleration(uint8_t MotorIndex);
		void SendGetErrorStatus(uint8_t MotorIndex, FinishedListener Callback = NULL);
		void SendGetPosition(uint8_t MotorIndex, FinishedListener Callback = NULL);
		void UpdateMotorParameters(uint8_t MotorIndex);
		void SendGetMotorSettings(uint8_t MotorIndex);
		MotorSettings GetMotorSettings();
		void UpdateMotorSetting(MotorSettings NewSettings);
	private:
		void PrintMotorIndexError();
		void CheckCommandQueue();
		void CheckForCommandReply();
		void CheckWaitAfterSending();
		void ClearCommandQueue();
		bool SendCurrentCommand();
		bool CommandQueueFull();
		bool CommandQueueEmpty();
		uint8_t CommandQueueCount();
		void CommandQueueAdvance();
		void CommandQueueRetreat();
		void UpdateAfterHoming();
		void CommandEnqueue(uint8_t MotorIndex, CommandType Type, float Parameter, CommandGetSetType GetOrSet);
		void CommandEnqueue(uint8_t MotorIndex, CommandType Type, float Parameter, CommandGetSetType GetOrSet, FinishedListener CommandCompleteCallback);
		void CommandEnqueue(uint8_t MotorIndex, const CommandStruct* CommandPointer, float Parameter, CommandGetSetType GetOrSet, FinishedListener CommandCompleteCallback);
		bool CommandQueuePullToCurrentCommand();
		void EnqueueGetLimitNegative(uint8_t MotorIndex);
		void EnqueueGetLimitPositive(uint8_t MotorIndex);
		void EnqueueErrorCommandRequest(uint8_t MotorIndex);
		void EnqueueErrorStatusRequest(uint8_t MotorIndex);
		void EnqueuePositionRequest(uint8_t MotorIndex);
		StatusType ConvertStatus(char* StatusChar);
		void ParseReply();
		void CheckErrorStatusPoll();
		void CheckPositionPoll();
		void PrepareErrorStatusPolling(uint8_t MotorIndex);
		void PrepareErrorStatusPolling(uint8_t MotorIndex, bool Enable);
		void PreparePositionPolling(uint8_t MotorIndex);
		void PreparePositionPolling(uint8_t MotorIndex, bool Enable);
		void PollPositionRealNeededMotors();
		void ModeTransitionToIdle();
		void ModeTransitionToWaitForReply();
		void ModeTransitionToWaitOnSentCommand();
		void UpdatePosition(uint8_t MotorIndex, float Position);
		void UpdateGPIOInput(uint8_t MotorAddress, uint8_t GPIOInputToSet);
		void UpdatePositionLimitPositive(uint8_t MotorAddress, float PositionLimitPositiveToSet);
		void UpdatePositionLimitNegative(uint8_t MotorAddress, float PositionLimitNegativeToSet);
		void UpdateAnalogue(uint8_t MotorAddress, float AnalogueToSet);
		void UpdateStatus(uint8_t MotorAddress, StatusType Status);
		void UpdateVelocity(uint8_t MotorAddress, float VelocityToSet);
		void UpdateAcceleration(uint8_t MotorAddress, float AccelerationToSet);
		void UpdateStateOnSending();
		bool ConvertMotorAddressToIndex(uint8_t Address, uint8_t* MotorIndexReturn);
		void CheckAllPollPosition();
		void CheckAllPollStatus();
		void SendErrorCommands(uint8_t MotorIndex);
		void UpdateCommandErrors(uint8_t MotorIndex, char ErrorCode);
		void ClearMotorSettings();
		bool IsMotorSetting(CommandType TypeToCheck);
		void PrintMotorSettings();
		bool CheckSettingsComplete();
		void UpdateMotorSetting(uint8_t MotorIndex, CommandType SettingType, float SettingValue);
		const char* ConvertToErrorString(char ErrorCode);
		static const uint32_t PollStatusTimeInterval;
		static const uint32_t PollPositionTimeInterval;
		static const CommandStruct CommandLibrary[];
		static const StatusCharSet StatusLibrary[];
		static const uint32_t CommandReplyTimeMax;
		static const uint32_t WipeInputEvery;
		static const char CarriageReturnCharacter;
		static const char NewLineCharacter;
		static const char GetCharacter;
		static const uint32_t WaitAfterSendingTimeMax;
		static const char NoErrorCharacter;
		MotorStatus MotorState[SMC100ChainedMaxMotors];
		MotorSettings CurrentSettings;
		uint8_t MotorCount;
		bool Busy;
		bool Verbose;
		ModeType Mode;
		HardwareSerial* SerialPort;
		FinishedListener AllCompleteCallback;
		FinishedListener MoveCompleteCallback;
		FinishedListener HomeCompleteCallback;
		FinishedListener GPIOReturnCallback;
		FinishedListener CurrentCommandCompleteCallback;
		bool NeedToFireMoveComplete;
		bool NeedToFireHomeComplete;
		bool PollStatus;
		bool PollPosition;
		bool AutomaticallyPoll;
		const CommandStruct* CurrentCommand;
		CommandGetSetType CurrentCommandGetOrSet;
		float CurrentCommandParameter;
		uint8_t CurrentCommandAddress;
		uint8_t CurrentCommandMotorIndex;
		uint32_t LastWipeTime;
		uint32_t TransmitTime;
		uint8_t ReplyBufferIndex;
		uint32_t PollStatusTimeLast;
		uint32_t PollPositionTimeLast;
		uint32_t SentDelay;
		uint32_t SentTime;
		char ReplyBuffer[SMC100ChainedReplyBufferSize];
		CommandQueueEntry CommandQueue[SMC100ChainedQueueCount];
		uint8_t CommandQueueHead;
		uint8_t CommandQueueTail;
		bool CommandQueueFullFlag;
};
#endif
