const static uint8_t msg_queue_len = 10; ///< size of ethercat message buffer

public:
	struct MessageHandler
	{
		static uint8_t LenQ;
		static uint8_t IndQ;
		static const int MsgDt;
		static uint32_t MsgTS;
		static MessageType MsgTp;
		static ErrorType ErrTp;

		uint8_t msgid;
		char msgtypstr[50];
		MessageType msgtype;
		ErrorType errtype;
		uint16_t Reg16[8];

		uint8_t U8i;
		uint8_t U16i;

		static RegUnion _U;

		MessageHandler();

		void set_all(uint8_t _msgid, MessageType _msgtype, const char *_msgtypstr, uint16_t *_Reg16);
		void set_msgid(uint8_t _msgid);
		void set_msg_type(MessageType _msgtype);
		void set_typ_str(const char *_msgtypstr);
		void set_int16(uint16_t *_Reg16);

		uint8_t get_id();
		MessageType get_msg_mt();
		const char *get_msg_type_str();
		uint16_t *get_int16();

		void set_ui16(int p_reg[]);
		void set_ui16(int reg_i16);
		void set_ui8(int reg_i18);
		void set_ui16(int reg_i16, uint8_t set_u16_i);
		void set_ui8(int reg_i8, uint8_t set_u8_i);
	};
	MessageHandler SndMH[msg_queue_len]; ///<  initialize array of MessageHandler structs for sending messages
	MessageHandler RcvMH[msg_queue_len]; ///<  initialize array of MessageHandler structs for receiving messages


    // Initialization of static members
uint8_t Wall_Operation::MessageHandler::LenQ = 0;
uint8_t Wall_Operation::MessageHandler::IndQ = 0;
const int Wall_Operation::MessageHandler::MsgDt = 0; // Adjust this as per your requirement
uint32_t Wall_Operation::MessageHandler::MsgTS = 0;
Wall_Operation::MessageType Wall_Operation::MessageHandler::MsgTp = Wall_Operation::MessageType::MSG_NONE;
Wall_Operation::ErrorType Wall_Operation::MessageHandler::ErrTp = Wall_Operation::ErrorType::ERROR_NONE;
Wall_Operation::RegUnion Wall_Operation::MessageHandler::_U;

Wall_Operation::MessageHandler::MessageHandler()
    : msgid(0), msgtype(MessageType::MSG_NONE)
{
    msgtypstr[0] = '\0';
    LenQ = msg_queue_len;
    MsgTp = MessageType::MSG_NONE;
    ErrTp = ErrorType::ERROR_NONE;
}

void Wall_Operation::MessageHandler::set_all(uint8_t _msgid, MessageType _msgtype, const char *_msgtypstr, uint16_t *_Reg16)
{
    msgid = _msgid;
    msgtype = _msgtype;
    strncpy(msgtypstr, _msgtypstr, sizeof(msgtypstr) - 1); // create message type string
    msgtypstr[sizeof(msgtypstr) - 1] = '\0'; // ensure null-termination
    for (int i = 0; i < 8; i++) // copy Reg16 array for 8 registers
        Reg16[i] = _Reg16[i];
}

void Wall_Operation::MessageHandler::set_msgid(uint8_t _msgid)
{
    msgid = _msgid;
}

void Wall_Operation::MessageHandler::set_msg_type(MessageType _msgtype)
{
    msgtype = _msgtype;
}

void Wall_Operation::MessageHandler::set_typ_str(const char *_msgtypstr)
{
    strncpy(msgtypstr, _msgtypstr, sizeof(msgtypstr) - 1); // create message type string
    msgtypstr[sizeof(msgtypstr) - 1] = '\0'; // ensure null-termination
}

void Wall_Operation::MessageHandler::set_int16(uint16_t *_Reg16)
{
    for (int i = 0; i < 8; i++) // copy Reg16 array for 8 registers
        Reg16[i] = _Reg16[i];
}

uint8_t Wall_Operation::MessageHandler::get_id()
{
    return msgid;
}

Wall_Operation::MessageType Wall_Operation::MessageHandler::get_msg_mt()
{
    return msgtype;
}

const char* Wall_Operation::MessageHandler::get_msg_type_str()
{
    return msgtypstr;
}

uint16_t* Wall_Operation::MessageHandler::get_int16()
{
    return Reg16;
}

void Wall_Operation::MessageHandler::set_ui16(int p_reg[])
{
    // set full register
    for (int U16i = 0; U16i < 8; U16i++)
        _U.ui16[U16i] = p_reg[U16i];
}

void Wall_Operation::MessageHandler::set_ui16(int reg_i16)
{
    // Method implementation...
    return;
}

void Wall_Operation::MessageHandler::set_ui8(int reg_i18)
{
    // Method implementation...
    return;
}

void Wall_Operation::MessageHandler::set_ui16(int reg_i16, uint8_t set_u16_i)
{
    // Method implementation...
    return;
}

void Wall_Operation::MessageHandler::set_ui8(int reg_i8, uint8_t set_u8_i)
{
    // Method implementation...
    return;
}


// Default Constructor
		MessageQueueStruct();

		// Struct Methods: Setters
		void set_msgid(uint8_t _msgid)
		{
			msgid = _msgid;
		}
		void set_msg_type(MessageType _msgtype)
		{
			msgtype = _msgtype;
		}
		void set_typ_str(const char *_msgtypstr)
		{
			strncpy(msgtypstr, _msgtypstr, sizeof(msgtypstr) - 1); // create message type string
			msgtypstr[sizeof(msgtypstr) - 1] = '\0';			   // ensure null-termination
		}
		void set_int16(uint16_t *_Reg16)
		{
			for (int i = 0; i < 8; i++) // copy Reg16 array for 8 registers
				Reg16[i] = _Reg16[i];
		}

		// Struct Methods: Getters
		uint8_t get_id()
		{
			return msgid;
		}
		MessageType get_msg_mt()
		{
			return msgtype;
		}
		const char *get_msg_type_str()
		{
			return msgtypstr;
		}

		uint16_t *get_int16()
		{
			return Reg16;
		}

		// Struct Methods: Setters RegUnion

		// Create a method that will work with the U union to set the 8 16-bit registers, tracking both 16 and 8 bit index
		void set_ui16(int p_reg[])
		{
			// set full register
			for (int U16i = 0; U16i < 8; U16i++)
				_U.ui16[U16i] = p_reg[U16i];
		}
		// Store reg data in ui16[8] and and incriment ui16 and ui8 index
		void set_ui16(int reg_i16)
		{
			return;
		}
		// Store reg data in ui8[16] and and incriment ui16 and ui8 index
		void set_ui8(int reg_i18)
		{
			return;
		}
		// Store reg data in ui16[16] at index set_u16_i and update ui16 and ui8 index
		void set_ui16(int reg_i16, uint8_t set_u16_i)
		{
			return;
		}
		// Store reg data in ui8[16] at index set_u8_i and incriment ui16 and ui8 index
		void set_ui8(int reg_i8, uint8_t set_u8_i)
		{
			return;
		}