

enum MessageType
{
	MSG_NONE = 0,
	CONFIRM_DONE = 1,
	HANDSHAKE = 2,
	MOVE_WALLS = 3,
	START_SESSION = 4,
	END_SESSION = 5,
	ERROR = 6,
	N_MessageType
};
enum ErrorType
{
	ERROR_NONE = 0,
	MESSAGE_ID_DISORDERED = 1,
	NO_MESSAGE_TYPE_MATCH = 2,
	REGISTER_LEFTOVERS = 3,
	MISSING_FOOTER = 4
};

struct EcatMessageStruct ///< class for handeling ethercat messages
{
	int msgDt = 10;															   ///< delay between message send/write (ms)
	int msgID = 0;															   ///< Ethercat message ID
	Wall_Operation::MessageType msgTp = Wall_Operation::MessageType::MSG_NONE; ///< Ethercat message error
	Wall_Operation::ErrorType errTp = ErrorType::ERROR_NONE;				   ///< Ethercat message error
	char msg_tp_str[50] = {0};												   ///< Ethercat message type string
	char err_tp_str[50] = {0};												   ///< Ethercat error type string
	uint8_t msg_tp_val = 0;													   ///< Ethercat message type value
	uint8_t msg_arg_lng = 0;												   ///< Ethercat message argument length in bytes
	uint8_t ArgDat[10] = {0};												   ///< Ethercat message arguments
	RegUnion RegU;															   ///< union for storing ethercat 8 16-bit reg entries
	uint8_t u8i = 0;														   ///< index for RegUnion.ui8[16]
	uint8_t u16i = 0;														   ///< index for RegUnion.ui16[8]
	bool isDone = false;													   ///< flag for message exicution completion
};

EcatMessageStruct sndEM; ///<  initialize message handler instance for sending messages
EcatMessageStruct rcvEM; ///<  initialize message handler instance for receiving messages