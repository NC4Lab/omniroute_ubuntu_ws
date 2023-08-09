/// @brief Update union 8 bit and 16 bit index
/// @return Last updated 8 bit index
uint8_t Wall_Operation::_get8ind(uint8_t b_i)
{
	_DB.printMsgTime("\t\t_upd8ind: u8i[%d] u16i[%d]", b8, b16); // TEMP
	b_i = b_i == 255 ? b8 : b_i;									 // if b_i is 255, use current union index
	b8 = b_i + 1;
	b16 = b8 / 2;
	return b_i;
}

/// @brief Update union 16 bit and 8 bit index
/// @return Last updated 16 bit index
uint8_t Wall_Operation::_get16ind(uint8_t b_i)
{
	_DB.printMsgTime("\t\_upd16ind: u8i[%d] u16i[%d]", b8, b16); // TEMP
	b_i = b_i == 255 ? b16 : b_i;								 // if b_i is 255, use current union index
	b16 = b_i + 1;
	b8 = b16 * 2;
	return b_i;
}