#ifndef EEPROM_HPP
#define EEPROM_HPP

uint8_t  * const flyer_feeder_threshold_eeprom_addr		= (uint8_t  * const)(MAPPED_EEPROM_START + 0x0000);
uint8_t  * const debug_port_eeprom_addr					= (uint8_t  * const)(MAPPED_EEPROM_START + 0x0001);

void flush_eeprom_page_buffer(void* const  addr)
{
	cli();
	while((NVM_STATUS & NVM_NVMBUSY_bm) != 0){};
	uint16_t addrv = uint16_t(addr);
	addrv -= MAPPED_EEPROM_START;
	NVM_ADDR0 = addrv;
	NVM_ADDR1 = addrv >> 8;
	NVM_CMD = NVM_CMD_ERASE_WRITE_EEPROM_PAGE_gc;
	CCP = CCP_IOREG_gc;
	NVM_CTRLA = NVM_CMDEX_bm;
	while((NVM_STATUS & NVM_NVMBUSY_bm) != 0){};
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;
	sei();
}

#endif